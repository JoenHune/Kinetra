"""
Unified LLM client — OpenAI-compatible API.

Works with:
  - OpenAI (GPT-4, etc.)
  - Anthropic (via OpenAI-compatible proxy or native)
  - Ollama (local models)
  - vLLM / TGI / LiteLLM
  - Any endpoint that speaks the OpenAI chat-completions protocol

Configuration is via config.yaml → llm section.
"""

from __future__ import annotations

import json
import logging
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:
    import requests
except ImportError:
    requests = None  # type: ignore

log = logging.getLogger("kinetra.agent.llm")


@dataclass
class Message:
    role: str          # "system" | "user" | "assistant"
    content: str


@dataclass
class LLMConfig:
    """Configuration for the LLM backend."""
    provider: str = "openai"           # openai | anthropic | ollama | custom
    model: str = "gpt-4o"
    api_base: str = ""                 # Override base URL (e.g. http://localhost:11434/v1)
    api_key: str = ""                  # Will also check env vars
    temperature: float = 0.3
    max_tokens: int = 4096
    timeout: int = 120

    def resolve_api_key(self) -> str:
        """Resolve API key from config or environment."""
        if self.api_key:
            return self.api_key
        env_map = {
            "openai": "OPENAI_API_KEY",
            "anthropic": "ANTHROPIC_API_KEY",
            "deepseek": "DEEPSEEK_API_KEY",
        }
        env_var = env_map.get(self.provider, "LLM_API_KEY")
        return os.environ.get(env_var, "")

    def resolve_api_base(self) -> str:
        """Resolve base URL."""
        if self.api_base:
            return self.api_base.rstrip("/")
        defaults = {
            "openai": "https://api.openai.com/v1",
            "anthropic": "https://api.anthropic.com/v1",
            "deepseek": "https://api.deepseek.com/v1",
            "ollama": "http://localhost:11434/v1",
        }
        return defaults.get(self.provider, "http://localhost:8000/v1")


@dataclass
class LLMResponse:
    content: str
    model: str = ""
    usage: dict[str, int] = field(default_factory=dict)
    finish_reason: str = ""
    raw: dict[str, Any] = field(default_factory=dict)


class LLMClient:
    """Unified chat-completions client."""

    def __init__(self, config: LLMConfig):
        self.config = config
        if requests is None:
            raise ImportError("requests library required for LLM client")

    def chat(
        self,
        messages: list[Message],
        temperature: float | None = None,
        max_tokens: int | None = None,
        json_mode: bool = False,
    ) -> LLMResponse:
        """Send a chat completion request and return the response."""
        api_base = self.config.resolve_api_base()
        api_key = self.config.resolve_api_key()

        url = f"{api_base}/chat/completions"
        headers: dict[str, str] = {"Content-Type": "application/json"}
        if api_key:
            headers["Authorization"] = f"Bearer {api_key}"

        payload: dict[str, Any] = {
            "model": self.config.model,
            "messages": [{"role": m.role, "content": m.content} for m in messages],
            "temperature": temperature if temperature is not None else self.config.temperature,
            "max_tokens": max_tokens or self.config.max_tokens,
        }
        if json_mode:
            payload["response_format"] = {"type": "json_object"}

        log.debug("LLM request → %s  model=%s  msgs=%d",
                  url, self.config.model, len(messages))

        t0 = time.monotonic()
        try:
            resp = requests.post(
                url,
                headers=headers,
                json=payload,
                timeout=self.config.timeout,
            )
        except requests.exceptions.RequestException as e:
            log.error("LLM request failed: %s", e)
            return LLMResponse(content="", finish_reason="error",
                               raw={"error": str(e)})

        elapsed = time.monotonic() - t0
        log.debug("LLM response ← %d  %.1fs", resp.status_code, elapsed)

        if resp.status_code != 200:
            log.error("LLM HTTP %d: %s", resp.status_code, resp.text[:500])
            return LLMResponse(
                content="",
                finish_reason="error",
                raw={"status_code": resp.status_code, "body": resp.text[:1000]},
            )

        data = resp.json()
        choice = data.get("choices", [{}])[0]
        message = choice.get("message", {})

        return LLMResponse(
            content=message.get("content", ""),
            model=data.get("model", self.config.model),
            usage=data.get("usage", {}),
            finish_reason=choice.get("finish_reason", ""),
            raw=data,
        )

    def chat_json(
        self,
        messages: list[Message],
        temperature: float | None = None,
    ) -> dict[str, Any] | None:
        """Chat and parse the response as JSON. Returns None on failure."""
        response = self.chat(messages, temperature=temperature, json_mode=True)
        if not response.content:
            return None
        try:
            return json.loads(response.content)
        except json.JSONDecodeError:
            # Try to extract JSON from markdown code block
            content = response.content
            if "```json" in content:
                content = content.split("```json", 1)[1]
                content = content.split("```", 1)[0]
            elif "```" in content:
                content = content.split("```", 1)[1]
                content = content.split("```", 1)[0]
            try:
                return json.loads(content.strip())
            except json.JSONDecodeError:
                log.error("Failed to parse LLM JSON response:\n%s",
                          response.content[:500])
                return None

    @staticmethod
    def from_config(cfg: dict[str, Any]) -> "LLMClient":
        """Create client from config.yaml llm section."""
        llm_cfg = LLMConfig(
            provider=cfg.get("provider", "openai"),
            model=cfg.get("model", "gpt-4o"),
            api_base=cfg.get("api_base", ""),
            api_key=cfg.get("api_key", ""),
            temperature=cfg.get("temperature", 0.3),
            max_tokens=cfg.get("max_tokens", 4096),
            timeout=cfg.get("timeout", 120),
        )
        return LLMClient(llm_cfg)
