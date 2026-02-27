"""
Web search / paper lookup tool.

When a proper search API key is configured this module queries arXiv and GitHub.
Otherwise it returns cached/stub results so the agent loop can still run offline.
"""

from __future__ import annotations

import hashlib
import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import requests
except ImportError:
    requests = None  # type: ignore


@dataclass
class Researcher:
    cache_dir: str = "logs/research_cache"
    enabled: bool = True

    def __post_init__(self):
        Path(self.cache_dir).mkdir(parents=True, exist_ok=True)

    def search(self, query: str, max_results: int = 5) -> list[dict[str, Any]]:
        """Return a list of references (title, url, snippet)."""
        if not self.enabled:
            return []

        # Check cache first
        cached = self._cache_get(query)
        if cached is not None:
            return cached

        results: list[dict[str, Any]] = []
        results.extend(self._search_arxiv(query, max_results))
        results.extend(self._search_github(query, max_results))

        self._cache_put(query, results)
        return results

    # ── arXiv ─────────────────────────────────────────────────────────────

    def _search_arxiv(self, query: str, max_results: int) -> list[dict[str, Any]]:
        """Query arXiv API (no key required)."""
        if requests is None:
            return []
        try:
            url = "http://export.arxiv.org/api/query"
            params = {
                "search_query": f"all:{query}",
                "max_results": max_results,
                "sortBy": "relevance",
            }
            resp = requests.get(url, params=params, timeout=15)
            if resp.status_code != 200:
                return []

            # Minimal XML parsing (avoid lxml dependency)
            import xml.etree.ElementTree as ET
            root = ET.fromstring(resp.text)
            ns = {"atom": "http://www.w3.org/2005/Atom"}
            results = []
            for entry in root.findall("atom:entry", ns):
                title = entry.findtext("atom:title", "", ns).strip()
                summary = entry.findtext("atom:summary", "", ns).strip()[:300]
                link_el = entry.find("atom:id", ns)
                link = link_el.text.strip() if link_el is not None and link_el.text else ""
                results.append({
                    "source": "arxiv",
                    "title": title,
                    "url": link,
                    "snippet": summary,
                })
            return results
        except Exception:
            return []

    # ── GitHub ────────────────────────────────────────────────────────────

    def _search_github(self, query: str, max_results: int) -> list[dict[str, Any]]:
        """Search GitHub repos (unauthenticated, rate-limited)."""
        if requests is None:
            return []
        try:
            url = "https://api.github.com/search/repositories"
            headers = {"Accept": "application/vnd.github+json"}
            token = os.environ.get("GITHUB_TOKEN")
            if token:
                headers["Authorization"] = f"Bearer {token}"
            params = {
                "q": query,
                "sort": "stars",
                "per_page": max_results,
            }
            resp = requests.get(url, params=params, headers=headers, timeout=15)
            if resp.status_code != 200:
                return []
            data = resp.json()
            results = []
            for item in data.get("items", []):
                results.append({
                    "source": "github",
                    "title": item.get("full_name", ""),
                    "url": item.get("html_url", ""),
                    "snippet": (item.get("description") or "")[:300],
                    "stars": item.get("stargazers_count", 0),
                })
            return results
        except Exception:
            return []

    # ── Cache ─────────────────────────────────────────────────────────────

    def _cache_key(self, query: str) -> str:
        return hashlib.sha256(query.encode()).hexdigest()[:16]

    def _cache_get(self, query: str) -> list[dict[str, Any]] | None:
        path = Path(self.cache_dir) / f"{self._cache_key(query)}.json"
        if path.exists():
            with open(path) as f:
                return json.load(f)
        return None

    def _cache_put(self, query: str, data: list[dict[str, Any]]) -> None:
        path = Path(self.cache_dir) / f"{self._cache_key(query)}.json"
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
