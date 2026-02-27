# Kinetra Development Agent

An AI-driven iterative development loop for the Kinetra trajectory planning library.

## Architecture

```
agent/
├── orchestrator.py      # Main loop: research → design → implement → test → benchmark → reflect
├── config.yaml          # Agent configuration (model, build paths, thresholds)
├── prompts/             # System & step prompts for each phase
│   ├── system.md
│   ├── research.md
│   ├── design.md
│   ├── implement.md
│   ├── test.md
│   ├── benchmark.md
│   └── reflect.md
├── tools/
│   ├── builder.py       # CMake configure + build wrapper
│   ├── tester.py        # CTest runner & result parser
│   ├── benchmarker.py   # Google Benchmark runner & comparison
│   ├── researcher.py    # Web search / paper lookup
│   └── reporter.py      # Markdown report & JSON log generation
├── logs/                # Per-iteration JSON logs
└── reports/             # Markdown & HTML summary reports
```

## Development Cycle

Each iteration follows the loop described by the project vision:

1. **Research** — Search the web for state-of-the-art algorithms, papers, and open-source implementations.
2. **Design** — Propose test cases and architectural changes based on findings.
3. **Implement** — Generate or modify C++ source code.
4. **Test** — Build and run the full test suite; analyse failures.
5. **Benchmark** — Run benchmarks, compare against previous iteration.
6. **Reflect** — Summarise learnings, update knowledge base, decide next focus.

## Usage

```bash
cd kinetra/agent
pip install -r requirements.txt
python orchestrator.py --iterations 10 --config config.yaml
```

## Configuration

See `config.yaml` for tuneable parameters such as:
- Build directory and CMake options
- Benchmark comparison thresholds (e.g., 5% regression alert)
- Research sources and API keys
- Logging verbosity
