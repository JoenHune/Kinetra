# Research Phase

Search for state-of-the-art papers, algorithms, and implementations related to the
current focus area: **{focus}**.

## Tasks

1. Query arXiv for recent papers on the focus topic + "trajectory planning" or
   "motion planning" or "trajectory optimization".
2. Query GitHub for high-star repositories implementing related algorithms.
3. Summarise the top findings: key idea, relevance to Kinetra, potential improvement.

## Output Format

Return a JSON list of references:
```json
[
  {
    "title": "...",
    "url": "...",
    "key_idea": "...",
    "relevance": "HIGH | MEDIUM | LOW",
    "actionable": "One sentence describing what we could implement."
  }
]
```
