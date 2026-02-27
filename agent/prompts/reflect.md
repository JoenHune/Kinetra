# Reflect Phase

Summarise this iteration's outcomes and update the knowledge base.

## Tasks

1. List what worked and what didn't.
2. Record any numerical insights (e.g., "ADMM converges in <20 iterations for QPs up to 64 vars").
3. Identify the next most impactful focus area.
4. If a regression was detected, propose a rollback or optimisation plan.

## Output Format

Append to the knowledge base (JSONL):
```json
{
  "iteration": N,
  "learnings": ["..."],
  "next_focus": "...",
  "open_issues": ["..."]
}
```
