# Contributing

1. Fork and create a feature branch (`git checkout -b feat/thing`).
2. Set up the environment: `python -m venv .venv && .venv/Scripts/pip install -r requirements.txt` (or `pip install -e ".[dev]"`).
3. Run the checks before opening a PR:
   ```bash
   .venv/Scripts/python -m ruff check src tests cad mesh simulation battery optimization
   .venv/Scripts/python -m mypy src
   .venv/Scripts/python -m pytest -v
   ```
4. Any physics-model change must keep its oracle test green (see `docs/verification-strategy.md`); update the oracle if the model genuinely changes.
5. Keep code style: full type hints, docstrings allowed, no inline comments.
6. Commit with a concise imperative message; PRs are squashed.
