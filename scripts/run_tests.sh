#!/usr/bin/env bash
# Run the full test suite with coverage
set -euo pipefail
./.venv/bin/python -m pytest tests/unit tests/property tests/integration -v
./.venv/bin/python -m pytest tests/unit tests/property --cov=drone_chassis --cov-report=term-missing
