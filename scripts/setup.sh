#!/usr/bin/env bash
# Reproducible setup: venv + pinned dependencies
set -euo pipefail
python3 -m venv .venv
./.venv/bin/pip install --upgrade pip
./.venv/bin/pip install -r requirements.txt
echo "Setup complete. Activate with: source .venv/bin/activate"
