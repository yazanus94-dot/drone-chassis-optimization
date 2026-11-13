FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libgl1 libglu1-mesa libglx-mesa0 libgomp1 libglib2.0-0 libx11-6 libxext6 libfontconfig1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY src/ ./src/
COPY cad/ mesh/ simulation/ battery/ optimization/ scripts/ benchmarks/ docs/ ./
COPY run_all.py ./
COPY pyproject.toml README.md LICENSE ./

RUN useradd --create-home pipeline && chown -R pipeline:pipeline /app
USER pipeline

ENV PYTHONPATH=/app/src
CMD ["python", "run_all.py", "--out", "results"]
