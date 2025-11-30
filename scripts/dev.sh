#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

mkdir -p "${ROOT_DIR}/datasets" "${ROOT_DIR}/models"

export HOST_DATASET_PATH="${HOST_DATASET_PATH:-${ROOT_DIR}/datasets}"
export HOST_MODEL_PATH="${HOST_MODEL_PATH:-${ROOT_DIR}/models}"

docker compose -f "${ROOT_DIR}/docker-compose.yml" up -d --build "$@"
