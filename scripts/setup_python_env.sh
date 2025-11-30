#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
VENV_PATH=${1:-"${PROJECT_ROOT}/.venv"}
PYTHON_BIN=${PYTHON:-python3}

if [[ ! -x "$(command -v ${PYTHON_BIN})" ]]; then
  echo "[setup] Python interpreter '${PYTHON_BIN}' not found" >&2
  exit 1
fi

if [[ ! -d "${VENV_PATH}" ]]; then
  echo "[setup] Creating virtual environment at ${VENV_PATH}" 
  "${PYTHON_BIN}" -m venv "${VENV_PATH}"
else
  echo "[setup] Reusing existing virtual environment at ${VENV_PATH}"
fi

# shellcheck disable=SC1090
source "${VENV_PATH}/bin/activate"

python -m pip install --upgrade pip wheel setuptools
python -m pip install -r "${PROJECT_ROOT}/requirements.txt"

echo "[setup] Environment ready. Activate with: source ${VENV_PATH}/bin/activate"
