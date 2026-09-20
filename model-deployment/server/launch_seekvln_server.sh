#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="/mnt/storage2/users/zmwang/SeekVLN:${ROOT}:${PYTHONPATH:-}"
export SEEKVLN_MODEL_PATH="${SEEKVLN_MODEL_PATH:-/mnt/storage2/users/zmwang/SeekVLN/model_exports/20260819_SeekVLN-Full-SFT-V2-3-4K-20260817-180914_global_step_9261_hf_full}"
# CUDA_VISIBLE_DEVICES remaps the selected physical GPU to cuda:0 inside the
# process, avoiding stale scheduler mappings and invalid device ordinals.
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-6}"
export SEEKVLN_DEVICE="${SEEKVLN_DEVICE:-cuda:0}"
export USE_TF="${USE_TF:-0}"
PYTHON="${SEEKVLN_PYTHON:-/mnt/storage2/users/zmwang/envs/seekvln-verl/bin/python3.10}"
exec "$PYTHON" -m uvicorn seekvln_server:app --host 127.0.0.1 --port 8012
