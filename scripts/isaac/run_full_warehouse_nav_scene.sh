#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-${HOME}/isaacsim/_build/linux-x86_64/release}"
ISAAC_PYTHON="${ISAAC_SIM_ROOT}/python.sh"

if [[ ! -x "${ISAAC_PYTHON}" ]]; then
  echo "找不到 Isaac Sim 的 python.sh: ${ISAAC_PYTHON}" >&2
  echo "请先设置 ISAAC_SIM_ROOT，例如：" >&2
  echo "  export ISAAC_SIM_ROOT=${HOME}/isaacsim/_build/linux-x86_64/release" >&2
  exit 1
fi

CONFIG_PATH="${REPO_ROOT}/scripts/isaac/full_warehouse_nav_robots.example.json"
OUTPUT_PATH=""

if [[ "${1:-}" != "" && "${1}" != -* ]]; then
  CONFIG_PATH="${1}"
  shift
fi

if [[ "${1:-}" != "" && "${1}" != -* ]]; then
  OUTPUT_PATH="${1}"
  shift
fi

EXTRA_ARGS=("$@")

CMD=(
  "${ISAAC_PYTHON}"
  "${REPO_ROOT}/scripts/isaac/build_full_warehouse_nav_scene.py"
  "--config"
  "${CONFIG_PATH}"
)

if [[ -n "${OUTPUT_PATH}" ]]; then
  CMD+=("--save-path" "${OUTPUT_PATH}")
fi

if [[ "${#EXTRA_ARGS[@]}" -gt 0 ]]; then
  CMD+=("${EXTRA_ARGS[@]}")
fi

echo "使用 Isaac Python: ${ISAAC_PYTHON}"
echo "导航场景配置文件: ${CONFIG_PATH}"
if [[ -n "${OUTPUT_PATH}" ]]; then
  echo "输出场景: ${OUTPUT_PATH}"
fi
if [[ "${#EXTRA_ARGS[@]}" -gt 0 ]]; then
  echo "额外参数: ${EXTRA_ARGS[*]}"
fi

exec "${CMD[@]}"
