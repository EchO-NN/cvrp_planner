#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-${HOME}/isaacsim/_build/linux-x86_64/release}"
ISAAC_PYTHON="${ISAAC_SIM_ROOT}/python.sh"

if [[ ! -x "${ISAAC_PYTHON}" ]]; then
  echo "Isaac Sim python.sh 不存在或不可执行: ${ISAAC_PYTHON}" >&2
  exit 1
fi

filter_path() {
  local input_path="${1:-}"
  local output_path=""
  local part=""

  IFS=':' read -r -a path_parts <<< "${input_path}"
  for part in "${path_parts[@]}"; do
    [[ -z "${part}" ]] && continue
    case "${part}" in
      *"/miniconda3/bin"|*"/miniconda3/condabin"|*"/anaconda3/bin"|*"/anaconda3/condabin")
        continue
        ;;
    esac
    if [[ -z "${output_path}" ]]; then
      output_path="${part}"
    else
      output_path="${output_path}:${part}"
    fi
  done

  printf '%s\n' "${output_path}"
}

CLEAN_PATH="$(filter_path "${PATH}")"

echo "使用 Isaac Python: ${ISAAC_PYTHON}"
echo "导出原生 occupancy map"
echo "已清理环境变量: CONDA_*, PYTHONPATH, LD_LIBRARY_PATH, 以及 PATH 中的 conda 路径"

exec env \
  -u CONDA_DEFAULT_ENV \
  -u CONDA_EXE \
  -u CONDA_PREFIX \
  -u CONDA_PROMPT_MODIFIER \
  -u CONDA_PYTHON_EXE \
  -u CONDA_SHLVL \
  -u PYTHONPATH \
  -u PYTHONHOME \
  -u LD_LIBRARY_PATH \
  PATH="${CLEAN_PATH}" \
  "${ISAAC_PYTHON}" "${REPO_ROOT}/scripts/isaac/export_scene_occupancy_map.py" \
  --stage "${REPO_ROOT}/scripts/isaac/generated/full_warehouse_nav_robots.example.usd" \
  --output-image "${REPO_ROOT}/cvrp_planner/maps/full_warehouse_native_map.pgm" \
  --output-yaml "${REPO_ROOT}/cvrp_planner/maps/full_warehouse_native_map.yaml" \
  --output-params "${REPO_ROOT}/scripts/isaac/generated/full_warehouse_native_map.params.yaml" \
  --output-meta "${REPO_ROOT}/scripts/isaac/generated/full_warehouse_native_map.meta.json" \
  --exclude-robots \
  "$@"
