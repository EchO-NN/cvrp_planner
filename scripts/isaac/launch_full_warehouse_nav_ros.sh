#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

CONFIG_PATH="${REPO_ROOT}/scripts/isaac/full_warehouse_nav_robots.example.json"
OUTPUT_PATH="${REPO_ROOT}/scripts/isaac/generated/full_warehouse_nav_robots.example.usd"
REBUILD=1

if [[ "${1:-}" != "" && "${1}" != -* ]]; then
  CONFIG_PATH="${1}"
  shift
fi

if [[ "${1:-}" != "" && "${1}" != -* ]]; then
  OUTPUT_PATH="${1}"
  shift
fi

for arg in "$@"; do
  case "${arg}" in
    --no-rebuild)
      REBUILD=0
      ;;
    *)
      echo "不支持的参数: ${arg}" >&2
      echo "用法: $0 [config.json] [output.usd] [--no-rebuild]" >&2
      exit 1
      ;;
  esac
done

if [[ "${REBUILD}" -eq 1 ]]; then
  echo "生成 ROS-ready 导航场景..."
  bash "${SCRIPT_DIR}/run_full_warehouse_nav_scene.sh" "${CONFIG_PATH}" "${OUTPUT_PATH}"
else
  echo "跳过重新生成，直接打开已有场景..."
fi

echo "启动 Isaac Sim 并打开导航场景..."
exec bash "${SCRIPT_DIR}/open_stage_with_ros.sh" "${OUTPUT_PATH}"
