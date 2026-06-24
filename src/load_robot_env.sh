#!/bin/bash

ROBOT_ENV_LOADER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_ENV_FILE="${ROBOT_ENV_LOADER_DIR}/robot.env"

if [ ! -f "${ROBOT_ENV_FILE}" ]; then
    echo "[ERROR] 找不到机器人环境配置: ${ROBOT_ENV_FILE}" >&2
    return 1 2>/dev/null || exit 1
fi

set -a
# shellcheck disable=SC1090
source "${ROBOT_ENV_FILE}"
set +a

: "${ROBOT_HOME:?robot.env 未配置 ROBOT_HOME}"
: "${ROBOT_WS:?robot.env 未配置 ROBOT_WS}"
: "${ROBOT_DATA_DIR:?robot.env 未配置 ROBOT_DATA_DIR}"

export ROBOT_HOME ROBOT_WS ROBOT_DATA_DIR
