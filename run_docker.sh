#!/bin/bash
set -e
set -o pipefail

IMAGE_NAME="docker_image:rk3588"
IMAGE_REPO="docker_image"

TAR_DIR="/robotfs/data/docker_ws"
TAR_GZ_FILE="${TAR_DIR}/docker_image_rk3588.tar.gz"
TAR_FILE="${TAR_DIR}/docker_image_rk3588.tar"

HOME_TAR_GZ_FILE="/home/jw/docker_image_rk3588.tar.gz"
HOME_TAR_FILE="/home/jw/docker_image_rk3588.tar"

DOCKER_DATA_ROOT="/robotfs/data/docker"
CONTAINER_NAME="robot_runtime"

echo "=================================================="
echo "[INFO] RK3588 Docker Runtime Launcher"
echo "=================================================="

# ==================================================
# 1. 先检查 Docker 是否可用
# ==================================================
echo ""
echo "[INFO] 检查 Docker 是否可用..."

DOCKER_AVAILABLE=false

if command -v docker > /dev/null 2>&1; then
    if sudo docker info > /dev/null 2>&1; then
        DOCKER_AVAILABLE=true
        echo "[INFO] Docker 可用。"
    else
        echo "[WARN] Docker 命令存在，但 Docker 服务不可用。"
    fi
else
    echo "[WARN] 未找到 docker 命令。"
fi

# ==================================================
# 2. 如果 Docker 不可用，才做 overlayfs 检查、daemon.json 配置、重启 Docker
# ==================================================
if [ "${DOCKER_AVAILABLE}" = false ]; then
    echo ""
    echo "[INFO] Docker 不可用，开始检查和配置 Docker 环境..."

    # ==================================================
    # 2.1 检查内核是否支持 overlayfs
    # ==================================================
    echo ""
    echo "[INFO] 检查内核是否支持 overlayfs..."

    if cat /proc/filesystems | grep -w "overlay" > /dev/null 2>&1; then
        echo "[INFO] overlayfs supported."
    else
        echo "[ERROR] 当前内核不支持 overlayfs，无法使用 overlay2 存储驱动。"
        echo "请检查内核配置是否启用 overlayfs。"
        exit 1
    fi

    # ==================================================
    # 2.2 配置 Docker daemon.json
    # ==================================================
    echo ""
    echo "[INFO] 配置 Docker daemon.json..."

    sudo mkdir -p /etc/docker

    sudo tee /etc/docker/daemon.json > /dev/null <<'EOF'
{
  "data-root": "/robotfs/data/docker",
  "storage-driver": "overlay2",
  "iptables": false,
  "ip6tables": false,
  "bridge": "none",
  "ip-forward": false,
  "ip-masq": false,
  "features": {
    "containerd-snapshotter": false
  }
}
EOF

    echo "[INFO] Docker daemon.json 已写入。"

    # ==================================================
    # 2.3 检查 /robotfs/data/docker 是否存在
    # ==================================================
    echo ""
    echo "[INFO] 检查 Docker data-root 是否存在: ${DOCKER_DATA_ROOT}"

    if [ ! -d "${DOCKER_DATA_ROOT}" ]; then
        echo "[WARN] ${DOCKER_DATA_ROOT} 不存在，正在创建..."
        sudo mkdir -p "${DOCKER_DATA_ROOT}"
    fi

    echo "[INFO] Docker data-root exists: ${DOCKER_DATA_ROOT}"

    # ==================================================
    # 2.4 启动 / 重启 Docker
    # ==================================================
    echo ""
    echo "[INFO] 启动 / 重启 Docker 服务..."

    if command -v systemctl > /dev/null 2>&1; then
        sudo systemctl daemon-reload || true
        sudo systemctl reset-failed docker || true
        sudo systemctl restart containerd || true
        sudo systemctl restart docker
        sudo systemctl enable docker >/dev/null 2>&1 || true
    else
        sudo service docker restart
    fi

    sleep 2

    echo "[INFO] Docker 当前状态："
    sudo docker info | grep -E "Docker Root Dir|Storage Driver" || true

    # ==================================================
    # 2.5 再次确认 Docker 是否可用
    # ==================================================
    echo ""
    echo "[INFO] 再次检查 Docker 是否可用..."

    if command -v docker > /dev/null 2>&1 && sudo docker info > /dev/null 2>&1; then
        DOCKER_AVAILABLE=true
        echo "[INFO] Docker 已可用。"
    else
        echo "[ERROR] Docker 仍不可用。"
        echo "该脚本不负责安装 Docker，请先确认系统已安装 Docker。"
        exit 1
    fi
fi

# ==================================================
# 3. Docker 可用后，优先检查 /home/jw 下是否存在新镜像包
#    如果存在新镜像包：
#    1. 删除本机所有容器
#    2. 删除本机所有镜像
#    3. 移动新镜像包到 /robotfs/data/docker_ws
#    4. 后续重新导入镜像
# ==================================================
echo ""
echo "[INFO] Docker 可用，开始检查镜像包和镜像..."

echo "[INFO] 优先检查 home 目录镜像包:"
echo "  ${HOME_TAR_GZ_FILE}"
echo "  ${HOME_TAR_FILE}"

sudo mkdir -p "${TAR_DIR}"

NEED_LOAD_IMAGE=false

if [ -f "${HOME_TAR_GZ_FILE}" ] || [ -f "${HOME_TAR_FILE}" ]; then
    echo ""
    echo "[INFO] 在 /home/jw 下发现新的镜像包。"
    echo "[WARN] 将删除本地所有 Docker 容器和所有 Docker 镜像，然后导入新镜像。"

    # ==================================================
    # 3.1 删除所有容器
    # ==================================================
    echo ""
    echo "[INFO] 删除本地所有 Docker 容器..."

    ALL_CONTAINERS="$(sudo docker ps -aq || true)"

    if [ -n "${ALL_CONTAINERS}" ]; then
        echo "[INFO] 当前容器列表:"
        sudo docker ps -a
        echo ""
        echo "[INFO] 正在删除所有容器..."
        echo "${ALL_CONTAINERS}" | xargs -r sudo docker rm -f
    else
        echo "[INFO] 当前没有任何 Docker 容器。"
    fi

    # ==================================================
    # 3.2 删除所有镜像
    # ==================================================
    echo ""
    echo "[INFO] 删除本地所有 Docker 镜像..."

    ALL_IMAGES="$(sudo docker images -aq || true)"

    if [ -n "${ALL_IMAGES}" ]; then
        echo "[INFO] 当前镜像列表:"
        sudo docker images
        echo ""
        echo "[INFO] 正在删除所有镜像..."
        echo "${ALL_IMAGES}" | xargs -r sudo docker rmi -f
    else
        echo "[INFO] 当前没有任何 Docker 镜像。"
    fi

    # ==================================================
    # 3.3 清理 dangling / build cache
    # ==================================================
    echo ""
    echo "[INFO] 清理 Docker 残留缓存..."

    sudo docker image prune -f || true
    sudo docker builder prune -f || true

    # ==================================================
    # 3.4 移动新镜像包到 /robotfs/data/docker_ws
    # ==================================================
    echo ""
    echo "[INFO] 移动新镜像包到: ${TAR_DIR}"

    # 删除目标目录旧包，避免混用旧 tar
    if [ -f "${TAR_GZ_FILE}" ]; then
        echo "[INFO] 删除目标目录旧压缩包: ${TAR_GZ_FILE}"
        sudo rm -f "${TAR_GZ_FILE}"
    fi

    if [ -f "${TAR_FILE}" ]; then
        echo "[INFO] 删除目标目录旧 tar 文件: ${TAR_FILE}"
        sudo rm -f "${TAR_FILE}"
    fi

    if [ -f "${HOME_TAR_GZ_FILE}" ]; then
        echo "[INFO] 移动: ${HOME_TAR_GZ_FILE} -> ${TAR_GZ_FILE}"
        sudo mv "${HOME_TAR_GZ_FILE}" "${TAR_GZ_FILE}"
    elif [ -f "${HOME_TAR_FILE}" ]; then
        echo "[INFO] 移动: ${HOME_TAR_FILE} -> ${TAR_FILE}"
        sudo mv "${HOME_TAR_FILE}" "${TAR_FILE}"
    fi

    NEED_LOAD_IMAGE=true

else
    echo ""
    echo "[INFO] /home/jw 下没有发现新的镜像包。"
    echo "[INFO] 开始检查当前 Docker 中是否已有镜像: ${IMAGE_NAME}"

    if sudo docker image inspect "${IMAGE_NAME}" > /dev/null 2>&1; then
        echo "[INFO] 镜像已存在: ${IMAGE_NAME}"
        NEED_LOAD_IMAGE=false
    else
        echo "[WARN] 镜像不存在: ${IMAGE_NAME}"
        echo "[INFO] 将尝试从目标目录导入镜像包: ${TAR_DIR}"
        NEED_LOAD_IMAGE=true
    fi
fi

# ==================================================
# 4. 需要导入镜像时，执行解压和 docker load
# ==================================================
if [ "${NEED_LOAD_IMAGE}" = true ]; then
    echo ""
    echo "[INFO] 准备导入镜像..."

    cd "${TAR_DIR}"

    echo ""
    echo "[INFO] 检查目标目录中的镜像包..."

    if [ -f "${TAR_GZ_FILE}" ]; then
        echo "[INFO] 找到压缩包: ${TAR_GZ_FILE}"
        echo "[INFO] 解压 docker_image_rk3588.tar.gz..."
        sudo gunzip -f "${TAR_GZ_FILE}"
    elif [ -f "${TAR_FILE}" ]; then
        echo "[INFO] 已存在 tar 文件: ${TAR_FILE}"
    else
        echo "[ERROR] 没有找到可导入的镜像包："
        echo "  home 目录: ${HOME_TAR_GZ_FILE}"
        echo "  home 目录: ${HOME_TAR_FILE}"
        echo "  目标目录: ${TAR_GZ_FILE}"
        echo "  目标目录: ${TAR_FILE}"
        exit 1
    fi

    echo ""
    echo "[INFO] 导入 Docker 镜像: ${TAR_FILE}"

    if [ ! -f "${TAR_FILE}" ]; then
        echo "[ERROR] 解压后仍未找到 tar 文件: ${TAR_FILE}"
        exit 1
    fi

    sudo docker load -i "${TAR_FILE}"

    echo ""
    echo "[INFO] 确认镜像是否导入成功: ${IMAGE_NAME}"

    if ! sudo docker image inspect "${IMAGE_NAME}" > /dev/null 2>&1; then
        echo "[ERROR] 镜像导入后仍找不到: ${IMAGE_NAME}"
        echo ""
        echo "当前 ${IMAGE_REPO} 相关镜像："
        sudo docker images | grep "${IMAGE_REPO}" || true
        echo ""
        echo "请确认 tar 包内部镜像名是否真的是: ${IMAGE_NAME}"
        exit 1
    fi

    echo "[INFO] 镜像导入成功: ${IMAGE_NAME}"
fi

# ==================================================
# 5. 镜像存在后，显示镜像
# ==================================================
echo ""
echo "[INFO] 当前 ${IMAGE_REPO} 相关镜像："
sudo docker images | grep "${IMAGE_REPO}" || true

# ==================================================
# 6. 查看并清理 dangling 镜像
# ==================================================
echo ""
echo "[INFO] 查看所有 dangling 镜像："
sudo docker images -f "dangling=true"

echo ""
echo "[INFO] 清理 dangling 镜像..."
sudo docker image prune -f

# ==================================================
# 7. 检查数据保存目录是否存在
# ==================================================
echo ""
echo "[INFO] 检查数据保存目录: ~/docker_ws/robot_data"

if [ ! -d ~/docker_ws/robot_data ]; then
    echo "[WARN] 数据保存目录不存在，正在创建: ~/docker_ws/robot_data"
    sudo mkdir -p ~/docker_ws/robot_data
fi

# ==================================================
# 8. 不管容器是否存在，都删除旧容器
# ==================================================
echo ""
echo "[INFO] 检查并删除旧容器: ${CONTAINER_NAME}"

if sudo docker ps -a --format '{{.Names}}' | grep -wq "${CONTAINER_NAME}"; then
    echo "[INFO] 发现旧容器，正在删除: ${CONTAINER_NAME}"
    sudo docker rm -f "${CONTAINER_NAME}"
else
    echo "[INFO] 没有发现旧容器: ${CONTAINER_NAME}"
fi

# ==================================================
# 9. 基于当前镜像重新新建并运行容器
# ==================================================
echo ""
echo "[INFO] 创建并启动新容器: ${CONTAINER_NAME}"
echo "[INFO] 使用镜像: ${IMAGE_NAME}"

if ! sudo docker image inspect "${IMAGE_NAME}" > /dev/null 2>&1; then
    echo "[ERROR] 启动前检查失败，镜像不存在: ${IMAGE_NAME}"
    exit 1
fi

# 允许容器访问宿主机 X11 图形界面
# if command -v xhost > /dev/null 2>&1; then
#     xhost +local:root || true
# else
#     echo "[WARN] 未找到 xhost，图形界面可能无法显示。"
# fi

sudo docker run -it \
  --name "${CONTAINER_NAME}" \
  --net=host \
  --ipc=host \
  --privileged \
  -e ROS_DOMAIN_ID=0 \
  -e DISPLAY=192.168.50.50:0.0 \
  -e XAUTHORITY=/tmp/.Xauthority \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/.Xauthority:/tmp/.Xauthority:ro \
  -v ~/docker_ws/robot_data:/root/robot_data \
  "${IMAGE_NAME}" \
  bash -lc "
    grep -qxF 'export DISPLAY=192.168.50.50:0.0' /root/.bashrc || echo 'export DISPLAY=192.168.50.50:0.0' >> /root/.bashrc
    export DISPLAY=192.168.50.50:0.0
    exec bash
  "
#   \
#   bash -lc "
#     source /opt/ros/humble/setup.bash &&
#     source /root/robot_rk3588_ws/install/setup.bash &&
#     python3 /root/robot_rk3588_ws/src/navigation_workflow_gui.py
#   "