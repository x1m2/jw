# Shared run directory and file paths for the offline launchers.
offline_new_run() {
    local prefix=$1
    mkdir -p "$OFFLINE_DIR/runs"
    DATA_DIR=$(mktemp -d "$OFFLINE_DIR/runs/$prefix.XXXXXX")
    mkdir -p "$DATA_DIR/log/ros" "$DATA_DIR/runtime"
    export LOG_ROOT="$DATA_DIR/log" ROS_LOG_DIR="$DATA_DIR/log/ros"
    export ROS_HOME="$DATA_DIR/runtime" TMPDIR="$DATA_DIR/runtime"
    export XDG_CACHE_HOME="$DATA_DIR/runtime/cache"
    export XDG_CONFIG_HOME="$DATA_DIR/runtime/config" XDG_DATA_HOME="$DATA_DIR/runtime/data"
    export PYTHONDONTWRITEBYTECODE=1
    printf '[INFO] 结果目录 %s\n' "$DATA_DIR"
    cd "$DATA_DIR"
}

offline_redirect_logs() {
    # FAST-LIO uses a compiled-in IMU log path. RViz stores settings under Qt's home path.
    local qt_cflags
    local -a qt_flags
    qt_cflags=$(pkg-config --cflags Qt5Core) || return 1
    read -r -a qt_flags <<< "$qt_cflags"
    export AIRY_IMU_LOG_SOURCE="$WORKSPACE_DIR/src/slam/mapping/fast_lio/Log/imu.txt"
    export AIRY_IMU_LOG_OUTPUT="$DATA_DIR/log/imu.txt"
    export AIRY_RVIZ_SETTINGS_HOME="$DATA_DIR/runtime"
    export AIRY_FILE_REDIRECT="$DATA_DIR/runtime/file_redirect.so"
    c++ -shared -fPIC "${qt_flags[@]}" -x c++ -o "$AIRY_FILE_REDIRECT" - -lQt5Core -ldl <<'CPP'
#include <QDir>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>

extern "C" FILE *fopen64(const char *path, const char *mode) {
    using OpenFile = FILE *(*)(const char *, const char *);
    const auto open_file = reinterpret_cast<OpenFile>(dlsym(RTLD_NEXT, "fopen64"));
    const char *source = std::getenv("AIRY_IMU_LOG_SOURCE");
    const char *output = std::getenv("AIRY_IMU_LOG_OUTPUT");
    if (source && output && std::strcmp(path, source) == 0) {
        path = output;
    }
    return open_file(path, mode);
}

QString QDir::homePath() {
    return QString::fromLocal8Bit(std::getenv("AIRY_RVIZ_SETTINGS_HOME"));
}
CPP
}
