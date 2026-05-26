#!/bin/bash
# 每次 colcon build 后运行，修复 ament_python 包的注册问题：
# 1. 将 ament_prefix_path hook 注入 package.dsv
# 2. 在 libexec 目录中创建可执行文件的符号链接

PACKAGE="fast_lio_localization"
REPO_DIR="$(cd "$(dirname "$0")/../../.." && pwd)"
INSTALL_DIR="${REPO_DIR}/install/${PACKAGE}"
HOOK_ENTRY="source;share/${PACKAGE}/hook/ament_prefix_path.sh"
DSV_FILE="${INSTALL_DIR}/share/${PACKAGE}/package.dsv"

# 1. 修复 package.dsv
if ! grep -q "${HOOK_ENTRY}" "${DSV_FILE}" 2>/dev/null; then
  echo "${HOOK_ENTRY}" >> "${DSV_FILE}"
  echo "Added ament_prefix_path hook to ${DSV_FILE}"
fi

# 2. 在 libexec 目录中创建可执行文件的符号链接
LIBEXEC_DIR="${INSTALL_DIR}/lib/${PACKAGE}"
BIN_DIR="${INSTALL_DIR}/bin"

mkdir -p "${LIBEXEC_DIR}"
if [ -d "${BIN_DIR}" ]; then
  for exe in "${BIN_DIR}"/*; do
    exe_name=$(basename "$exe")
    link_path="${LIBEXEC_DIR}/${exe_name}"
    if [ ! -e "${link_path}" ] || [ -L "${link_path}" ]; then
      ln -sf "${exe}" "${link_path}"
      echo "Linked ${link_path} -> ${exe}"
    fi
  done
fi
