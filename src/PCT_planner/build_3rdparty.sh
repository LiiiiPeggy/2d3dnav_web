#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SOURCE_DIR="${ROOT_DIR}/3rdparty/src"
INSTALL_DIR="${ROOT_DIR}/3rdparty/install"
BUILD_DIR="${ROOT_DIR}/3rdparty/build"
BUILD_JOBS="${PCT_BUILD_JOBS:-2}"

mkdir -p "${SOURCE_DIR}" "${INSTALL_DIR}" "${BUILD_DIR}"

clone_tag() {
  local repository=$1
  local tag=$2
  local destination=$3
  [[ -d "${destination}/.git" ]] && return
  rm -rf -- "${destination}"
  local attempt
  for attempt in 1 2 3; do
    if git clone --depth 1 --filter=blob:none --single-branch \
      --branch "${tag}" --recurse-submodules --shallow-submodules \
      "${repository}" "${destination}"; then
      return
    fi
    echo "Clone attempt ${attempt} failed for ${repository}; retrying..." >&2
    rm -rf -- "${destination}"
  done
  echo "Unable to clone ${repository} after three attempts" >&2
  exit 1
}

build_gtsam() {
  local source="${SOURCE_DIR}/gtsam-4.2.0"
  local build="${BUILD_DIR}/gtsam-4.2.0"
  local prefix="${INSTALL_DIR}/gtsam"
  clone_tag https://github.com/borglab/gtsam.git 4.2.0 "${source}"
  cmake -S "${source}" -B "${build}" -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${prefix}" \
    -DCMAKE_BUILD_RPATH_USE_ORIGIN=ON \
    '-DCMAKE_INSTALL_RPATH=$ORIGIN' \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
  cmake --build "${build}" --parallel "${BUILD_JOBS}"
  cmake --install "${build}"
}

build_osqp() {
  local source="${SOURCE_DIR}/osqp-1.0.0"
  local build="${BUILD_DIR}/osqp-1.0.0"
  local prefix="${INSTALL_DIR}/osqp"
  clone_tag https://github.com/osqp/osqp.git v1.0.0 "${source}"
  cmake -S "${source}" -B "${build}" -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${prefix}" \
    -DOSQP_BUILD_UNITTESTS=OFF \
    -DOSQP_BUILD_DEMO_EXE=OFF
  cmake --build "${build}" --parallel "${BUILD_JOBS}"
  cmake --install "${build}"
}

case "${1:-all}" in
  gtsam) build_gtsam ;;
  osqp) build_osqp ;;
  all)
    build_gtsam
    build_osqp
    ;;
  *)
    echo "Usage: $0 {gtsam|osqp|all}" >&2
    exit 2
    ;;
esac

echo "PCT third-party dependencies installed under ${INSTALL_DIR}"
