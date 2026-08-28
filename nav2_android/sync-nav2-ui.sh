#!/usr/bin/env bash
set -euo pipefail

android_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd -- "${android_dir}/.." && pwd)"
web_source="${workspace_root}/src/web/nav2_web/web"
android_assets="${android_dir}/nav2-app/src/main/assets/nav2_web"

for asset in index.html style.css app.js; do
  if [[ ! -f "${web_source}/${asset}" ]]; then
    echo "错误：找不到 Web UI 文件 ${web_source}/${asset}" >&2
    exit 1
  fi
done

mkdir -p "${android_assets}"
for asset in index.html style.css app.js; do
  install -m 0644 "${web_source}/${asset}" "${android_assets}/${asset}"
done

echo "已同步 Web UI → Android assets"
echo "源目录：${web_source}"
echo "目标目录：${android_assets}"
