#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run with sudo: sudo bash /home/mason/AutoCar/fix_apt_sources.sh" >&2
  exit 1
fi

cp -n /etc/apt/sources.list /etc/apt/sources.list.bak.autocar
sed -i \
  -e 's|archive.ubuntu.com|mirrors.aliyun.com|g' \
  -e 's|security.ubuntu.com|mirrors.aliyun.com|g' \
  /etc/apt/sources.list

if [[ -f /etc/apt/sources.list.d/nodesource.sources ]]; then
  mv /etc/apt/sources.list.d/nodesource.sources /etc/apt/sources.list.d/nodesource.sources.disabled
fi

apt clean
apt update

echo
echo "Current Ubuntu apt sources:"
grep -E 'archive|security|aliyun' /etc/apt/sources.list || true
