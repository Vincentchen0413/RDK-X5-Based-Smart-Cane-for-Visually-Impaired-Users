#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEMPLATE="${ROOT}/deploy/systemd/smart-cane.service.in"
TARGET="/etc/systemd/system/smart-cane.service"
RUN_USER="${SUDO_USER:-$(id -un)}"
RUN_GROUP="$(id -gn "${RUN_USER}")"

mkdir -p "${ROOT}/logs" "${ROOT}/run"
chmod +x "${ROOT}/scripts/"*.sh
chmod +x "${ROOT}/bringup/"*.py "${ROOT}/adapters/"*.py \
  "${ROOT}/detection/runtime/"*.py "${ROOT}/scripts/"*.py

sed \
  -e "s|@ROOT@|${ROOT}|g" \
  -e "s|@USER@|${RUN_USER}|g" \
  -e "s|@GROUP@|${RUN_GROUP}|g" \
  "${TEMPLATE}" | sudo tee "${TARGET}" >/dev/null

sudo systemctl daemon-reload
sudo systemctl enable smart-cane.service

echo "Installed ${TARGET}"
echo "Start with: sudo systemctl start smart-cane.service"
echo "Logs: journalctl -u smart-cane.service -f"
