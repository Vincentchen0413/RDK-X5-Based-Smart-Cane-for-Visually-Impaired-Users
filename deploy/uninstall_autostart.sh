#!/usr/bin/env bash
set -Eeuo pipefail
sudo systemctl disable --now smart-cane.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/smart-cane.service
sudo systemctl daemon-reload
echo "smart-cane.service removed"
