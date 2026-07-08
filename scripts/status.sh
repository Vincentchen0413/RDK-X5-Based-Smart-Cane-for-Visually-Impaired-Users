#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if systemctl list-unit-files | grep -q '^smart-cane.service'; then
  systemctl --no-pager --full status smart-cane.service || true
else
  pgrep -af "smart_cane_supervisor.py|smart_cane_|rtabmap|openvins" || true
fi

echo
echo "Recent logs:"
find "${ROOT}/logs" -maxdepth 1 -type f -name '*.log' -printf '%TY-%Tm-%Td %TH:%TM %p\n' 2>/dev/null \
  | sort -r | head -n 10
