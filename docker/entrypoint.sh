#!/bin/bash
set -e
# shellcheck disable=SC1091
source /ws/docker/env.sh
if ! grep -q 'source /ws/docker/env.sh' /root/.bashrc 2>/dev/null; then
  echo 'source /ws/docker/env.sh' >> /root/.bashrc
fi
cd /ws
exec "${@:-sleep}" infinity
