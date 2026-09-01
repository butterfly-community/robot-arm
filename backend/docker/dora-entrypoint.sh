#!/usr/bin/env bash
set -e

export DORA_COORDINATOR_ADDR="$(
  getent ahostsv4 "${DORA_COORDINATOR_HOST:-dora-coordinator}" | awk 'NR == 1 { print $1 }'
)"
exec "$@"
