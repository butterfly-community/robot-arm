#!/usr/bin/env bash
set -e

device_root=/opt/devices/stararm-102
project_root="$device_root/project"
vendor_root="$device_root/vendor"
archive=/opt/build/downloads/stararm-102.tar.gz

curl --fail --silent --show-error --location \
  --retry 5 --retry-all-errors --retry-delay 2 \
  -o "$archive" "$1"
echo "$2  $archive" | sha256sum --check --strict
mkdir -p "$vendor_root"
tar -xzf "$archive" --strip-components=1 -C "$vendor_root"
git -C "$vendor_root" init --quiet
python3 "$project_root/tools/verify-model.py" "$vendor_root" vendor
for patch in model dynamics topic-io; do
  git -C "$vendor_root" apply --ignore-space-change --ignore-whitespace \
    "$project_root/patches/$patch.patch"
done
python3 "$project_root/tools/verify-model.py" "$vendor_root" patched
