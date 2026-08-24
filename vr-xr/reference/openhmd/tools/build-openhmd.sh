#!/bin/sh

set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
project_dir=$(dirname -- "$script_dir")
openhmd_source=${OPENHMD_SOURCE:-/home/life/Develop/temp/openhmd-src}
openhmd_build=${OPENHMD_BUILD:-"$openhmd_source/build"}
openhmd_prefix=${OPENHMD_PREFIX:-/home/life/Develop/temp/openhmd-install}
openhmd_base_revision=85075b0c7e3c723ded2577edb79d00ee11aac339
openhmd_patch="$project_dir/patches/openhmd-nolo-cv1.patch"
openhmd_patch_sha256=01413dca19ef873a11040024553c19e7669b1b79592e40c190dcdec91d8b6c59
fusion_source=${FUSION_SOURCE:-/home/life/Develop/temp/Fusion}
fusion_revision=d69784c8f7058a6545802b8852d26d6fcfd5e119
openhmd_test=/tmp/openhmd-nolo-unittests
build_jobs=${OPENHMD_BUILD_JOBS:-2}
check_only=false

if [ "$#" -gt 1 ] || { [ "$#" -eq 1 ] && [ "$1" != "--check" ]; }; then
	printf '%s\n' "usage: $0 [--check]" >&2
	exit 2
fi
if [ "$#" -eq 1 ]; then
	check_only=true
fi

actual_patch_sha256=$(sha256sum "$openhmd_patch" | awk '{print $1}')
if [ "$actual_patch_sha256" != "$openhmd_patch_sha256" ]; then
	printf '%s\n' "OpenHMD patch checksum mismatch: expected $openhmd_patch_sha256, got $actual_patch_sha256" >&2
	exit 1
fi

if ! git -C "$openhmd_source" cat-file -e "$openhmd_base_revision^{commit}"; then
	printf '%s\n' "OpenHMD base revision is missing: $openhmd_base_revision" >&2
	exit 1
fi

if ! git -C "$openhmd_source" merge-base --is-ancestor "$openhmd_base_revision" HEAD; then
	printf '%s\n' "OpenHMD HEAD is not based on $openhmd_base_revision" >&2
	exit 1
fi

if ! git -C "$openhmd_source" apply --reverse --check "$openhmd_patch"; then
	printf '%s\n' "Project OpenHMD patch is not fully applied: $openhmd_patch" >&2
	printf '%s\n' "Historical instructions: vr-xr/reference/openhmd/README.md." >&2
	exit 1
fi

actual_fusion_revision=$(git -C "$fusion_source" rev-parse HEAD)
if [ "$actual_fusion_revision" != "$fusion_revision" ]; then
	printf '%s\n' "Fusion revision mismatch: expected $fusion_revision, got $actual_fusion_revision" >&2
	exit 1
fi
if [ -n "$(git -C "$fusion_source" status --porcelain --untracked-files=all)" ]; then
	printf '%s\n' "Fusion source tree is not clean: $fusion_source" >&2
	exit 1
fi

cc -std=c99 -Wall -Wextra -Werror -Wno-unused-parameter \
	-I"$openhmd_source/include" \
	-I"$openhmd_source/src" \
	-I"$fusion_source/Fusion" \
	-I/usr/include/hidapi \
	"$openhmd_source/tests/unittests/nolo.c" \
	"$openhmd_source/src/drv_nolo/packet.c" \
	"$openhmd_source/src/drv_nolo/nolo_fusion.c" \
	"$fusion_source/Fusion/FusionAhrs.c" \
	"$fusion_source/Fusion/FusionBias.c" \
	-lm -o "$openhmd_test"
"$openhmd_test"

if [ "$check_only" = true ]; then
	printf '%s\n' "OpenHMD patch, Fusion source and NOLO unit tests: ok"
	exit 0
fi

cmake -S "$openhmd_source" -B "$openhmd_build" \
	-DBUILD_SHARED_LIBS=ON \
	-DCMAKE_BUILD_TYPE=RelWithDebInfo \
	-DCMAKE_INSTALL_PREFIX="$openhmd_prefix" \
	-DNOLO_FUSION_SOURCE_DIR="$fusion_source/Fusion" \
	-DOPENHMD_DRIVER_OCULUS_RIFT=OFF \
	-DOPENHMD_DRIVER_OCULUS_RIFT_S=OFF \
	-DOPENHMD_DRIVER_DEEPOON=OFF \
	-DOPENHMD_DRIVER_WMR=OFF \
	-DOPENHMD_DRIVER_PSVR=OFF \
	-DOPENHMD_DRIVER_HTC_VIVE=OFF \
	-DOPENHMD_DRIVER_NOLO=ON \
	-DOPENHMD_DRIVER_XGVR=OFF \
	-DOPENHMD_DRIVER_VRTEK=OFF \
	-DOPENHMD_DRIVER_EXTERNAL=OFF \
	-DOPENHMD_EXAMPLE_SIMPLE=ON

cmake --build "$openhmd_build" --parallel "$build_jobs"
cmake --install "$openhmd_build"

printf '%s\n' "OpenHMD installed in $openhmd_prefix"
