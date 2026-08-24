#!/bin/sh

set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
project_dir=$(dirname -- "$script_dir")
monado_source=${MONADO_SOURCE:-/home/life/Develop/temp/monado-src}
openhmd_prefix=${OPENHMD_PREFIX:-/home/life/Develop/temp/openhmd-install}
output_dir=${VR_XR_BUILD_DIR:-"$project_dir/build"}
c_compiler=${CC:-cc}

openxr_include="$monado_source/src/external/openxr_includes"
if [ ! -f "$openxr_include/openxr/openxr.h" ]; then
	printf '%s\n' "OpenXR headers not found: $openxr_include" >&2
	exit 1
fi
if [ ! -f "$openhmd_prefix/include/openhmd.h" ]; then
	printf '%s\n' "OpenHMD headers not found: $openhmd_prefix/include" >&2
	exit 1
fi

mkdir -p "$output_dir"

"$c_compiler" -std=c11 -O2 -Wall -Wextra -Werror \
	-I"$openhmd_prefix/include" \
	"$project_dir/tests/openhmd-enumerate.c" \
	-L"$openhmd_prefix/lib" -Wl,-rpath,"$openhmd_prefix/lib" \
	-lopenhmd -lm -o "$output_dir/openhmd-enumerate"

"$c_compiler" -std=c11 -O2 -Wall -Wextra -Werror \
	-I"$openhmd_prefix/include" \
	"$project_dir/tests/openhmd-pose-monitor.c" \
	-L"$openhmd_prefix/lib" -Wl,-rpath,"$openhmd_prefix/lib" \
	-lopenhmd -lm -o "$output_dir/openhmd-pose-monitor"

"$c_compiler" -std=c11 -O2 -Wall -Wextra -Werror \
	-I"$openxr_include" \
	"$project_dir/tests/nolo-xdev-monitor.c" \
	-lopenxr_loader -o "$output_dir/nolo-xdev-monitor"

"$c_compiler" -std=c11 -O2 -Wall -Wextra -Werror \
	-I"$openxr_include" \
	"$project_dir/tests/nolo-controller-stream.c" \
	-lopenxr_loader -o "$output_dir/nolo-controller-stream"

printf '%s\n' "VR/XR tools built in $output_dir"
