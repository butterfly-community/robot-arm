#define _POSIX_C_SOURCE 200809L

#include <openhmd.h>

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static bool
parse_device_index(const char *text, int *value)
{
	char *end = NULL;
	errno = 0;
	long parsed = strtol(text, &end, 10);
	if (errno != 0 || end == text || *end != '\0' || parsed < 0 || parsed > INT_MAX) {
		return false;
	}
	*value = (int)parsed;
	return true;
}

static void
sleep_ms(long milliseconds)
{
	struct timespec delay = {
	    .tv_sec = milliseconds / 1000,
	    .tv_nsec = (milliseconds % 1000) * 1000000L,
	};
	nanosleep(&delay, NULL);
}

int
main(int argc, char **argv)
{
	ohmd_context *ctx = ohmd_ctx_create();
	if (ctx == NULL) {
		fprintf(stderr, "ohmd_ctx_create failed\n");
		return 1;
	}

	int count = ohmd_ctx_probe(ctx);
	if (count < 0) {
		fprintf(stderr, "ohmd_ctx_probe failed: %s\n", ohmd_ctx_get_error(ctx));
		ohmd_ctx_destroy(ctx);
		return 1;
	}

	int indices[16];
	ohmd_device *devices[16] = {0};
	int device_count = argc > 1 ? argc - 1 : 1;
	if (device_count > 16) {
		device_count = 16;
	}

	for (int i = 0; i < device_count; i++) {
		if (argc > 1 && !parse_device_index(argv[i + 1], &indices[i])) {
			fprintf(stderr, "invalid device index: %s\n", argv[i + 1]);
			ohmd_ctx_destroy(ctx);
			return 2;
		}
		if (argc == 1) {
			indices[i] = 0;
		}
		if (indices[i] < 0 || indices[i] >= count) {
			fprintf(stderr, "device index %d out of range (count=%d)\n", indices[i], count);
			ohmd_ctx_destroy(ctx);
			return 1;
		}

		devices[i] = ohmd_list_open_device(ctx, indices[i]);
		printf("open index=%d product='%s': %s\n", indices[i],
		       ohmd_list_gets(ctx, indices[i], OHMD_PRODUCT), devices[i] != NULL ? "ok" : "failed");
		if (devices[i] == NULL) {
			fprintf(stderr, "%s\n", ohmd_ctx_get_error(ctx));
		}
	}
	int open_count = 0;
	for (int i = 0; i < device_count; i++) {
		open_count += devices[i] != NULL;
	}
	if (open_count == 0) {
		ohmd_ctx_destroy(ctx);
		return 1;
	}

	for (int sample = 0; sample < 50; sample++) {
		ohmd_ctx_update(ctx);
		for (int i = 0; i < device_count; i++) {
			if (devices[i] == NULL) {
				continue;
			}
			float position[3] = {0};
			float orientation[4] = {0};
			ohmd_device_getf(devices[i], OHMD_POSITION_VECTOR, position);
			ohmd_device_getf(devices[i], OHMD_ROTATION_QUAT, orientation);
			printf("%04d index=%d p=(% .5f % .5f % .5f) q=(% .5f % .5f % .5f % .5f)\n", sample,
			       indices[i], position[0], position[1], position[2], orientation[0], orientation[1],
			       orientation[2], orientation[3]);
		}
		sleep_ms(100);
	}

	for (int i = 0; i < device_count; i++) {
		if (devices[i] != NULL) {
			ohmd_close_device(devices[i]);
		}
	}
	ohmd_ctx_destroy(ctx);
	return 0;
}
