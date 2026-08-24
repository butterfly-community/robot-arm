#include <openhmd.h>

#include <stdio.h>

static const char *
class_name(int device_class)
{
	switch (device_class) {
	case OHMD_DEVICE_CLASS_HMD: return "HMD";
	case OHMD_DEVICE_CLASS_CONTROLLER: return "CONTROLLER";
	case OHMD_DEVICE_CLASS_GENERIC_TRACKER: return "GENERIC_TRACKER";
	default: return "UNKNOWN";
	}
}

int
main(void)
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

	printf("devices=%d\n", count);
	for (int i = 0; i < count; i++) {
		int device_class = -1;
		int device_flags = 0;
		ohmd_list_geti(ctx, i, OHMD_DEVICE_CLASS, &device_class);
		ohmd_list_geti(ctx, i, OHMD_DEVICE_FLAGS, &device_flags);
		printf("%d: product='%s' class=%d (%s) flags=0x%x path='%s'\n", i,
		       ohmd_list_gets(ctx, i, OHMD_PRODUCT), device_class, class_name(device_class), device_flags,
		       ohmd_list_gets(ctx, i, OHMD_PATH));
	}

	ohmd_ctx_destroy(ctx);
	return 0;
}
