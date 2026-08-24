#define _POSIX_C_SOURCE 200809L

#include <openxr/openxr.h>
#include <openxr/XR_MNDX_xdev_space.h>

#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define CHECK_XR(call)                                                                                                 \
	do {                                                                                                           \
		XrResult result_ = (call);                                                                              \
		if (XR_FAILED(result_)) {                                                                               \
			fprintf(stderr, "%s failed: %d\n", #call, result_);                                             \
			goto cleanup;                                                                                     \
		}                                                                                                      \
	} while (false)

#define LOAD_XR(name)                                                                                                  \
	do {                                                                                                           \
		CHECK_XR(xrGetInstanceProcAddr(instance, #name, (PFN_xrVoidFunction *)&pfn_##name));                    \
	} while (false)

struct monitored_device
{
	XrXDevIdMNDX id;
	XrSpace space;
	char name[256];
	bool has_space;
};

static XrTime
monotonic_now_ns(void)
{
	struct timespec ts = {0};
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (XrTime)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static bool
has_extension(const XrExtensionProperties *extensions, uint32_t count, const char *name)
{
	for (uint32_t i = 0; i < count; i++) {
		if (strcmp(extensions[i].extensionName, name) == 0) {
			return true;
		}
	}
	return false;
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

static bool
parse_positive_int(const char *text, int *value)
{
	char *end = NULL;
	errno = 0;
	long parsed = strtol(text, &end, 10);
	if (errno != 0 || end == text || *end != '\0' || parsed <= 0 || parsed > INT_MAX) {
		return false;
	}
	*value = (int)parsed;
	return true;
}

int
main(int argc, char **argv)
{
	int samples = 100;
	int interval_ms = 100;
	if ((argc > 1 && !parse_positive_int(argv[1], &samples)) ||
	    (argc > 2 && !parse_positive_int(argv[2], &interval_ms)) || argc > 3) {
		fprintf(stderr, "usage: %s [samples] [interval_ms]\n", argv[0]);
		return 2;
	}

	XrInstance instance = XR_NULL_HANDLE;
	XrSession session = XR_NULL_HANDLE;
	XrSpace local_space = XR_NULL_HANDLE;
	XrXDevListMNDX xdev_list = XR_NULL_HANDLE;
	struct monitored_device *devices = NULL;
	XrExtensionProperties *extensions = NULL;
	XrXDevIdMNDX *ids = NULL;
	uint32_t device_count = 0;
	int exit_code = 1;
	PFN_xrCreateXDevListMNDX pfn_xrCreateXDevListMNDX = NULL;
	PFN_xrEnumerateXDevsMNDX pfn_xrEnumerateXDevsMNDX = NULL;
	PFN_xrGetXDevPropertiesMNDX pfn_xrGetXDevPropertiesMNDX = NULL;
	PFN_xrCreateXDevSpaceMNDX pfn_xrCreateXDevSpaceMNDX = NULL;
	PFN_xrDestroyXDevListMNDX pfn_xrDestroyXDevListMNDX = NULL;

	uint32_t extension_count = 0;
	CHECK_XR(xrEnumerateInstanceExtensionProperties(NULL, 0, &extension_count, NULL));
	extensions = calloc(extension_count, sizeof(*extensions));
	if (extensions == NULL) {
		fprintf(stderr, "out of memory\n");
		goto cleanup;
	}
	for (uint32_t i = 0; i < extension_count; i++) {
		extensions[i].type = XR_TYPE_EXTENSION_PROPERTIES;
	}
	CHECK_XR(xrEnumerateInstanceExtensionProperties(NULL, extension_count, &extension_count, extensions));
	if (!has_extension(extensions, extension_count, XR_MND_HEADLESS_EXTENSION_NAME) ||
	    !has_extension(extensions, extension_count, XR_MNDX_XDEV_SPACE_EXTENSION_NAME)) {
		fprintf(stderr, "runtime lacks XR_MND_headless or XR_MNDX_xdev_space\n");
		goto cleanup;
	}
	free(extensions);
	extensions = NULL;

	const char *enabled_extensions[] = {
	    XR_MND_HEADLESS_EXTENSION_NAME,
	    XR_MNDX_XDEV_SPACE_EXTENSION_NAME,
	};
	XrInstanceCreateInfo instance_info = {
	    .type = XR_TYPE_INSTANCE_CREATE_INFO,
	    .applicationInfo =
	        {
	            .applicationName = "NOLO xdev monitor",
	            .applicationVersion = 1,
	            .engineName = "none",
	            .engineVersion = 0,
	            .apiVersion = XR_CURRENT_API_VERSION,
	        },
	    .enabledExtensionCount = sizeof(enabled_extensions) / sizeof(enabled_extensions[0]),
	    .enabledExtensionNames = enabled_extensions,
	};
	CHECK_XR(xrCreateInstance(&instance_info, &instance));

	LOAD_XR(xrCreateXDevListMNDX);
	LOAD_XR(xrEnumerateXDevsMNDX);
	LOAD_XR(xrGetXDevPropertiesMNDX);
	LOAD_XR(xrCreateXDevSpaceMNDX);
	LOAD_XR(xrDestroyXDevListMNDX);

	XrSystemId system_id = XR_NULL_SYSTEM_ID;
	XrSystemGetInfo system_info = {
	    .type = XR_TYPE_SYSTEM_GET_INFO,
	    .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY,
	};
	CHECK_XR(xrGetSystem(instance, &system_info, &system_id));

	XrSystemXDevSpacePropertiesMNDX xdev_support = {
	    .type = XR_TYPE_SYSTEM_XDEV_SPACE_PROPERTIES_MNDX,
	};
	XrSystemProperties system_properties = {
	    .type = XR_TYPE_SYSTEM_PROPERTIES,
	    .next = &xdev_support,
	};
	CHECK_XR(xrGetSystemProperties(instance, system_id, &system_properties));
	if (!xdev_support.supportsXDevSpace) {
		fprintf(stderr, "runtime reports xdev space unsupported\n");
		goto cleanup;
	}

	XrSessionCreateInfo session_info = {
	    .type = XR_TYPE_SESSION_CREATE_INFO,
	    .systemId = system_id,
	};
	CHECK_XR(xrCreateSession(instance, &session_info, &session));

	XrReferenceSpaceCreateInfo reference_info = {
	    .type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
	    .referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL,
	    .poseInReferenceSpace = {.orientation = {.w = 1.0f}},
	};
	CHECK_XR(xrCreateReferenceSpace(session, &reference_info, &local_space));

	XrCreateXDevListInfoMNDX list_info = {
	    .type = XR_TYPE_CREATE_XDEV_LIST_INFO_MNDX,
	};
	CHECK_XR(pfn_xrCreateXDevListMNDX(session, &list_info, &xdev_list));
	CHECK_XR(pfn_xrEnumerateXDevsMNDX(xdev_list, 0, &device_count, NULL));
	ids = calloc(device_count, sizeof(*ids));
	devices = calloc(device_count, sizeof(*devices));
	if (ids == NULL || devices == NULL) {
		fprintf(stderr, "out of memory\n");
		goto cleanup;
	}
	CHECK_XR(pfn_xrEnumerateXDevsMNDX(xdev_list, device_count, &device_count, ids));

	printf("devices=%u samples=%d interval_ms=%d\n", device_count, samples, interval_ms);
	for (uint32_t i = 0; i < device_count; i++) {
		devices[i].id = ids[i];
		XrGetXDevInfoMNDX get_info = {
		    .type = XR_TYPE_GET_XDEV_INFO_MNDX,
		    .id = ids[i],
		};
		XrXDevPropertiesMNDX properties = {
		    .type = XR_TYPE_XDEV_PROPERTIES_MNDX,
		};
		CHECK_XR(pfn_xrGetXDevPropertiesMNDX(xdev_list, &get_info, &properties));
		snprintf(devices[i].name, sizeof(devices[i].name), "%s", properties.name);
		devices[i].has_space = properties.canCreateSpace;
		printf("%u: id=%" PRIu64 " space=%s name='%s' serial='%s'\n", i, (uint64_t)ids[i],
		       properties.canCreateSpace ? "yes" : "no", properties.name, properties.serial);

		if (properties.canCreateSpace) {
			XrCreateXDevSpaceInfoMNDX space_info = {
			    .type = XR_TYPE_CREATE_XDEV_SPACE_INFO_MNDX,
			    .xdevList = xdev_list,
			    .id = ids[i],
			    .offset = {.orientation = {.w = 1.0f}},
			};
			CHECK_XR(pfn_xrCreateXDevSpaceMNDX(session, &space_info, &devices[i].space));
		}
	}
	free(ids);
	ids = NULL;

	for (int sample = 0; sample < samples; sample++) {
		XrTime now = monotonic_now_ns();
		for (uint32_t i = 0; i < device_count; i++) {
			if (!devices[i].has_space) {
				continue;
			}
			XrSpaceLocation location = {.type = XR_TYPE_SPACE_LOCATION};
			CHECK_XR(xrLocateSpace(devices[i].space, local_space, now, &location));
			XrPosef pose = location.pose;
			printf("%04d %-42s flags=0x%02" PRIx64 " p=(% .5f % .5f % .5f) q=(% .5f % .5f % .5f % .5f)\n",
			       sample, devices[i].name, (uint64_t)location.locationFlags, pose.position.x, pose.position.y,
			       pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z,
			       pose.orientation.w);
		}
		fflush(stdout);
		sleep_ms(interval_ms);
	}

	exit_code = 0;

cleanup:
	free(ids);
	free(extensions);
	if (devices != NULL) {
		for (uint32_t i = 0; i < device_count; i++) {
			if (devices[i].space != XR_NULL_HANDLE) {
				xrDestroySpace(devices[i].space);
			}
		}
	}
	free(devices);
	if (xdev_list != XR_NULL_HANDLE && pfn_xrDestroyXDevListMNDX != NULL) {
		pfn_xrDestroyXDevListMNDX(xdev_list);
	}
	if (local_space != XR_NULL_HANDLE) {
		xrDestroySpace(local_space);
	}
	if (session != XR_NULL_HANDLE) {
		xrDestroySession(session);
	}
	if (instance != XR_NULL_HANDLE) {
		xrDestroyInstance(instance);
	}
	return exit_code;
}
