#define _POSIX_C_SOURCE 200809L

#include <openxr/XR_MNDX_xdev_space.h>
#include <openxr/openxr.h>

#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TARGET_DEVICE "NOLO CV1: Controller 0 (OpenHMD)"

#define CHECK_XR(call)                                                         \
  do {                                                                         \
    XrResult result_ = (call);                                                 \
    if (XR_FAILED(result_)) {                                                  \
      fprintf(stderr, "%s failed: %d\n", #call, result_);                      \
      goto cleanup;                                                            \
    }                                                                          \
  } while (false)

#define LOAD_XR(name)                                                          \
  do {                                                                         \
    CHECK_XR(xrGetInstanceProcAddr(instance, #name,                            \
                                   (PFN_xrVoidFunction *)&pfn_##name));        \
  } while (false)

static volatile sig_atomic_t keep_running = 1;

static void stop_handler(int signal_number) {
  (void)signal_number;
  keep_running = 0;
}

static XrTime monotonic_now_ns(void) {
  struct timespec ts = {0};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (XrTime)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void sleep_ms(long milliseconds) {
  struct timespec delay = {
      .tv_sec = milliseconds / 1000,
      .tv_nsec = (milliseconds % 1000) * 1000000L,
  };
  nanosleep(&delay, NULL);
}

static bool parse_positive_int(const char *text, int *value) {
  char *end = NULL;
  errno = 0;
  long parsed = strtol(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || parsed <= 0 ||
      parsed > INT_MAX) {
    return false;
  }
  *value = (int)parsed;
  return true;
}

static bool has_extension(const XrExtensionProperties *extensions,
                          uint32_t count, const char *name) {
  for (uint32_t i = 0; i < count; i++) {
    if (strcmp(extensions[i].extensionName, name) == 0) {
      return true;
    }
  }
  return false;
}

static bool same_pose(const XrPosef *a, const XrPosef *b) {
  return a->position.x == b->position.x &&
         a->position.y == b->position.y &&
         a->position.z == b->position.z &&
         a->orientation.x == b->orientation.x &&
         a->orientation.y == b->orientation.y &&
         a->orientation.z == b->orientation.z &&
         a->orientation.w == b->orientation.w;
}

int main(int argc, char **argv) {
  int interval_ms = 20;
  if ((argc > 1 && !parse_positive_int(argv[1], &interval_ms)) || argc > 2) {
    fprintf(stderr, "usage: %s [interval_ms]\n", argv[0]);
    return 2;
  }

  signal(SIGINT, stop_handler);
  signal(SIGTERM, stop_handler);

  XrInstance instance = XR_NULL_HANDLE;
  XrSession session = XR_NULL_HANDLE;
  XrSpace local_space = XR_NULL_HANDLE;
  XrSpace controller_space = XR_NULL_HANDLE;
  XrXDevListMNDX xdev_list = XR_NULL_HANDLE;
  XrActionSet action_set = XR_NULL_HANDLE;
  XrAction menu_action = XR_NULL_HANDLE;
  XrPath right_hand_path = XR_NULL_PATH;
  XrExtensionProperties *extensions = NULL;
  XrXDevIdMNDX *ids = NULL;
  bool session_running = false;
  bool exit_requested = false;
  bool have_previous_pose = false;
  XrPosef previous_pose = {.orientation = {.w = 1.0f}};
  XrTime last_pose_change_ns = 0;
  int exit_code = 1;
  PFN_xrCreateXDevListMNDX pfn_xrCreateXDevListMNDX = NULL;
  PFN_xrEnumerateXDevsMNDX pfn_xrEnumerateXDevsMNDX = NULL;
  PFN_xrGetXDevPropertiesMNDX pfn_xrGetXDevPropertiesMNDX = NULL;
  PFN_xrCreateXDevSpaceMNDX pfn_xrCreateXDevSpaceMNDX = NULL;
  PFN_xrDestroyXDevListMNDX pfn_xrDestroyXDevListMNDX = NULL;

  uint32_t extension_count = 0;
  CHECK_XR(
      xrEnumerateInstanceExtensionProperties(NULL, 0, &extension_count, NULL));
  extensions = calloc(extension_count, sizeof(*extensions));
  if (extensions == NULL) {
    fprintf(stderr, "out of memory\n");
    goto cleanup;
  }
  for (uint32_t i = 0; i < extension_count; i++) {
    extensions[i].type = XR_TYPE_EXTENSION_PROPERTIES;
  }
  CHECK_XR(xrEnumerateInstanceExtensionProperties(
      NULL, extension_count, &extension_count, extensions));
  if (!has_extension(extensions, extension_count,
                     XR_MND_HEADLESS_EXTENSION_NAME) ||
      !has_extension(extensions, extension_count,
                     XR_MNDX_XDEV_SPACE_EXTENSION_NAME)) {
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
              .applicationName = "NOLO controller stream",
              .applicationVersion = 1,
              .engineName = "none",
              .engineVersion = 0,
              .apiVersion = XR_CURRENT_API_VERSION,
          },
      .enabledExtensionCount =
          sizeof(enabled_extensions) / sizeof(enabled_extensions[0]),
      .enabledExtensionNames = enabled_extensions,
  };
  CHECK_XR(xrCreateInstance(&instance_info, &instance));

  LOAD_XR(xrCreateXDevListMNDX);
  LOAD_XR(xrEnumerateXDevsMNDX);
  LOAD_XR(xrGetXDevPropertiesMNDX);
  LOAD_XR(xrCreateXDevSpaceMNDX);
  LOAD_XR(xrDestroyXDevListMNDX);

  CHECK_XR(xrStringToPath(instance, "/user/hand/right", &right_hand_path));
  XrActionSetCreateInfo action_set_info = {
      .type = XR_TYPE_ACTION_SET_CREATE_INFO,
      .priority = 0,
  };
  snprintf(action_set_info.actionSetName, sizeof(action_set_info.actionSetName),
           "controller");
  snprintf(action_set_info.localizedActionSetName,
           sizeof(action_set_info.localizedActionSetName), "Controller");
  CHECK_XR(xrCreateActionSet(instance, &action_set_info, &action_set));

  XrActionCreateInfo action_info = {
      .type = XR_TYPE_ACTION_CREATE_INFO,
      .actionType = XR_ACTION_TYPE_BOOLEAN_INPUT,
      .countSubactionPaths = 1,
      .subactionPaths = &right_hand_path,
  };
  snprintf(action_info.actionName, sizeof(action_info.actionName),
           "reset_front");
  snprintf(action_info.localizedActionName,
           sizeof(action_info.localizedActionName), "Reset front");
  CHECK_XR(xrCreateAction(action_set, &action_info, &menu_action));

  XrPath simple_controller_path = XR_NULL_PATH;
  XrPath menu_path = XR_NULL_PATH;
  CHECK_XR(xrStringToPath(instance,
                          "/interaction_profiles/khr/simple_controller",
                          &simple_controller_path));
  CHECK_XR(xrStringToPath(instance, "/user/hand/right/input/menu/click",
                          &menu_path));
  XrActionSuggestedBinding menu_binding = {
      .action = menu_action,
      .binding = menu_path,
  };
  XrInteractionProfileSuggestedBinding suggested_bindings = {
      .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
      .interactionProfile = simple_controller_path,
      .countSuggestedBindings = 1,
      .suggestedBindings = &menu_binding,
  };
  CHECK_XR(xrSuggestInteractionProfileBindings(instance, &suggested_bindings));

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

  XrSessionActionSetsAttachInfo attach_info = {
      .type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO,
      .countActionSets = 1,
      .actionSets = &action_set,
  };
  CHECK_XR(xrAttachSessionActionSets(session, &attach_info));

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
  uint32_t device_count = 0;
  CHECK_XR(pfn_xrEnumerateXDevsMNDX(xdev_list, 0, &device_count, NULL));
  ids = calloc(device_count, sizeof(*ids));
  if (ids == NULL) {
    fprintf(stderr, "out of memory\n");
    goto cleanup;
  }
  CHECK_XR(
      pfn_xrEnumerateXDevsMNDX(xdev_list, device_count, &device_count, ids));
  for (uint32_t i = 0; i < device_count; i++) {
    XrGetXDevInfoMNDX get_info = {
        .type = XR_TYPE_GET_XDEV_INFO_MNDX,
        .id = ids[i],
    };
    XrXDevPropertiesMNDX properties = {
        .type = XR_TYPE_XDEV_PROPERTIES_MNDX,
    };
    CHECK_XR(pfn_xrGetXDevPropertiesMNDX(xdev_list, &get_info, &properties));
    if (strcmp(properties.name, TARGET_DEVICE) != 0 ||
        !properties.canCreateSpace) {
      continue;
    }
    XrCreateXDevSpaceInfoMNDX space_info = {
        .type = XR_TYPE_CREATE_XDEV_SPACE_INFO_MNDX,
        .xdevList = xdev_list,
        .id = ids[i],
        .offset = {.orientation = {.w = 1.0f}},
    };
    CHECK_XR(
        pfn_xrCreateXDevSpaceMNDX(session, &space_info, &controller_space));
    break;
  }
  free(ids);
  ids = NULL;
  if (controller_space == XR_NULL_HANDLE) {
    fprintf(stderr, "target device not found: %s\n", TARGET_DEVICE);
    goto cleanup;
  }

  fprintf(stderr, "streaming %s every %d ms\n", TARGET_DEVICE, interval_ms);
  while (keep_running && !exit_requested) {
    XrEventDataBuffer event = {.type = XR_TYPE_EVENT_DATA_BUFFER};
    XrResult poll_result = XR_SUCCESS;
    while ((poll_result = xrPollEvent(instance, &event)) == XR_SUCCESS) {
      if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
        const XrEventDataSessionStateChanged *changed =
            (const XrEventDataSessionStateChanged *)&event;
        if (changed->state == XR_SESSION_STATE_READY && !session_running) {
          XrSessionBeginInfo begin_info = {
              .type = XR_TYPE_SESSION_BEGIN_INFO,
              .primaryViewConfigurationType =
                  XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
          };
          CHECK_XR(xrBeginSession(session, &begin_info));
          session_running = true;
        } else if (changed->state == XR_SESSION_STATE_STOPPING &&
                   session_running) {
          xrEndSession(session);
          session_running = false;
        } else if (changed->state == XR_SESSION_STATE_EXITING ||
                   changed->state == XR_SESSION_STATE_LOSS_PENDING) {
          exit_requested = true;
        }
      }
      event = (XrEventDataBuffer){.type = XR_TYPE_EVENT_DATA_BUFFER};
    }
    if (XR_FAILED(poll_result) && poll_result != XR_EVENT_UNAVAILABLE) {
      fprintf(stderr, "xrPollEvent failed: %d\n", poll_result);
      goto cleanup;
    }

    bool menu_active = false;
    bool menu_pressed = false;
    if (session_running) {
      XrActiveActionSet active_set = {
          .actionSet = action_set,
          .subactionPath = right_hand_path,
      };
      XrActionsSyncInfo sync_info = {
          .type = XR_TYPE_ACTIONS_SYNC_INFO,
          .countActiveActionSets = 1,
          .activeActionSets = &active_set,
      };
      XrResult sync_result = xrSyncActions(session, &sync_info);
      if (XR_SUCCEEDED(sync_result)) {
        XrActionStateGetInfo get_state_info = {
            .type = XR_TYPE_ACTION_STATE_GET_INFO,
            .action = menu_action,
            .subactionPath = right_hand_path,
        };
        XrActionStateBoolean menu_state = {.type =
                                               XR_TYPE_ACTION_STATE_BOOLEAN};
        CHECK_XR(
            xrGetActionStateBoolean(session, &get_state_info, &menu_state));
        menu_active = menu_state.isActive;
        menu_pressed = menu_active && menu_state.currentState;
      } else if (sync_result != XR_SESSION_NOT_FOCUSED) {
        fprintf(stderr, "xrSyncActions failed: %d\n", sync_result);
        goto cleanup;
      }
    }

    XrTime now = monotonic_now_ns();
    XrSpaceLocation location = {.type = XR_TYPE_SPACE_LOCATION};
    CHECK_XR(xrLocateSpace(controller_space, local_space, now, &location));
    const XrPosef pose = location.pose;
    // OpenXR exposes the query time here, not the time of the last USB sample.
    // Track exact pose changes separately so repeated last-known relations are
    // visible to diagnostics.
    if (!have_previous_pose || !same_pose(&pose, &previous_pose)) {
      previous_pose = pose;
      last_pose_change_ns = now;
      have_previous_pose = true;
    }
    const int64_t unchanged_ms = have_previous_pose
                                     ? (int64_t)((now - last_pose_change_ns) /
                                                 1000000LL)
                                     : 0;
    printf(
        "{\"time_ns\":%" PRId64 ",\"device\":\"%s\",\"flags\":%" PRIu64
        ",\"position\":[%.8g,%.8g,%.8g],\"orientation\":[%.8g,%.8g,%.8g,%.8g],"
        "\"unchanged_ms\":%" PRId64
        ",\"menu_active\":%s,\"menu_pressed\":%s}\n",
        (int64_t)now, TARGET_DEVICE, (uint64_t)location.locationFlags,
        pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
        pose.orientation.y, pose.orientation.z, pose.orientation.w, unchanged_ms,
        menu_active ? "true" : "false", menu_pressed ? "true" : "false");
    fflush(stdout);
    sleep_ms(interval_ms);
  }

  exit_code = 0;

cleanup:
  free(ids);
  free(extensions);
  if (controller_space != XR_NULL_HANDLE) {
    xrDestroySpace(controller_space);
  }
  if (xdev_list != XR_NULL_HANDLE && pfn_xrDestroyXDevListMNDX != NULL) {
    pfn_xrDestroyXDevListMNDX(xdev_list);
  }
  if (local_space != XR_NULL_HANDLE) {
    xrDestroySpace(local_space);
  }
  if (session != XR_NULL_HANDLE) {
    xrDestroySession(session);
  }
  if (menu_action != XR_NULL_HANDLE) {
    xrDestroyAction(menu_action);
  }
  if (action_set != XR_NULL_HANDLE) {
    xrDestroyActionSet(action_set);
  }
  if (instance != XR_NULL_HANDLE) {
    xrDestroyInstance(instance);
  }
  return exit_code;
}
