import threading
import unittest

from stararm_102_motion_node.dora_motion_node import MotionNode


class MotionSequencingTests(unittest.TestCase):
    def test_motion_waits_for_inflight_controller_sync(self) -> None:
        node = object.__new__(MotionNode)
        node._latest_arm_state = {"joints_rad": [0.0] * 6}
        node._controller_sync_requested = True
        node._controller_sync_phase = None
        node._motion_waiting_for_sync = None
        node._motion_status = MotionNode._idle_motion_status()
        outputs = []
        sync_calls = []
        node._enqueue = lambda output, value: outputs.append((output, value.copy()))
        node._synchronize_controllers = lambda: sync_calls.append(True)

        target = [0.0, 0.0, -0.1, 0.0, 0.0, 0.0]
        node._begin_motion_plan("request", target, {})

        self.assertEqual(node._motion_status["state"], "planning")
        self.assertEqual(
            node._motion_status["result_message"],
            "等待 ros2_control 同步当前反馈",
        )
        self.assertEqual(
            node._motion_waiting_for_sync,
            ("request", target, {}, False),
        )
        self.assertEqual(sync_calls, [True])
        self.assertEqual(outputs[-1][0], "motion_status")

    def test_sync_completion_starts_the_waiting_motion_once(self) -> None:
        node = object.__new__(MotionNode)
        node._lock = threading.RLock()
        waiting = ("request", [0.0] * 6, {}, False)
        node._motion_waiting_for_sync = waiting
        calls = []
        node._start_motion_plan = lambda *args: calls.append(args)

        node._start_motion_after_controller_sync()
        node._start_motion_after_controller_sync()

        self.assertIsNone(node._motion_waiting_for_sync)
        self.assertEqual(calls, [waiting])

    def test_second_motion_does_not_overwrite_active_request(self) -> None:
        node = object.__new__(MotionNode)
        node._motion_status = {
            **MotionNode._idle_motion_status(),
            "request_id": "active",
            "state": "executing",
        }
        outputs = []
        node._enqueue = lambda output, value: outputs.append((output, value))

        node._handle_motion_request({"request_id": "second", "action": "apply"})

        self.assertEqual(node._motion_status["request_id"], "active")
        self.assertEqual(node._motion_status["state"], "executing")
        self.assertEqual(outputs[-1][0], "motion_request_result")
        self.assertEqual(outputs[-1][1]["request_id"], "second")
        self.assertEqual(
            outputs[-1][1]["original_error"], "已有普通运动正在执行"
        )


if __name__ == "__main__":
    unittest.main()
