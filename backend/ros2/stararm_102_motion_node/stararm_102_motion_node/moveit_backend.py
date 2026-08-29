"""Blocking adapter around MoveIt's public actions and planning-scene services."""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any

from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningSceneComponents,
)
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node

from .model_catalog import JOINTS


class PlanningError(RuntimeError):
    def __init__(
        self, code: int, collision_pairs: tuple[tuple[str, str], ...] = ()
    ) -> None:
        super().__init__(f"MoveIt 规划失败，错误码 {code}")
        self.code = code
        self.collision_pairs = collision_pairs


@dataclass(frozen=True)
class MotionPlan:
    trajectory: Any
    points: int
    duration_s: float
    collision_pairs: tuple[tuple[str, str], ...]


@dataclass(frozen=True)
class ExecutionResult:
    success: bool
    code: int


class MoveItBackend:
    """Keep ROS mechanics out of the Dora-facing node.

    rclpy spins on the main thread while the motion worker blocks on these futures.
    This keeps one linear request flow without a callback state machine.
    """

    def __init__(self, node: Node) -> None:
        self._move_group = ActionClient(node, MoveGroup, "/move_action")
        self._execute = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
        self._state_validity = node.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self._planning_scene = node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self._lock = threading.Lock()
        self._plan_handle: Any | None = None
        self._execute_handle: Any | None = None

    def ready(self) -> bool:
        return self._move_group.server_is_ready() and self._execute.server_is_ready()

    def plan(
        self,
        current: list[float],
        target: list[float],
        options: dict[str, float],
    ) -> MotionPlan:
        result = self._request_plan(current, target, options)
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            return self._motion_plan(result, ())

        pairs = self._collision_pairs(current)
        if not pairs:
            raise PlanningError(result.error_code.val)
        matrix = self._allowed_collision_matrix(pairs)
        result = self._request_plan(current, target, options, matrix)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise PlanningError(result.error_code.val, pairs)
        return self._motion_plan(result, pairs)

    def execute(self, plan: MotionPlan) -> ExecutionResult:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = plan.trajectory
        goal.controller_names = ["arm_controller"]
        handle = self._accepted(self._execute.send_goal_async(goal), "执行")
        with self._lock:
            self._execute_handle = handle
        try:
            result = self._wait(handle.get_result_async()).result
        finally:
            with self._lock:
                self._execute_handle = None
        return ExecutionResult(
            success=result.error_code.val == MoveItErrorCodes.SUCCESS,
            code=result.error_code.val,
        )

    def cancel(self) -> None:
        with self._lock:
            handles = (self._plan_handle, self._execute_handle)
        for handle in handles:
            if handle is not None:
                handle.cancel_goal_async()

    def _request_plan(
        self,
        current: list[float],
        target: list[float],
        options: dict[str, float],
        matrix: Any | None = None,
    ) -> Any:
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.start_state.joint_state.name = list(JOINTS)
        goal.request.start_state.joint_state.position = current
        goal.request.start_state.is_diff = False
        if "velocity_scaling" in options:
            goal.request.max_velocity_scaling_factor = options["velocity_scaling"]
        if "acceleration_scaling" in options:
            goal.request.max_acceleration_scaling_factor = options[
                "acceleration_scaling"
            ]
        constraints = Constraints(name="joint_target")
        constraints.joint_constraints = [
            JointConstraint(joint_name=name, position=position, weight=1.0)
            for name, position in zip(JOINTS, target, strict=True)
        ]
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        if matrix is not None:
            scene = goal.planning_options.planning_scene_diff
            scene.is_diff = True
            scene.allowed_collision_matrix = matrix

        handle = self._accepted(self._move_group.send_goal_async(goal), "规划")
        with self._lock:
            self._plan_handle = handle
        try:
            return self._wait(handle.get_result_async()).result
        finally:
            with self._lock:
                self._plan_handle = None

    def _collision_pairs(self, current: list[float]) -> tuple[tuple[str, str], ...]:
        request = GetStateValidity.Request()
        request.robot_state.joint_state.name = list(JOINTS)
        request.robot_state.joint_state.position = current
        request.group_name = "arm"
        response = self._wait(self._state_validity.call_async(request))
        if response.valid:
            return ()
        return tuple(
            sorted(
                {
                    tuple(sorted((contact.contact_body_1, contact.contact_body_2)))
                    for contact in response.contacts
                    if contact.body_type_1 == contact.ROBOT_LINK
                    and contact.body_type_2 == contact.ROBOT_LINK
                    and contact.contact_body_1 != contact.contact_body_2
                }
            )
        )

    def _allowed_collision_matrix(self, pairs: tuple[tuple[str, str], ...]) -> Any:
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix = self._wait(
            self._planning_scene.call_async(request)
        ).scene.allowed_collision_matrix
        self.allow_pairs(matrix, pairs)
        return matrix

    @staticmethod
    def allow_pairs(matrix: Any, pairs: tuple[tuple[str, str], ...]) -> None:
        if len(matrix.entry_values) != len(matrix.entry_names) or any(
            len(row.enabled) != len(matrix.entry_names) for row in matrix.entry_values
        ):
            raise ValueError("MoveIt AllowedCollisionMatrix 不是方阵")
        for name in sorted({name for pair in pairs for name in pair}):
            if name in matrix.entry_names:
                continue
            matrix.entry_names.append(name)
            for row in matrix.entry_values:
                row.enabled.append(False)
            entry = AllowedCollisionEntry()
            entry.enabled = [False] * len(matrix.entry_names)
            matrix.entry_values.append(entry)
        indices = {name: index for index, name in enumerate(matrix.entry_names)}
        for first, second in pairs:
            matrix.entry_values[indices[first]].enabled[indices[second]] = True
            matrix.entry_values[indices[second]].enabled[indices[first]] = True

    @staticmethod
    def _motion_plan(result: Any, pairs: tuple[tuple[str, str], ...]) -> MotionPlan:
        points = result.planned_trajectory.joint_trajectory.points
        if not points:
            raise RuntimeError("MoveIt 返回空轨迹")
        end = points[-1].time_from_start
        return MotionPlan(
            trajectory=result.planned_trajectory,
            points=len(points),
            duration_s=end.sec + end.nanosec * 1e-9,
            collision_pairs=pairs,
        )

    @classmethod
    def _accepted(cls, future: Any, action: str) -> Any:
        handle = cls._wait(future)
        if not handle.accepted:
            raise RuntimeError(f"MoveIt 拒绝{action}请求")
        return handle

    @staticmethod
    def _wait(future: Any) -> Any:
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        completed.wait()
        error = future.exception()
        if error is not None:
            raise error
        return future.result()
