#!/usr/bin/python3

import json
import math
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from controller_manager_msgs.srv import ConfigureController
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import LoadController
from controller_manager_msgs.srv import SwitchController
from franka_msgs.action import Grasp
from franka_msgs.action import Homing
from franka_msgs.action import Move
from franka_msgs.action import ErrorRecovery
from franka_msgs.action import PTPMotion
from franka_msgs.msg import FrankaRobotState
from franka_msgs.srv import SetFullCollisionBehavior
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.srv import GetPositionIK
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.srv import SetParametersAtomically
from sensor_msgs.msg import JointState
from serl_franka_controllers_ros2.msg import CartesianImpedanceCommand
from serl_franka_controllers_ros2.msg import ZeroJacobian
from std_srvs.srv import Trigger


class FrankaHTTPBridge(Node):
    def __init__(self) -> None:
        super().__init__("serl_franka_http_server")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5000)
        self.declare_parameter("base_frame", "fr3_link0")
        self.declare_parameter("robot_type", "fr3")
        self.declare_parameter("arm_prefix", "")
        self.declare_parameter("load_gripper", False)
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("impedance_controller", "cartesian_impedance_controller")
        self.declare_parameter("cartesian_pose_controller", "cartesian_pose_command_controller")
        self.declare_parameter("joint_controller", "joint_position_controller")
        self.declare_parameter(
            "reset_joint_target", [0.0, -0.78539816339, 0.0, -2.35619449019, 0.0, 1.57079632679, 0.78539816339]
        )
        self.declare_parameter("current_pose_topic", "/franka_robot_state_broadcaster/current_pose")
        self.declare_parameter(
            "stiffness_wrench_topic", "/franka_robot_state_broadcaster/external_wrench_in_stiffness_frame"
        )
        self.declare_parameter("measured_joint_states_topic", "/franka_robot_state_broadcaster/measured_joint_states")
        self.declare_parameter("franka_state_topic", "/franka_robot_state_broadcaster/robot_state")
        self.declare_parameter("jacobian_topic", "/cartesian_impedance_controller/franka_jacobian")
        self.declare_parameter("equilibrium_pose_topic", "/cartesian_impedance_controller/equilibrium_pose")
        self.declare_parameter("cartesian_pose_command_topic", "/cartesian_pose_command_controller/target_pose")
        self.declare_parameter("error_recovery_action", "/action_server/error_recovery")
        self.declare_parameter("ptp_motion_action", "/action_server/ptp_motion")
        self.declare_parameter("compute_ik_service", "compute_ik")
        self.declare_parameter("compute_cartesian_path_service", "compute_cartesian_path")
        self.declare_parameter("execute_trajectory_action", "execute_trajectory")
        self.declare_parameter("pose_fallback_to_ik", False)
        self.declare_parameter("pose_auto_activate_impedance", False)
        self.declare_parameter("pose_ik_timeout_sec", 5.0)
        self.declare_parameter("pose_fallback_goal_tolerance", 0.005)
        self.declare_parameter("pose_fallback_activation_settle_sec", 0.25)
        self.declare_parameter("precise_controller", "fr3_arm_controller")
        self.declare_parameter("precise_cartesian_max_step", 0.005)
        self.declare_parameter("precise_cartesian_jump_threshold", 0.0)
        self.declare_parameter("precise_cartesian_prismatic_jump_threshold", 0.0)
        self.declare_parameter("precise_cartesian_revolute_jump_threshold", 0.0)
        self.declare_parameter("precise_cartesian_min_fraction", 0.999)
        self.declare_parameter("precise_cartesian_avoid_collisions", True)
        self.declare_parameter("joint_reset_use_ptp_action", True)
        self.declare_parameter("joint_reset_maximum_joint_velocities", [0.25, 0.25, 0.25, 0.25, 0.3, 0.3, 0.3])
        self.declare_parameter("joint_reset_goal_tolerance", 0.01)
        self.declare_parameter("request_timeout_sec", 10.0)
        self.declare_parameter("joint_reset_timeout_sec", 30.0)
        self.declare_parameter("gripper_prefix", "franka_gripper")
        self.declare_parameter("gripper_joint_states_topic", "franka_gripper/joint_states")
        self.declare_parameter("gripper_move_speed", 0.05)
        self.declare_parameter("gripper_grasp_speed", 0.03)
        self.declare_parameter("gripper_grasp_force", 40.0)
        self.declare_parameter("gripper_epsilon_inner", 0.005)
        self.declare_parameter("gripper_epsilon_outer", 0.005)
        self.declare_parameter("gripper_open_width", 0.08)
        self.declare_parameter("gripper_closed_width", 0.0)
        self.declare_parameter("gripper_timeout_sec", 15.0)
        self.declare_parameter("apply_default_collision_behavior", True)
        self.declare_parameter("collision_behavior_service", "/service_server/set_full_collision_behavior")
        self.declare_parameter("lower_torque_thresholds_nominal", [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0])
        self.declare_parameter("upper_torque_thresholds_nominal", [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0])
        self.declare_parameter(
            "lower_torque_thresholds_acceleration", [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0]
        )
        self.declare_parameter(
            "upper_torque_thresholds_acceleration", [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0]
        )
        self.declare_parameter("lower_force_thresholds_nominal", [30.0, 30.0, 30.0, 25.0, 25.0, 25.0])
        self.declare_parameter("upper_force_thresholds_nominal", [40.0, 40.0, 40.0, 35.0, 35.0, 35.0])
        self.declare_parameter("lower_force_thresholds_acceleration", [30.0, 30.0, 30.0, 25.0, 25.0, 25.0])
        self.declare_parameter("upper_force_thresholds_acceleration", [40.0, 40.0, 40.0, 35.0, 35.0, 35.0])
        self.declare_parameter("auto_clear_error", False)
        self.declare_parameter("auto_start_impedance", False)
        self.declare_parameter("auto_start_delay_sec", 3.0)
        self.declare_parameter("auto_start_wait_timeout_sec", 20.0)
        self.declare_parameter("auto_start_retry_count", 5)
        self.declare_parameter("auto_start_retry_interval_sec", 2.0)

        self._state_lock = threading.RLock()
        self.pose = [0.0] * 7
        self.vel = [0.0] * 6
        self.force = [0.0] * 3
        self.torque = [0.0] * 3
        self.q = [0.0] * 7
        self.dq = [0.0] * 7
        self.jacobian = [[0.0] * 7 for _ in range(6)]
        self.gripper_pos = 0.0
        self.have_pose = False
        self.have_jacobian = False
        self.have_joint_state = False
        self.have_gripper = False
        self.default_collision_behavior_applied = False
        self.default_collision_behavior_future = None
        self.auto_start_started = False
        self.auto_start_thread: Optional[threading.Thread] = None

        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.robot_type = self.get_parameter("robot_type").get_parameter_value().string_value
        self.arm_prefix = self.get_parameter("arm_prefix").get_parameter_value().string_value
        self.load_gripper = self.get_parameter("load_gripper").get_parameter_value().bool_value
        self.controller_manager = self.get_parameter("controller_manager").get_parameter_value().string_value
        self.impedance_controller = (
            self.get_parameter("impedance_controller").get_parameter_value().string_value
        )
        self.cartesian_pose_controller = (
            self.get_parameter("cartesian_pose_controller").get_parameter_value().string_value
        )
        self.precise_controller = self.get_parameter("precise_controller").get_parameter_value().string_value
        self.joint_controller = self.get_parameter("joint_controller").get_parameter_value().string_value
        self.request_timeout_sec = (
            self.get_parameter("request_timeout_sec").get_parameter_value().double_value
        )
        self.joint_reset_timeout_sec = (
            self.get_parameter("joint_reset_timeout_sec").get_parameter_value().double_value
        )
        self.pose_fallback_to_ik = self.get_parameter("pose_fallback_to_ik").get_parameter_value().bool_value
        self.pose_auto_activate_impedance = (
            self.get_parameter("pose_auto_activate_impedance").get_parameter_value().bool_value
        )
        self.pose_ik_timeout_sec = self.get_parameter("pose_ik_timeout_sec").get_parameter_value().double_value
        self.pose_fallback_activation_settle_sec = (
            self.get_parameter("pose_fallback_activation_settle_sec").get_parameter_value().double_value
        )
        self.joint_reset_use_ptp_action = (
            self.get_parameter("joint_reset_use_ptp_action").get_parameter_value().bool_value
        )
        self.gripper_timeout_sec = self.get_parameter("gripper_timeout_sec").get_parameter_value().double_value
        self.gripper_prefix = self.get_parameter("gripper_prefix").get_parameter_value().string_value.strip("/")
        self.apply_default_collision_behavior = (
            self.get_parameter("apply_default_collision_behavior").get_parameter_value().bool_value
        )
        self.auto_clear_error = self.get_parameter("auto_clear_error").get_parameter_value().bool_value
        self.auto_start_impedance = self.get_parameter("auto_start_impedance").get_parameter_value().bool_value
        self.auto_start_delay_sec = self.get_parameter("auto_start_delay_sec").get_parameter_value().double_value
        self.auto_start_wait_timeout_sec = (
            self.get_parameter("auto_start_wait_timeout_sec").get_parameter_value().double_value
        )
        self.auto_start_retry_count = (
            self.get_parameter("auto_start_retry_count").get_parameter_value().integer_value
        )
        self.auto_start_retry_interval_sec = (
            self.get_parameter("auto_start_retry_interval_sec").get_parameter_value().double_value
        )

        self.pose_publisher = self.create_publisher(
            CartesianImpedanceCommand,
            self.get_parameter("equilibrium_pose_topic").get_parameter_value().string_value,
            10,
        )
        self.cartesian_pose_publisher = self.create_publisher(
            PoseStamped,
            self.get_parameter("cartesian_pose_command_topic").get_parameter_value().string_value,
            10,
        )
        convenience_qos = QoSProfile(depth=10)
        convenience_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            PoseStamped,
            self.get_parameter("current_pose_topic").get_parameter_value().string_value,
            self._pose_callback,
            convenience_qos,
        )
        self.create_subscription(
            WrenchStamped,
            self.get_parameter("stiffness_wrench_topic").get_parameter_value().string_value,
            self._wrench_callback,
            convenience_qos,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("measured_joint_states_topic").get_parameter_value().string_value,
            self._joint_state_callback,
            convenience_qos,
        )
        self.create_subscription(
            FrankaRobotState,
            self.get_parameter("franka_state_topic").get_parameter_value().string_value,
            self._franka_state_callback,
            10,
        )
        self.create_subscription(
            ZeroJacobian,
            self.get_parameter("jacobian_topic").get_parameter_value().string_value,
            self._jacobian_callback,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("gripper_joint_states_topic").get_parameter_value().string_value,
            self._gripper_joint_state_callback,
            10,
        )

        self.list_controllers_client = self.create_client(
            ListControllers, f"{self.controller_manager}/list_controllers"
        )
        self.load_controller_client = self.create_client(
            LoadController, f"{self.controller_manager}/load_controller"
        )
        self.configure_controller_client = self.create_client(
            ConfigureController, f"{self.controller_manager}/configure_controller"
        )
        self.switch_controller_client = self.create_client(
            SwitchController, f"{self.controller_manager}/switch_controller"
        )

        self.impedance_get_params_client = self.create_client(
            GetParameters, f"{self.impedance_controller}/get_parameters"
        )
        self.impedance_set_params_client = self.create_client(
            SetParameters, f"{self.impedance_controller}/set_parameters"
        )
        self.joint_set_params_client = self.create_client(
            SetParameters, f"{self.joint_controller}/set_parameters"
        )
        self.compute_ik_client = self.create_client(
            GetPositionIK,
            self.get_parameter("compute_ik_service").get_parameter_value().string_value,
        )
        self.compute_cartesian_path_client = self.create_client(
            GetCartesianPath,
            self.get_parameter("compute_cartesian_path_service").get_parameter_value().string_value,
        )
        self.error_recovery_client = ActionClient(
            self,
            ErrorRecovery,
            self.get_parameter("error_recovery_action").get_parameter_value().string_value,
        )
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            self.get_parameter("execute_trajectory_action").get_parameter_value().string_value,
        )
        self.ptp_motion_client = ActionClient(
            self,
            PTPMotion,
            self.get_parameter("ptp_motion_action").get_parameter_value().string_value,
        )
        self.gripper_homing_client = ActionClient(self, Homing, self._gripper_resource("homing"))
        self.gripper_move_client = ActionClient(self, Move, self._gripper_resource("move"))
        self.gripper_grasp_client = ActionClient(self, Grasp, self._gripper_resource("grasp"))
        self.gripper_stop_client = self.create_client(Trigger, self._gripper_resource("stop"))
        self.collision_behavior_client = self.create_client(
            SetFullCollisionBehavior,
            self.get_parameter("collision_behavior_service").get_parameter_value().string_value,
        )
        self.collision_behavior_timer = self.create_timer(1.0, self._apply_default_collision_behavior_once)
        self.auto_start_timer = self.create_timer(0.5, self._maybe_start_auto_start_sequence)

        self.http_server: Optional[ThreadingHTTPServer] = None
        self.http_thread: Optional[threading.Thread] = None

    def _pose_callback(self, msg: PoseStamped) -> None:
        pose = [
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        ]
        with self._state_lock:
            self.pose = pose
            self.have_pose = True

    def _wrench_callback(self, msg: WrenchStamped) -> None:
        with self._state_lock:
            self.force = [float(msg.wrench.force.x), float(msg.wrench.force.y), float(msg.wrench.force.z)]
            self.torque = [
                float(msg.wrench.torque.x),
                float(msg.wrench.torque.y),
                float(msg.wrench.torque.z),
            ]

    def _joint_state_callback(self, msg: JointState) -> None:
        if len(msg.position) < 7 or len(msg.velocity) < 7:
            return
        with self._state_lock:
            self.q = [float(value) for value in msg.position[:7]]
            self.dq = [float(value) for value in msg.velocity[:7]]
            self.have_joint_state = True
            if self.have_jacobian:
                self.vel = self._matrix_vector_multiply(self.jacobian, self.dq)

    def _franka_state_callback(self, msg: FrankaRobotState) -> None:
        pose = [
            float(msg.o_t_ee.pose.position.x),
            float(msg.o_t_ee.pose.position.y),
            float(msg.o_t_ee.pose.position.z),
            float(msg.o_t_ee.pose.orientation.x),
            float(msg.o_t_ee.pose.orientation.y),
            float(msg.o_t_ee.pose.orientation.z),
            float(msg.o_t_ee.pose.orientation.w),
        ]
        joint_state = msg.measured_joint_state
        has_joint_state = len(joint_state.position) >= 7 and len(joint_state.velocity) >= 7
        with self._state_lock:
            self.pose = pose
            self.have_pose = True
            self.force = [
                float(msg.k_f_ext_hat_k.wrench.force.x),
                float(msg.k_f_ext_hat_k.wrench.force.y),
                float(msg.k_f_ext_hat_k.wrench.force.z),
            ]
            self.torque = [
                float(msg.k_f_ext_hat_k.wrench.torque.x),
                float(msg.k_f_ext_hat_k.wrench.torque.y),
                float(msg.k_f_ext_hat_k.wrench.torque.z),
            ]
            if has_joint_state:
                self.q = [float(value) for value in joint_state.position[:7]]
                self.dq = [float(value) for value in joint_state.velocity[:7]]
                self.have_joint_state = True
                if self.have_jacobian:
                    self.vel = self._matrix_vector_multiply(self.jacobian, self.dq)

    def _jacobian_callback(self, msg: ZeroJacobian) -> None:
        with self._state_lock:
            values = [float(value) for value in msg.zero_jacobian]
            self.jacobian = [[values[row + 6 * col] for col in range(7)] for row in range(6)]
            self.have_jacobian = True
            if self.have_joint_state:
                self.vel = self._matrix_vector_multiply(self.jacobian, self.dq)

    def _gripper_joint_state_callback(self, msg: JointState) -> None:
        if len(msg.position) < 2:
            return
        with self._state_lock:
            self.gripper_pos = float(msg.position[0] + msg.position[1])
            self.have_gripper = True

    def start_http_server(self) -> None:
        host = self.get_parameter("host").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        handler = self._make_handler()
        self.http_server = ThreadingHTTPServer((host, port), handler)
        self.http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f"HTTP server listening on http://{host}:{port}")

    def shutdown_http_server(self) -> None:
        if self.http_server is not None:
            self.http_server.shutdown()
            self.http_server.server_close()
        if self.http_thread is not None:
            self.http_thread.join(timeout=2.0)

    def _make_handler(self):
        bridge = self

        class RequestHandler(BaseHTTPRequestHandler):
            def do_POST(self):  # noqa: N802
                bridge._handle_request(self)

            def do_GET(self):  # noqa: N802
                bridge._handle_request(self)

            def log_message(self, fmt: str, *args) -> None:
                bridge.get_logger().info("HTTP %s - %s" % (self.command, fmt % args))

        return RequestHandler

    def _handle_request(self, handler: BaseHTTPRequestHandler) -> None:
        try:
            payload = self._read_json(handler)
            response = self._dispatch(handler.path, payload)
            self._send_json(handler, HTTPStatus.OK, response)
        except ValueError as exc:
            self._send_json(handler, HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
        except NotImplementedError as exc:
            self._send_json(handler, HTTPStatus.NOT_IMPLEMENTED, {"ok": False, "error": str(exc)})
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"HTTP request handling failed: {exc}")
            self._send_json(handler, HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": str(exc)})

    @staticmethod
    def _read_json(handler: BaseHTTPRequestHandler) -> Dict[str, Any]:
        length = int(handler.headers.get("Content-Length", "0"))
        if length == 0:
            return {}
        body = handler.rfile.read(length)
        if not body:
            return {}
        return json.loads(body.decode("utf-8"))

    @staticmethod
    def _send_json(handler: BaseHTTPRequestHandler, status: HTTPStatus, payload: Dict[str, Any]) -> None:
        data = json.dumps(payload).encode("utf-8")
        handler.send_response(status.value)
        handler.send_header("Content-Type", "application/json")
        handler.send_header("Content-Length", str(len(data)))
        handler.end_headers()
        handler.wfile.write(data)

    def _dispatch(self, path: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        routes = {
            "/health": self.health,
            "/startimp": self.start_impedance,
            "/stopimp": self.stop_impedance,
            "/pose": lambda: self.move_pose(payload["arr"], payload.get("q")),
            "/pose_precise": lambda: self.move_pose_precise(payload["arr"], payload),
            "/getpos": self.get_pose,
            "/getpos_euler": self.get_pose_euler,
            "/getvel": self.get_velocity,
            "/getforce": self.get_force,
            "/gettorque": self.get_torque,
            "/getq": self.get_q,
            "/getdq": self.get_dq,
            "/getjacobian": self.get_jacobian,
            "/getstate": self.get_state,
            "/jointreset": lambda: self.reset_joint(payload.get("arr"), payload),
            "/clearerr": self.clear_error,
            "/update_param": lambda: self.update_params(payload),
            "/get_gripper": self.get_gripper,
            "/activate_gripper": self.activate_gripper,
            "/reset_gripper": self.reset_gripper,
            "/close_gripper": lambda: self.close_gripper(payload),
            "/open_gripper": lambda: self.open_gripper(payload),
            "/move_gripper": lambda: self.move_gripper(payload),
        }
        if path not in routes:
            raise ValueError(f"Unknown endpoint: {path}")
        return routes[path]()

    def health(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "have_pose": self.have_pose,
            "have_joint_state": self.have_joint_state,
            "have_jacobian": self.have_jacobian,
        }

    def get_pose(self) -> Dict[str, Any]:
        with self._state_lock:
            return {"ok": True, "pose": list(self.pose)}

    def get_pose_euler(self) -> Dict[str, Any]:
        with self._state_lock:
            quat = self.pose[3:]
            euler = self._quat_xyzw_to_euler_xyz(quat)
            return {"ok": True, "pose": list(self.pose[:3]) + euler}

    def get_velocity(self) -> Dict[str, Any]:
        with self._state_lock:
            return {"ok": True, "vel": list(self.vel)}

    def get_force(self) -> Dict[str, Any]:
        with self._state_lock:
            return {"ok": True, "force": list(self.force)}

    def get_torque(self) -> Dict[str, Any]:
        with self._state_lock:
            return {"ok": True, "torque": list(self.torque)}

    def get_q(self) -> Dict[str, Any]:
        with self._state_lock:
            return {"ok": True, "q": list(self.q)}

    def get_dq(self) -> Dict[str, Any]:
        with self._state_lock:
            return {"ok": True, "dq": list(self.dq)}

    def get_jacobian(self) -> Dict[str, Any]:
        with self._state_lock:
            return {"ok": True, "jacobian": [list(row) for row in self.jacobian]}

    def get_state(self) -> Dict[str, Any]:
        with self._state_lock:
            return {
                "ok": True,
                "pose": list(self.pose),
                "vel": list(self.vel),
                "force": list(self.force),
                "torque": list(self.torque),
                "q": list(self.q),
                "dq": list(self.dq),
                "jacobian": [list(row) for row in self.jacobian],
                "gripper_pos": self.gripper_pos if self.have_gripper else None,
                "have_gripper": self.have_gripper,
            }

    def get_gripper(self) -> Dict[str, Any]:
        with self._state_lock:
            return {
                "ok": True,
                "gripper_pos": self.gripper_pos if self.have_gripper else None,
                "have_gripper": self.have_gripper,
            }

    def activate_gripper(self) -> Dict[str, Any]:
        result = self._execute_homing()
        return {
            "ok": result.success,
            "message": "Gripper homing finished" if result.success else "Gripper homing failed",
            "error": result.error,
            "gripper_pos": self._gripper_width_or_none(),
        }

    def reset_gripper(self) -> Dict[str, Any]:
        stop_response = self._stop_gripper(raise_on_failure=False)
        result = self._execute_homing()
        return {
            "ok": result.success,
            "message": "Gripper reset finished" if result.success else "Gripper reset failed",
            "error": result.error,
            "stop_message": stop_response.get("message", ""),
            "gripper_pos": self._gripper_width_or_none(),
        }

    def open_gripper(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        width = self._coerce_gripper_width(payload.get("width"), "gripper_open_width")
        speed = self._coerce_positive_float(payload.get("speed"), "gripper_move_speed")
        result = self._execute_move(width=width, speed=speed)
        return {
            "ok": result.success,
            "message": "Gripper opened" if result.success else "Failed to open gripper",
            "error": result.error,
            "gripper_pos": self._gripper_width_or_none(),
            "target_width": width,
            "speed": speed,
        }

    def close_gripper(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        width = self._coerce_gripper_width(payload.get("width"), "gripper_closed_width")
        speed = self._coerce_positive_float(payload.get("speed"), "gripper_grasp_speed")
        force = self._coerce_positive_float(payload.get("force"), "gripper_grasp_force")
        epsilon_inner = self._coerce_nonnegative_float(payload.get("epsilon_inner"), "gripper_epsilon_inner")
        epsilon_outer = self._coerce_nonnegative_float(payload.get("epsilon_outer"), "gripper_epsilon_outer")
        result = self._execute_grasp(
            width=width,
            speed=speed,
            force=force,
            epsilon_inner=epsilon_inner,
            epsilon_outer=epsilon_outer,
        )
        return {
            "ok": result.success,
            "message": "Gripper closed" if result.success else "Failed to close gripper",
            "error": result.error,
            "gripper_pos": self._gripper_width_or_none(),
            "target_width": width,
            "speed": speed,
            "force": force,
        }

    def move_gripper(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        width = payload.get("width")
        if width is None and "arr" in payload:
            arr = payload["arr"]
            if not isinstance(arr, list) or not arr:
                raise ValueError("move_gripper arr must be a non-empty list")
            width = arr[0]
        width = self._coerce_gripper_width(width, "gripper_open_width")
        speed = self._coerce_positive_float(payload.get("speed"), "gripper_move_speed")
        result = self._execute_move(width=width, speed=speed)
        return {
            "ok": result.success,
            "message": "Gripper moved" if result.success else "Failed to move gripper",
            "error": result.error,
            "gripper_pos": self._gripper_width_or_none(),
            "target_width": width,
            "speed": speed,
        }

    def move_pose(self, pose: List[float], q: Optional[List[float]] = None) -> Dict[str, Any]:
        if len(pose) != 7:
            raise ValueError("pose must contain [x, y, z, qx, qy, qz, qw]")
        if q is not None and len(q) != 7:
            raise ValueError("q must contain 7 joint angles")

        states = self._controller_states()
        if states.get(self.impedance_controller) != "active":
            if self.pose_auto_activate_impedance:
                self.start_impedance()
                time.sleep(max(0.0, self.pose_fallback_activation_settle_sec))
            elif self.pose_fallback_to_ik:
                return self._move_pose_with_fallback_controller(pose, states)
            else:
                raise RuntimeError("Impedance controller is inactive and no fallback is enabled")

        msg = CartesianImpedanceCommand()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pose[0])
        msg.pose.position.y = float(pose[1])
        msg.pose.position.z = float(pose[2])
        msg.pose.orientation.x = float(pose[3])
        msg.pose.orientation.y = float(pose[4])
        msg.pose.orientation.z = float(pose[5])
        msg.pose.orientation.w = float(pose[6])
        msg.has_master_q = q is not None
        if q is not None:
            msg.master_q = [float(v) for v in q]
        self.pose_publisher.publish(msg)
        method = "cartesian_impedance_controller"
        if states.get(self.impedance_controller) != "active":
            method = "auto_started_cartesian_impedance_controller"
        response: Dict[str, Any] = {"ok": True, "message": "Moved", "pose": pose, "method": method}
        if q is not None:
            response["q"] = [float(v) for v in q]
        return response

    def move_pose_precise(self, pose: List[float], payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        if len(pose) != 7:
            raise ValueError("pose must contain [x, y, z, qx, qy, qz, qw]")
        if not self.have_pose or not self.have_joint_state:
            raise RuntimeError("Current robot pose and joint state are required before precise Cartesian motion")

        payload = payload or {}
        self._start_precise_controller()
        response = self._compute_cartesian_path(pose, payload)
        result = self._execute_trajectory(response.solution)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"Execute trajectory failed with code {result.error_code.val}")
        return {
            "ok": True,
            "message": "Moved",
            "pose": [float(value) for value in pose],
            "method": "moveit_cartesian_path",
            "fraction": float(response.fraction),
            "controller": self.precise_controller,
        }

    def clear_error(self) -> Dict[str, Any]:
        if not self.error_recovery_client.wait_for_server(timeout_sec=self.request_timeout_sec):
            raise RuntimeError("Error recovery action server is unavailable")

        future = self.error_recovery_client.send_goal_async(ErrorRecovery.Goal())
        goal_handle = self._await_future(future, self.request_timeout_sec)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Error recovery goal was rejected")
        result_future = goal_handle.get_result_async()
        self._await_future(result_future, self.request_timeout_sec)
        return {"ok": True, "message": "Clear"}

    def update_params(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        if not payload:
            raise ValueError("update_param expects a JSON object with parameter names and values")

        self._wait_for_service(self.impedance_set_params_client, "impedance_set_parameters")
        request = SetParameters.Request()
        request.parameters = [self._to_parameter_msg(name, value) for name, value in payload.items()]
        future = self.impedance_set_params_client.call_async(request)
        results = self._await_future(future, self.request_timeout_sec)
        failures = [result.reason for result in results.results if not result.successful]
        if failures:
            raise RuntimeError("; ".join(failures))
        return {"ok": True, "message": "Updated compliance parameters", "updated": list(payload.keys())}

    def start_impedance(self) -> Dict[str, Any]:
        self._ensure_controller_loaded(self.impedance_controller)
        states = self._controller_states()
        if states.get(self.impedance_controller) == "active":
            return {"ok": True, "message": "Impedance already active"}

        deactivate = []
        if states.get(self.joint_controller) == "active":
            deactivate.append(self.joint_controller)
        if states.get(self.cartesian_pose_controller) == "active":
            deactivate.append(self.cartesian_pose_controller)
        if states.get(self.precise_controller) == "active":
            deactivate.append(self.precise_controller)

        self._switch_controllers([self.impedance_controller], deactivate)
        return {
            "ok": True,
            "message": "Started impedance",
            "deactivated": deactivate,
        }

    def stop_impedance(self) -> Dict[str, Any]:
        self._switch_controllers([], [self.impedance_controller], strict=False)
        return {"ok": True, "message": "Stopped impedance"}

    def _start_precise_controller(self) -> None:
        self._ensure_controller_loaded(self.precise_controller)
        states = self._controller_states()
        if states.get(self.precise_controller) == "active":
            return

        deactivate = [
            controller
            for controller in [self.impedance_controller, self.cartesian_pose_controller, self.joint_controller]
            if states.get(controller) == "active"
        ]
        self._switch_controllers([self.precise_controller], deactivate, strict=False)

    def reset_joint(self, target: Optional[List[float]], payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        payload = payload or {}
        if target is None:
            target = list(
                self.get_parameter("reset_joint_target").get_parameter_value().double_array_value
            )
        if len(target) != 7:
            raise ValueError("joint reset target must contain 7 joint values")

        if self.joint_reset_use_ptp_action and not payload.get("use_controller", False):
            return self._reset_joint_with_ptp(target, payload)

        return self._reset_joint_with_controller(target)

    def _reset_joint_with_controller(self, target: List[float]) -> Dict[str, Any]:
        self._ensure_controller_loaded(self.joint_controller)
        self._set_joint_target(target)
        self._switch_controllers([self.joint_controller], [self.impedance_controller], strict=False)

        start_time = time.time()
        while time.time() - start_time < self.joint_reset_timeout_sec:
            with self._state_lock:
                if self.have_joint_state and self._all_close(self.q, target, atol=1e-2, rtol=1e-2):
                    break
            time.sleep(0.1)
        else:
            raise RuntimeError("joint reset TIMEOUT")

        self._switch_controllers([self.impedance_controller], [self.joint_controller], strict=False)
        return {"ok": True, "message": "Reset Joint", "target": target, "method": "joint_position_controller"}

    def _reset_joint_with_ptp(self, target: List[float], payload: Dict[str, Any]) -> Dict[str, Any]:
        states = self._controller_states()
        restore_impedance = states.get(self.impedance_controller) == "active"
        restore_cartesian_pose = states.get(self.cartesian_pose_controller) == "active"
        deactivate = [
            controller
            for controller in [self.impedance_controller, self.cartesian_pose_controller, self.joint_controller]
            if states.get(controller) == "active"
        ]
        if deactivate:
            self._switch_controllers([], deactivate, strict=False)

        maximum_joint_velocities = payload.get("maximum_joint_velocities")
        if maximum_joint_velocities is None:
            maximum_joint_velocities = self._double_array_parameter("joint_reset_maximum_joint_velocities")
        if len(maximum_joint_velocities) != 7:
            raise ValueError("maximum_joint_velocities must contain 7 values")

        goal_tolerance = payload.get("goal_tolerance")
        if goal_tolerance is None:
            goal_tolerance = self.get_parameter("joint_reset_goal_tolerance").get_parameter_value().double_value

        try:
            result = self._execute_ptp_motion(
                target=target,
                maximum_joint_velocities=[float(value) for value in maximum_joint_velocities],
                goal_tolerance=float(goal_tolerance),
            )
        finally:
            if restore_impedance:
                self._switch_controllers([self.impedance_controller], [], strict=False)
            elif restore_cartesian_pose:
                self._switch_controllers([self.cartesian_pose_controller], [], strict=False)

        if result.target_status.status != 2:
            raise RuntimeError(result.error_message or f"PTP motion failed with status {result.target_status.status}")
        return {
            "ok": True,
            "message": "Reset Joint",
            "target": target,
            "method": "ptp_motion_action",
            "maximum_joint_velocities": [float(value) for value in maximum_joint_velocities],
            "goal_tolerance": float(goal_tolerance),
        }

    def _move_pose_with_fallback_controller(
        self, pose: List[float], states: Optional[Dict[str, str]] = None
    ) -> Dict[str, Any]:
        if states is None:
            states = self._controller_states()

        target = [float(value) for value in pose]
        self._ensure_controller_loaded(self.cartesian_pose_controller)
        activate = []
        deactivate = []
        if states.get(self.cartesian_pose_controller) != "active":
            activate.append(self.cartesian_pose_controller)
        for controller in [self.impedance_controller, self.joint_controller]:
            if states.get(controller) == "active":
                deactivate.append(controller)
        if activate or deactivate:
            self._switch_controllers(activate, deactivate, strict=False)
            self._publish_pose_to_fallback_controller(self.pose)
            time.sleep(max(0.0, self.pose_fallback_activation_settle_sec))

        self._publish_pose_to_fallback_controller(target)

        return {
            "ok": True,
            "message": "Moved",
            "pose": target,
            "method": "cartesian_pose_command_controller",
        }

    def _publish_pose_to_fallback_controller(self, pose: List[float]) -> None:
        msg = PoseStamped()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pose[0])
        msg.pose.position.y = float(pose[1])
        msg.pose.position.z = float(pose[2])
        msg.pose.orientation.x = float(pose[3])
        msg.pose.orientation.y = float(pose[4])
        msg.pose.orientation.z = float(pose[5])
        msg.pose.orientation.w = float(pose[6])
        self.cartesian_pose_publisher.publish(msg)

    def _compute_ik(self, pose: List[float]):
        self._wait_for_service(self.compute_ik_client, "compute_ik")
        request = GetPositionIK.Request()
        request.ik_request.group_name = self._ik_group_name()
        request.ik_request.pose_stamped.header.frame_id = self.base_frame
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose.position.x = float(pose[0])
        request.ik_request.pose_stamped.pose.position.y = float(pose[1])
        request.ik_request.pose_stamped.pose.position.z = float(pose[2])
        request.ik_request.pose_stamped.pose.orientation.x = float(pose[3])
        request.ik_request.pose_stamped.pose.orientation.y = float(pose[4])
        request.ik_request.pose_stamped.pose.orientation.z = float(pose[5])
        request.ik_request.pose_stamped.pose.orientation.w = float(pose[6])
        request.ik_request.ik_link_name = self._ik_link_name()
        request.ik_request.robot_state.joint_state.name = self._joint_names()
        with self._state_lock:
            request.ik_request.robot_state.joint_state.position = [float(value) for value in self.q]
            request.ik_request.robot_state.joint_state.velocity = [float(value) for value in self.dq]
            request.ik_request.robot_state.joint_state.effort = [0.0] * 7
        future = self.compute_ik_client.call_async(request)
        response = self._await_future(future, self.pose_ik_timeout_sec)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"IK failed with code {response.error_code.val} for pose fallback"
            )
        return response

    def _compute_cartesian_path(
        self, pose: List[float], payload: Optional[Dict[str, Any]] = None
    ) -> GetCartesianPath.Response:
        payload = payload or {}
        self._wait_for_service(self.compute_cartesian_path_client, "compute_cartesian_path")
        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = self._ik_group_name()
        request.link_name = self._ik_link_name()
        request.max_step = float(payload.get("max_step", self._double_parameter("precise_cartesian_max_step")))
        request.jump_threshold = float(
            payload.get("jump_threshold", self._double_parameter("precise_cartesian_jump_threshold"))
        )
        request.prismatic_jump_threshold = float(
            payload.get(
                "prismatic_jump_threshold",
                self._double_parameter("precise_cartesian_prismatic_jump_threshold"),
            )
        )
        request.revolute_jump_threshold = float(
            payload.get(
                "revolute_jump_threshold",
                self._double_parameter("precise_cartesian_revolute_jump_threshold"),
            )
        )
        request.avoid_collisions = bool(
            payload.get(
                "avoid_collisions",
                self.get_parameter("precise_cartesian_avoid_collisions").get_parameter_value().bool_value,
            )
        )
        with self._state_lock:
            request.start_state.joint_state.name = self._joint_names()
            request.start_state.joint_state.position = [float(value) for value in self.q]
            request.start_state.joint_state.velocity = [float(value) for value in self.dq]
            request.start_state.joint_state.effort = [0.0] * 7

        waypoint = PoseStamped()
        waypoint.header.frame_id = self.base_frame
        waypoint.pose.position.x = float(pose[0])
        waypoint.pose.position.y = float(pose[1])
        waypoint.pose.position.z = float(pose[2])
        waypoint.pose.orientation.x = float(pose[3])
        waypoint.pose.orientation.y = float(pose[4])
        waypoint.pose.orientation.z = float(pose[5])
        waypoint.pose.orientation.w = float(pose[6])
        request.waypoints = [waypoint.pose]

        response = self._await_future(
            self.compute_cartesian_path_client.call_async(request),
            self.pose_ik_timeout_sec,
        )
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"Cartesian path planning failed with code {response.error_code.val}")

        min_fraction = float(payload.get("min_fraction", self._double_parameter("precise_cartesian_min_fraction")))
        if response.fraction < min_fraction:
            raise RuntimeError(
                f"Cartesian path planning only achieved fraction {response.fraction:.3f}, "
                f"required {min_fraction:.3f}"
            )
        if not response.solution.joint_trajectory.points:
            raise RuntimeError("Cartesian path planning returned an empty joint trajectory")
        return response

    def _execute_trajectory(self, trajectory) -> ExecuteTrajectory.Result:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        return self._run_action(
            self.execute_trajectory_client,
            self.get_parameter("execute_trajectory_action").get_parameter_value().string_value,
            goal,
            timeout_sec=self.joint_reset_timeout_sec,
        )

    def _robot_name_prefix(self) -> str:
        arm_prefix = self.arm_prefix.strip("_")
        if arm_prefix:
            return f"{arm_prefix}_{self.robot_type}"
        return self.robot_type

    def _joint_names(self) -> List[str]:
        prefix = self._robot_name_prefix()
        return [f"{prefix}_joint{index}" for index in range(1, 8)]

    def _ik_group_name(self) -> str:
        return f"{self._robot_name_prefix()}_arm"

    def _ik_link_name(self) -> str:
        prefix = self._robot_name_prefix()
        if self.load_gripper:
            return f"{prefix}_hand_tcp"
        return f"{prefix}_link8"

    def _controller_states(self) -> Dict[str, str]:
        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        response = self._await_future(future, self.request_timeout_sec)
        return {controller.name: controller.state for controller in response.controller}

    def _ensure_controller_loaded(self, controller_name: str) -> None:
        self._wait_for_service(self.list_controllers_client, "list_controllers")
        states = self._controller_states()
        if controller_name in states:
            if states[controller_name] == "unconfigured":
                self._configure_controller(controller_name)
            return

        self._wait_for_service(self.load_controller_client, "load_controller")
        load_request = LoadController.Request()
        load_request.name = controller_name
        load_response = self._await_future(
            self.load_controller_client.call_async(load_request), self.request_timeout_sec
        )
        if not load_response.ok:
            raise RuntimeError(f"Failed to load controller {controller_name}")

        self._configure_controller(controller_name)

    def _configure_controller(self, controller_name: str) -> None:
        self._wait_for_service(self.configure_controller_client, "configure_controller")
        request = ConfigureController.Request()
        request.name = controller_name
        response = self._await_future(
            self.configure_controller_client.call_async(request), self.request_timeout_sec
        )
        if not response.ok:
            raise RuntimeError(f"Failed to configure controller {controller_name}")

    def _switch_controllers(
        self, activate: List[str], deactivate: List[str], strict: bool = True
    ) -> None:
        self._wait_for_service(self.switch_controller_client, "switch_controller")
        request = SwitchController.Request()
        request.activate_controllers = [name for name in activate if name]
        request.deactivate_controllers = [name for name in deactivate if name]
        request.strictness = (
            SwitchController.Request.STRICT if strict else SwitchController.Request.BEST_EFFORT
        )
        request.activate_asap = True
        request.timeout.sec = 5
        response = self._await_future(
            self.switch_controller_client.call_async(request), self.request_timeout_sec
        )
        if not response.ok:
            raise RuntimeError(
                f"Failed to switch controllers activate={request.activate_controllers} "
                f"deactivate={request.deactivate_controllers}"
            )

    def _set_joint_target(self, target: List[float]) -> None:
        self._wait_for_service(self.joint_set_params_client, "joint_set_parameters")
        request = SetParameters.Request()
        request.parameters = [self._to_parameter_msg("target_joint_positions", [float(v) for v in target])]
        future = self.joint_set_params_client.call_async(request)
        results = self._await_future(future, self.request_timeout_sec)
        failures = [result.reason for result in results.results if not result.successful]
        if failures:
            raise RuntimeError("; ".join(failures))

    def _wait_for_service(self, client, name: str) -> None:
        if not client.wait_for_service(timeout_sec=self.request_timeout_sec):
            raise RuntimeError(f"Service {name} is unavailable")

    def _maybe_start_auto_start_sequence(self) -> None:
        if self.auto_start_started or not (self.auto_clear_error or self.auto_start_impedance):
            return

        self.auto_start_started = True
        self.auto_start_thread = threading.Thread(
            target=self._run_auto_start_sequence,
            name="serl_franka_auto_start",
            daemon=True,
        )
        self.auto_start_thread.start()

    def _run_auto_start_sequence(self) -> None:
        time.sleep(max(0.0, self.auto_start_delay_sec))

        if self.auto_clear_error:
            try:
                self.clear_error()
                self.get_logger().info("Automatic Franka error recovery finished.")
            except Exception as exc:
                self.get_logger().warn(f"Automatic Franka error recovery failed: {exc}")

        if not self.auto_start_impedance:
            return

        self._wait_for_impedance_controller_before_auto_start()
        retry_count = max(1, self.auto_start_retry_count)
        for attempt in range(1, retry_count + 1):
            try:
                result = self.start_impedance()
                self.get_logger().info(
                    f"Automatic impedance start finished: {result.get('message', 'ok')}"
                )
                return
            except Exception as exc:
                if attempt >= retry_count:
                    self.get_logger().error(f"Automatic impedance start failed: {exc}")
                    return
                self.get_logger().warn(
                    f"Automatic impedance start attempt {attempt}/{retry_count} failed: {exc}"
                )
                time.sleep(max(0.1, self.auto_start_retry_interval_sec))

    def _wait_for_impedance_controller_before_auto_start(self) -> None:
        timeout = max(0.0, self.auto_start_wait_timeout_sec)
        deadline = time.time() + timeout
        last_error = None
        while time.time() < deadline:
            try:
                states = self._controller_states()
                state = states.get(self.impedance_controller)
                if state in ("inactive", "active"):
                    self.get_logger().info(
                        "Automatic impedance start found controller ready: "
                        f"{self.impedance_controller}={state}"
                    )
                    return
                if state == "unconfigured":
                    self.get_logger().info(
                        "Automatic impedance start is waiting for controller configuration: "
                        f"{self.impedance_controller}=unconfigured"
                    )
            except Exception as exc:  # pylint: disable=broad-except
                last_error = exc
            time.sleep(0.2)

        if last_error is not None:
            self.get_logger().warn(
                "Timed out waiting for controller spawner before automatic impedance start; "
                f"continuing with internal load/configure path. Last error: {last_error}"
            )
        else:
            self.get_logger().warn(
                "Timed out waiting for controller spawner before automatic impedance start; "
                "continuing with internal load/configure path."
            )

    def _apply_default_collision_behavior_once(self) -> None:
        if not self.apply_default_collision_behavior or self.default_collision_behavior_applied:
            return

        if self.default_collision_behavior_future is not None:
            if not self.default_collision_behavior_future.done():
                return
            response = self.default_collision_behavior_future.result()
            if response.success:
                self.default_collision_behavior_applied = True
                self.get_logger().info("Default Franka collision behavior set.")
            else:
                self.default_collision_behavior_future = None
                self.get_logger().warn(f"Failed to set default collision behavior: {response.error}")
            return

        if not self.collision_behavior_client.service_is_ready():
            return

        self.default_collision_behavior_future = self.collision_behavior_client.call_async(
            self._default_collision_behavior_request()
        )

    def _default_collision_behavior_request(self) -> SetFullCollisionBehavior.Request:
        request = SetFullCollisionBehavior.Request()
        request.lower_torque_thresholds_nominal = self._double_array_parameter("lower_torque_thresholds_nominal")
        request.upper_torque_thresholds_nominal = self._double_array_parameter("upper_torque_thresholds_nominal")
        request.lower_torque_thresholds_acceleration = self._double_array_parameter(
            "lower_torque_thresholds_acceleration"
        )
        request.upper_torque_thresholds_acceleration = self._double_array_parameter(
            "upper_torque_thresholds_acceleration"
        )
        request.lower_force_thresholds_nominal = self._double_array_parameter("lower_force_thresholds_nominal")
        request.upper_force_thresholds_nominal = self._double_array_parameter("upper_force_thresholds_nominal")
        request.lower_force_thresholds_acceleration = self._double_array_parameter(
            "lower_force_thresholds_acceleration"
        )
        request.upper_force_thresholds_acceleration = self._double_array_parameter(
            "upper_force_thresholds_acceleration"
        )
        return request

    def _double_array_parameter(self, name: str) -> List[float]:
        return [float(value) for value in self.get_parameter(name).get_parameter_value().double_array_value]

    def _double_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).get_parameter_value().double_value)

    @staticmethod
    def _matrix_vector_multiply(matrix: List[List[float]], vector: List[float]) -> List[float]:
        return [sum(value * vector[index] for index, value in enumerate(row)) for row in matrix]

    @staticmethod
    def _all_close(values: List[float], target: List[float], atol: float, rtol: float) -> bool:
        return all(abs(a - b) <= (atol + rtol * abs(b)) for a, b in zip(values, target))

    @staticmethod
    def _quat_xyzw_to_euler_xyz(quat: List[float]) -> List[float]:
        x, y, z, w = [float(value) for value in quat]

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]

    def _wait_for_action_server(self, client: ActionClient, name: str, timeout_sec: Optional[float] = None) -> None:
        if timeout_sec is None:
            timeout_sec = self.gripper_timeout_sec
        if not client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(f"Action server {name} is unavailable")

    def _gripper_resource(self, suffix: str) -> str:
        return f"{self.gripper_prefix}/{suffix}" if self.gripper_prefix else suffix

    def _execute_homing(self):
        goal = Homing.Goal()
        return self._run_action(self.gripper_homing_client, self._gripper_resource("homing"), goal)

    def _execute_move(self, width: float, speed: float):
        goal = Move.Goal()
        goal.width = float(width)
        goal.speed = float(speed)
        return self._run_action(self.gripper_move_client, self._gripper_resource("move"), goal)

    def _execute_grasp(
        self, width: float, speed: float, force: float, epsilon_inner: float, epsilon_outer: float
    ):
        goal = Grasp.Goal()
        goal.width = float(width)
        goal.speed = float(speed)
        goal.force = float(force)
        goal.epsilon.inner = float(epsilon_inner)
        goal.epsilon.outer = float(epsilon_outer)
        return self._run_action(self.gripper_grasp_client, self._gripper_resource("grasp"), goal)

    def _execute_ptp_motion(
        self, target: List[float], maximum_joint_velocities: List[float], goal_tolerance: float
    ):
        goal = PTPMotion.Goal()
        goal.goal_joint_configuration = [float(value) for value in target]
        goal.maximum_joint_velocities = [float(value) for value in maximum_joint_velocities]
        goal.goal_tolerance = float(goal_tolerance)
        return self._run_action(
            self.ptp_motion_client,
            self.get_parameter("ptp_motion_action").get_parameter_value().string_value,
            goal,
            timeout_sec=self.joint_reset_timeout_sec,
        )

    def _run_action(self, client: ActionClient, name: str, goal, timeout_sec: Optional[float] = None) -> Any:
        if timeout_sec is None:
            timeout_sec = self.gripper_timeout_sec
        self._wait_for_action_server(client, name, timeout_sec=timeout_sec)
        goal_future = client.send_goal_async(goal)
        goal_handle = self._await_future(goal_future, timeout_sec)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Goal rejected by {name}")
        result_future = goal_handle.get_result_async()
        result = self._await_future(result_future, timeout_sec)
        return result.result

    def _stop_gripper(self, raise_on_failure: bool = True) -> Dict[str, Any]:
        self._wait_for_service(self.gripper_stop_client, "gripper_stop")
        future = self.gripper_stop_client.call_async(Trigger.Request())
        response = self._await_future(future, self.gripper_timeout_sec)
        result = {"ok": bool(response.success), "message": str(response.message)}
        if raise_on_failure and not response.success:
            raise RuntimeError(response.message or "Failed to stop gripper")
        return result

    def _gripper_width_or_none(self) -> Optional[float]:
        with self._state_lock:
            return self.gripper_pos if self.have_gripper else None

    def _coerce_gripper_width(self, value: Any, default_parameter_name: str) -> float:
        if value is None:
            value = self.get_parameter(default_parameter_name).get_parameter_value().double_value
        width = float(value)
        if width < 0.0:
            raise ValueError("gripper width must be >= 0")
        return width

    def _coerce_positive_float(self, value: Any, default_parameter_name: str) -> float:
        if value is None:
            value = self.get_parameter(default_parameter_name).get_parameter_value().double_value
        result = float(value)
        if result <= 0.0:
            raise ValueError(f"{default_parameter_name} must be > 0")
        return result

    def _coerce_nonnegative_float(self, value: Any, default_parameter_name: str) -> float:
        if value is None:
            value = self.get_parameter(default_parameter_name).get_parameter_value().double_value
        result = float(value)
        if result < 0.0:
            raise ValueError(f"{default_parameter_name} must be >= 0")
        return result

    @staticmethod
    def _await_future(future, timeout_sec: float):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.01)
        raise RuntimeError("Timed out waiting for ROS response")

    @staticmethod
    def _to_parameter_msg(name: str, value: Any) -> ParameterMsg:
        parameter = ParameterMsg()
        parameter.name = name
        parameter.value = FrankaHTTPBridge._to_parameter_value(value)
        return parameter

    @staticmethod
    def _to_parameter_value(value: Any) -> ParameterValue:
        parameter_value = ParameterValue()
        if isinstance(value, bool):
            parameter_value.type = ParameterType.PARAMETER_BOOL
            parameter_value.bool_value = bool(value)
            return parameter_value
        if isinstance(value, int) and not isinstance(value, bool):
            parameter_value.type = ParameterType.PARAMETER_INTEGER
            parameter_value.integer_value = int(value)
            return parameter_value
        if isinstance(value, float):
            parameter_value.type = ParameterType.PARAMETER_DOUBLE
            parameter_value.double_value = float(value)
            return parameter_value
        if isinstance(value, str):
            parameter_value.type = ParameterType.PARAMETER_STRING
            parameter_value.string_value = value
            return parameter_value
        if isinstance(value, list):
            if not value:
                parameter_value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                parameter_value.double_array_value = []
                return parameter_value
            if all(isinstance(v, bool) for v in value):
                parameter_value.type = ParameterType.PARAMETER_BOOL_ARRAY
                parameter_value.bool_array_value = [bool(v) for v in value]
                return parameter_value
            if all(isinstance(v, int) and not isinstance(v, bool) for v in value):
                parameter_value.type = ParameterType.PARAMETER_INTEGER_ARRAY
                parameter_value.integer_array_value = [int(v) for v in value]
                return parameter_value
            if all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value):
                parameter_value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                parameter_value.double_array_value = [float(v) for v in value]
                return parameter_value
            if all(isinstance(v, str) for v in value):
                parameter_value.type = ParameterType.PARAMETER_STRING_ARRAY
                parameter_value.string_array_value = [str(v) for v in value]
                return parameter_value
        raise ValueError(f"Unsupported parameter value: {value!r}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrankaHTTPBridge()
    node.start_http_server()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown_http_server()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
