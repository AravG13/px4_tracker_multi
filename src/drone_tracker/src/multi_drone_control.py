#!/usr/bin/env python3
"""
Multi-Vehicle PX4 Controller - Control ONE drone at a time
Uses namespacing: /drone_{vehicle_id}/ for ROS topics
                  /px4_{vehicle_id}/fmu/ for PX4 topics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    VehicleCommand,
    OffboardControlMode, 
    TrajectorySetpoint,
    VehicleStatus,
    VehicleLocalPosition
)
from geometry_msgs.msg import PointStamped, Vector3
from std_srvs.srv import Empty

import time
import math
import numpy as np
from enum import Enum
from dataclasses import dataclass

class DroneState(Enum):
    DISARMED = 0
    ARMED = 1
    TAKING_OFF = 2
    HOVERING = 3
    TRACKING = 4
    LANDING = 5
    EMERGENCY = 6

@dataclass
class TrackingData:
    pixel_x: int = 0
    pixel_y: int = 0
    frame_width: int = 640
    frame_height: int = 480
    bbox_area: float = 0.0
    confidence: float = 0.0
    boundary_violation: bool = False
    timestamp: float = 0.0

class StablePIDController:
    def __init__(self, kp, ki, kd, max_output=1.0, max_integral=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.max_integral = max_integral
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0
        
    def compute(self, error, dt):
        proportional = self.kp * error
        
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        integral_term = self.ki * self.integral
        
        derivative = (error - self.prev_error) / dt
        derivative = 0.8 * self.prev_derivative + 0.2 * derivative
        self.prev_derivative = derivative
        derivative_term = self.kd * derivative
        
        output = proportional + integral_term + derivative_term
        output = max(min(output, self.max_output), -self.max_output)
        
        self.prev_error = error
        return output
        
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0

class ExponentialSmoother:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.value = 0.0
        self.initialized = False
        
    def filter(self, new_value):
        if not self.initialized:
            self.value = new_value
            self.initialized = True
        else:
            self.value = self.alpha * new_value + (1.0 - self.alpha) * self.value
        return self.value
        
    def reset(self):
        self.value = 0.0
        self.initialized = False

class MultiVehiclePX4Controller(Node):
    def __init__(self):
        super().__init__('multi_vehicle_px4_controller')
        
        # ============================================
        # MULTI-VEHICLE CONFIGURATION
        # ============================================
        self.declare_parameter('vehicle_id', 1)
        self.vehicle_id = self.get_parameter('vehicle_id').value
        
        # PX4 namespace (from PX4 multi-vehicle)
        self.px4_ns = f'/px4_{self.vehicle_id}'
        
        # ROS namespace (for our topics)
        self.ros_ns = f'/drone_{self.vehicle_id}'
        
        self.get_logger().info("="*60)
        self.get_logger().info(f"  Multi-Vehicle Controller - Vehicle {self.vehicle_id}")
        self.get_logger().info("="*60)
        self.get_logger().info(f"PX4 Topics:  {self.px4_ns}/fmu/*")
        self.get_logger().info(f"ROS Topics:  {self.ros_ns}/*")
        self.get_logger().info(f"Services:    {self.ros_ns}/*")
        self.get_logger().info("="*60)
        
        # QoS profile for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # ============================================
        # PX4 Publishers (namespaced by vehicle)
        # ============================================
        self.command_pub = self.create_publisher(
            VehicleCommand, f'{self.px4_ns}/fmu/in/vehicle_command', qos_profile)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, f'{self.px4_ns}/fmu/in/offboard_control_mode', qos_profile)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f'{self.px4_ns}/fmu/in/trajectory_setpoint', qos_profile)
        
        # ============================================
        # PX4 Subscribers (namespaced by vehicle)
        # ============================================
        self.status_sub = self.create_subscription(
            VehicleStatus, f'{self.px4_ns}/fmu/out/vehicle_status', 
            self.status_callback, qos_profile)
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, f'{self.px4_ns}/fmu/out/vehicle_local_position',
            self.position_callback, qos_profile)
        
        # ============================================
        # Tracking Subscribers (namespaced by vehicle)
        # ============================================
        self.target_sub = self.create_subscription(
            PointStamped, f'{self.ros_ns}/detected_target',
            self.target_callback, 10)
        self.bbox_sub = self.create_subscription(
            Vector3, f'{self.ros_ns}/target_bbox_info',
            self.bbox_callback, 10)
        
        # ============================================
        # Services (namespaced by vehicle)
        # ============================================
        self.create_service(Empty, f'{self.ros_ns}/arm', self.arm_callback)
        self.create_service(Empty, f'{self.ros_ns}/disarm', self.disarm_callback)
        self.create_service(Empty, f'{self.ros_ns}/takeoff', self.takeoff_callback)
        self.create_service(Empty, f'{self.ros_ns}/land', self.land_callback)
        self.create_service(Empty, f'{self.ros_ns}/emergency', self.emergency_callback)
        self.create_service(Empty, f'{self.ros_ns}/start_tracking', self.start_tracking_callback)
        self.create_service(Empty, f'{self.ros_ns}/stop_tracking', self.stop_tracking_callback)
        
        # ============================================
        # State variables
        # ============================================
        self.state = DroneState.DISARMED
        self.armed = False
        self.nav_state = 0
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.current_vel = np.array([0.0, 0.0, 0.0])
        self.home_pos = None
        self.tracking_enabled = False
        
        # Flight parameters
        self.takeoff_altitude = -5.0
        self.TARGET_FOLLOW_DISTANCE = 3.5
        self.MIN_ALTITUDE = 3.0
        self.SAFETY_ALTITUDE = 4.0
        self.hover_altitude = -5.0
        
        # PID Controllers
        self.pid_x = StablePIDController(0.8, 0.01, 0.1, 1.5, 0.3)
        self.pid_y = StablePIDController(1.2, 0.02, 0.15, 2.0, 0.4)
        self.pid_z = StablePIDController(0.3, 0.005, 0.02, 0.8, 0.1)
        self.pid_distance = StablePIDController(0.6, 0.015, 0.08, 1.5, 0.25)
        
        # Smoothers
        self.smoother_x = ExponentialSmoother(0.30)
        self.smoother_y = ExponentialSmoother(0.20)
        self.smoother_z = ExponentialSmoother(0.10)
        self.smoother_distance = ExponentialSmoother(0.15)
        
        # Tracking data
        self.tracking_data = TrackingData()
        self.last_target_time = None
        self.target_timeout = 2.0
        self.last_control_time = time.time()
        
        # Backing away state
        self.backing_away_mode = False
        self.backup_start_time = None
        self.backup_duration = 2.0
        
        # State machine
        self.command_start_time = None
        self.sequence_step = 0
        
        # Ground safety
        self.estimated_altitude = 5.0
        self.prev_bbox_size = 0.0
        self.prev_bbox_time = time.time()
        self.size_rate = 0.0
        
        # Timers
        self.offboard_timer = self.create_timer(0.02, self.publish_offboard_mode)
        self.state_timer = self.create_timer(0.1, self.state_machine)
        self.control_timer = self.create_timer(0.05, self.tracking_control_loop)
        
        self.get_logger().info(f"✓ Vehicle {self.vehicle_id} controller ready!")
        self.get_logger().info(f"✓ Services: {self.ros_ns}/[arm|takeoff|start_tracking|land]")
        self.get_logger().info("Waiting for PX4 connection...")
    
    def status_callback(self, msg):
        old_armed = self.armed
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state
        
        if old_armed != self.armed:
            status = "ARMED" if self.armed else "DISARMED"
            self.get_logger().info(f"[V{self.vehicle_id}] {status}")
        
        if not self.armed and self.state != DroneState.DISARMED:
            self.state = DroneState.DISARMED
            self.tracking_enabled = False
            self.reset_controllers()
    
    def position_callback(self, msg):
        self.current_pos = np.array([msg.x, msg.y, msg.z])
        self.current_vel = np.array([msg.vx, msg.vy, msg.vz])
        self.estimated_altitude = -msg.z
        self.current_heading = msg.heading
        
        if self.home_pos is None and self.armed:
            self.home_pos = self.current_pos.copy()
            self.hover_altitude = msg.z
            self.get_logger().info(f"[V{self.vehicle_id}] Home: [{msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}]")
    
    def target_callback(self, msg):
        cx = self.tracking_data.frame_width / 2
        cy = self.tracking_data.frame_height / 2
        
        norm_x = msg.point.x / 1.5
        norm_y = msg.point.y / 1.5
        
        self.tracking_data.pixel_x = int(norm_x * cx + cx)
        self.tracking_data.pixel_y = int(norm_y * cy + cy)
        self.tracking_data.timestamp = time.time()
        self.last_target_time = self.get_clock().now()
    
    def bbox_callback(self, msg):
        self.tracking_data.bbox_area = msg.x
        self.tracking_data.confidence = msg.y
        self.tracking_data.boundary_violation = (msg.z > 0.5)
    
    def tracking_control_loop(self):
        if not self.tracking_enabled or self.state != DroneState.TRACKING:
            return
        
        if self.tracking_data.bbox_area <= 0:
            setpoint = TrajectorySetpoint()
            setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            setpoint.position = [self.current_pos[0], self.current_pos[1], self.takeoff_altitude]
            setpoint.velocity = [0.0, 0.0, 0.0]
            setpoint.yaw = float('nan')
            self.setpoint_pub.publish(setpoint)
            return
        
        if self.last_target_time:
            time_since_target = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
            if time_since_target > self.target_timeout:
                self.state = DroneState.HOVERING
                self.tracking_enabled = False
                return
        
        current_time = time.time()
        dt = current_time - self.last_control_time
        dt = max(min(dt, 0.1), 0.01)
        self.last_control_time = current_time
        
        cx = self.tracking_data.frame_width / 2
        cy = self.tracking_data.frame_height / 2
        
        err_x = (self.tracking_data.pixel_x - cx) / cx
        err_y = (self.tracking_data.pixel_y - cy) / cy
        
        current_size = math.sqrt(self.tracking_data.bbox_area)
        boundary_violation = self.tracking_data.boundary_violation
        
        boundary_error = 0.0
        if self.tracking_data.bbox_area > 0:
            margin = 50
            frame_w = self.tracking_data.frame_width
            frame_h = self.tracking_data.frame_height
            
            bbox_half_size = math.sqrt(self.tracking_data.bbox_area) / 2
            bbox_x = self.tracking_data.pixel_x - bbox_half_size
            bbox_y = self.tracking_data.pixel_y - bbox_half_size
            bbox_x2 = self.tracking_data.pixel_x + bbox_half_size
            bbox_y2 = self.tracking_data.pixel_y + bbox_half_size
            
            if bbox_x < margin: boundary_error += (margin - bbox_x) / frame_w
            if bbox_y < margin: boundary_error += (margin - bbox_y) / frame_h
            if bbox_x2 > frame_w - margin: boundary_error += (bbox_x2 - (frame_w - margin)) / frame_w
            if bbox_y2 > frame_h - margin: boundary_error += (bbox_y2 - (frame_h - margin)) / frame_h
        
        if not self.backing_away_mode and (boundary_violation or boundary_error > 0.05):
            self.backing_away_mode = True
            self.backup_start_time = current_time
            self.get_logger().info(f"[V{self.vehicle_id}] BACKUP MODE")
        elif self.backing_away_mode:
            backup_elapsed = current_time - self.backup_start_time
            if backup_elapsed >= self.backup_duration and not boundary_violation and boundary_error < 0.02:
                self.backing_away_mode = False
                self.get_logger().info(f"[V{self.vehicle_id}] BACKUP COMPLETE")
        
        if self.backing_away_mode:
            backup_elapsed = current_time - self.backup_start_time
            if backup_elapsed < 0.5:
                backup_intensity = 0.2 + (0.6 * (backup_elapsed / 0.5))
            else:
                backup_intensity = 0.8 * math.exp(-(backup_elapsed - 0.5) / 2.0)
            distance_error = backup_intensity
            if boundary_violation or boundary_error > 0.03:
                distance_error = max(distance_error, 0.3)
        else:
            TARGET_DISTANCE_PIXELS = 70.0
            
            if not hasattr(self, 'prev_bbox_size'):
                self.prev_bbox_size = current_size
                self.prev_bbox_time = current_time
                self.size_rate = 0.0
            
            size_dt = current_time - self.prev_bbox_time
            if size_dt > 0.1:
                self.size_rate = (current_size - self.prev_bbox_size) / size_dt
                self.prev_bbox_size = current_size
                self.prev_bbox_time = current_time
            
            size_error = (current_size - TARGET_DISTANCE_PIXELS) / TARGET_DISTANCE_PIXELS
            
            rate_threshold = 3.0
            if abs(self.size_rate) > rate_threshold:
                rate_error = self.size_rate / 30.0
                distance_error = size_error + rate_error * 0.6
            else:
                distance_error = size_error
            
            distance_error = max(min(distance_error, 0.4), -0.4)
        
        POSITION_DEADZONE = 0.08
        ALTITUDE_DEADZONE = 0.15
        DISTANCE_DEADZONE = 0.12
        
        ctrl_x = 0.0
        ctrl_y = 0.0
        
        if abs(err_x) > POSITION_DEADZONE:
            ctrl_y = self.pid_y.compute(err_x, dt)
        else:
            self.pid_y.reset()
        
        if abs(distance_error) > DISTANCE_DEADZONE:
            ctrl_x = -self.pid_distance.compute(distance_error, dt)
        else:
            ctrl_x = 0.0
            self.pid_distance.reset()
        
        if not hasattr(self, 'prev_ctrl_x'):
            self.prev_ctrl_x = 0.0
            self.prev_ctrl_y = 0.0
        
        MAX_ACCEL = 0.4
        delta_x = ctrl_x - self.prev_ctrl_x
        delta_y = ctrl_y - self.prev_ctrl_y
        
        if abs(delta_x) > MAX_ACCEL:
            ctrl_x = self.prev_ctrl_x + (MAX_ACCEL if delta_x > 0 else -MAX_ACCEL)
        if abs(delta_y) > MAX_ACCEL:
            ctrl_y = self.prev_ctrl_y + (MAX_ACCEL if delta_y > 0 else -MAX_ACCEL)
        
        self.prev_ctrl_x = ctrl_x
        self.prev_ctrl_y = ctrl_y
        
        vx = self.smoother_x.filter(ctrl_x)
        vy = self.smoother_y.filter(ctrl_y)
        
        MAX_VEL_XY = 1.0
        vx = max(min(vx, MAX_VEL_XY), -MAX_VEL_XY)
        vy = max(min(vy, MAX_VEL_XY), -MAX_VEL_XY)
        
        if abs(vx) < 0.05: vx = 0.0
        if abs(vy) < 0.05: vy = 0.0
        
        yaw = getattr(self, 'current_heading', 0.0)
        ned_vx = vx * math.cos(yaw) - vy * math.sin(yaw)
        ned_vy = vx * math.sin(yaw) + vy * math.cos(yaw)
        
        if not hasattr(self, 'target_altitude'):
            self.target_altitude = self.takeoff_altitude
        
        if self.estimated_altitude < 2.0:
            self.target_altitude = -6.0
            self.get_logger().error(f"[V{self.vehicle_id}] GROUND SAFETY: {self.estimated_altitude:.1f}m - CLIMBING")
        elif abs(err_y) > ALTITUDE_DEADZONE and self.estimated_altitude > 3.0:
            altitude_adjustment = err_y * 0.8
            self.target_altitude += altitude_adjustment * 0.04
            self.target_altitude = max(min(self.target_altitude, -2.5), -8.0)
        
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint.position = [float('nan'), float('nan'), self.target_altitude]
        setpoint.velocity = [float(ned_vx), float(ned_vy), float('nan')]
        setpoint.yaw = float('nan')
        
        self.setpoint_pub.publish(setpoint)
    
    def reset_controllers(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_distance.reset()
        self.smoother_x.reset()
        self.smoother_y.reset()
        self.smoother_z.reset()
        self.smoother_distance.reset()
    
    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)
    
    def send_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
    
    def state_machine(self):
        now = self.get_clock().now()
        
        if self.state == DroneState.ARMED and self.command_start_time:
            elapsed = (now - self.command_start_time).nanoseconds / 1e9
            
            if self.sequence_step == 1 and elapsed > 1.0:
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.sequence_step = 2
            elif self.sequence_step == 2 and elapsed > 2.0:
                self.state = DroneState.TAKING_OFF
                self.sequence_step = 0
                
        elif self.state == DroneState.TAKING_OFF:
            self.send_takeoff_setpoint()
            if self.nav_state != 14:
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            
            current_altitude = -self.current_pos[2]
            target_altitude = -self.takeoff_altitude
            
            if abs(current_altitude - target_altitude) < 0.8:
                self.get_logger().info(f"[V{self.vehicle_id}] ✓ Takeoff complete at {current_altitude:.1f}m")
                self.state = DroneState.HOVERING
                
        elif self.state == DroneState.HOVERING:
            self.send_hover_setpoint()
        elif self.state == DroneState.TRACKING:
            if self.nav_state != 14:
                self.get_logger().warn(f"[V{self.vehicle_id}] Lost OFFBOARD!", throttle_duration_sec=2.0)
    
    def send_takeoff_setpoint(self):
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint.position = [self.current_pos[0], self.current_pos[1], self.takeoff_altitude]
        setpoint.velocity = [float('nan')] * 3
        setpoint.yaw = float('nan')
        self.setpoint_pub.publish(setpoint)
    
    def send_hover_setpoint(self):
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint.position = [self.current_pos[0], self.current_pos[1], self.takeoff_altitude]
        setpoint.velocity = [0.0, 0.0, 0.0]
        setpoint.yaw = float('nan')
        self.setpoint_pub.publish(setpoint)
    
    # Service callbacks
    def arm_callback(self, request, response):
        if self.state != DroneState.DISARMED:
            return response
        
        self.get_logger().info(f"[V{self.vehicle_id}] ARM sequence...")
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        time.sleep(2.0)
        
        for i in range(100):
            setpoint = TrajectorySetpoint()
            setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            setpoint.position = [0.0, 0.0, self.takeoff_altitude] 
            setpoint.velocity = [0.0, 0.0, 0.0]
            setpoint.yaw = float('nan')
            self.setpoint_pub.publish(setpoint)
            time.sleep(0.02)
        
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        time.sleep(1.0)
        self.state = DroneState.ARMED
        self.command_start_time = self.get_clock().now()
        self.sequence_step = 1
        return response
    
    def disarm_callback(self, request, response):
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.state = DroneState.DISARMED
        self.tracking_enabled = False
        return response
    
    def takeoff_callback(self, request, response):
        if self.state == DroneState.DISARMED:
            return self.arm_callback(request, response)
        return response
    
    def land_callback(self, request, response):
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.state = DroneState.LANDING
        self.tracking_enabled = False
        return response
    
    def emergency_callback(self, request, response):
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0)
        self.state = DroneState.EMERGENCY
        self.tracking_enabled = False
        return response
    
    def start_tracking_callback(self, request, response):
        if self.state not in [DroneState.HOVERING, DroneState.TRACKING]:
            self.get_logger().warn(f"[V{self.vehicle_id}] Can only track from HOVER")
            return response
        
        if self.tracking_data.bbox_area <= 0:
            self.get_logger().warn(f"[V{self.vehicle_id}] No target selected!")
            return response
        
        self.get_logger().info(f"[V{self.vehicle_id}] ✓ Starting tracking")
        self.tracking_enabled = True
        self.state = DroneState.TRACKING
        self.reset_controllers()
        self.last_control_time = time.time()
        return response
    
    def stop_tracking_callback(self, request, response):
        self.get_logger().info(f"[V{self.vehicle_id}] Stopping tracking")
        self.tracking_enabled = False
        self.state = DroneState.HOVERING
        self.reset_controllers()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MultiVehiclePX4Controller()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()