#!/usr/bin/env python3
# launch/fixed_drone_tracking.launch.py - Multi-drone support

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():
    
    # Launch arguments
    px4_instance_arg = DeclareLaunchArgument(
        'px4_instance',
        default_value='0',
        description='PX4 instance number (0=first drone, 1=second, etc.)'
    )
    
    enable_manual_control_arg = DeclareLaunchArgument(
        'enable_manual_control',
        default_value='false',
        description='Start manual control interface'
    )
    
    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude',
        default_value='5.0',
        description='Takeoff altitude in meters'
    )
    
    camera_source_arg = DeclareLaunchArgument(
        'camera_source',
        default_value='auto',
        description='Camera source: auto, gstreamer, usb, or test'
    )
    
    return LaunchDescription([
        # Arguments
        px4_instance_arg,
        enable_manual_control_arg,
        takeoff_altitude_arg,
        camera_source_arg,
        
        # System information
        LogInfo(msg="=== MULTI-DRONE TRACKING SYSTEM ==="),
        LogInfo(msg="Prerequisites:"),
        LogInfo(msg="1. PX4 SITL with multiple instances running"),
        LogInfo(msg="2. QGroundControl closed"),
        LogInfo(msg="3. Display available for OpenCV"),
        LogInfo(msg="====================================="),
        
        # System check with instance detection
        ExecuteProcess(
            cmd=['bash', '-c', '''
                echo "Checking system..."
                
                PX4_INSTANCE=${1:-0}
                echo "Checking for PX4 instance: $PX4_INSTANCE"
                
                if pgrep -f "px4.*instance.*$PX4_INSTANCE" > /dev/null; then
                    echo "✓ PX4 instance $PX4_INSTANCE running"
                else
                    echo "⚠ PX4 instance $PX4_INSTANCE not detected"
                    echo "  Start with: make px4_sitl gazebo-classic ARGS=\"-i $PX4_INSTANCE\""
                fi
                
                # Check video port
                VIDEO_PORT=$((5600 + PX4_INSTANCE * 10))
                echo "Expected video port: $VIDEO_PORT"
                
                if netstat -tuln 2>/dev/null | grep -q ":$VIDEO_PORT "; then
                    echo "✓ Port $VIDEO_PORT is listening"
                else
                    echo "⚠ Port $VIDEO_PORT not listening (video may not be streaming)"
                fi
                
                if [ -n "$DISPLAY" ]; then
                    echo "✓ Display available: $DISPLAY"
                else
                    echo "✗ No DISPLAY - OpenCV windows may fail"
                fi
                
                if [ -n "$ROS_DISTRO" ]; then
                    echo "✓ ROS2 $ROS_DISTRO active"
                else
                    echo "✗ ROS2 environment not sourced"
                fi
                
                # Check for QGroundControl
                if pgrep -f "QGroundControl" > /dev/null; then
                    echo "⚠ WARNING: QGroundControl is running - may conflict with video stream"
                fi
            '''],
            output='screen',
            additional_env={'1': LaunchConfiguration('px4_instance')}
        ),
        
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="Starting camera and tracking nodes..."),
                
                # Camera node with instance-aware configuration
                Node(
                    package='drone_tracker',
                    executable='gstreamer_camera_node',
                    name='camera_node',
                    output='screen',
                    parameters=[{
                        'px4_instance': LaunchConfiguration('px4_instance'),
                        'camera_source': LaunchConfiguration('camera_source'),
                        'base_port': 5600,  # PX4 SITL base video port
                        'frame_width': 640,
                        'frame_height': 480,
                        'fps': 30,
                        'use_sim_time': False
                    }],
                    respawn=True,
                    respawn_delay=2.0
                ),
                
                # Tracker node with namespace support
                Node(
                    package='drone_tracker',
                    executable='tracker_node',
                    name='object_tracker',
                    output='screen',
                    parameters=[{
                        'px4_instance': LaunchConfiguration('px4_instance'),
                        'use_sim_time': False
                    }],
                    # Remap topics based on instance
                    remappings=[
                        # Camera will publish to /px4_N/camera/image_raw for N>0
                        # Tracker will subscribe from correct namespace automatically
                    ],
                    additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')},
                    emulate_tty=True
                ),
                
                # Drone controller with instance-aware topics
                Node(
                    package='drone_tracker',
                    executable='working_px4_control.py',
                    name='drone_controller',
                    output='screen',
                    parameters=[{
                        'px4_instance': LaunchConfiguration('px4_instance'),
                        'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                        'use_sim_time': False
                    }],
                    # Remap PX4 topics based on instance
                    remappings=[
                        # For instance > 0, PX4 topics are in /px4_N/ namespace
                        # Services stay in root namespace
                    ],
                    emulate_tty=True
                ),
            ]
        ),
        
        # Manual control (optional)
        Node(
            condition=IfCondition(LaunchConfiguration('enable_manual_control')),
            package='drone_tracker',
            executable='manual_drone_control.py',
            name='manual_controller',
            output='screen',
            parameters=[{
                'px4_instance': LaunchConfiguration('px4_instance')
            }],
            emulate_tty=True,
            prefix='gnome-terminal -- '
        ),
        
        # Status monitor with instance info
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', '''
                        PX4_INSTANCE=${1:-0}
                        echo ""
                        echo "=== DRONE $PX4_INSTANCE STATUS ==="
                        
                        # Determine namespace
                        if [ "$PX4_INSTANCE" -eq 0 ]; then
                            NAMESPACE=""
                            PREFIX=""
                        else
                            NAMESPACE="/px4_$PX4_INSTANCE"
                            PREFIX="px4_$PX4_INSTANCE/"
                        fi
                        
                        echo "Namespace: ${NAMESPACE:-root}"
                        echo ""
                        
                        echo "Active Nodes:"
                        ros2 node list 2>/dev/null | grep -E "(camera|tracker|controller)" | head -5
                        
                        echo ""
                        echo "Key Topics:"
                        
                        # Camera feed
                        CAMERA_TOPIC="${NAMESPACE}/camera/image_raw"
                        if ros2 topic list 2>/dev/null | grep -q "$CAMERA_TOPIC"; then
                            echo "✓ Camera: $CAMERA_TOPIC"
                            timeout 2 ros2 topic hz "$CAMERA_TOPIC" 2>/dev/null | head -1 || echo "  (checking rate...)"
                        else
                            echo "✗ No camera feed at $CAMERA_TOPIC"
                        fi
                        
                        # Tracking topics
                        if ros2 topic list 2>/dev/null | grep -q "/detected_target"; then
                            echo "✓ Target detection: /detected_target"
                        else
                            echo "- Target detection: waiting for selection"
                        fi
                        
                        # PX4 connection
                        PX4_TOPIC="${NAMESPACE}/fmu/out/vehicle_status"
                        if ros2 topic list 2>/dev/null | grep -q "$PX4_TOPIC"; then
                            echo "✓ PX4 connection: $PX4_TOPIC"
                        else
                            echo "✗ PX4 not connected at $PX4_TOPIC"
                        fi
                        
                        echo ""
                        echo "=== CONTROL COMMANDS ==="
                        echo "Service calls (add namespace prefix for instance > 0):"
                        echo "  ros2 service call /drone/arm std_srvs/srv/Empty"
                        echo "  ros2 service call /drone/takeoff std_srvs/srv/Empty"
                        echo "  ros2 service call /drone/start_tracking std_srvs/srv/Empty"
                        echo "  ros2 service call /drone/land std_srvs/srv/Empty"
                        echo ""
                        echo "For testing without PX4:"
                        echo "  ros2 launch drone_tracker fixed_drone_tracking.launch.py camera_source:=test"
                        echo "========================"
                    '''],
                    output='screen',
                    additional_env={'1': LaunchConfiguration('px4_instance')}
                )
            ]
        )
    ])