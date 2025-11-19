#!/usr/bin/env python3
"""
launch/drone_tracking_multi.launch.py
Complete multi-vehicle launch file - Control ONE drone at a time
All topics namespaced: /drone_{vehicle_id}/*
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    
    # ============================================
    # LAUNCH ARGUMENTS
    # ============================================
    vehicle_id_arg = DeclareLaunchArgument(
        'vehicle_id',
        default_value='1',
        description='Which drone to control (1, 2, or 3)'
    )
    
    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude',
        default_value='5.0',
        description='Takeoff altitude in meters'
    )
    
    # Create LaunchConfiguration objects
    vehicle_id = LaunchConfiguration('vehicle_id')
    takeoff_altitude = LaunchConfiguration('takeoff_altitude')  # THIS WAS MISSING!
    
    return LaunchDescription([
        vehicle_id_arg,
        takeoff_altitude_arg,
        
        # ============================================
        # STARTUP INFO
        # ============================================
        LogInfo(msg="============================================================"),
        LogInfo(msg="   MULTI-VEHICLE DRONE TRACKING SYSTEM"),
        LogInfo(msg="============================================================"),
        LogInfo(msg=["Controlling Vehicle: ", vehicle_id]),
        LogInfo(msg=""),
        LogInfo(msg="Prerequisites:"),
        LogInfo(msg="1. PX4 Multi-Vehicle SITL running:"),
        LogInfo(msg="   cd ~/PX4-Autopilot"),
        LogInfo(msg="   ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m typhoon_h480 -n 3"),
        LogInfo(msg="2. QGroundControl closed"),
        LogInfo(msg="3. Only ONE drone controller active at a time"),
        LogInfo(msg="============================================================"),
        
        # ============================================
        # SYSTEM CHECK
        # ============================================
        ExecuteProcess(
            cmd=['bash', '-c', '''
                echo ""
                echo "=== PRE-FLIGHT CHECK ==="
                
                # Check PX4 processes
                count=$(pgrep -c "px4" || echo 0)
                echo "PX4 Processes: $count"
                if [ $count -ge 3 ]; then
                    echo "  ✓ Multi-vehicle simulation detected"
                else
                    echo "  ⚠ Warning: Expected 3 PX4 processes, found $count"
                    echo "  Start with: ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m typhoon_h480 -n 3"
                fi
                
                # Check ROS environment
                if [ -n "$ROS_DISTRO" ]; then
                    echo "  ✓ ROS2 $ROS_DISTRO active"
                else
                    echo "  ✗ ROS2 not sourced!"
                fi
                
                # Check for existing drone controllers
                existing=$(ros2 node list 2>/dev/null | grep -c "multi_vehicle_px4_controller" || echo 0)
                if [ $existing -gt 0 ]; then
                    echo "  ⚠ Warning: $existing drone controller(s) already running"
                    echo "  Note: Only control ONE drone at a time!"
                fi
                
                echo ""
                sleep 2
                
                # Check PX4 topics
                echo "Checking PX4 topics..."
                timeout 3 ros2 topic list 2>/dev/null | grep -E "/px4_[0-9]/" > /dev/null
                if [ $? -eq 0 ]; then
                    echo "  ✓ PX4 topics found:"
                    timeout 2 ros2 topic list 2>/dev/null | grep -E "/px4_[0-9]/fmu/out/vehicle_status" | cut -d'/' -f2 | sort -u | sed 's/^/    /'
                else
                    echo "  ⚠ No PX4 topics found yet (may still be starting)"
                fi
                
                echo "==========================="
                echo ""
            '''],
            output='screen'
        ),
        
        # ============================================
        # CAMERA NODE (Only ONE subscriber allowed!)
        # ============================================
        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg=[">>> Starting nodes for Vehicle ", vehicle_id, " <<<"]),
                
                Node(
                    package='drone_tracker',
                    executable='gstreamer_camera_node',
                    name='camera_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'vehicle_id': vehicle_id
                    }],
                    respawn=True,
                    respawn_delay=3.0
                ),
            ]
        ),
        
        # ============================================
        # TRACKER NODE
        # ============================================
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='drone_tracker',
                    executable='tracker_node',
                    name='object_tracker',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'vehicle_id': vehicle_id
                    }],
                    additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')},
                    emulate_tty=True
                ),
            ]
        ),
        
        # ============================================
        # CONTROLLER NODE
        # ============================================
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='drone_tracker',
                    executable='multi_drone_control.py',
                    name='drone_controller',
                    output='screen',
                    parameters=[{
                        'vehicle_id': vehicle_id,
                        'takeoff_altitude': takeoff_altitude,
                        'use_sim_time': False
                    }],
                    emulate_tty=True
                ),
            ]
        ),
        
        # ============================================
        # STATUS MONITOR
        # ============================================
        TimerAction(
            period=12.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', '''
                        # Get vehicle_id from parameter (will be available after nodes start)
                        vid=$(ros2 param get /drone_controller vehicle_id 2>/dev/null | grep -oE '[0-9]+' | head -1 || echo 1)
                        
                        echo ""
                        echo "============================================================"
                        echo "   VEHICLE $vid STATUS"
                        echo "============================================================"
                        
                        # Check nodes
                        echo ""
                        echo "Active Nodes:"
                        ros2 node list 2>/dev/null | grep -E "(camera|tracker|controller)" | sed 's/^/  /' || echo "  (none)"
                        
                        # Check PX4 connection
                        echo ""
                        echo "PX4 Connection:"
                        if timeout 2 ros2 topic list 2>/dev/null | grep -q "/px4_$vid/fmu/out/vehicle_status"; then
                            echo "  ✓ Connected to /px4_$vid/fmu/*"
                        else
                            echo "  ✗ No connection to /px4_$vid/fmu/*"
                            echo "  Available PX4 vehicles:"
                            timeout 2 ros2 topic list 2>/dev/null | grep -E "/px4_[0-9]/fmu/out/vehicle_status" | cut -d'/' -f2 | sort -u | sed 's/^/    /' || echo "    (none)"
                        fi
                        
                        # Check camera
                        echo ""
                        echo "Camera Feed:"
                        if timeout 2 ros2 topic list 2>/dev/null | grep -q "/drone_$vid/camera/image_raw"; then
                            echo "  ✓ Topic: /drone_$vid/camera/image_raw"
                            timeout 3 ros2 topic hz /drone_$vid/camera/image_raw 2>/dev/null | head -1 | sed 's/^/    /' || echo "    (waiting for data)"
                        else
                            echo "  ✗ No camera topic for vehicle $vid"
                        fi
                        
                        # Check tracking topics
                        echo ""
                        echo "Tracking Topics:"
                        for topic in "detected_target" "target_bbox_info"; do
                            if timeout 2 ros2 topic list 2>/dev/null | grep -q "/drone_$vid/$topic"; then
                                echo "  ✓ /drone_$vid/$topic"
                            else
                                echo "  ✗ /drone_$vid/$topic"
                            fi
                        done
                        
                        # Check services
                        echo ""
                        echo "Available Services:"
                        services=$(ros2 service list 2>/dev/null | grep "/drone_$vid/" | wc -l)
                        if [ $services -gt 0 ]; then
                            echo "  ✓ $services services for vehicle $vid"
                            ros2 service list 2>/dev/null | grep "/drone_$vid/" | head -5 | sed 's/^/    /'
                        else
                            echo "  ✗ No services found for vehicle $vid"
                        fi
                        
                        echo ""
                        echo "============================================================"
                        echo "   CONTROL COMMANDS"
                        echo "============================================================"
                        echo ""
                        echo "Basic Flight:"
                        echo "  ros2 service call /drone_$vid/arm std_srvs/srv/Empty"
                        echo "  ros2 service call /drone_$vid/takeoff std_srvs/srv/Empty"
                        echo "  ros2 service call /drone_$vid/land std_srvs/srv/Empty"
                        echo "  ros2 service call /drone_$vid/emergency std_srvs/srv/Empty"
                        echo ""
                        echo "Tracking Control:"
                        echo "  1. Select target in OpenCV window (click and drag)"
                        echo "  2. ros2 service call /drone_$vid/start_tracking std_srvs/srv/Empty"
                        echo "  3. ros2 service call /drone_$vid/stop_tracking std_srvs/srv/Empty"
                        echo ""
                        echo "Monitoring:"
                        echo "  ros2 topic echo /px4_$vid/fmu/out/vehicle_local_position"
                        echo "  ros2 topic echo /drone_$vid/detected_target"
                        echo "  ros2 param get /drone_controller vehicle_id"
                        echo ""
                        echo "To control different vehicle:"
                        echo "  1. Stop this launch (Ctrl+C)"
                        echo "  2. ros2 launch drone_tracker drone_tracking_multi.launch.py vehicle_id:=2"
                        echo "============================================================"
                        echo ""
                    '''],
                    output='screen'
                )
            ]
        )
    ])