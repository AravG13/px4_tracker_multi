#!/usr/bin/env python3
"""
Diagnostic PX4 Controller - Shows exactly what topics are being used
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
import time

class DiagnosticController(Node):
    def __init__(self):
        super().__init__('diagnostic_controller')
        
        # Get px4_instance parameter
        self.declare_parameter('px4_instance', 1)
        self.px4_instance = self.get_parameter('px4_instance').value
        
        # Determine namespace
        if self.px4_instance == 0:
            self.topic_prefix = ""
        else:
            self.topic_prefix = f"/px4_{self.px4_instance}"
        
        print("\n" + "="*60)
        print(f"DIAGNOSTIC CONTROLLER - Instance {self.px4_instance}")
        print("="*60)
        print(f"Topic Prefix: '{self.topic_prefix}'")
        print(f"MAV_SYS_ID: {self.px4_instance + 1}")
        print("="*60 + "\n")
        
        # Exact QoS matching PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Build exact topic names
        status_topic = f'{self.topic_prefix}/fmu/out/vehicle_status_v1'
        position_topic = f'{self.topic_prefix}/fmu/out/vehicle_local_position_v1'
        
        print(f"Will subscribe to:")
        print(f"  Status:   {status_topic}")
        print(f"  Position: {position_topic}")
        print()
        
        # Create subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            status_topic,
            self.status_callback,
            qos_profile
        )
        
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            position_topic,
            self.position_callback,
            qos_profile
        )
        
        print("✓ Subscriptions created\n")
        
        # Tracking variables
        self.status_count = 0
        self.position_count = 0
        self.last_status_time = None
        self.last_position_time = None
        
        # Check timer
        self.check_timer = self.create_timer(2.0, self.check_connection)
        self.start_time = time.time()
        
        print("Waiting for data...")
        print("(Press Ctrl+C to stop)\n")
    
    def status_callback(self, msg):
        self.status_count += 1
        self.last_status_time = time.time()
        
        if self.status_count == 1:
            print(f"\n{'='*60}")
            print("✓✓✓ FIRST STATUS MESSAGE RECEIVED! ✓✓✓")
            print(f"{'='*60}\n")
            print(f"Armed: {msg.arming_state == 2}")
            print(f"Nav State: {msg.nav_state}")
            print(f"System ID: {msg.system_id}")
            print(f"Component ID: {msg.component_id}")
            print()
        
        if self.status_count % 50 == 0:
            print(f"Status messages received: {self.status_count}")
    
    def position_callback(self, msg):
        self.position_count += 1
        self.last_position_time = time.time()
        
        if self.position_count == 1:
            print(f"\n{'='*60}")
            print("✓✓✓ FIRST POSITION MESSAGE RECEIVED! ✓✓✓")
            print(f"{'='*60}\n")
            print(f"Position: [{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}]")
            print(f"Velocity: [{msg.vx:.2f}, {msg.vy:.2f}, {msg.vz:.2f}]")
            print()
        
        if self.position_count % 50 == 0:
            print(f"Position messages received: {self.position_count}")
    
    def check_connection(self):
        elapsed = time.time() - self.start_time
        
        if self.status_count == 0 and self.position_count == 0:
            print(f"[{elapsed:.0f}s] ⚠ Still waiting for data...")
            
            if elapsed > 10:
                print("\n" + "="*60)
                print("ERROR: No data received after 10 seconds!")
                print("="*60)
                print("\nTroubleshooting:")
                print("1. Check PX4 is publishing:")
                print(f"   ros2 topic echo /px4_{self.px4_instance}/fmu/out/vehicle_status_v1 --once\n")
                print("2. Check topic exists:")
                print(f"   ros2 topic list | grep /px4_{self.px4_instance}/fmu/out\n")
                print("3. Check subscription count:")
                print(f"   ros2 topic info /px4_{self.px4_instance}/fmu/out/vehicle_status_v1 --verbose")
                print("   (Should show 'Subscription count: 1')\n")
                print("4. Check namespace:")
                print(f"   ros2 node list | grep diagnostic")
                print(f"   ros2 node info /diagnostic_controller\n")
                
                # Don't exit, keep trying
        elif self.status_count > 0 or self.position_count > 0:
            # Got data! Report and exit
            print(f"\n{'='*60}")
            print("SUCCESS! Connection working!")
            print(f"{'='*60}")
            print(f"Status messages:   {self.status_count}")
            print(f"Position messages: {self.position_count}")
            print(f"\nThe controller can now work. Issues were:")
            print("  - Topic namespace")
            print("  - QoS settings")
            print("  - Subscription setup")
            print(f"\n{'='*60}\n")
            
            # Keep running to show it's working
            self.check_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nStopped by user")
        print(f"Final count - Status: {node.status_count}, Position: {node.position_count}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()