#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
import time

class DamagedObjectTrackerClient(Node):
    def __init__(self):
        super().__init__('damaged_object_tracker_client')
        
        # Create service clients
        self.start_stop_client = self.create_client(
            SetBool, 
            '/damaged_object_tracker_server/start_inspection'
        )
        
        self.status_client = self.create_client(
            Trigger, 
            '/damaged_object_tracker_server/get_status'
        )
        
        # Wait for services to be available
        self.get_logger().info('Waiting for services...')
        self.start_stop_client.wait_for_service()
        self.status_client.wait_for_service()
        self.get_logger().info('Services are ready!')
    
    def start_inspection(self):
        """Start the inspection process"""
        request = SetBool.Request()
        request.data = True
        
        future = self.start_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Start inspection: {response.success} - {response.message}')
            return response.success
        else:
            self.get_logger().error('Failed to call start_inspection service')
            return False
    
    def stop_inspection(self):
        """Stop the inspection process"""
        request = SetBool.Request()
        request.data = False
        
        future = self.start_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Stop inspection: {response.success} - {response.message}')
            return response.success
        else:
            self.get_logger().error('Failed to call stop_inspection service')
            return False
    
    def get_status(self):
        """Get the current inspection status"""
        request = Trigger.Request()
        
        future = self.status_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Status: {response.message}')
            return response.success, response.message
        else:
            self.get_logger().error('Failed to call get_status service')
            return False, "Service call failed"
    
    def run_inspection_demo(self, duration=10):
        """Run a demonstration of the inspection process"""
        self.get_logger().info('=== Damaged Object Tracker Demo ===')
        
        # Check initial status
        self.get_logger().info('Checking initial status...')
        self.get_status()
        
        # Start inspection
        self.get_logger().info('Starting inspection...')
        if self.start_inspection():
            self.get_logger().info(f'Inspection will run for {duration} seconds')
            
            # Monitor status during inspection
            for i in range(duration):
                time.sleep(1)
                if i % 5 == 0:  # Check status every 5 seconds
                    self.get_status()
            
            # Stop inspection
            self.get_logger().info('Stopping inspection...')
            self.stop_inspection()
            
            # Check final status
            self.get_logger().info('Checking final status...')
            self.get_status()
        
        self.get_logger().info('=== Demo Complete ===')


def main(args=None):
    rclpy.init(args=args)
    
    # Check if demo mode is requested
    import sys
    demo_mode = len(sys.argv) > 1 and sys.argv[1] == 'demo'
    
    client = DamagedObjectTrackerClient()
    
    if demo_mode:
        # Run automatic demo
        duration = 30  # seconds
        if len(sys.argv) > 2:
            try:
                duration = int(sys.argv[2])
            except ValueError:
                client.get_logger().warn('Invalid duration, using default 30 seconds')
        
        client.run_inspection_demo(duration)
    else:
        # Interactive mode
        client.get_logger().info('Interactive mode - use the following commands:')
        client.get_logger().info('  ros2 service call /damaged_object_tracker_server/start_inspection std_srvs/srv/SetBool "{data: true}"')
        client.get_logger().info('  ros2 service call /damaged_object_tracker_server/get_status std_srvs/srv/Trigger "{}"')
        client.get_logger().info('  ros2 service call /damaged_object_tracker_server/start_inspection std_srvs/srv/SetBool "{data: false}"')
        client.get_logger().info('Or run: ros2 run camera_inspection damaged_object_tracker_client demo [duration_seconds]')
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()