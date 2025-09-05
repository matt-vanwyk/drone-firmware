#!/usr/bin/env python3
import time
import asyncio
import threading
import rclpy
from rclpy.node import Node
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from drone_interfaces.msg import DroneState

# Using friend's threading pattern for asyncio/ROS2 integration
async def spin(node: Node):
    def _spin_func():
        while rclpy.ok():
            rclpy.spin_once(node)
        node.get_logger().info("ROS 2 spin thread finished")
    
    spin_thread = threading.Thread(target=_spin_func, daemon=True)
    spin_thread.start()
    
    try:
        while rclpy.ok():
            await asyncio.sleep(1.0)
    finally:
        node.get_logger().info("Waiting for ROS 2 spin thread to finish")
        spin_thread.join()

class MAVSDKNode(Node):
    def __init__(self):
        super().__init__('basic_mavsdk_node')
        self.drone = System()  # Create MAVSDK system

        # Publisher for drone state
        self.state_publisher = self.create_publisher(DroneState, 'drone/state', 10)

        # Telemetry Variables
        self.position = None
        self.armed = False
        self.flight_mode = None
        self.battery = None
        self.in_air = False
        self.num_satellites = None
        self.landed_state = None
        self.velocity = None
        self.heading = None
        self.mission_complete = False

        self.get_logger().info('MAVSDK Node started')
        self.loop = asyncio.get_event_loop()

    # ASYNC METHODS START HERE
    ##########################################################

    async def start(self):
        """Initialize connection and run mission"""
        await self.connect_to_drone()
        await self.start_telemetry_streams()

        # Create timer to publish telemetry
        self.create_timer(0.5, self.publish_telemetry)

    async def connect_to_drone(self):
        try:
            self.get_logger().info('Attempting to connect to "udp://:14550...')
            await self.drone.connect(system_address="udp://:14550")
            
            self.get_logger().info('Waiting for drone connection...')
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.get_logger().info('Drone connected successfully!')
                    break
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')

    async def start_telemetry_streams(self):
        """Start all telemetry streams as concurrent tasks"""
        try:
            self.get_logger().info('Starting telemetry streams...')

            # Start all telemetry tasks 
            self.loop.create_task(self.update_position())
            self.loop.create_task(self.update_armed_state())
            self.loop.create_task(self.update_flight_mode())
            self.loop.create_task(self.update_battery_state())
            self.loop.create_task(self.update_in_air())
            self.loop.create_task(self.update_num_satellites())
            self.loop.create_task(self.update_landed_state())
            self.loop.create_task(self.update_velocity())
            self.loop.create_task(self.update_drone_heading())

            await asyncio.sleep(2.0)  # Allow some time for initial data to be received
            self.get_logger().info('Telemetry streams started.')

        except Exception as e:
            self.get_logger().error(f'Failed to start telemetry streams: {e}')

    # Telemetry update methods
    async def update_position(self):
        try:
            async for position in self.drone.telemetry.position():
                self.position = position
        except Exception as e:
            self.get_logger().error(f"Error in position update: {str(e)}")

    async def update_armed_state(self):
        try:
            async for armed in self.drone.telemetry.armed():
                self.armed = armed
        except Exception as e:
            self.get_logger().error(f"Error in armed state update: {str(e)}")

    async def update_flight_mode(self):
        try:
            async for flight_mode in self.drone.telemetry.flight_mode():
                self.flight_mode = flight_mode
        except Exception as e:
            self.get_logger().error(f"Error in flight mode update: {str(e)}")

    async def update_battery_state(self):
        try:
            async for battery in self.drone.telemetry.battery():
                self.battery = battery
        except Exception as e:
            self.get_logger().error(f"Error in battery state update: {str(e)}")

    async def update_in_air(self):
        try:
            async for in_air in self.drone.telemetry.in_air():
                self.in_air = in_air
        except Exception as e:
            self.get_logger().error(f"Error in in-air state update: {str(e)}")

    async def update_num_satellites(self):
        try:
            async for gps_info in self.drone.telemetry.gps_info():
                self.num_satellites = gps_info
        except Exception as e:
            self.get_logger().error(f"Error in GPS info update: {str(e)}")   
            
    async def update_landed_state(self):
        try:
            async for landed_state in self.drone.telemetry.landed_state():
                self.landed_state = landed_state
        except Exception as e:
            self.get_logger().error(f"Error in landed state update: {str(e)}")    
            
    async def update_velocity(self):
        try:
            async for velocity in self.drone.telemetry.velocity_ned():
                self.velocity = velocity
        except Exception as e:
            self.get_logger().error(f"Error in velocity update: {str(e)}")
    
    async def update_drone_heading(self):
        try:
            async for heading in self.drone.telemetry.heading():
                self.heading = heading
        except Exception as e:
            self.get_logger().error(f"Error in heading update: {str(e)}")

    ##########################################################
    # ASYNC METHODS END HERE

    def publish_telemetry(self):
        """Publish telemetry as DroneState message"""
        # Only publish if we have basic position data
        if not self.position:
            return

        try:
            state_msg = DroneState()

            # Header with timestamp
            state_msg.header.stamp = self.get_clock().now().to_msg()

            # Position Data
            state_msg.latitude = self.position.latitude_deg
            state_msg.longitude = self.position.longitude_deg
            state_msg.altitude = self.position.relative_altitude_m

            # Flight State
            state_msg.armed = self.armed
            state_msg.flight_mode = str(self.flight_mode) if self.flight_mode else "UNKNOWN"
            state_msg.is_in_air = self.in_air
            state_msg.landed_state = str(self.landed_state) if self.landed_state else "UNKNOWN"

            # GPS Info and Power
            state_msg.num_satellites = self.num_satellites.num_satellites if self.num_satellites else 0
            state_msg.battery_percentage = self.battery.remaining_percent if self.battery else 0.0

            # Motion Data
            if self.velocity:
                state_msg.velocity_x = self.velocity.north_m_s
                state_msg.velocity_y = self.velocity.east_m_s
                state_msg.velocity_z = self.velocity.down_m_s
            else:
                state_msg.velocity_x = 0.0
                state_msg.velocity_y = 0.0
                state_msg.velocity_z = 0.0

            state_msg.current_yaw = self.heading.heading_deg if self.heading else 0.0

            # Mission Status
            state_msg.mission_complete = self.mission_complete

            # Add drone ID
            if hasattr(state_msg, 'drone_id'):
                state_msg.drone_id = "drone_001"

            self.state_publisher.publish(state_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing telemetry: {str(e)}")
            return

def main():
    rclpy.init()
    node = MAVSDKNode()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(node.start())
    
    try:
        loop.run_until_complete(spin(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        loop.close()

if __name__ == '__main__':
    main()