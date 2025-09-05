#!/usr/bin/env python3
import asyncio
import threading
import rclpy
from rclpy.node import Node
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

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

class BasicMAVSDKNode(Node):
    def __init__(self):
        super().__init__('basic_mavsdk_node')
        self.drone = System()  # Create MAVSDK system
        self.get_logger().info('Basic MAVSDK Node started')
        self.loop = asyncio.get_event_loop()

    async def start(self):
        """Initialize connection and run mission"""
        await self.connect_to_drone()
        await self.test_basic_telemetry()
        await self.run_waypoint_mission()

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

    async def test_basic_telemetry(self):
        """Test basic telemetry retrieval"""
        try:
            # Get one sample of each telemetry type
            async for health in self.drone.telemetry.health():
                self.get_logger().info(f'Health: {health}')
                break
            
            async for battery in self.drone.telemetry.battery():
                self.get_logger().info(f'Battery: {battery.voltage_v:.1f}V, {battery.remaining_percent:.0f}%')
                break
            
            async for flight_mode in self.drone.telemetry.flight_mode():
                self.get_logger().info(f'Flight mode: {flight_mode}')
                break
            
            async for gps_info in self.drone.telemetry.gps_info():
                self.get_logger().info(f'GPS: {gps_info.num_satellites} satellites, fix: {gps_info.fix_type}')
                break
            
            async for position in self.drone.telemetry.position():
                self.get_logger().info(f'Position: lat={position.latitude_deg:.6f}, lon={position.longitude_deg:.6f}, alt={position.relative_altitude_m:.1f}m')
                break
            
            self.get_logger().info('Basic telemetry test completed successfully!')
            
        except Exception as e:
            self.get_logger().error(f'Telemetry test failed: {e}')

    async def run_waypoint_mission(self):
        """Create and execute a waypoint mission"""
        try:
            self.get_logger().info('Starting waypoint mission...')
            
            # Wait for global position estimate
            self.get_logger().info('Waiting for global position estimate...')
            async for health in self.drone.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    self.get_logger().info('Global position OK')
                    break
            
            # Get current position to create waypoints relative to home
            home_position = None
            async for position in self.drone.telemetry.position():
                home_position = position
                self.get_logger().info(f'Home position: {position.latitude_deg:.6f}, {position.longitude_deg:.6f}')
                break
            
            if not home_position:
                self.get_logger().error('Could not get home position')
                return
            
            # Create mission items (waypoints around home position)
            mission_items = []
            
            # Takeoff to 20m
            mission_items.append(MissionItem(
                home_position.latitude_deg,
                home_position.longitude_deg,
                20,  # altitude in meters
                10,  # speed in m/s
                True,  # is_fly_through
                float('nan'),  # gimbal_pitch_deg
                float('nan'),  # gimbal_yaw_deg
                MissionItem.CameraAction.NONE,
                float('nan'),  # loiter_time_s
                float('nan'),  # camera_photo_interval_s
                float('nan'),  # acceptance_radius_m
                float('nan'),  # yaw_deg
                float('nan'),  # camera_photo_distance_m
                MissionItem.VehicleAction.NONE  # vehicle_action
            ))
            
            # Waypoint 1: North 100m
            mission_items.append(MissionItem(
                home_position.latitude_deg + 0.0009,  # ~100m north
                home_position.longitude_deg,
                20,
                10,
                True,
                float('nan'),
                float('nan'),
                MissionItem.CameraAction.NONE,
                float('nan'),
                float('nan'),
                float('nan'),
                float('nan'),
                float('nan'),
                MissionItem.VehicleAction.NONE
            ))
            
            # Waypoint 2: East 100m
            mission_items.append(MissionItem(
                home_position.latitude_deg + 0.0009,
                home_position.longitude_deg + 0.0012,  # ~100m east
                20,
                10,
                True,
                float('nan'),
                float('nan'),
                MissionItem.CameraAction.NONE,
                float('nan'),
                float('nan'),
                float('nan'),
                float('nan'),
                float('nan'),
                MissionItem.VehicleAction.NONE
            ))
            
            # Waypoint 3: South (back toward home)
            mission_items.append(MissionItem(
                home_position.latitude_deg,
                home_position.longitude_deg + 0.0012,
                20,
                10,
                True,
                float('nan'),
                float('nan'),
                MissionItem.CameraAction.NONE,
                float('nan'),
                float('nan'),
                float('nan'),
                float('nan'),
                float('nan'),
                MissionItem.VehicleAction.NONE
            ))
            
            # Waypoint 4: Return to home
            mission_items.append(MissionItem(
                home_position.latitude_deg,
                home_position.longitude_deg,
                20,
                10,
                True,
                float('nan'),
                float('nan'),
                MissionItem.CameraAction.NONE,
                5.0,  # loiter for 5 seconds
                float('nan'),
                float('nan'),
                float('nan'),
                float('nan'),
                MissionItem.VehicleAction.NONE
            ))
            
            # Create mission plan
            mission_plan = MissionPlan(mission_items)
            
            # Upload mission
            self.get_logger().info('Uploading mission...')
            await self.drone.mission.upload_mission(mission_plan)
            self.get_logger().info('Mission uploaded successfully!')
            
            # Arm the drone
            self.get_logger().info('Arming...')
            await self.drone.action.arm()
            self.get_logger().info('Armed!')
            
            # Start mission
            self.get_logger().info('Starting mission...')
            await self.drone.mission.start_mission()
            self.get_logger().info('Mission started!')
            
            # Monitor mission progress
            await self.monitor_mission()
            
        except Exception as e:
            self.get_logger().error(f'Mission failed: {e}')

    async def monitor_mission(self):
        """Monitor mission progress and provide updates"""
        try:
            self.get_logger().info('Monitoring mission progress...')
            
            # Monitor mission progress
            async for mission_progress in self.drone.mission.mission_progress():
                self.get_logger().info(f'Mission progress: {mission_progress.current}/{mission_progress.total}')
                
                # Also log current position
                async for position in self.drone.telemetry.position():
                    self.get_logger().info(f'Current position: lat={position.latitude_deg:.6f}, lon={position.longitude_deg:.6f}, alt={position.relative_altitude_m:.1f}m')
                    break
                
                # Check if mission is complete
                if mission_progress.current == mission_progress.total:
                    self.get_logger().info('Mission completed!')
                    break
            
            # Land the drone
            self.get_logger().info('Landing...')
            await self.drone.action.land()
            
            # Wait for landing
            async for in_air in self.drone.telemetry.in_air():
                if not in_air:
                    self.get_logger().info('Landed successfully!')
                    break
            
            # Disarm
            await self.drone.action.disarm()
            self.get_logger().info('Disarmed!')
            
        except Exception as e:
            self.get_logger().error(f'Mission monitoring failed: {e}')

def main():
    rclpy.init()
    node = BasicMAVSDKNode()
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