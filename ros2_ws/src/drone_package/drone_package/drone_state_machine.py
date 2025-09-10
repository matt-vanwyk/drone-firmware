#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from statemachine import State, StateMachine
from shared_interfaces.msg import DroneState
from shared_interfaces.srv import PrepareForMission, UploadMission
from drone_interfaces.msg import Telemetry

class DroneStateMachine(StateMachine):
    """State machine definitions"""

    # Define States
    idle = State(initial=True)
    ready_to_fly = State()
    mission_received = State()
    mission_in_progress = State()
    landed = State()

    # Define Transitions
    prepare_for_flight = idle.to(ready_to_fly)
    received_mission = ready_to_fly.to(mission_received)
    start_mission = mission_received.to(mission_in_progress)
    mission_complete = mission_in_progress.to(landed)
    power_off = landed.to(idle)

class DroneStateMachineNode(Node):
    def __init__(self):
        super().__init__('drone_state_machine_node')
        self.state_machine = DroneStateMachine(model=self)
        
        # Subscriber and Publisher Declarations
        self.telemetry_subscription = self.create_subscription(
            Telemetry,
            'drone/telemetry',
            self.telemetry_callback,
            10
        )
        self.drone_state_publisher = self.create_publisher(DroneState, 'drone/state', 10)
        self.create_timer(1.0, self.publish_drone_state)

        # Service Server Declarations (base_station_state_machine communication)
        self.prepare_mission_service = self.create_service(
            PrepareForMission,
            'drone/prepare_for_mission',
            self.handle_prepare_for_mission
        )
        self.upload_mission_service = self.create_service(
            UploadMission,
            'drone/upload_mission',
            self.handle_upload_mission
        )

        # Initialize DroneState attributes
        self.drone_state = DroneState()
        self.drone_state.latitude = -500.0
        self.drone_state.longitude = -500.0
        self.drone_state.altitude = -500.0
        self.drone_state.armed = False
        self.drone_state.flight_mode = ''
        self.drone_state.is_in_air = False
        self.drone_state.battery_percentage = -500.0
        self.drone_state.num_satellites = -500
        self.drone_state.landed_state = ''
        self.drone_state.velocity_x = -500.0
        self.drone_state.velocity_y = -500.0
        self.drone_state.velocity_z = -500.0
        self.drone_state.current_yaw = -500.0
        self.drone_state.mission_complete = False

        # Mission Variables
        self.current_mission_id = ""
        self.current_waypoints = []

        # THRESHOLDS TO BE PLACED IN CONFIG FILE LATER?
        self.BATTERY_CHARGED_THRESHOLD = 90.0
        self.BATTERY_DEAD_THRESHOLD = 20.0
        self.SATELLITES_LOCKED_THRESHOLD = 10

        # Telemetry validation flags
        self.received_first_telemetry_flag = False
        self.telemetry_valid = False

        # Dictionary to track if all topics are ready
        self.telemetry_ready = {
            'latitude': False,
            'longitude': False,
            'altitude': False,
            'battery_percentage': False,
            'num_satellites': False,
            'flight_mode': False,
            'landed_state': False
        }

        self.get_logger().info('Drone State Machine started - waiting for telemetry...')

    def telemetry_callback(self, msg):
        """Process incoming telemetry and update state machine"""
        # Store latest telemetry
        self.drone_state = msg

        if self.received_first_telemetry_flag is False:
            self.get_logger().info('Received first telemetry message! Publishing drone state...')
            self.received_first_telemetry_flag = True

        self.update_state_machine()

    def publish_drone_state(self):
        """Publish the current drone state"""
        msg = DroneState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.latitude = self.drone_state.latitude
        msg.longitude = self.drone_state.longitude
        msg.altitude = self.drone_state.altitude
        msg.armed = self.drone_state.armed
        msg.flight_mode = self.drone_state.flight_mode
        msg.is_in_air = self.drone_state.is_in_air
        msg.battery_percentage = self.drone_state.battery_percentage
        msg.num_satellites = self.drone_state.num_satellites
        msg.landed_state = self.drone_state.landed_state
        msg.velocity_x = self.drone_state.velocity_x
        msg.velocity_y = self.drone_state.velocity_y
        msg.velocity_z = self.drone_state.velocity_z
        msg.current_yaw = self.drone_state.current_yaw
        msg.mission_complete = self.drone_state.mission_complete
        msg.current_state = str(self.state_machine.current_state) if self.state_machine.current_state else "UNKNOWN"

        self.drone_state_publisher.publish(msg)
    
    def check_telemetry_validity(self):
        """Check if received telemetry data is valid"""
        if not self.received_first_telemetry_flag:
            return False
            
        # Check each critical field
        if self.drone_state.latitude != -500.0 and not self.telemetry_ready['latitude']:
            self.telemetry_ready['latitude'] = True

        if self.drone_state.longitude != -500.0 and not self.telemetry_ready['longitude']:
            self.telemetry_ready['longitude'] = True
    
        if self.drone_state.altitude != -500.0 and not self.telemetry_ready['altitude']:
            self.telemetry_ready['altitude'] = True

        if self.drone_state.battery_percentage != -500.0 and not self.telemetry_ready['battery_percentage']:
            self.telemetry_ready['battery_percentage'] = True

        if self.drone_state.num_satellites != -500 and not self.telemetry_ready['num_satellites']:
            self.telemetry_ready['num_satellites'] = True

        if self.drone_state.flight_mode != '' and not self.telemetry_ready['flight_mode']:
            self.telemetry_ready['flight_mode'] = True

        if self.drone_state.landed_state != '' and not self.telemetry_ready['landed_state']:
            self.telemetry_ready['landed_state'] = True

        # Check if all critical telemetry is ready
        critical_fields = ['latitude', 'longitude', 'altitude', 'battery_percentage', 'num_satellites']
        all_critical_ready = all(self.telemetry_ready[field] for field in critical_fields)
        
        if all_critical_ready and not self.telemetry_valid:
            self.telemetry_valid = True
            self.get_logger().info('All critical telemetry data is now valid')
            
        return self.telemetry_valid

    def update_state_machine(self):
        """Handle all state transitions and requests"""
        current_state = self.state_machine.current_state.id

        self.check_telemetry_validity()

        # Log state changes
        if current_state != getattr(self, 'prev_state', None):
            self.get_logger().info(f'Drone state: {current_state}')
            self.prev_state = current_state

###################################
# HANDLERS FOR SERVICE REQUESTS FROM BASE STATION STATE MACHINE
###################################

    def handle_prepare_for_mission(self, request, response):
        """Handle prepare for mission service request with all validation inline"""
        self.get_logger().info(f'Mission preparation request: {request.mission_id}')

        current_state = self.state_machine.current_state.id

        # Fill in current status
        response.battery_percentage = self.drone_state.battery_percentage
        response.num_satellites = self.drone_state.num_satellites
        response.drone_state = current_state

        # Check if all conditions are met for preparation
        if not self.telemetry_valid:
            response.success = False
            response.error_message = "Telemetry not valid yet"
        elif current_state != 'idle':
            response.success = False
            response.error_message = f"Not in idle state! Current state: {current_state}"
        elif self.drone_state.battery_percentage < self.BATTERY_CHARGED_THRESHOLD:
            response.success = False
            response.error_message = f"Battery too low: {self.drone_state.battery_percentage}%"
        elif self.drone_state.num_satellites < self.SATELLITES_LOCKED_THRESHOLD:
            response.success = False
            response.error_message = f"Not enough satellites: {self.drone_state.num_satellites}"
        else:
            # All checks passed - make the transition
            try:
                self.state_machine.prepare_for_flight()
                response.success = True
                response.error_message = ""
                response.drone_state = str(self.state_machine.current_state.id)
                self.get_logger().info(f'Drone prepared for mission: {request.mission_id}')
            except Exception as e:
                response.success = False
                response.error_message = f"Transition failed: {str(e)}"

        return response

    def handle_upload_mission(self, request, response):
        """Handle upload mission service request with all validation inline"""
        self.get_logger().info(f'Mission upload request: {request.mission_id}')

        current_state = self.state_machine.current_state.id

        # Fill in current status
        response.drone_state = current_state
        response.num_waypoints_received = len(request.waypoints)

        # Check if in correct state to fly
        if current_state != 'ready_to_fly':
            response.success = False
            response.error_message = f"Not in ready_to_fly state! Current state: {current_state}"
        else:
            # Store mission details
            try:
                self.current_mission_id = request.mission_id
                self.current_waypoints = request.waypoints
                self.state_machine.received_mission()
                # TODO After this, we need to try and uplaod the mission to the drone
                # via the mavsdk_node. To do this we will make a service request using
                # drone/interfaces/srv/UploadMissionMAVSDK.srv (not implemented yet).
                # Only when we receive a successful response from the mavsdk_node server
                # will we respond to the base station state machine to say that the mission
                # upload was successful.
                response.success = True
                response.error_message = ""
                response.drone_state = str(self.state_machine.current_state.id)
                self.get_logger().info(f'Mission uploaded: {len(request.waypoints)} waypoints')
            except Exception as e:
                response.success = False
                response.error_message = f"Mission upload failed: {str(e)}"

        return response

###################################
# HANDLERS FOR SERVICE REQUESTS FROM BASE STATION STATE MACHINE
###################################

def main():
    rclpy.init()
    
    # Create and run the state machine
    node = DroneStateMachineNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Drone State Machine...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()