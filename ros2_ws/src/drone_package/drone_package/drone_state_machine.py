#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from statemachine import State, StateMachine
from shared_interfaces.msg import DroneState
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

        self.received_first_telemetry_flag = False

        self.get_logger().info('Drone State Machine started - waiting for telemetry...')


    def telemetry_callback(self, msg):
        """Process incoming telemetry and update state machine"""

        # Store latest telemetry
        self.drone_state = msg

        if self.received_first_telemetry_flag is False:
            self.get_logger().info('Received first telemetry message! Publishing drone state...')
            self.received_first_telemetry_flag = True

        # State machine transitions based on telemetry
        # self.get_logger().info(f"Received telemetry: Latitude {msg.latitude} | Longitude {msg.longitude} | Altitude {msg.altitude} | Armed {msg.armed} | Mode {msg.flight_mode} | Landed State {msg.landed_state} | Battery {msg.battery_percentage}% | In Air: {msg.is_in_air} | ")

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