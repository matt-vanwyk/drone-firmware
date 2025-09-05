#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from statemachine import State, StateMachine
from drone_interfaces.msg import DroneState

class DroneStateMachine(StateMachine):
    """State machine definitions"""

    # Define States
    initializing = State(initial=True)
    idle = State()
    charging = State()
    ready_to_fly = State()
    armed_on_ground = State()
    in_flight = State()
    hovering = State()
    mission_active = State()
    returning_home= State()

    # Define Transitions
    connect = initializing.to(idle)
    start_charging = idle.to(charging) | ready_to_fly.to(charging)
    finish_charging = charging.to(ready_to_fly)
    prepare_flight = idle.to(ready_to_fly)
    arm_drone = ready_to_fly.to(armed_on_ground)
    take_off = armed_on_ground.to(in_flight)
    start_hover = in_flight.to(hovering)
    start_mission = hovering.to(mission_active)
    return_home = mission_active.to(returning_home) | hovering.to(returning_home)
    land = in_flight.to(armed_on_ground) | hovering.to(armed_on_ground) | returning_home.to(armed_on_ground)
    disarm = armed_on_ground.to(idle)

class DroneStateMachineNode(Node):
    def __init__(self):
        super().__init__('drone_state_machine_node')

        # Create state machine instance
        self.state_machine = DroneStateMachine(model=self)

        # Create subscription to drone/state topic (drone telemetry)
        self.state_subscription = self.create_subscription(
            DroneState,
            'drone/state',
            self.telemetry_callback,
            10
        )

        self.drone_state = None
        self.telemetry_received = False

        # State transition thresholds
        self.BATTERY_LOW_THRESHOLD = 20.0  # Percent
        self.BATTERY_FULL_THRESHOLD = 90.0  # Percent

        self.get_logger().info('State Machine started - waiting for telemetry...')

    def telemetry_callback(self, msg):
        """Process incoming telemetry and update state machine"""

        # Store latest telemetry
        self.drone_state = msg

        # Handle first telemetry reception
        if not self.telemetry_received:
            self.telemetry_received = True
            self.get_logger().info('First telemetry received!')
            self.state_machine.connect()  # Transition from initializing to idle
            self.log_current_state("Connected to telemetry")

        self.update_state_machine()
    
    def update_state_machine(self):
        """Update state machine based on current telemetry"""
        current_state = self.state_machine.current_state.id

        # State transition logic based on telemetry
        if current_state == 'idle':
            if self.drone_state.battery_percentage < self.BATTERY_LOW_THRESHOLD:
                self.state_machine.start_charging()
                self.log_current_state(f"Battery low: {self.drone_state.battery_percentage:.1f}%")
            elif self.drone_state.battery_percentage >= self.BATTERY_FULL_THRESHOLD:
                self.state_machine.prepare_flight()
                self.log_current_state(f"Battery ready: {self.drone_state.battery_percentage:.1f}%")

        elif current_state == 'charging':
            if self.drone_state.battery_percentage >= self.BATTERY_FULL_THRESHOLD:
                self.state_machine.finish_charging()
                self.log_current_state(f"Charging complete: {self.drone_state.battery_percentage:.1f}%")

        elif current_state == 'ready_to_fly':
            if self.drone_state.battery_percentage < self.BATTERY_LOW_THRESHOLD:
                self.state_machine.start_charging()
                self.log_current_state(f"Battery low: {self.drone_state.battery_percentage:.1f}%")
            elif self.drone_state.armed and not self.drone_state.is_in_air:
                self.state_machine.arm_drone()
                self.log_current_state("Drone armed on ground")

        elif current_state == 'armed_on_ground':
            if not self.drone_state.armed:
                self.state_machine.disarm()
                self.log_current_state("Drone disarmed")
            elif self.drone_state.is_in_air:
                self.state_machine.take_off()
                self.log_current_state("Drone took off")

        elif current_state == "in_flight":
            if not self.drone_state.is_in_air:
                self.state_machine.land()
                self.log_current_state("Drone landed")
            elif self.drone_state.flight_mode == "HOLD":
                self.state_machine.start_hover()
                self.log_current_state("Entered hover mode")
        
        elif current_state == "hovering":
            if not self.drone_state.is_in_air:
                self.state_machine.land()
                self.log_current_state("Drone landed from hover")
            elif self.drone_state.flight_mode == "MISSION":
                self.state_machine.start_mission()
                self.log_current_state("Mission started")
            elif self.drone_state.flight_mode == "RETURN":
                self.state_machine.return_home()
                self.log_current_state("Returning to launch")
        
        elif current_state == "mission_active":
            if not self.drone_state.is_in_air:
                self.state_machine.land()
                self.log_current_state("Drone landed during mission")
            elif self.drone_state.flight_mode == "RETURN":
                self.state_machine.return_home()
                self.log_current_state("Mission aborted - returning home")
        
        elif current_state == "returning_home":
            if not self.drone_state.is_in_air:
                self.state_machine.land()
                self.log_current_state("Completed return to launch")

    def log_current_state(self, reason):
        """Log current state with context"""
        state = self.state_machine.current_state.id
        telemetry_summary = ""
        
        if self.drone_state:
            telemetry_summary = (f"armed={self.drone_state.armed}, "
                               f"in_air={self.drone_state.is_in_air}, "
                               f"battery={self.drone_state.battery_percentage:.1f}%, "
                               f"mode={self.drone_state.flight_mode}")
        
        self.get_logger().info(f'STATE: {state.upper()} | {reason}')
        self.get_logger().info(f'  Telemetry: {telemetry_summary}')

def main():
    rclpy.init()
    
    # Create and run the state machine
    node = DroneStateMachineNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down state machine...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



