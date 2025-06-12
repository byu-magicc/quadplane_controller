import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorServos, VehicleCommand, TrajectorySetpoint
import numpy as np

#creates the period for the quick timer
quickTimerPeriod = 0.1
slowTimerPeriod = 1.0

class ServoCommander(Node):
    def __init__(self):
        super().__init__('servo_commander')

        # Publisher for servo messages
        self.servo_pub = self.create_publisher(
            msg_type=ActuatorServos,
            topic='/fmu/in/actuator_servos',
            qos_profile=10
        )

        # Publisher for vehicle commands (mode, arm/disarm)
        self.command_pub = self.create_publisher(
            msg_type=VehicleCommand,
            topic='/fmu/in/vehicle_command',
            qos_profile=10
        )

        #publisher for the setpoint
        self.setpoint_pub = self.create_publisher(
            msg_type=TrajectorySetpoint,
            topic='/fmu/in/trajectory_setpoint',
            qos_profile=10
        )

        # Timer to publish servo signals regularly
        self.servo_timer = self.create_timer(quickTimerPeriod, self.servo_callback)

        #Timer to publish the trajectory setpoints
        self.setpoint_timer = self.create_timer(quickTimerPeriod, self.setpoint_callback)
        # Timer to send mode and arm command once after boot
        self.initialization_timer = self.create_timer(slowTimerPeriod, self.init_vehicle_commands)
        self.init_sent = False

    def servo_callback(self):
        msg = ActuatorServos()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        t_seconds = self.get_clock().now().nanoseconds / 1e9
        outputControl = 0.5 + 0.2*np.cos(t_seconds)
        msg.control = [outputControl] * 8  # Example oscillating servo
        self.servo_pub.publish(msg)
        self.get_logger().info(f'Servo Command: {outputControl}')

    #creates the setpoint callback
    def setpoint_callback(self):
        #creates the message
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [0.0, 0.0, 0.0]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.yaw = 0.0
        self.setpoint_pub.publish(msg)
 

    def init_vehicle_commands(self):
        if self.init_sent:
            return  # Only run once
        self.get_logger().info('Sending OFFBOARD mode and ARM commands')

        # Set mode to OFFBOARD (MAV_CMD_DO_SET_MODE = 176, param1 = 1=custom, param2 = 6=OFFBOARD)
        self.send_vehicle_command(command=176, param1=1.0, param2=6.0)

        # Arm the vehicle (MAV_CMD_COMPONENT_ARM_DISARM = 400, param1 = 1.0 = arm)
        self.send_vehicle_command(command=400, param1=1.0)

        self.init_sent = True  # prevent running again
        self.initialization_timer.cancel()

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent VehicleCommand: {command} (param1={param1}, param2={param2})')

def main():
    rclpy.init()
    node = ServoCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

