import rclpy
from rclpy.node import Node
#imports the VehicheCommand, which we publish
#and we import the VehicleCommandAck, which we subscribe to
from px4_msgs.msg import VehicleCommand, VehicleCommandAck
from time import time

#defines the periods for the timer
arm_timer_period = 1.0



#creates the PX4 commanded state node
class ServoCommander(Node):

    #inits
    def __init__(self):

        super().__init__('servo_commander')

        #creates the publisher to send the vehicle command
        self.command_pub = self.create_publisher(
            msg_type=VehicleCommand,
            topic='fmu/in/vehicle_command',
            qos_profile=10
        )

        #creates the subscriber to read in the acknowledge
        #bit in from the vehicle
        self.ack_subscription = self.create_subscription(
            msg_type=VehicleCommandAck,
            topic='fmu/in/vehicle_command_ack',
            callback=self.ack_callback,
            qos_profile=10
        )

        #creates the timer to send the arm commanded
        self.arm_timer = self.create_timer(timer_period_sec=arm_timer_period,
                                           callback=self.send_arm_command)
        #variable for whether the command has been sent
        self.command_sent = False


    #creates the send arm command to be sent
    def send_arm_command(self):

        #checks if the command has been sent
        if self.command_sent:
            #then we just return without doing anything
            return

        #creates the vehicle command class instance
        cmd = VehicleCommand()
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        cmd.timestamp = timestamp
        #sends the arm command
        # 1 = ARM, 0 = DISARM
        cmd.param1 = 1.0
        #the command from the VehicleCommand plethora used to arm or disarm the vehicle
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        #sets the target system
        cmd.target_system = 1
        #sets the target component
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1

        cmd.from_external = True
        #publishes the command just defined using command_pub defined above 
        self.command_pub.publish(cmd)
        self.get_logger().info("ARM command sent.")
        #sets the command sent variable to true, so we don't keep sending it over and over
        self.command_sent = True

    #creates the function to acknowledge a callback
    def ack_callback(self, msg):
        #reates the result map
        result_map = {
            0: 'ACCEPTED',
            1: 'TEMPORARILY_REJECTED',
            2: 'DENIED',
            3: 'UNSUPPORTED',
            4: 'FAILED',
            5: 'IN_PROGRESS',
        }

        #CREATES THE RESULT STRING
        result_str = result_map.get(msg.result, f'UNKNOWN({msg.result})')

        #function to get the logger
        self.get_logger().info(
            f"Reeived ACK: command={msg.command}, result={result_str}"
        )


#defines the main function to run everything
def main(args=None):
    rclpy.init(args=args)
    node = ServoCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
This is a list of all of the ros2 topics, at least so
far for this project.
/fmu/in/actuator_motors
/fmu/in/actuator_servos
/fmu/in/arming_check_reply
/fmu/in/aux_global_position
/fmu/in/config_control_setpoints
/fmu/in/config_overrides_request
/fmu/in/differential_drive_setpoint
/fmu/in/goto_setpoint
/fmu/in/manual_control_input
/fmu/in/message_format_request
/fmu/in/mode_completed
/fmu/in/obstacle_distance
/fmu/in/offboard_control_mode
/fmu/in/onboard_computer_status
/fmu/in/register_ext_component_request
/fmu/in/sensor_optical_flow
/fmu/in/telemetry_status
/fmu/in/trajectory_setpoint
/fmu/in/unregister_ext_component
/fmu/in/vehicle_attitude_setpoint
/fmu/in/vehicle_command
/fmu/in/vehicle_command_mode_executor
/fmu/in/vehicle_mocap_odometry
/fmu/in/vehicle_rates_setpoint
/fmu/in/vehicle_thrust_setpoint
/fmu/in/vehicle_torque_setpoint
/fmu/in/vehicle_trajectory_bezier
/fmu/in/vehicle_trajectory_waypoint
/fmu/in/vehicle_visual_odometry
/fmu/out/battery_status
/fmu/out/estimator_status_flags
/fmu/out/failsafe_flags
/fmu/out/manual_control_setpoint
/fmu/out/position_setpoint_triplet
/fmu/out/sensor_combined
/fmu/out/timesync_status
/fmu/out/vehicle_attitude
/fmu/out/vehicle_command_ack
/fmu/out/vehicle_control_mode
/fmu/out/vehicle_local_position
/fmu/out/vehicle_odometry
/fmu/out/vehicle_status
#'''
