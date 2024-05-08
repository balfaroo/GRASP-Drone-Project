#!/usr/bin/env python3
''' implementation for a node that subscribed to the depth camera topic to drop off the target object '''
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from bryan_msgs.msg import DepthCamera, BottomCamera
import depthai as dai
import sys
import numpy as np
import cv2
from pupil_apriltags import Detector, Detection
import datetime

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.depth_tracking=True

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.subscription_depth = self.create_subscription(DepthCamera, 'depth_camera', self.depth_listener_callback, 10)


        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -0.45

        self.last_yaw_positive = True

        self.x_local = 0.0 # vars for tracking positions
        self.y_local = 0.0 

        self.target_heading = 0.0

        self.armed = False

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # hard coded an angle to stop yaw deviations
        self.REF_YAW = np.radians(277.0)
    

    def body_to_local(self, x, y):
        ''' function for converting body dx and dy commands into the PX4 local coordinate system, which is defined by
        an x-axis aligned with North, y-axis aligned with East, and a heading angle increasing from North to East'''
        hding = self.vehicle_local_position.heading
        return np.cos(hding)*x-np.sin(hding)*y, np.sin(hding)*x+np.cos(hding)*y


    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
        self.armed = True


    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)



    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float): 
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw        
        msg.yaw = np.mod(msg.yaw+np.pi, 2*np.pi)-np.pi # clipping to [-pi, pi]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        if x == 0.0 and y == 0.0:
            self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def depth_listener_callback(self, msg):
            
        if abs(self.vehicle_local_position.z-self.takeoff_height) < 0.02: # only move once at the appropriate height
            if msg.spotted:
                dx = msg.dx/1000
                dy = msg.dy/1000 # currently using the indexing method
                self.get_logger().info(f"detected front dx, dy as {[dx, dy]}")
                yaw = np.radians(msg.yaw)

                # in case the drone yaws too much and loses sight, this is to tell it to yaw back in the correct direction. unused currently since just hard coding
                # a heading
                if yaw > 0.0:
                    self.last_yaw_positive = True
                else:
                    self.last_yaw_positive = False

                if (dx>2.0 or (dx == 0.0 and dy ==0.0)): # sometimes returns dx and dy 0.0 if too far
                    dx, dy = self.body_to_local(min(0.1, abs(2.0-dx)), -np.sign(dy)*min(0.1,abs(dy))) # clipping step size so that drone moves slowly
                    self.x_local = self.vehicle_local_position.x+dx
                    self.y_local = self.vehicle_local_position.y+dy
                    self.target_heading = self.REF_YAW # remove when not hard coding
                    self.target_heading = np.mod(self.target_heading+np.pi, 2*np.pi)-np.pi

                elif dx<=2.0 and abs(dy < 0.05):
                    self.get_logger().info('DROP OBJECT') # need to run code for servo in another terminal
                elif dx <= 2.0:
                    self.x_local = self.vehicle_local_position.x # stay at this distance, only move laterally to be in front of tag
                    self.y_local = self.vehicle_local_position.y+dy 

            else:
                if self.last_yaw_positive:
                    self.target_heading = self.vehicle_local_position.heading - np.radians(5)
                else:
                    self.target_heading = self.vehicle_local_position.heading + np.radians(5)
                self.target_heading = self.REF_YAW # remove this line if not hard coding heading anymore
  

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 15: ## raised delay to 1.5 s for heading to stabilitze
            self.target_heading = self.vehicle_local_position.heading
            self.target_heading = self.REF_YAW
            self.engage_offboard_mode()
            self.get_logger().info('arming drone')
            self.arm()
            self.offboard_setpoint_counter+=1


        elif self.armed and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(self.x_local, self.y_local, self.takeoff_height, self.target_heading)
    
       
        if self.offboard_setpoint_counter < 16:
            self.offboard_setpoint_counter += 1
            self.initial_heading = self.offboard_setpoint_counter+self.initial_heading/(self.offboard_setpoint_counter+1) + self.vehicle_local_position.heading/(self.offboard_setpoint_counter+1)


def main(args=None) -> None:

    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)