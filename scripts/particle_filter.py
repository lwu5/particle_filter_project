#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

import random

from likelihood_field import LikelihoodField

def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(choices, weights, n):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    return random.choices(choices, weights = weights, k = n)


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # TODO comment
        self.likelihood_field = LikelihoodField()

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 100

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        rospy.sleep(1)

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):

        self.map = data
    

    def initialize_particle_cloud(self):
        
        ((x_lower, x_upper), (y_lower, y_upper)) = self.likelihood_field.get_obstacle_bounding_box()
        resol = self.map.info.resolution

        x_list = draw_random_sample(np.arange(x_lower, x_upper + resol, resol), None, self.num_particles)
        y_list = draw_random_sample(np.arange(y_lower, y_upper + resol, resol), None, self.num_particles)
        yaw_list = draw_random_sample(np.arange(0, 2 * math.pi, math.pi/180), None, self.num_particles)

        for i in range(self.num_particles):
            p = Pose()
            p.position = Point()
            p.position.x = x_list[i]
            p.position.y = y_list[i]
            p.position.z = 0
            p.orientation = Quaternion()
            q = quaternion_from_euler(0.0, 0.0, yaw_list[i])
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            # initialize the new particle, where all will have the same weight (1.0)
            new_particle = Particle(p, 1.0)

            # append the particle to the particle cloud
            self.particle_cloud.append(new_particle)

        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        w_tot = 0

        for i in range(self.num_particles):
            w_tot += self.particle_cloud[i].w
        
        for i in range(self.num_particles):
            self.particle_cloud[i].w /= w_tot


    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)



    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        
        weights = []

        for i in range(self.num_particles):
            weights.append(self.particle_cloud[i].w)

        print(weights)
        
        new_particle_indexes = draw_random_sample(np.arange(0, self.num_particles), weights, self.num_particles)
        
        new_particle_cloud = []

        for i in new_particle_indexes:
            new_particle_cloud.append(self.particle_cloud[i])

        print(new_particle_indexes)
        
        self.particle_cloud = new_particle_cloud


    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                print("updating motion ...")
                self.update_particles_with_motion_model()
                print("motion finished!")

                print("updating measurement ...")
                self.update_particle_weights_with_measurement_model(data)
                print("measurement finished!")

                print("normalizing ...")
                self.normalize_particles()
                print("normalizing finished!")

                print("resampling ...")
                self.resample_particles()
                print("resampling finished!")

                print("estimating robot pose ...")
                self.update_estimated_robot_pose()
                print("est. robot pose finished!")

                print("publishing ...")
                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()
                print("est. robot pose published!")

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        tot_x = 0
        tot_y = 0
        tot_yaw = 0

        for p in self.particle_cloud:
            tot_x += p.pose.position.x
            tot_y += p.pose.position.y
            tot_yaw += get_yaw_from_pose(p.pose)
        
        self.robot_estimate.position.x = tot_x / self.num_particles
        self.robot_estimate.position.y = tot_y / self.num_particles
        q = quaternion_from_euler(0.0, 0.0, tot_yaw / self.num_particles)
        self.robot_estimate.orientation.x = q[0]
        self.robot_estimate.orientation.y = q[1]
        self.robot_estimate.orientation.z = q[2]
        self.robot_estimate.orientation.w = q[3]
            

    def update_particle_weights_with_measurement_model(self, data):
        cardinal_direction_idxs = [0, 90, 180, 270]
        z_max = 0.0
        ranges = data.ranges

        # testing
        weights = []
        for i in range(self.num_particles):
            weights.append(self.particle_cloud[i].w)
        print(weights)
        # testing

        for i in range(self.num_particles):
            q = 1
            x = self.particle_cloud[i].pose.position.x
            y = self.particle_cloud[i].pose.position.y
            yaw = get_yaw_from_pose(self.particle_cloud[i].pose)
            
            for k in cardinal_direction_idxs:
                if ranges[k] != z_max:
                    x_trans = x + ranges[k] * math.cos(yaw + k)
                    y_trans = y + ranges[k] * math.sin(yaw + k)
                    dist = self.likelihood_field.get_closest_obstacle_distance(x_trans, y_trans)
                    q = q * compute_prob_zero_centered_gaussian(dist, 0.2)
            
            self.particle_cloud[i].w = q
        # testing
        weights_ = []
        for i in range(self.num_particles):
            weights_.append(self.particle_cloud[i].w)
        print(weights_)
        # testing

        
        
    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly
        
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        diff_x = curr_x - old_x
        diff_y = curr_y - old_y
        diff_yaw = curr_yaw - old_yaw

        # TODO need noise

        for i in range(self.num_particles):
            self.particle_cloud[i].pose.position.x += diff_x
            self.particle_cloud[i].pose.position.y += diff_y
            new_yaw = get_yaw_from_pose(self.odom_pose.pose) + diff_yaw
            q = quaternion_from_euler(0.0, 0.0, new_yaw)
            self.particle_cloud[i].pose.orientation.x = q[0]
            self.particle_cloud[i].pose.orientation.y = q[1]
            self.particle_cloud[i].pose.orientation.z = q[2]
            self.particle_cloud[i].pose.orientation.w = q[3]


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









