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
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw """

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(choices, weights, n):
    """ Draws a random sample of n elements from a given list of choices and their specified 
            probabilities / weights. """

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

        # import LikelihoodField class
        self.likelihood_field = LikelihoodField()

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 5000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        # initialize pose of last motion
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

        # give map enough time to be initialized
        rospy.sleep(1)

        # intialize the particle cloud
        self.initialize_particle_cloud()
        self.initialized = True
        print("particle cloud initialized!")


    def get_map(self, data):

        self.map = data
    

    def initialize_particle_cloud(self):
        
        # get the map boundary data and its resolution
        ((x_lower, x_upper), (y_lower, y_upper)) = self.likelihood_field.get_obstacle_bounding_box()
        resol = self.map.info.resolution

        # generate particles' x, y, and yaw values randomly within the map
        # since all particles have equal weights during initialization, we set list of prob to None. 
        x_list = draw_random_sample(np.arange(x_lower, x_upper + resol, resol), None, self.num_particles)
        y_list = draw_random_sample(np.arange(y_lower, y_upper + resol, resol), None, self.num_particles)
        yaw_list = draw_random_sample(np.arange(0, 2 * math.pi, math.pi/180), None, self.num_particles)

        # create particles based on the randomly-generated x, y, and yaw value
        for i in range(self.num_particles):
            p = Pose()
            p.position = Point()
            p.position.x = x_list[i]
            p.position.y = y_list[i]
            p.position.z = 0
            p.orientation = Quaternion()
            q = quaternion_from_euler(0.0, 0.0, yaw_list[i]) # convert yaw to quaternion
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
        # generate a new particle cloud to replace the old one;
        # particles in the new particle cloud are generated randomly with weighted probabilities
        
        # get a probability list
        weights = []
        for i in range(self.num_particles):
            weights.append(self.particle_cloud[i].w)
        
        new_particle_indexes = draw_random_sample(np.arange(0, self.num_particles), weights, self.num_particles)
        
        new_particle_cloud = []

        for i in new_particle_indexes:
            # map index back to the particle in the old cloud
            resampled_particle = self.particle_cloud[i]    

            # initialize new particle
            new_particle = Pose()
            new_particle.position = Point()
            new_particle.orientation = Quaternion()
            
            # giving new particle values based on the resampled particle
            new_particle.position.x = resampled_particle.pose.position.x
            new_particle.position.y = resampled_particle.pose.position.y
            new_particle.position.z = 0.0
            new_particle.orientation.x = resampled_particle.pose.orientation.x
            new_particle.orientation.y = resampled_particle.pose.orientation.y
            new_particle.orientation.z = resampled_particle.pose.orientation.z
            new_particle.orientation.w = resampled_particle.pose.orientation.w

            # add new particle to the new cloud
            new_particle_cloud.append(
                Particle(new_particle, resampled_particle.w)
            )
        
        # replace old cloud by new cloud
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

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose


    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate

        tot_x = 0
        tot_y = 0
        tot_yaw = 0
        tot_w = 0

        # get the weighted total x, y, and yaw values
        for p in self.particle_cloud:
            tot_x += p.pose.position.x * p.w
            tot_y += p.pose.position.y * p.w
            tot_yaw += get_yaw_from_pose(p.pose) * p.w
            tot_w += p.w
        
        # calculate and assign the weighted average as robot's estimated pose
        self.robot_estimate.position.x = tot_x / tot_w
        self.robot_estimate.position.y = tot_y / tot_w
        q = quaternion_from_euler(0.0, 0.0, tot_yaw / tot_w)
        self.robot_estimate.orientation.x = q[0]
        self.robot_estimate.orientation.y = q[1]
        self.robot_estimate.orientation.z = q[2]
        self.robot_estimate.orientation.w = q[3]
            

    def update_particle_weights_with_measurement_model(self, data):
        # for each particle, based on the likelihood field, we'll update its weight

        # cardinal directions of particles we take into account
        #direction_idxs = [0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 260, 280, 300, 320, 340, 359]
        direction_idxs = [0, 45, 90, 135, 180, 225, 270, 315]
        #direction_idxs = [0,30,60,90,120,150,180,210,240,270,300,330]
        #direction_idxs = [0,90,180,270]
        z_max = 0.0
        ranges = data.ranges

        for i in range(self.num_particles):
            q = 1 # initialize probability
            x = self.particle_cloud[i].pose.position.x
            y = self.particle_cloud[i].pose.position.y
            yaw = get_yaw_from_pose(self.particle_cloud[i].pose)
            
            for k in direction_idxs:
                if ranges[k] != z_max: # if laser finds obstable 
                    x_trans = x + ranges[k] * math.cos(yaw + k)
                    y_trans = y + ranges[k] * math.sin(yaw + k)
                    dist = self.likelihood_field.get_closest_obstacle_distance(x_trans, y_trans)
                    if dist == np.float('nan'): # if particle is outside map boundaries
                        q = q * 0.001 # makes the particle's weight small
                    else: # particle is in the map
                        q = q * compute_prob_zero_centered_gaussian(dist, 0.1)
            
            self.particle_cloud[i].w = q
        
        
    def update_particles_with_motion_model(self):
        # based on the how the robot has moved (calculated from its odometry), we'll move
        # all of the particles correspondingly
        
        # get robot's current and previous poses
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        # calculate robot's motion change in terms of x, y, and yaw
        diff_x = curr_x - old_x
        diff_y = curr_y - old_y
        diff_yaw = curr_yaw - old_yaw

        # calculate robot's motion change in terms of a rotation followed by translation and a second rotation
        rot_1 = math.atan2(diff_y, diff_x) - old_yaw
        trans = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        rot_2 = diff_yaw - rot_1

        # update each particle's pose to match robot's motion change
        # translate each particle by distance traveled first,
        #   followed by rotating it by how much robot has rotated
        for i in range(self.num_particles):
            i_old_yaw = get_yaw_from_pose(self.particle_cloud[i].pose)
            self.particle_cloud[i].pose.position.x += trans * math.cos(i_old_yaw + rot_1) + random.gauss(0, 0.1) # gaussian noise added
            self.particle_cloud[i].pose.position.y += trans * math.sin(i_old_yaw + rot_1) + random.gauss(0, 0.1) # gaussian noise added
            i_new_yaw = i_old_yaw + rot_1 + rot_2 #+ random.gauss(0, math.pi/180) # gaussian noise added
            q = quaternion_from_euler(0.0, 0.0, i_new_yaw)
            self.particle_cloud[i].pose.orientation.x = q[0]
            self.particle_cloud[i].pose.orientation.y = q[1]
            self.particle_cloud[i].pose.orientation.z = q[2]
            self.particle_cloud[i].pose.orientation.w = q[3]


if __name__=="__main__":
    
    pf = ParticleFilter()

    rospy.spin() # keep the program running
