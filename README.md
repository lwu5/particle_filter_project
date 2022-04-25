# particle_filter_project

Name: Suha Chang, Liuhao Wu

---

# Final Write-Up 4/24/2022
## Objectives Description

- The goal of this project was to solve the problem of robot localization by implementing the particle filter algorithm with Monte Carlo localization. The particle filter uses information from a map and measurements from the robot’s odometry data to locate the robot in space, and it uses probabilistic sampling methods to update a belief of the robot’s position. 

## Behavior

Below, we provide a demo for this particle localization algorithm. We recorded this demo in two ways: one view includes both the robot's movement in maze and Rviz view; the other only recorded Rviz view directly from computer with higher clearity. We also recorded a bag with the following topics: `/map`, `/scan`, `/cmd_vel`, `/particle_cloud`, `/estimated_robot_pose` and this is the (bag)[https://github.com/lwu5/particle_filter_project/tree/main/bags]'s location.

Note: Please discard the virtual turtlebot in the Rviz, it does not correctly reflect on the robot's position and movement in reality. Please refer to the demo video for the actually robot position and movement.

- https://user-images.githubusercontent.com/59663733/165006261-a7e43fef-a816-493f-aea0-e0860cc4aed0.mp4

- https://user-images.githubusercontent.com/59663733/165006340-0f6fdf12-74ef-4554-b17f-afd8ba6c8a84.mp4


## High-Level Description

- We solved the problem of robot localization by implementing the particle filter algorithm with Monte Carlo Localization. This algorithm first randomly distributes many different particles (which each represent a guess about where a robot is placed/oriented) throughout a map of a maze environment that we collected using the SLAM method. As the robot moves around and senses obstacles in the maze, the particles also move around like the robot and update their own hypothetical laser scan measurements. The algorithm then compares the robot’s sensor measurements to what different particles are sensing in their vicinity; particles that more closely match what the robot is sensing are weighted more highly as being candidates for the robot’s true location. Particles are then probabilistically resampled with replacement depending on their weight. With every iteration of the model it is expected that a particle cloud which likely consists of particles that had larger weights (representing the best estimates of the robot’s location) will converge onto the robot’s true location. 

## Main Steps Code Explanation
1. **Initialization of particle cloud**
- **Code Location**: Implemented with function `initialize_particle_cloud()`
- **Code Description**: We first access the map's resolution and boundaries using `get_obstacle_bounding_box()` in the `likelihood_field.py`. Then we use those information to generate particles' x, y, and yaw values randomly within the map using our own `draw_random_sample()`. What the `draw_random_sample()` does is essentailly calling python function `random.choices()` to draws a random sample of n elements from a given list of choices and their specified probabilities / weights. During the initialization, we let the list of choices to be every single resolution in map in terms of x and y and every integer degree in `[0,360)` for yaw and set every resolution to be equal probability. We use those randomly generated x, y, and yaw values to create particles with every particle's weight set to `1`. Before we publish the partcile cloud, we also normalize all particle weights so that all probabilities sum to `1`.

2. **Movement model**
- **Code Location**: Implemented with function `update_particles_with_motion_model()`
- **Code Description**: We first get robot's current and previous poses (x, y, and yaw). With this information, we can calculate robot's motion changes in terms of x, y, and yaw and in terms of a rotation followed by translation and a second rotation. Then we use a loop to update each particle's pose to match robot's motion change (using the algorithm given on the slack): translate each particle by distance traveled first, followed by rotating it by how much robot has rotated. Note, here we add Gaussian noise to each particle's x and y values using python `random.gauss()` function and we do not add noise for yaw because we realized through our testing that the estimation of robot's pose performs better without adding noise to yaw than with noises. We convert the new yaw for each partcle back to quaternion at the end.

3. **Measurement model**
- **Code Location**: Implemented with function `update_particle_weights_with_measurement_model()`
- **Code Description**: We use a nested loop to iterate through every particle in the cloud and each of the eight directions (i.e., `[0, 45, 90, 135, 180, 225, 270, 315]`). Using the `get_closest_obstacle_distance` function from `likelihood_field.py`, we find the distance to the closest obstacle for each particle in designated directions and with the measurement model, we can calculate each particle's weights. Note, if at some direction, a particle seems to be outside the map boundaries (i.e., `get_closest_obstacle_distance` returns `nan`), we have its weight timed by a small number to decreate its probability to get resampled later.

4. **Resampling**
- **Code Location**: Implemented with function `resample_particles()`
- **Code Description**: We use `draw_random_sample()` to generate particles for our new particle cloud randomly with their weights as probabilities. What the `draw_random_sample()` does is essentailly calling python function `random.choices()` to draws a random sample of n elements from a given list of choices and their specified probabilities / weights. During the resampling step, we let the list of choices to be every particle in the old cloud and use particles' weights as probabilities, and the total number of particles in the old and new cloud remains the same. The function `draw_random_sample()` would return a list of indexes of particles in the old cloud and we create new particles for our new cloud based on mapping of this index list and partciles in the old cloud. Finally, we replace the old particle cloud with our new particle cloud.

5. **Incorporation of noise**
- **Code Location**: Incorporated the python function `random.gauss()` to `update_particles_with_motion_model()`
- **Code Description**: We feed `random.gauss()` 0 as mean and 0.1 as standard deviation to get a random gaussian distribution number to move particles' x and y values with a random noise mostly within `[-0.1, 0.1]`. We do not add noise for yaw because we realized through our testing that the estimation of robot's pose performs better without adding noise to yaw than with noises.

6. **Updating estimated robot pose**
- **Code Location**: Implemented with function `update_estimated_robot_pose()`
- **Code Description**: We simply interate through every particle in the cloud to calculate the weighted average in terms of x, y, and yaw and convert yaw to quaternion before assign them to robot's estimated pose. 

7. **Optimization of parameters**
- **Code Location**: We optimized the following parameters: particle numbers (`self.num_particles` in `ParticleFilter` class), mean and standard deviations for Gaussian noises (`random.gauss` in `update_particles_with_motion_model()` function), standard deviations for gaussian probability (`compute_prob_zero_centered_gaussian()` in `update_particle_weights_with_measurement_model()` function), and number of ranges we taken into account when updating particle weights (`direction_idxs` in `update_particle_weights_with_measurement_model()` function).
- **Code Description**: For particle numbers, we realized that too many particles can make program laggy and give timing error but too less particles drop the algorithm's accuracy, so we found particle 5,000 is an acceptable number. For mean and standard deviations for Gaussian noises, we just tried a bunch of different numbers for optimization. We do not add gaussian noise for yaw because we realized through our testing that the estimation of robot's pose performs better without adding noise to yaw than with noises. For the standard deviation for `compute_prob_zero_centered_gaussian()`, we realized that as the std div becomes smaller, particles converge faster but also lower the accuracy; we took 0.1 at the end for short converging time and accuracy. For the number of ranges we taken into account when updating particle weights, we realized that if we only consider four cardinal directions, the accuracy is a bit low; but if we consider all 360 degrees, the program becomes too computationally expensive; so we ended up taking eight directions into consideration.

## Challenges

- Some initial challenges we faced was figuring out the motion model and how to make the particle move like the robot, but with respect to its own orientation. In order to understand what was going wrong, we overcame our issues by drawing out lots of diagrams and doing careful testing with small numbers of particles to check their position and orientation in space. This approach solidified our intuitions for what was going wrong and helped us better understand the motion model we ultimately implemented (the one suggested on Slack). 
- Another challenge we faced was trying to figure out why our weights were becoming very small and ultimately breaking our particle filter model since it would only resample one particle when all the weights had gone to zero. With the help of TAs we found that it was because we had not made a deep copy of the particles when resampling and that was potentially severely impacting the weight updating of particles that had been resampled more than once. This mistake taught us a lesson about making sure that lists updated in object oriented programming (for future projects) should not include these shallow copies.
- One challenge we also faced was modulating the addition of noise and optimizing those parameters. We were stumped for a bit by a lot of our particles seemingly disappearing from the map, but we realized through debugging that it was because we were resampling so many of the same particles since the noise was being implemented in the wrong place. Through trial and error we were able to find the right places to insert noise and also try out a bunch of different values to make the cloud converge to the optimal position of the robot.

## Future Work

- If we had more time, we would want to improve the localization so that it converges faster. We were generally limited in how many particles we could include in the model because we got the error that rviz’s time goes backwards if we included too many. Therefore we would like to be able to increase the number of particles to see how well it can perform with more. Also we would want to spend more time further optimizing parameters like noise and the standard deviations for likelihood so that the robot can be accurately localized in space given this larger particle amount. It would also be cool to be able to obtain different maps and test the localization ability in different environments that are not so narrow or similar like the maze. 

## Takeaways

1. One key takeaway from this project was learning how to manipulate variables in different coordinate systems and thinking about how to transform between online maps and real-life spaces. Managing all the different conversions from the data sent from the robot and testing how they are oriented relative to the RViz environment.
2. TODO
3. Another key takeaway was thinking about how to work with the limitation of hardware. Our computers became super laggy when we set particle cloud size to 10,000 and took too many lazer directions into account for updating particles with measurement models. This forces us to learn to find a balance between performance and hardware limitation. We have to spend time testing what number of particles and directions our computers can handle and the program performs well.

---

# Implementation Plan 4/12/2022
## Implementation + Testing
1. How you will initialize your particle cloud (`initialize_particle_cloud`)?

- **Implementation**: We will determine the position boundaries of the map and use a random function to generate a large sample of particles with coordinates that will be randomly distributed within that space. We will put those particle objects in the array `particle_cloud`.
- **Testing**: Check the array length and visualize the particles on the map using rviz tools. We want to make sure that the particles are roughly randomly distributed across the empty spaces of the map.

2. How you will update the position of the particles will be updated based on the movements of the robot (`update_particles_with_motion_model`)?

- **Implementation**: We first capture the position of the robot after the movement (i.e. listening to messages about the robot’s position and orientation). We would then iterate through the array to update the position of particles to match the same movement of the robot and we also make sure to keep particles within the map’s boundaries if it goes outside of the map after the movement.
- **Testing**: We can either visualize the particle cloud to see if it matches the robot's movement or take a couple particles as samples to see if their position changes match the robot's movement.

3. How you will compute the importance weights of each particle after receiving the robot's laser scan data? (`update_particle_weights_with_measurement_model`)?

- **Implementation**: We will iterate through the particle cloud array and use the measurement model / equation discussed in the class to calculate and update the importance weights for each particle based on its relative position to the data we receive from the robot's laser scan.
- **Testing**: We would choose a couple particles both far away from and close to the robot (when testing, we know the robot’s location), and compare their weights to see if the closer the particle is to the robot, the larger weight it has. We could also further visualize over a larger area what the weights look like by plotting a heat map indicating the weights for each particle in the 2D space and seeing how they are distributed over the map relative to the robot. 

4. How you will normalize the particles' importance weights (`normalize_particles`) and resample the particles (`resample_particles`)?

- **Implementation**: We would iterate through the array to get the total weight sum of the all particles and iterate again to normalize the particle’s importance weight by multiplying their current weights by 1 / {total weights}. We would create another array and assign each particle a range (this range should be between 0 and 1; if the weight is 0.1 for a particle, then the range is 0-0.1) based on their weight so that no two particles have an overlapping range of weights. Then we use a random function to generate numbers between 0 - 1 (equal amount to the number of particles we want to resample) and see which range (thus particle) it corresponds to. We create new particles this way and then replace them with the old particles. 
- **Testing**: For testing normalized weights, we just add weights of all particles together to check if the sum equals to 1. For resampling, since we would know the robot's location when testing, we should just visualize the particle cloud and see if more particles are clustered around the robot. 

5. How you will update the estimated pose of the robot (`update_estimated_robot_pose`)?

- **Implementation**: We would take the average pose of all particles on the map as the estimated pose of the robot.
- **Testing**: We visualize all particles on the map and see if our estimated pose of the robot moves towards where the majority of particles are clustered after the resampling step.

6. How you will incorporate noise into your particle filter localization?

- **Implementation**: We will generate artificial Gaussian noise and implement it in the measurement of the particle’s positions after resampling the particles in the cloud. 
- **Testing**: We will first confirm that particles are moving in the expected direction after the addition of noise (likely with large noise values to see the difference more clearly) and then fine tune the parameters of the Gaussian noise generator to make sure that we are not jittering the particle positions too far from their original position such that the particle cloud is compromised and the estimated position of the robot is very different from before the addition of noise. 

---

## Timeline
- initialize_particle_cloud:  4/17
- update_particles_with_motion_model:  4/17
- update_particle_weights_with_measurement_model:  4/17
- normalize_particles 4/17
- resample_particles: 4/22
- update_estimated_robot_pose: 4/22
- noise: 4/22
