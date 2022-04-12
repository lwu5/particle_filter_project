# particle_filter_project

Name: Suha Chang, Liuhao Wu

# Implementation Plan 4/12
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

## Timeline
- initialize_particle_cloud:  4/17
- update_particles_with_motion_model:  4/17
- update_particle_weights_with_measurement_model:  4/17
- normalize_particles 4/17
- resample_particles: 4/22
- update_estimated_robot_pose: 4/22
- noise: Wednesday 4/22
