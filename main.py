import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np
import time

def save_data(data_filepath, N, pos_data, vel_data, param_data):
    pos_filename = "position_data.txt"
    vel_filename = "velocity_data.txt"
    param_filename = "param_data.txt"
    np.savetxt(data_filepath+pos_filename, pos_data, delimiter=',', fmt='%.8f')
    np.savetxt(data_filepath+vel_filename, vel_data, delimiter=',', fmt='%.8f')
    with open(data_filepath+param_filename, 'w') as file:
        for key, value in param_data.items():
            file.write(f'{key}: {value}\n')
    return

# Instantiate Robotarium object
N = 7
# Define goal points by removing orientation from poses
#goal_points = generate_initial_conditions(N)
# print(goal_points)
r = robotarium.Robotarium(number_of_robots=N, show_figure=True, sim_in_real_time=False)

# Define the constants
velocity_mag_limit = 0.15
offset = 0.3
visual_range = 0.8
separation_distance = 0.25

turnfactor = 0.07
avoid_factor = 0.008
centering_factor = 0.0002
matching_factor = 0.002

# Save data variables declared
data_filepath = "simulation_data/"
vel_data = []
position_data = []
parameters_list = ["velocity_mag_limit", "offset", "turnfactor", "visual_range", "avoid_factor", "centering_factor", "matching_factor", "separation_distance"]
parameters_values = [velocity_mag_limit, offset, turnfactor, visual_range, avoid_factor, centering_factor, matching_factor, separation_distance]
parameter_data = {}
for i in range(len(parameters_list)):
    parameter_data[parameters_list[i]] = parameters_values[i]


# Plotting parameters
separation_radius_marker_size = determine_marker_size(r,separation_distance)
visual_radius_marker_size = determine_marker_size(r,visual_range)


# Create single integrator position controller
#single_integrator_position_controller = create_si_position_controller()

# Create barrier certificates to avoid collision
# si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()

_, uni_to_si_states = create_si_to_uni_mapping()

# Create mapping from single integrator velocity commands to unicycle velocity commands
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()

# Function to implement Boids behavior
def boids_behavior(x_si, dxi, velocity_mag_limit, offset, turnfactor, visual_range, avoid_factor, centering_factor, matching_factor, separation_distance):
    # print ('x_si: ',x_si)
    # print("dxi: ",dxi)
    # Got from si_position controller

    for i in range(N):
        xpos_avg, ypos_avg, xvel_avg, yvel_avg, neighboring_boids, close_dx, close_dy = 0,0,0,0,0,0,0
        # Calculate separation vector
        
        for j in range(N):
            if i != j:
                temp_close_dx = x_si[0,i] - x_si[0,j] #x
                temp_close_dy = x_si[1,i] - x_si[1,j] #y
                if (abs(temp_close_dx)<visual_range and abs(temp_close_dy)<visual_range):
                    squared_distance = temp_close_dx**2 + temp_close_dy**2
                
                    if (squared_distance < (separation_distance**2)):
                        # pos
                        close_dx += temp_close_dx
                        close_dy += temp_close_dy 

                    elif (squared_distance < (visual_range**2)):  
                        
                        xpos_avg+=x_si[0,j]
                        ypos_avg+=x_si[1,j]
                        xvel_avg+= dxi[0,j]
                        yvel_avg+= dxi[1,j]
                        neighboring_boids += 1 
            # If there were any boids in the visual range . . .            
        if (neighboring_boids > 0): 

            # Divide accumulator variables by number of boids in visual range
            xpos_avg = xpos_avg/neighboring_boids 
            ypos_avg = ypos_avg/neighboring_boids
            xvel_avg = xvel_avg/neighboring_boids
            yvel_avg = yvel_avg/neighboring_boids

            # Add the centering/matching contributions to velocity
            dxi[0,i]= (dxi[0,i] + 
                    (xpos_avg - x_si[0,i])*centering_factor + 
                    (xvel_avg - dxi[0,i])*matching_factor)

            dxi[1,i] = (dxi[1,i] + 
                    (ypos_avg - x_si[1,i])*centering_factor + 
                    (yvel_avg - dxi[1,i])*matching_factor)


        dxi[0,i] += close_dx*avoid_factor
        dxi[1,i] += close_dy*avoid_factor

        # top margin
        if x_si[1,i] >= 1 - offset:
            dxi[1,i] = dxi[1,i] - turnfactor
        # bottom
        if x_si[1,i] <= -1 + offset:
            dxi[1,i] = dxi[1,i] + turnfactor
         # left margin
        if x_si[0,i] <= -1.6+offset:
            dxi[0,i] = dxi[0,i] + turnfactor
        # right 
        if x_si[0,i] >= 1.6-offset:
            dxi[0,i] = dxi[0,i] - turnfactor
    
    # Clip the velocities to maximum limits
    norms = np.linalg.norm(dxi, axis=0)
    idxs = np.where(norms >= velocity_mag_limit)
    if norms[idxs].size != 0:
        dxi[:, idxs] *= velocity_mag_limit/norms[idxs]
    
    for i in range(len(dxi)):
        if dxi[0][i]>velocity_mag_limit or dxi[1][i]>velocity_mag_limit:
            print(f"i: {i}\ndxi: {dxi}")
            exit()

    return dxi

### define x initially ###
x = r.get_poses()
CM = np.random.rand(N,3) # Random Colors
linewidth_val = 2
g1 = r.axes.scatter(x[0,:], x[1,:], s=np.pi/4*separation_radius_marker_size, marker='o', facecolors='none',edgecolors=CM,linewidth=linewidth_val)
g2 = r.axes.scatter(x[0,:], x[1,:], s=np.pi/4*visual_radius_marker_size, marker='o', facecolors='none',edgecolors=CM,linewidth=linewidth_val)

# Get the x,y coordinates from the unicycle pose above
x_si = uni_to_si_states(x)
# Initialize the velocities
velocity_mag_limit = 0.15
theta0 = np.random.uniform(0,2*3.14,size=(N)) # It is a 1 X N array
mag_vel0 = np.random.uniform(0,velocity_mag_limit, size=(N)) # It is a 1 X N array
dxi = np.zeros((2,N))
for i in range(N):
    # X component of velocity
    dxi[0,i] = mag_vel0[i] * np.cos(theta0[i])
    # Y component of velocity
    dxi[1,i] = mag_vel0[i] * np.sin(theta0[i])

print(f"dxi: {dxi}")
# print(f"Flatten dxi: {dxi.flatten(order='F')}")
position_data.append(x_si.flatten(order='F'))
vel_data.append(dxi.flatten(order='F'))

dxu = si_to_uni_dyn(dxi, x)
r.step()

counter = 0
max_counter = 5000
# While the number of robots at the required poses is less
# than N...
# while (np.size(at_pose(np.vstack((x_si,x[2,:])), goal_points, rotation_error=100)) != N):
# while (counter<=max_counter):
while counter<=max_counter:
    if (counter%1000==0):
        print(f"Counter: {counter}")
    # Get poses of agents
    x = r.get_poses()
    # Update Plotted Visualization
    g1.set_offsets(x[:2,:].T)
    g2.set_offsets(x[:2,:].T)
    # This updates the marker sizes if the figure window size is changed. 
    # This should be removed when submitting to the Robotarium.
    g1.set_sizes([determine_marker_size(r,separation_distance)])
    g2.set_sizes([determine_marker_size(r,visual_range)])
    
    # Convert the unicycle pose to single integrator pose
    x_si = uni_to_si_states(x)

    # Apply Boids behavior
    dxi = boids_behavior(x_si, dxi, velocity_mag_limit, offset, turnfactor, visual_range, avoid_factor, centering_factor, matching_factor, separation_distance)
    
    # Save the data
    position_data.append(x_si.flatten(order='F'))
    vel_data.append(dxi.flatten(order='F'))
    # print("dxi: ",dxi)
    # Create safe control inputs (i.e., no collisions)
    # dxi = si_barrier_cert(dxi, x_si)
    # print(dxi)
    # Transform single integrator velocity commands to unicycle
    dxu = si_to_uni_dyn(dxi, x)

    # Set the velocities by mapping the single-integrator inputs to unciycle inputs
    r.set_velocities(np.arange(N), dxu)
    # Iterate the simulation
    r.step()
    # time.sleep(0.01)
    counter += 1

# Save all the data
save_data(data_filepath, N, position_data, vel_data, parameter_data)

# Call at the end of the script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()
