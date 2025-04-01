import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# -----------------------------
# Connect and initialize simulation
# -----------------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)

plane_id = p.loadURDF("plane.urdf")
fish_id = p.loadURDF("fish.urdf", basePosition=[0, 0, 1.0], useFixedBase=False)
num_joints = p.getNumJoints(fish_id)
for joint in range(num_joints):
    p.changeDynamics(fish_id, joint, linearDamping=0.5, angularDamping=2.0)

wall_position = [-10, -3, 1]  # Target wall position
wall_size = [0.2, 10, 3]
wall_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_size)
wall_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=wall_size, rgbaColor=[1,0,0,1])
wall_id = p.createMultiBody(baseMass=0,
                            baseCollisionShapeIndex=wall_collision_shape,
                            baseVisualShapeIndex=wall_visual_shape,
                            basePosition=wall_position)

# -----------------------------
# Experiment parameters
# -----------------------------
# Water flow settings (adjustable)
water_flow_amplitude = 2.0  # N
water_flow_frequency = 0.3  # Hz

# Target performance: arrival time < 30 s, average trajectory deviation < 0.2 m
T_target = 30.0  # s

# -----------------------------
# Initialize variables
start_time = time.time()
gravity_phase = True         # Gravity transitions from -1 to 0 during the first 5 seconds
actual_start_time = None     # Time when actual motion starts
dt = 1.0 / 600.0             # Simulation time step

propulsion_energy = 0.0      # Propulsion energy integration (N·s)
water_flow_energy = 0.0      # Water resistance energy integration (N·s)

initial_pos = np.array([0, 0])   # Assume initial horizontal position (0, 0)
positions = []    # To record horizontal position data
head_angles = []  # To record fish head angles (rad)

# Variables for adaptive control
propulsion_multiplier = 1.0    # Initial multiplier
update_interval = 1.0          # Update every 1 second
last_update_time = None
prev_update_pos = None

# Record initial theoretical propulsion force of the fish (only once, using 100 * |angle_2| when motion starts)
initial_propulsion_force = None
initial_tail_amplitude = None

# Ideal straight-line direction (from initial position to target wall, horizontal component only)
target_pos = np.array([wall_position[0], wall_position[1]])
ideal_direction = target_pos - initial_pos
if np.linalg.norm(ideal_direction) > 0:
    ideal_direction = ideal_direction / np.linalg.norm(ideal_direction)
else:
    ideal_direction = np.array([1, 0])

# -----------------------------
# Main simulation loop
# -----------------------------
while True:
    current_time = time.time() - start_time
    # Gravity transitions from -1 to 0 (within 5 seconds)
    gravity_factor = min(current_time / 5.0, 1)
    p.setGravity(0, 0, -1 + gravity_factor * 1)
    if gravity_factor < 1:
        gravity_phase = True
    else:
        gravity_phase = False

    # Begin motion after gravity adjustment is complete
    if not gravity_phase:
        if actual_start_time is None:
            actual_start_time = time.time()
            # Record initial propulsion force and tail swing amplitude at motion start
            temp_angle_2 = 0.1 * math.sin(current_time * 3)  # Calculate tail swing amplitude at this moment
            initial_propulsion_force = 200 * abs(temp_angle_2)
            initial_tail_amplitude = abs(temp_angle_2)
            # Initialize adaptive update parameters
            last_update_time = current_time
            fish_pos_now, _ = p.getBasePositionAndOrientation(fish_id)
            prev_update_pos = np.array([fish_pos_now[0], fish_pos_now[1]])
        
        # Get current fish position and record it
        fish_pos, _ = p.getBasePositionAndOrientation(fish_id)
        fish_x, fish_y, _ = fish_pos
        current_pos = np.array([fish_x, fish_y])
        positions.append((fish_x, fish_y))
        
        # Adjust fish head orientation toward the target wall
        dir_vector = np.array(wall_position[:2]) - current_pos
        target_angle = math.atan2(dir_vector[1], dir_vector[0])
        _, current_ori, _, _, _, _ = p.getLinkState(fish_id, 0)
        current_angle = math.atan2(current_ori[1], current_ori[0])
        head_angles.append(current_angle)
        angle_error = (target_angle - current_angle + math.pi) % (2*math.pi) - math.pi
        p.setJointMotorControl2(fish_id, 0, p.TORQUE_CONTROL, force=10 * angle_error)
        
        # Control fish body swing
        angle_1 = 0.05 * math.sin(current_time * 2)
        angle_2 = 0.1 * math.sin(current_time * 3)  # Base swing amplitude, not affected by adaptation here
        p.setJointMotorControl2(fish_id, 0, p.POSITION_CONTROL, targetPosition=angle_1, force=50)
        p.setJointMotorControl2(fish_id, 1, p.POSITION_CONTROL, targetPosition=angle_2, force=50)
        
        # Adaptive update: update propulsion_multiplier every update_interval seconds
        if current_time - last_update_time >= update_interval:
            # Calculate current speed (relative to last update)
            displacement = np.linalg.norm(current_pos - prev_update_pos)
            time_interval = current_time - last_update_time
            v_avg = displacement / time_interval if time_interval > 0 else 0.001  # Prevent division by zero
            # Remaining distance along the ideal direction
            d_remaining = np.dot(target_pos - current_pos, ideal_direction)
            # Estimated arrival time = elapsed time + d_remaining / v_avg
            t_elapsed = current_time
            estimated_arrival = t_elapsed + d_remaining / v_avg if v_avg > 0 else 1000
            # Compute current average trajectory deviation (based on all recorded positions)
            deviations = []
            p0 = initial_pos
            for pos in positions:
                p_val = np.array(pos)
                # 2D point-to-line distance
                cross_val = abs(ideal_direction[0]*(p_val[1]-p0[1]) - ideal_direction[1]*(p_val[0]-p0[0]))
                deviations.append(cross_val)
            avg_deviation = np.mean(deviations) if deviations else 0
            # Simple adaptive control:
            delta = 0.05
            # Increase multiplier if estimated arrival time is too long or deviation is too high
            if estimated_arrival > T_target or avg_deviation > 0.2:
                propulsion_multiplier += delta
            # Decrease multiplier if both metrics are well below target, but not below 1.0
            elif estimated_arrival < (T_target - 1) and avg_deviation < 0.19:
                propulsion_multiplier = max(1.0, propulsion_multiplier - delta)
            # Update last record
            last_update_time = current_time
            prev_update_pos = current_pos
        
        # Apply propulsion force using adaptive multiplier
        sim_force_strength = (200 * propulsion_multiplier) * abs(angle_2)
        if current_time % 0.2 < (1.0/240.0):
            norm = np.linalg.norm(dir_vector)
            if norm > 0:
                u_x = dir_vector[0] / norm
                u_y = dir_vector[1] / norm
            else:
                u_x = u_y = 0
            p.applyExternalForce(fish_id, -1,
                                 forceObj=[sim_force_strength * u_x, sim_force_strength * u_y, 0],
                                 posObj=[0,0,0], flags=p.LINK_FRAME)
            propulsion_energy += sim_force_strength * dt
        
        # Water resistance
        water_flow_force = water_flow_amplitude * math.sin(current_time * water_flow_frequency)
        p.applyExternalForce(fish_id, -1,
                             forceObj=[water_flow_force * u_x, water_flow_force * u_y, 0],
                             posObj=[0,0,0], flags=p.LINK_FRAME)
        water_flow_energy += abs(water_flow_force) * dt
        
        # Check for collision (target reached)
        contact_points = p.getContactPoints(fish_id, wall_id)
        if len(contact_points) > 0:
            sim_run_time = time.time() - actual_start_time
            arrival_time = sim_run_time
            break

    p.stepSimulation()
    time.sleep(dt)

# -----------------------------
# Calculate trajectory stability
# -----------------------------
p0 = initial_pos
final_pos, _ = p.getBasePositionAndOrientation(fish_id)
p1 = np.array([final_pos[0], final_pos[1]])
line_vec = p1 - p0
line_length = np.linalg.norm(line_vec)
deviations = []
if line_length > 0:
    for pos in positions:
        p_val = np.array(pos)
        cross_val = abs(line_vec[0]*(p_val[1]-p0[1]) - line_vec[1]*(p_val[0]-p0[0]))
        deviations.append(cross_val / line_length)
    avg_deviation_final = np.mean(deviations)
else:
    avg_deviation_final = 0

head_angle_std = np.std(head_angles)

# -----------------------------
# Output results
# -----------------------------
print("Experiment completed!")
print("Water flow parameters: water_flow_amplitude = {} N, water_flow_frequency = {} Hz".format(water_flow_amplitude, water_flow_frequency))
print("Time to reach target: {} s".format(arrival_time))
print("Propulsion energy: {} N·s".format(propulsion_energy))
print("Average trajectory deviation: {} m".format(avg_deviation_final))
print("Final propulsion multiplier: {:.2f}".format(propulsion_multiplier))

# Keep the simulation window open
while True:
    p.stepSimulation()
    time.sleep(dt)
