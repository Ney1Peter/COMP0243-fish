import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Use GUI mode to display the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Initialize gravity to 0
p.setGravity(0, 0, 0)

# Load the ground
plane_id = p.loadURDF("plane.urdf")

# Load the fish model, initial position at [0, 0, 1.0]
fish_id = p.loadURDF("fish.urdf", basePosition=[0, 0, 1.0], useFixedBase=False)
num_joints = p.getNumJoints(fish_id)
for joint in range(num_joints):
    p.changeDynamics(fish_id, joint, linearDamping=0.5, angularDamping=2.0)

# Create a wall (collision target)
wall_position = [-10, -3, 1]  # Wall position
wall_size = [0.2, 10, 3]      # Wall dimensions
wall_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_size)
wall_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=wall_size, rgbaColor=[1, 0, 0, 1])
wall_id = p.createMultiBody(baseMass=0,
                            baseCollisionShapeIndex=wall_collision_shape,
                            baseVisualShapeIndex=wall_visual_shape,
                            basePosition=wall_position)

# -------------------------------
# Experiment parameter settings
# -------------------------------
# Water flow parameters (please modify these two values before each run)
water_flow_amplitude = 0.5  # Water resistance strength (N)
water_flow_frequency = 0.3  # Frequency of the water flow (Hz)

# -------------------------------
# Initialize variables
start_time = time.time()
gravity_phase = True       # Initial phase: gravity gradually changes from -1 to 0
actual_start_time = None   # The moment actual movement starts (after gravity is adjusted)
dt = 1.0 / 600.0           # Simulation time step
propulsion_energy = 0.0    # Propulsion energy consumption (approximate integration, unit: N·s)
water_flow_energy = 0.0    # Water flow resistance energy consumption (approximate integration, unit: N·s)

# Assume fish's initial horizontal position is (0, 0)
initial_pos = [0, 0]

# Record fish trajectory (horizontal position) and head angles
positions = []    # Each element is (x, y)
head_angles = []  # Each element is the fish head angle (rad)

# Record initial propulsion force and tail swing amplitude
initial_propulsion_force = None
initial_tail_amplitude = None

# -------------------------------
# Main simulation loop: runs until fish hits the wall
# -------------------------------
while True:
    current_time = time.time() - start_time
    
    # Gravity gradually changes from -1 to 0 over 5 seconds
    gravity_factor = min(current_time / 5.0, 1)
    p.setGravity(0, 0, -1 + gravity_factor * 1)
    if gravity_factor < 1:
        gravity_phase = True
    else:
        gravity_phase = False
        
    if not gravity_phase:  # Gravity is now 0, real movement begins
        if actual_start_time is None:
            actual_start_time = time.time()
            # At this moment, record the fish's initial propulsion and tail swing amplitude
            # First calculate tail swing amplitude (angle_2):
            # To calculate angle_2, use the current time
            # The calculated angle_2 here serves as the fish's initial swing amplitude
            temp_angle_2 = 0.1 * math.sin(current_time * 3)
            initial_propulsion_force = 200 * abs(temp_angle_2)
            initial_tail_amplitude = abs(temp_angle_2)
        
        # Get fish's current position (horizontal x, y)
        fish_pos, _ = p.getBasePositionAndOrientation(fish_id)
        fish_x, fish_y, _ = fish_pos
        positions.append((fish_x, fish_y))
        
        # Adjust fish head orientation toward the target wall
        dir_x = wall_position[0] - fish_x
        dir_y = wall_position[1] - fish_y
        target_angle = math.atan2(dir_y, dir_x)
        # Get fish head's current orientation (via link0's orientation)
        _, current_ori, _, _, _, _ = p.getLinkState(fish_id, 0)
        current_angle = math.atan2(current_ori[1], current_ori[0])
        head_angles.append(current_angle)
        
        angle_error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
        p.setJointMotorControl2(fish_id, 0, p.TORQUE_CONTROL, force=10 * angle_error)
        
        # Control fish body swing (simulate tail swing using sine functions)
        angle_1 = 0.05 * math.sin(current_time * 2)  # Head-body swing
        angle_2 = 0.1 * math.sin(current_time * 3)  # Tail swing amplitude
        p.setJointMotorControl2(fish_id, 0, p.POSITION_CONTROL, targetPosition=angle_1, force=50)
        p.setJointMotorControl2(fish_id, 1, p.POSITION_CONTROL, targetPosition=angle_2, force=50)
        
        # Apply propulsion force calculated in simulation (doesn't affect initial record)
        sim_force_strength = 200 * abs(angle_2)
        if current_time % 0.2 < (1.0 / 240.0):
            norm = math.sqrt(dir_x ** 2 + dir_y ** 2)
            if norm > 0:
                u_x = dir_x / norm
                u_y = dir_y / norm
            else:
                u_x = u_y = 0
            p.applyExternalForce(fish_id, -1,
                                 forceObj=[sim_force_strength * u_x, sim_force_strength * u_y, 0],
                                 posObj=[0, 0, 0], flags=p.LINK_FRAME)
            propulsion_energy += sim_force_strength * dt
        
        # Water resistance force (simulate periodic water flow using sine function)
        water_flow_force = water_flow_amplitude * math.sin(current_time * water_flow_frequency)
        p.applyExternalForce(fish_id, -1,
                             forceObj=[water_flow_force * u_x, water_flow_force * u_y, 0],
                             posObj=[0, 0, 0], flags=p.LINK_FRAME)
        water_flow_energy += abs(water_flow_force) * dt
        
        # Check for collision (between fish and wall)
        contact_points = p.getContactPoints(fish_id, wall_id)
        if len(contact_points) > 0:
            sim_run_time = time.time() - actual_start_time
            arrival_time = sim_run_time   # Time taken to reach the target
            break
    
    p.stepSimulation()
    time.sleep(dt)

# -------------------------------
# Compute trajectory stability metric
# -------------------------------
p0 = np.array(initial_pos)
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
    avg_deviation = np.mean(deviations)
else:
    avg_deviation = 0

head_angle_std = np.std(head_angles)

# -------------------------------
# Output experimental evaluation metrics
# -------------------------------
print("Experiment completed!")
print("Water flow settings: water_flow_amplitude = {} N, water_flow_frequency = {} Hz".format(water_flow_amplitude, water_flow_frequency))
print("Fish's initial propulsion force: {} N".format(initial_propulsion_force))
print("Fish tail swing amplitude: {} rad".format(initial_tail_amplitude))
print("Time taken to reach the target: {} s".format(arrival_time))
print("Propulsion energy consumption: {} N·s".format(propulsion_energy))
print("Average trajectory deviation: {} m".format(avg_deviation))

# Keep the simulation window open for observation (exit with Ctrl+C)
while True:
    p.stepSimulation()
    time.sleep(dt)
