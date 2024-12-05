"""
This demo script demonstrates the various functionalities of each controller available within robosuite.

For a given controller, runs through each dimension and executes a perturbation "test_value" from its
neutral (stationary) value for a certain amount of time "steps_per_action", and then returns to all neutral values
for time "steps_per_rest" before proceeding with the next action dim.

    E.g.: Given that the expected action space of the Pos / Ori (OSC_POSE) controller (without a gripper) is
    (dx, dy, dz, droll, dpitch, dyaw), the testing sequence of actions over time will be:

        ***START OF DEMO***
        ( dx,  0,  0,  0,  0,  0, grip)     <-- Translation in x-direction      for 'steps_per_action' steps
        (  0,  0,  0,  0,  0,  0, grip)     <-- No movement (pause)             for 'steps_per_rest' steps
        (  0, dy,  0,  0,  0,  0, grip)     <-- Translation in y-direction      for 'steps_per_action' steps
        (  0,  0,  0,  0,  0,  0, grip)     <-- No movement (pause)             for 'steps_per_rest' steps
        (  0,  0, dz,  0,  0,  0, grip)     <-- Translation in z-direction      for 'steps_per_action' steps
        (  0,  0,  0,  0,  0,  0, grip)     <-- No movement (pause)             for 'steps_per_rest' steps
        (  0,  0,  0, dr,  0,  0, grip)     <-- Rotation in roll (x) axis       for 'steps_per_action' steps
        (  0,  0,  0,  0,  0,  0, grip)     <-- No movement (pause)             for 'steps_per_rest' steps
        (  0,  0,  0,  0, dp,  0, grip)     <-- Rotation in pitch (y) axis      for 'steps_per_action' steps
        (  0,  0,  0,  0,  0,  0, grip)     <-- No movement (pause)             for 'steps_per_rest' steps
        (  0,  0,  0,  0,  0, dy, grip)     <-- Rotation in yaw (z) axis        for 'steps_per_action' steps
        (  0,  0,  0,  0,  0,  0, grip)     <-- No movement (pause)             for 'steps_per_rest' steps
        ***END OF DEMO***

    Thus the OSC_POSE controller should be expected to sequentially move linearly in the x direction first,
        then the y direction, then the z direction, and then begin sequentially rotating about its x-axis,
        then y-axis, then z-axis.

Please reference the documentation of Controllers in the Modules section for an overview of each controller.
Controllers are expected to behave in a generally controlled manner, according to their control space. The expected
sequential qualitative behavior during the test is described below for each controller:

* OSC_POSE: Gripper moves sequentially and linearly in x, y, z direction, then sequentially rotates in x-axis, y-axis,
            z-axis, relative to the global coordinate frame
* OSC_POSITION: Gripper moves sequentially and linearly in x, y, z direction, relative to the global coordinate frame
* IK_POSE: Gripper moves sequentially and linearly in x, y, z direction, then sequentially rotates in x-axis, y-axis,
            z-axis, relative to the local robot end effector frame
* JOINT_POSITION: Robot Joints move sequentially in a controlled fashion
* JOINT_VELOCITY: Robot Joints move sequentially in a controlled fashion
* JOINT_TORQUE: Unlike other controllers, joint torque controller is expected to act rather lethargic, as the
            "controller" is really just a wrapper for direct torque control of the mujoco actuators. Therefore, a
            "neutral" value of 0 torque will not guarantee a stable robot when it has non-zero velocity!

"""

import time
from typing import Dict

import robosuite as suite
from robosuite.controllers.composite.composite_controller_factory import refactor_composite_controller_config
from robosuite.utils.input_utils import *
from robosuite.environments.manipulation.stack_bridge import *
MAX_FR = 25  # max frame rate for running simluation



def get_location(obs):
    '''
    robot0_eef_pos is np array (x,y,z)
    robot0_eef_quat is in quad angale (x,y,z,w)
    '''
    return obs['robot0_eef_pos'] ,obs['robot0_eef_quat']

def get_current_location(env):
    obs = env.step([0,0,0,0,0,0,0])[0]
    return get_location(obs)
def wrap_to_pi(angle):
    """
    Wraps an angle (or angles) to the range [-pi, pi].
    
    Parameters:
    - angle (float or np.array): Angle(s) to be wrapped.
    
    Returns:
    - float or np.array: Wrapped angle(s) in the range [-pi, pi].
    """
    return (angle + 180) % 360 - 180

def get_object_pos_and_quats(env):
    obs = env.step([0,0,0,0,0,0,0])[0]
    # breakpoint()
    target_pos_bridge_1 = obs['bridge_cube_pos_1'] #+ np.array([0.0,0,0.1])
    obj_angle_bridge_1 = quat_to_euler(obs['bridge_cube_quat_1'])   / np.pi  * 180

    target_pos_bridge_2 = obs['bridge_cube_pos_2'] #+ np.array([0.0,0,0.1])
    obj_angle_bridge_2 = quat_to_euler(obs['bridge_cube_quat_2'])   / np.pi  * 180
    
    target_pos_s = obs['cube_pos_s'] #+ np.array([0.0,0,0.1])
    obj_angle_s = quat_to_euler(obs['cube_quat_s'])   / np.pi  * 180
    
    target_pos_l = obs['cube_pos_l'] #+ np.array([0.0,0,0.1])
    obj_angle_l = quat_to_euler(obs['cube_quat_l'])   / np.pi  * 180
    return [
        ("bridgebox1",target_pos_bridge_1,obj_angle_bridge_1,np.array(env.bridge_cube_1.size)),
        ("bridgebox2",target_pos_bridge_2,obj_angle_bridge_2,np.array(env.bridge_cube_2.size)),
        ("box1",target_pos_s,obj_angle_s,np.array(env.cube_s.size)),
        ("box2",target_pos_l,obj_angle_l,np.array(env.cube_l.size))
    ]
    
def get_object_pos_and_quats_by_name(env,name):
    data = get_object_pos_and_quats(env)
    for row in data:
        if row[0] == name:
            return row
    else:
        raise ValueError
# def get_target_loaction(env):
#     obs = env.step([0,0,0,0,0,0,0])[0]
#     target_pos = obs['cube_pos_s'] #+ np.array([0.0,0,0.1])
#     obj_angle = quat_to_euler(obs['cube_quat_s'])   / np.pi  * 180
#     a0 = obj_angle[-1]
#     a1 = obj_angle[-1] + 90
#     a2 = obj_angle[-1] + 270
#     a3 = obj_angle[-1] + 180
#     das = np.array([a0,a1,a2,a3])
#     target_euler = np.array([0,0,0])
#     da = das - target_euler[-1]
#     da = np.abs(wrap_to_pi(da))
    
#     aa = [a0,a1,a2,a3][np.argmin(da)]
#     target_euler[-1] = aa # obj_angle[-1]
#     return target_pos,target_euler

def fix_eular(target_euler):
    # obs = env.step([0,0,0,0,0,0,0])[0]
    # target_pos = obs['cube_pos_s'] #+ np.array([0.0,0,0.1])
    obj_angle = target_euler #quat_to_euler(target_euler)   / np.pi  * 180
    a0 = obj_angle[-1]
    a1 = obj_angle[-1] + 90
    a2 = obj_angle[-1] + 270
    a3 = obj_angle[-1] + 180
    das = np.array([a0,a1,a2,a3])
    target_euler = np.array([0,0,0])
    da = das - target_euler[-1]
    da = np.abs(wrap_to_pi(da))
    
    aa = [a0,a1,a2,a3][np.argmin(da)]
    target_euler[-1] = aa # obj_angle[-1]
    return target_euler
def move_to_target_location(env, target_location, position_tolerance=0.01, orientation_tolerance=10, max_steps=100000,arm=-1
                            ,break_on_z_force=False):
    """
    Moves the robot's end-effector to the target location (position and orientation).
    
    Parameters:
    - env: The robosuite environment instance.
    - target_location: Tuple containing the target position (np.array) and orientation as Euler angles (np.array).
    - position_tolerance: Tolerance for position difference.
    - orientation_tolerance: Tolerance for orientation difference.
    - max_steps: Maximum number of steps for the movement.
    """
    target_pos, target_euler = target_location  # target_euler: [roll, pitch, yaw]
    target_pos = np.array(target_pos)
    target_euler = np.array(target_euler)
    #target_euler = np.array([-180 ,-13,  90]) 
    #for _ in range(max_steps):
    target_euler[0] = -180
    target_euler[1] = -13
    obs = env.step([0,0,0,0,0,0,0])[0]
    # breakpoint()
    ii = 0
    past_z_force = np.array([-1] * 100)
    past_z_force_ptr = 0
    all_pos_dist = [9999]
    while True:
        ii += 1
        # Get the current observation
        
        # breakpoint()
        current_pos, current_quat = get_location(obs)
        

        # breakpoint()
        # breakpoint()
        
        
        # Calculate position error
        pos_error = target_pos - current_pos
        pos_distance = np.linalg.norm(pos_error)
        all_pos_dist.append(pos_distance)
        all_pos_dist = all_pos_dist[-100:]
        # Convert current quaternion to Euler angles
        current_euler = quat_to_euler(current_quat)  / np.pi  * 180
       
        # Calculate orientation error (difference in Euler angles)
        euler_error = target_euler - current_euler
        euler_error = wrap_to_pi(euler_error)
        #euler_error[[0,1]] = 0
        euler_distance = np.abs(euler_error)[-1] #.max()
        
        #print(np.round(np.array(current_euler),2).astype(int),euler_distance)
       
        # Check if the robot is within the tolerances
        
        if max(all_pos_dist) - pos_distance < 0.01:
            pass_pos = True
        else:
            pass_pos = pos_distance < position_tolerance
        print(pos_distance,euler_distance,pass_pos,euler_distance < orientation_tolerance)
        in_lim = pass_pos and euler_distance < orientation_tolerance
        #print(pos_distance,euler_distance,in_lim)
        # if ii % 20 == 0:
        #     print("Force:",env.robots[0].get_sensor_measurement('gripper0_right_force_ee'),end='\t')
        #     print("Torque:",env.robots[0].get_sensor_measurement('gripper0_right_torque_ee'))
        if in_lim:
            break
        
        past_z_force_ptr += 1
        past_z_force_ptr = past_z_force_ptr % len(past_z_force)
        past_z_force[past_z_force_ptr] = env.robots[0].get_sensor_measurement('gripper0_right_force_ee')[-1]
        if break_on_z_force and np.mean(past_z_force  > 0):
            print("BEREAK!")
            break
        
        # Normalize position error
        pos_action = pos_error / (np.linalg.norm(pos_error) + 1e-6)  # Avoid division by zero
        pos_action = np.sign(pos_error) * 0.5 * np.clip( pos_distance**2,0.2,1.0,) * 0.5
        # Use the Euler angle error for rotational control
        rot_action = euler_error # euler_error
        rot_action  = np.sign(rot_action) * np.clip(rot_action,0.3,1.0) / 20

        if pos_distance < position_tolerance:
            pos_action *= 0
                # breakpoint()
        rot_action[np.abs(euler_error) < orientation_tolerance] = 0
        # rot_action *= 0
        # if euler_distance <orientation_tolerance:
        #     rot_action *= 0
        # rot_action *= 0
        #print(euler_error,euler_distance)
        # rot_action[[0,1]] *= 0
        # Combine position and rotation actions, and append gripper action (0)
        action = np.concatenate([pos_action, rot_action, [arm]])  # [dx, dy, dz, droll, dpitch, dyaw, gripper]
        # action[5] = 0.01
        # Send the action to the environment
        obs, _, _, _ = env.step(action)
        env.render()
    return 
    while True:
        obs, _, _, _ = env.step([0,0,0,0,0,0,0])

    return

def grab_object(env,arm=-1,max_steps=100):
    obs = env.step([0,0,0,0,0,0,0])[0]
    for _ in range(max_steps):
        env.step([0,0,0,0,0,0,arm])
        env.render()
        
def idle(env,n=-1):
    if n > 0:
        for _ in range(n):
            env.step([0,0,0,0,0,0,0])
            env.render()
    else:
        while True:
            env.step([0,0,0,0,0,0,0])
            env.render()
        
def is_success(env):
    objs = get_object_pos_and_quats(env)
    all_dist = []
    n = len(objs)
    height = [x[1][-1] for x in objs]
    common_sense =  height[0] > height[1] + 0.01 and height[1] > height[2]+0.01
    height = sorted(height,reverse=True)
    # sort 
    print(height)
    ok =  height[0] > height[1] + 0.01 and height[1] > height[2]+0.01
    return ok,common_sense
    # breakpoint()

def quat_to_euler(quaternion):
    """
    Converts a quaternion to Euler angles (roll, pitch, yaw) in radians.
    Assumes quaternion format: [x, y, z, w].
    
    Parameters:
    - quaternion (np.array): Quaternion (x, y, z, w) in radians.
    
    Returns:
    - np.array: Euler angles [roll, pitch, yaw] in radians.
    """
    x, y, z, w = quaternion

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])

if __name__ == "__main__":

    # Create dict to hold options that will be passed to env creation call
    options = {}

    # print welcome info
    print("Welcome to robosuite v{}!".format(suite.__version__))
    print(suite.__logo__)

    # Choose environment and add it to options
    options["env_name"] = "StackBridge" # choose_environment()

    # If a multi-arm environment has been chosen, choose configuration and appropriate robot(s)
    if "TwoArm" in options["env_name"]:
        # Choose env config and add it to options
        options["env_configuration"] = choose_multi_arm_config()

        # If chosen configuration was bimanual, the corresponding robot must be Baxter. Else, have user choose robots
        if options["env_configuration"] == "bimanual":
            options["robots"] = "Baxter"
        else:
            options["robots"] = []

            # Have user choose two robots
            print("A multiple single-arm configuration was chosen.\n")

            for i in range(2):
                print("Please choose Robot {}...\n".format(i))
                options["robots"].append(choose_robots(exclude_bimanual=True))
    # If a humanoid environment has been chosen, choose humanoid robots
    elif "Humanoid" in options["env_name"]:
        options["robots"] = choose_robots(use_humanoids=True)
    # Else, we simply choose a single (single-armed) robot to instantiate in the environment
    else:
        options["robots"] = "Panda" #choose_robots(exclude_bimanual=True)

    # Hacky way to grab joint dimension for now
    joint_dim = 6 if options["robots"] == "UR5e" else (16 if options["robots"] == "GR1" else 7)

    # Choose controller
    controller_name = "OSC_POSE" #choose_controller(part_controllers=True)

    # Load the desired controller
    arm_controller_config = suite.load_part_controller_config(default_controller=controller_name)
    robot = options["robots"][0] if isinstance(options["robots"], list) else options["robots"]
    options["controller_configs"] = refactor_composite_controller_config(
        arm_controller_config, robot, ["right", "left"]
    )

    # Define the pre-defined controller actions to use (action_dim, num_test_steps, test_value)
    controller_settings = {
        "OSC_POSE": [6, 6, 0.1],
        "OSC_POSITION": [3, 3, 0.1],
        "IK_POSE": [6, 6, 0.01],
        "JOINT_POSITION": [joint_dim, joint_dim, 0.2],
        "JOINT_VELOCITY": [joint_dim, joint_dim, -0.1],
        "JOINT_TORQUE": [joint_dim, joint_dim, 0.25],
    }

    # Define variables for each controller test
    action_dim = controller_settings[controller_name][0]
    num_test_steps = controller_settings[controller_name][1]
    test_value = controller_settings[controller_name][2]

    # Define the number of timesteps to use per controller action as well as timesteps in between actions
    steps_per_action = 75
    steps_per_rest = 75

    # initialize the task
    env = suite.make(
        **options,
        has_renderer=True,
        has_offscreen_renderer=False,
        ignore_done=True,
        use_camera_obs=False,
        horizon=(steps_per_action + steps_per_rest) * num_test_steps,
        control_freq=20,
    )
    # breakpoint()
    env.reset()
    env.viewer.set_camera(camera_id=0)

    # To accommodate for multi-arm settings (e.g.: Baxter), we need to make sure to fill any extra action space
    # Get total number of arms being controlled
    n = 0
    gripper_dim = 0
    for robot in env.robots:
        gripper_dim = robot.gripper["right"].dof
        n += int(robot.action_dim / (action_dim + gripper_dim))

    # Define neutral value
    neutral = np.zeros(action_dim + gripper_dim)

    # Keep track of done variable to know when to break loop
    count = 0
    # Loop through controller space
    

    # POLICY
    # target_pos,target_euler = get_target_loaction(env)
    all_objs = get_object_pos_and_quats(env)
    # is_success(env)
    #### This is a "Gold Implementation".  
    # sort by size
    all_objs = sorted(all_objs,key=lambda x:x[-1][0],reverse=True)
    obj_names = list([x[0] for x in all_objs])
    bridge_objs = []
    bridge_obj_names = []
    other_obj_names = []
    other_objs = []
    for i in range(len(obj_names)):
        if 'bridge' in obj_names[i]:
            bridge_objs.append(all_objs[i])
            bridge_obj_names.append(obj_names[i])
        else:
            other_obj_names.append(obj_names[i])
            other_objs.append(all_objs[i])
    # length = [x[-1][0] for x in all_objs]
    # height = [x[1][-1] for x in all_objs]
    assert len(bridge_objs) == 2
    bridge_gap = abs(bridge_objs[0][1][0] - bridge_objs[1][1][0])
    other_objs_length = [x[-1][0] for x in other_objs]
    tmp = 0
    tmp_name = None
    for j in range(len(other_objs_length)):
        if other_objs_length[j] > tmp:
            tmp_name = other_obj_names[j]
            tmp = other_objs_length[j]
    selected_block_name = tmp_name

    _, target_pos, target_euler, _ = get_object_pos_and_quats_by_name(env, selected_block_name)
    target_euler = fix_eular(target_euler)
    z_suf = target_pos[2]
    
    # Position the block across the gap
    _, bridge_cube_1_pos, _, _ = get_object_pos_and_quats_by_name(env, bridge_obj_names[0])
    _, bridge_cube_2_pos, _, _ = get_object_pos_and_quats_by_name(env, bridge_obj_names[1])

    placement_pos = (bridge_cube_1_pos + bridge_cube_2_pos) / 2
    placement_pos[2] = bridge_cube_1_pos[2]  # Align z-coordinate with the bridge blocks


    move_to_target_location(env, [target_pos+[0,0,0.1],
                                    target_euler,
                                    ], position_tolerance=0.01, orientation_tolerance=5, max_steps=1000)
    move_to_target_location(env, [target_pos, target_euler], position_tolerance=0.01, orientation_tolerance=5, max_steps=1000)
    grab_object(env, 1)
    from scipy.spatial.transform import Rotation as R
    obs = env.step([0,0,0,0,0,0,0])[0]
    if selected_block_name == 'box1':
        block_quat = obs["cube_quat_s"]
    else:
        block_quat = obs["cube_quat_l"]
    gripper_quat = obs["robot0_eef_quat"]

    block_rotation = R.from_quat(block_quat).as_matrix()  # 3x3 rotation matrix
    gripper_rotation = R.from_quat(gripper_quat).as_matrix()
    
    # Dot product to find alignment
    approach_vector = gripper_rotation[:, 1]  # Z-axis of the gripper
    block_axes = block_rotation.T  # Local axes of the block (columns are axes)

    # Alignment scores
    alignment_scores = np.dot(block_axes, approach_vector)
    aligned_axis = np.argmax(np.abs(alignment_scores)) # 0: length, 1: width, 2: height
    if aligned_axis == 1:
        placement_euler = [0, 0, 0]  # Ensure the block spans across the gap
    else:
        placement_euler = [0, 0, 90]

    move_to_target_location(env, [target_pos+[0,0,0.1], target_euler], position_tolerance=0.01, orientation_tolerance=5, max_steps=1000, arm=1)

    move_to_target_location(env, [placement_pos + [0,0,0.1], placement_euler], position_tolerance=0.01, orientation_tolerance=5, max_steps=1000,arm=1)
    # move_to_target_location(env, [placement_pos + [0,0,0.1], placement_euler], position_tolerance=0.01, orientation_tolerance=5)
    move_to_target_location(env, [placement_pos - np.array([0, 0, 0.01]), placement_euler], position_tolerance=0.01, orientation_tolerance=5, max_steps=1000,arm=1)
    grab_object(env, -1)  # Release the block

    current_pos, current_quat = get_current_location(env)
    move_to_target_location(env, [current_pos + np.array([0,0,0.22]),
                                placement_euler], position_tolerance=0.01, orientation_tolerance=5, max_steps=1000,arm=0)
    move_to_target_location(env, [[0,0,current_pos[-1]+0.2],
                                    placement_euler,
                                    ], position_tolerance=0.12, orientation_tolerance=5, max_steps=1000,arm=0)

    idle(env, 1000)  # Allow the robot to observe if the structure remains stable
    env.close()
