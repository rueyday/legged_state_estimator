import robotoc
import numpy as np


def create_robot():
    path_to_urdf = 'a1_description/urdf/a1.urdf'
    contact_frames = ['FL_foot', 'RL_foot', 'FR_foot', 'RR_foot'] 
    contact_types = [robotoc.ContactType.PointContact for i in contact_frames]
    baumgarte_time_step = 0.05
    robot = robotoc.Robot(path_to_urdf, robotoc.BaseJointType.FloatingBase, 
                        contact_frames, contact_types, baumgarte_time_step)
    return robot


def create_mpc_trot():
    robot = create_robot()
    LF_foot_id, LH_foot_id, RF_foot_id, RH_foot_id = robot.contact_frames()

    step_length = np.array([0.15, 0, 0]) 
    # step_length = np.array([0.1, 0, 0]) 
    # step_length = np.array([-0.1, 0, 0]) 
    # step_length = np.array([0, 0.1, 0]) 
    # step_length = np.array([0.1, -0.1, 0]) 
    yaw_step = 0.0
    # yaw_step = np.pi / 6

    swing_height = 0.1
    swing_time = 0.25
    stance_time = 0.05
    stance_time = 0.0
    swing_start_time = 0.5

    vcom_cmd = 0.5 * step_length / (swing_time + stance_time)
    yaw_rate_cmd = yaw_step / swing_time

    T = 0.5
    N = 18
    nthreads = 4
    mpc = robotoc.MPCTrot(robot, T, N, nthreads)

    planner = robotoc.TrotFootStepPlanner(robot)
    planner.set_gait_pattern(step_length, yaw_step, (stance_time > 0.))
    mpc.set_gait_pattern(planner, swing_height, swing_time, stance_time, swing_start_time)

    q = np.array([0, 0, 0.3181, 0, 0, 0, 1, 
                  0.0,  0.67, -1.3, 
                  0.0,  0.67, -1.3, 
                  0.0,  0.67, -1.3, 
                  0.0,  0.67, -1.3])
    v = np.zeros(robot.dimv())
    t = 0.0
    option_init = robotoc.SolverOptions()
    option_init.max_iter = 10

    mpc.init(t, q, v, option_init)
    option_mpc = robotoc.SolverOptions()
    option_mpc.max_iter = 2 # MPC iterations
    option_mpc.max_iter = 1 # MPC iterations
    mpc.set_solver_options(option_mpc)

    return mpc, planner


def create_mpc_flying_trot():
    robot = create_robot()
    LF_foot_id, LH_foot_id, RF_foot_id, RH_foot_id = robot.contact_frames()

    # step_length = np.array([0.15, 0, 0]) 
    step_length = np.array([0.0, 0.0, 0.0]) 
    yaw_cmd = np.pi / 8.

    swing_height = 0.1
    stance_time = 0.15
    flying_time = 0.10
    swing_start_time = 0.5

    v_com_cmd = step_length / (stance_time + flying_time)
    yaw_rate_cmd = yaw_cmd / (stance_time + flying_time)

    T = 0.5
    N = 18
    nthreads = 4
    mpc = robotoc.MPCFlyingTrot(robot, T, N, nthreads)

    planner = robotoc.FlyingTrotFootStepPlanner(robot)
    planner.set_gait_pattern(step_length, yaw_cmd)
    mpc.set_gait_pattern(planner, swing_height, flying_time, stance_time, swing_start_time)

    q = np.array([0, 0, 0.3181, 0, 0, 0, 1, 
                  0.0,  0.67, -1.3, 
                  0.0,  0.67, -1.3, 
                  0.0,  0.67, -1.3, 
                  0.0,  0.67, -1.3])
    v = np.zeros(robot.dimv())
    t = 0.0
    option_init = robotoc.SolverOptions()
    option_init.max_iter = 10

    mpc.init(t, q, v, option_init)
    option_mpc = robotoc.SolverOptions()
    option_mpc.max_iter = 2 # MPC iterations
    mpc.set_solver_options(option_mpc)

    return mpc, planner


def create_mpc_jump():
    jump_type = 'longitudinal'
    # jump_type = 'lateral'
    # jump_type = 'back'
    # jump_type = 'rotational'

    if jump_type == 'longitudinal':
        # jump_length = [0.5, 0, 0]
        jump_length = [0.4, 0, 0]
        jump_yaw = 0
    elif jump_type == 'lateral':
        jump_length = [0, 0.4, 0]
        jump_yaw = 0
    elif jump_type == 'back':
        jump_length = [-0.3, 0, 0]
        jump_yaw = 0
    elif jump_type == 'rotational':
        jump_length = [0.1, 0.0, 0]
        jump_yaw = np.pi / 6

    robot = create_robot()
    LF_foot_id, LH_foot_id, RF_foot_id, RH_foot_id = robot.contact_frames()

    T = 0.8
    N = 18
    nthreads = 4
    mpc = robotoc.MPCJump(robot, T, N, nthreads)

    planner = robotoc.JumpFootStepPlanner(robot)
    planner.set_jump_pattern(jump_length, jump_yaw)
    mpc.set_jump_pattern(planner, flying_time=0.3, min_flying_time=0.2, 
                        ground_time=0.3, min_ground_time=0.2)

    q = np.array([0, 0, 0.3181, 0, 0, 0, 1, 
                0.0,  0.67, -1.3, 
                0.0,  0.67, -1.3, 
                0.0,  0.67, -1.3, 
                0.0,  0.67, -1.3])
    v = np.zeros(robot.dimv())
    t = 0.0
    option_init = robotoc.SolverOptions()
    option_init.max_iter = 200
    option_init.initial_sto_reg_iter = 50
    mpc.init(t, q, v, option_init, sto=True)  

    option_mpc = robotoc.SolverOptions()
    option_mpc.max_iter = 2 # MPC iterations
    option_mpc.initial_sto_reg_iter = 0
    option_mpc.max_dt_mesh = T / N
    mpc.set_solver_options(option_mpc)

    return mpc, planner