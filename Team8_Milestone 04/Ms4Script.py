import math
import numpy as np
import sympy as sp
import time

q1, q2, q3 = sp.symbols('q1 q2 q3')
L1, L2, L3 = 0.0404, 0.12, 0.214  # Link lengths as constants
desired_position = [-0.00574921, -0.001185442, 0.037296507]
#desired_position = [-0.00574921, -0.0026541311, 0.005901091497]
r = -0.00574921

def sysCall_init():
    sim = require('sim')

    self.n = 3  # Number of joints
    self.q1 = 130
    self.q2 = 0
    self.q3 = 90

    self.is_once = True

    # Getting joint objects and storing them in a dictionary
    self.joints = {}
    for i in range(1, self.n + 1):
        self.joints[i] = sim.getObject(f"/q{i}")  # Get joint handles

    # DH Parameters for each joint
    self.DH_params = {
        'q1_q2': {'theta': self.q1, 'd': 0.0404, 'r': 0, 'alpha': 90},
        'q2_q3': {'theta': -90+self.q2, 'd': 0.0, 'r': -0.12, 'alpha': 180},
        'q3_end_effector': {'theta': 90+self.q3, 'd':0, 'r': 0.214, 'alpha': 0},
    }

    # Prepare list of transformation matrices parameters
    self.transformation_params = [[params['theta'], params['d'], params['r'], params['alpha']] 
    for params in self.DH_params.values()]

    # Set target positions (in radians) for each joint
    self.target_positions = [self.q1 * (math.pi / 180) , self.q2 * (math.pi / 180), self.q3 * (math.pi / 180)]
    self.required_duration = 20  # Duration over which to move the joints
    self.start_time = sim.getSimulationTime()  # Store the start time of the simulation

    # Set the initial joint positions
    for i in range(1, self.n + 1):
        sim.setJointPosition(self.joints[i], 0)  # Set initial position to 0 radians

    EE_handle = sim.getObject("/ReferenceFrameEE")
    base_handle = sim.getObject("/q1")
    initial_position = sim.getObjectPosition(EE_handle, base_handle)
    
    # Generate trajectory data
    self.trajectory_data = CircularPathTrajectory(initial_position, desired_position)
    #self.trajectory_data = aLinearPathTrajectory (initial_position, desired_position)




def transformation_func(paramth,paramd,parama,paramalpha):
    # Extract theta, d, a, alpha from params
   # th, d, a, alpha = 0.0, 0.0, 0.0, 0.0
    th = paramth  # Z-axis orientation
    d = paramd       # Z-axis displacement

    # Assign the remaining parameters
    a = parama          # X-axis displacement
    alpha = paramalpha       # X-axis orientation
    

    # Compute cosines and sines for theta and alpha
    cth = np.cos(np.radians(th))
    sth = np.sin(np.radians(th))
    calpha = np.cos(np.radians(alpha))
    salpha = np.sin(np.radians(alpha))
    
    cth = np.round(cth, decimals=3)
    sth = np.round(sth, decimals=3)
    calpha = np.round(calpha, decimals=3)
    salpha = np.round(salpha, decimals=3)
    

    # Compute the transformation matrix
    A = np.array([
        [cth, -sth * calpha,  sth * salpha, a * cth],
        [sth,  cth * calpha, -cth * salpha, a * sth],
        [0,    salpha,        calpha,        d],
        [0,    0,             0,             1]
    ])
    
    A = np.round(A, decimals=3)

    return A
    
def getAi(joints, DH_params):
    Ai = []  # Initialize empty list for transformation matrices

    # Iterate over the joints
    for i in range(len(joints) - 1):
        Ai.append(transformation_func(DH_params[i][0],DH_params[i][1],DH_params[i][2],DH_params[i][3]))  # Compute DH for each joint and store in Ai

    return Ai  # Return the list of transformation matrices

def forward_kinematics_func():
    Ai = getAi(self.joints, self.transformation_params)
    i = 0
    result = Ai[0]
    for list in Ai[1:]:
        result = np.dot(result,list)
    return result


def sysCall_actuation():
    sim = require('sim')
    current_time = sim.getSimulationTime() - self.start_time  # Calculate elapsed time
    time_step = 0.1  # Time step of the trajectory

    if current_time <= self.required_duration:
        # Find the current trajectory point based on elapsed time
        index = int(current_time / time_step)
        if index < len(self.trajectory_data):
            t, x, y, z, q1, q2, q3 = self.trajectory_data[index]
            
            # Set the joint target positions based on inverse kinematics
            sim.setJointTargetPosition(self.joints[1], q1 * (math.pi / 180))
            sim.setJointTargetPosition(self.joints[2], q2 * (math.pi / 180))
            sim.setJointTargetPosition(self.joints[3], q3 * (math.pi / 180))
            
            # Print the current position and joint angles
            print(f"Time: {t:.1f} s")
            print(f"Position -> x: {x:.3f}, y: {y:.3f}, z: {z:.3f}")
            print(f"Joint Angles -> q1: {q1:.3f}, q2: {q2:.3f}, q3: {q3:.3f}")
               
                    
def CircularPathTrajectory(initial_position, final_position):
    xc = (initial_position[0] + final_position[0]) / 2
    yc = (initial_position[1] + final_position[1]) / 2
    zc = initial_position[2]  # Assuming z remains constant
    
    radius = np.sqrt((xc - initial_position[0])**2 + (yc - initial_position[1])**2)
    
    # Define time samples
    t_samples = np.arange(0, 10.1, 0.1)
    trajectory = []
    
    for t in t_samples:
        x_t = xc + radius * np.cos(0.2 * np.pi * t)
        y_t = yc + radius * np.sin(0.2 * np.pi * t)
        z_t = zc  # Assuming z remains constant
        
        q1, q2, q3 = Inverse_Position([x_t, y_t, z_t])
        
        # Store the trajectory (t, x, y, z, q1, q2, q3) for each time sample
        trajectory.append((t, x_t, y_t, z_t, q1, q2, q3))
        
    return trajectory
    
def abLinearPathTrajectory(initial_position, desired_position):
    """
    Generates a linear trajectory from the initial position to the desired position.
    
    Parameters:
    initial_position (list or array): Initial position [x, y, z].
    desired_position (list or array): Desired position [x, y, z].
    duration (float): Duration of the trajectory in seconds.
    time_step (float): Time step of the trajectory in seconds.
    
    Returns:
    trajectory_data (list): Trajectory data containing time and positions.
    """
    duration =10
    time_step=0.1
    trajectory_data = []
    num_steps = int(duration / time_step)
    
    # Linear interpolation for each coordinate
    x_vals = np.linspace(initial_position[0], desired_position[0], num_steps)
    y_vals = np.linspace(initial_position[1], desired_position[1], num_steps)
    z_vals = np.linspace(initial_position[2], desired_position[2], num_steps)
    
    for i in range(num_steps):
        t = i * time_step
        x = x_vals[i]
        y = y_vals[i]
        z = z_vals[i]
        
        # Solve inverse kinematics to get joint angles for each position
        joint_angles = Inverse_Position([x, y, z])
        
        q1, q2, q3 = joint_angles
        trajectory_data.append((t, x, y, z, q1, q2, q3))
    
    return trajectory_data

def LinearPathTrajectory(initial_position, final_position):
    x0, y0, z0 = initial_position
    xf, yf, zf = final_position
    # Define time samples for forward and backward motion
    t_samples_forward = np.arange(0, 10.1, 0.1)
    t_samples_backward = np.arange(0, 10.1, 0.1)
    
    trajectory = []
    
    # Forward trajectory
    for t in t_samples_forward:
        # Linear interpolation for x, y, z
        x_t = x0 + (xf - x0) * (t / 10)
        y_t = y0 + (yf - y0) * (t / 10)
        z_t = z0 + (zf - z0) * (t / 10)
        
        q1, q2, q3 = Inverse_Position([x_t, y_t, z_t])
        
        # Store the trajectory (t, x, y, z, q1, q2, q3) for each time sample
        trajectory.append((t, x_t, y_t, z_t, q1, q2, q3))
    
    # Backward trajectory
    for t in t_samples_backward:
        # Linear interpolation for x, y, z
        x_t = xf + (x0 - xf) * (t / 10)
        y_t = yf + (y0 - yf) * (t / 10)
        z_t = zf + (z0 - zf) * (t / 10)
        
        q1, q2, q3 = Inverse_Position([x_t, y_t, z_t])
        
        # Store the trajectory (t, x, y, z, q1, q2, q3) for each time sample
        trajectory.append((t + 10, x_t, y_t, z_t, q1, q2, q3))  # Adjust time for backward motion
    
    return trajectory

    
def aLinearPathTrajectory(initial_position, final_position):
    x0, y0, z0 = initial_position
    xf, yf, zf = final_position
    # Define time samples
    t_samples = np.arange(0, 10.1, 0.1)
    trajectory = []
    
    for t in t_samples:
        # Linear interpolation for x, y, z
        x_t = x0 + (xf - x0) * (t / 10)
        y_t = y0 + (yf - y0) * (t / 10)
        z_t = z0 + (zf - z0) * (t / 10)
        
        q1, q2, q3 = Inverse_Position([x_t, y_t, z_t])
        
        # Store the trajectory (t, x, y, z, q1, q2, q3) for each time sample
        trajectory.append((t, x_t, y_t, z_t, q1, q2, q3))
        
    return trajectory
    
    

def Inverse_Position(matrix):
    d = 0.0404
    r2 = 0.12
    r3 = 0.214
    
    # Assuming matrix provides x, y, z
    x, y, z = matrix
    
    # Calculate q1
    q1 = math.degrees(math.atan2(y, x))  # atan2 provides correct quadrant
    
    # Calculate intermediate values
    s = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
    r = z - d
    w = math.sqrt(math.pow(s, 2) + math.pow(r, 2))
    
    # Calculate q3
    nomQ3 = math.pow(s, 2) + math.pow(r, 2) - math.pow(r2, 2) - math.pow(r3, 2)
    demQ3 = 2 * r2 * r3
    angle_cosine = nomQ3 / demQ3
    angle_sine = math.sqrt(1 - math.pow(angle_cosine, 2))
    q3 = math.degrees(math.atan2(angle_sine, angle_cosine))
    
    # Calculate alpha
    alpha = math.degrees(math.atan2(r, s))  # atan2 ensures correct quadrant for alpha
    
    # Calculate q5 using atan2 for correct quadrant
    nomQ5 = r3 * np.sin(np.radians(q3))
    demQ5 = w
    q5 = math.degrees(math.atan2(nomQ5, math.sqrt(demQ5**2 - nomQ5**2)))  # atan2 handles the sign
    
    # Adjust q2 for the correct quadrant
    q2 = alpha - q5
    
    # Print and return results
    print(q1, q2, q3)
    return q1, q2, q3

    
    
# Forward Kinematics - Define the end effector position symbolically
def forward_kinematics_funcSP():
    # Define the forward kinematics expressions for X, Y, Z based on the 4-DOF arm
    # Assuming planar arm with movement in XY plane only, with Z = 0
    X = -L2 * sp.sin(q1) * sp.sin(q2) + L3 * (sp.sin(q1) * sp.sin(q2) * sp.sin(q3) + sp.sin(q1) * sp.cos(q2) * sp.cos(q3) + sp.cos(q1) * sp.cos(q3))
    Y = L2 * sp.cos(q1) * sp.sin(q2) + L3 * (-sp.cos(q1) * sp.sin(q2) * sp.sin(q3) - sp.cos(q1) * sp.cos(q2) * sp.cos(q3) + sp.sin(q1) * sp.cos(q3))
    Z = L1 + L2 * sp.cos(q2) + L3 * (sp.cos(q2) * sp.sin(q3) - sp.sin(q2) * sp.cos(q3))
    return X, Y, Z
    
# Jacobian Matrix Calculation
def jacobian_matrix():
    # Get symbolic expressions for the end effector's position
    X, Y, Z = forward_kinematics_funcSP()
    
    # Compute partial derivatives for the Jacobian with respect to each joint angle
    J11 = sp.diff(X, q1)
    J12 = sp.diff(X, q2)
    J13 = sp.diff(X, q3)
    
    J21 = sp.diff(Y, q1)
    J22 = sp.diff(Y, q2)
    J23 = sp.diff(Y, q3)
    
    J31 = sp.diff(Z, q1)
    J32 = sp.diff(Z, q2)
    J33 = sp.diff(Z, q3)
    
    # Construct the Jacobian matrix
    J = sp.Matrix([
        [J11, J12, J13],
        [J21, J22, J23],
        [J31, J32, J33]
    ])
    
    return J

# Forward Velocity Kinematics
def forward_velocity_kinematics(q, q_dot):
    # Obtain the Jacobian matrix
    J = jacobian_matrix()
    
    # Substitute joint angles into Jacobian if needed
    J_num = J.subs({q1: q[0], q2: q[1], q3: q[2]})

    
    # Convert J_num to a numpy array for numerical computation
    J_np = np.array(J_num).astype(np.float64)
    
    # Calculate forward velocity (V_F = J * q_dot)
    V_F = J_np @ np.array(q_dot)
    
    return V_F

def inverse_jacobian_matrix(q):
    # Obtain the Jacobian matrix
    J = jacobian_matrix()
    
    # Substitute joint angles into Jacobian if needed
    J_num = J.subs({q1: q[0], q2: q[1], q3: q[2]})
    
    # Convert J_num to a numpy array for numerical computation
    J_np = np.array(J_num).astype(np.float64)
    
    # Calculate the inverse of the Jacobian matrix
    J_inv = np.linalg.inv(J_np)
    
    return J_inv

def inverse_kinematics_func(q0, X_d):
    """
    Computes the joint velocities (q_dot) given the desired end-effector velocity (X_d).
    
    Parameters:
    q0 (list or array): Initial joint angles [q1, q2, q3].
    X_d (list or array): Desired end-effector velocities [Vx, Vy, Vz].
    
    Returns:
    q_dot (array): Joint velocities [q1_dot, q2_dot, q3_dot].
    """
    # Initial guess
    q = np.array(q0, dtype=np.float64)
    
    # Obtain the inverse Jacobian matrix
    J_inv = inverse_jacobian_matrix(q)
    
    # Convert X_d to a numpy array for computation
    X_d_np = np.array(X_d, dtype=np.float64)
    
    # Compute joint velocities (q_dot = J_inv * X_d)
    q_dot = J_inv @ X_d_np
    
    return q_dot

def sysCall_cleanup():
    EE_handle = sim.getObject("/ReferenceFrameEE")
    base_handle = sim.getObject("/q1")
    EE_position = sim.getObjectPosition(EE_handle, base_handle)
    
    trajectory_data = CircularPathTrajectory(EE_position, desired_position)
    #trajectory_data = LinearPathTrajectory(EE_position, desired_position)
    
    for data in trajectory_data:
        t, x, y, z, q1, q2, q3 = data
        print(f"t = {t:.1f}, x = {x:.3f}, y = {y:.3f}, z = {z:.3f}, q1 = {q1:.3f}, q2 = {q2:.3f}, q3 = {q3:.3f}")
    
    for i in range(1, self.n + 1):
        self.is_once = False
        
        sim.setJointTargetPosition(self.joints[i], self.target_positions[i - 1])
        if i == self.n:  # Last joint iteration
            # Calculate the DH transformation
            end_effector_position = forward_kinematics_func()
            print("End-Effector Position after final actuation:", end_effector_position)

            
            
            print("Rasputin",EE_position)
            
            q1, q2, q3 = Inverse_Position(EE_position)
            
            V_I = inverse_kinematics_func([q1,q2,q3],EE_position)
            
            print(V_I)
            
            V_F = forward_velocity_kinematics([q1,q2,q3] , V_I)
            
            print(V_F)
    # Clean-up finished
    pass    


def calculateDistances():
    # Get handles for the objects
    Q1_handle = sim.getObject('/q1')
    Q2_handle = sim.getObject('/q2')
    Q3_handle = sim.getObject('/q3')
    ReferenceFrameEE_handle = sim.getObject('/ReferenceFrameEE')

    # Get positions of all objects relative to the world frame
    Q1_position = sim.getObjectPosition(Q1_handle, -1)
    Q2_position = sim.getObjectPosition(Q2_handle, Q1_handle)
    Q3_position = sim.getObjectPosition(Q3_handle, Q2_handle)
    ReferenceFrameEE_position = sim.getObjectPosition(ReferenceFrameEE_handle, Q3_handle)

    # Compute translations
    translationX_Q1_Q2 = Q2_position[0] - Q1_position[0]
    translationZ_Q1_Q2 = Q2_position[2] - Q1_position[2]

    translationX_Q3_Q2 = Q3_position[0] - Q2_position[0]
    translationZ_Q3_Q2 = Q3_position[2] - Q2_position[2]

    translationX_ReferenceFrameEE_Q3 = ReferenceFrameEE_position[0] - Q3_position[0]
    translationZ_ReferenceFrameEE_Q3 = ReferenceFrameEE_position[2] - Q3_position[2]

    # Compute distances
    distance1 = math.sqrt(sum((Q2_position[i] - Q1_position[i]) ** 2 for i in range(3)))
    distance2 = math.sqrt(sum((Q3_position[i] - Q2_position[i]) ** 2 for i in range(3)))
    distance3 = math.sqrt(sum((ReferenceFrameEE_position[i] - Q3_position[i]) ** 2 for i in range(3)))

    # Print results
    print(f"Distance 1 (Q1 -> Q2): {distance1}")
    print(f"Distance 2 (Q2 -> Q3): {distance2}")
    print(f"Distance 3 (Q3 -> ReferenceFrameEE): {distance3}")
    print("####################")

    return distance1, distance2, distance3
    