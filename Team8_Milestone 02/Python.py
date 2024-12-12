import math
import numpy as np

def sysCall_init():
    sim = require('sim')

    self.n = 5  # Number of joints
    self.q = 90
    self.is_once = True

    # Getting joint objects and storing them in a dictionary
    self.joints = {}
    for i in range(1, self.n + 1):
        self.joints[i] = sim.getObject(f"/q{i}")  # Get joint handles

    # DH Parameters for each joint
    self.DH_params = {
        'q1_q2': {'theta': 90+self.q, 'd': 0.0404, 'r': 0.0139, 'alpha': 90},
        'q2_q3': {'theta': 90+self.q, 'd': 0.0, 'r': 0.12, 'alpha': 0.0},
        'q3_q4': {'theta': 180+self.q, 'd':0.0134, 'r': 0.0051, 'alpha': 90},
        'q4_q5': {'theta': 0.0+self.q, 'd': 0.0558 , 'r': 0.005, 'alpha': 90},
        'q5_endEffector': {'theta': 90.0+self.q, 'd': 0.0, 'r': 0.0951, 'alpha': 90}
    }

    # Prepare list of transformation matrices parameters
    self.transformation_params = [[params['theta'], params['d'], params['r'], params['alpha']] for params in self.DH_params.values()]

    # Set target positions (in radians) for each joint
    self.target_positions = [90 * (math.pi / 180)] * self.n  # Convert degrees to radians for each joint
    self.required_duration = 5  # Duration over which to move the joints
    self.start_time = sim.getSimulationTime()  # Store the start time of the simulation

    # Set the initial joint positions
    for i in range(1, self.n + 1):
        sim.setJointPosition(self.joints[i], 0)  # Set initial position to 0 radians

    print("Initialization complete: Joints initialized with starting positions.")


def transformation_func(paramth,paramd,parama,paramalpha):
    # Extract theta, d, a, alpha from params
    th, d, a, alpha = 0.0, 0.0, 0.0, 0.0
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
    for i in range(len(joints)):
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
    curr_time = sim.getSimulationTime() - self.start_time
    if curr_time < self.required_duration:
        # Move all joints smoothly to target positions
        for i in range(1, self.n + 1):
            new_position = (curr_time / self.required_duration) * self.target_positions[i - 1]
            sim.setJointTargetPosition(self.joints[i], new_position)
    else:
        # Set final positions if the required duration has passed
        for i in range(1, self.n + 1):
            sim.setJointTargetPosition(self.joints[i], self.target_positions[i - 1])
            if i == self.n:  # Last joint iteration
                # Calculate the DH transformation
                end_effector_position = forward_kinematics_func()
                print("End-Effector Position after final actuation:", end_effector_position)
                
            if self.is_once:
                self.is_once = False
                EE_handle = sim.getObject("/ReferenceFrameEE")
                base_handle = sim.getObject("/ReferenceFrame")
                EE_position = sim.getObjectPosition(EE_handle, base_handle)
                print("positioon1",EE_position)
                


    # Calculate the final transformation using DH parameters
    transformation_result = forward_kinematics_func()
                
      #  box = sim.getObject("/softBody")
      #  sim.setObjectPosition(box, EE_handle,?last_column)

    
def sysCall_cleanup():
    # Get the handles for the end-effector and base reference frames
    EE_handle = sim.getObject("/ReferenceFrameEE")
    base_handle = sim.getObject("/ReferenceFrame")


    # Calculate the final transformation using DH parameters
    transformation_result = forward_kinematics_func()
    transformation_result= np.round(transformation_result,decimals= 2)
    # Extract the final position from the transformation matrix
    print("DH Calculated End-Effector Position:", transformation_result)

 
 
    # Optionally calculate distances between joints
    calculateDistances()

    # Clean-up finished
    pass    

def calculateDistances():
    self.Q1_handle = sim.getObject("/q1")
    self.Q1_position = sim.getObjectPosition(self.Q1_handle)

    self.Q2_handle = sim.getObject("/q2")
    self.Q2_position = sim.getObjectPosition(self.Q2_handle)

    self.translationX_Q1_Q2 = self.Q2_position[0] - self.Q1_position[0]
    self.translationZ_Q1_Q2 = self.Q2_position[2] - self.Q1_position[2]

    self.distance1 = math.sqrt(
        (self.Q2_position[0] - self.Q1_position[0]) ** 2 +
        (self.Q2_position[1] - self.Q1_position[1]) ** 2 +
        (self.Q2_position[2] - self.Q1_position[2]) ** 2)

    self.Q3_handle = sim.getObject("/q3")
    self.Q3_position = sim.getObjectPosition(self.Q3_handle)

    self.translationX_Q3_Q2 = self.Q3_position[0] - self.Q2_position[0]
    self.translationZ_Q3_Q2 = self.Q3_position[2] - self.Q2_position[2]
    self.translationX_Q3_Q2 = np.round(self.translationX_Q3_Q2, decimals=4)

    self.distance2 = math.sqrt(
        (-self.Q2_position[0] + self.Q3_position[0]) ** 2 +
        (-self.Q2_position[1] + self.Q3_position[1]) ** 2 +
        (-self.Q2_position[2] + self.Q3_position[2]) ** 2)

    self.Q4_handle = sim.getObject("/q4")
    self.Q4_position = sim.getObjectPosition(self.Q4_handle)
    
    self.translationX_Q4_Q3 = self.Q4_position[0] - self.Q3_position[0]
    self.translationZ_Q4_Q3 = self.Q4_position[2] - self.Q3_position[2]

    self.distance3 = math.sqrt(
        (self.Q4_position[0] - self.Q3_position[0]) ** 2 +
        (self.Q4_position[1] - self.Q3_position[1]) ** 2 +
        (self.Q4_position[2] - self.Q3_position[2]) ** 2)
    
    self.Q5_handle = sim.getObject("/q5")
    self.Q5_position = sim.getObjectPosition(self.Q5_handle)
    self.translationX_Q5_Q4 = self.Q5_position[0] - self.Q4_position[0]
    self.translationZ_Q5_Q4 = self.Q5_position[2] - self.Q4_position[2]

    self.distance3 = math.sqrt(
        (self.Q5_position[0] - self.Q4_position[0]) ** 2 +
        (self.Q5_position[1] - self.Q4_position[1]) ** 2 +
        (self.Q5_position[2] - self.Q4_position[2]) ** 2)

    self.ReferenceFrameEE_handle = sim.getObject("/ReferenceFrameEE")
    self.ReferenceFrameEE_position = sim.getObjectPosition(self.ReferenceFrameEE_handle)

    self.translationX_ReferenceFrameEE_Q5 = self.ReferenceFrameEE_position[0] - self.Q5_position[0]
    self.translationz_ReferenceFrameEE_Q5 = self.ReferenceFrameEE_position[2] - self.Q5_position[2]

    self.distance5 = math.sqrt(
        (self.ReferenceFrameEE_position[0] - self.Q5_position[0]) ** 2 +
        (self.ReferenceFrameEE_position[1] - self.Q5_position[1]) ** 2 +
        (self.ReferenceFrameEE_position[2] - self.Q5_position[2]) ** 2)
