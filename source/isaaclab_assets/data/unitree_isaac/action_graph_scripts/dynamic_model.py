import omni.graph.core as og
import numpy as np
from scipy.spatial.transform import Rotation as R

class RobotPose():
    def __init__(self, x, y, theta, dt=0.1):
        self.x = x
        self.y = y
        self.theta = theta

        self.dt = dt

    def update_pose(self, vx, vy ,vw):
        vx = self.velocity_constraint(vx, 0.0)
        vy = self.velocity_constraint(vy, 0.0)
        vw = self.velocity_constraint(vw, 0.11)

        dth = vw*self.dt
        dx = vx*self.dt*np.cos(dth/2) - vy*self.dt*np.sin(dth/2)
        dy = vx*self.dt*np.sin(dth/2) + vy*self.dt*np.cos(dth/2)

        self.theta += dth
        self.x += dx*np.cos(self.theta) - dy*np.sin(self.theta)
        self.y += dx*np.sin(self.theta) + dy*np.cos(self.theta)

    def velocity_constraint(self, velocity, velocity_out):
        if velocity < 0.1 and velocity > 0.0:
            velocity = velocity_out
        
        if velocity > -0.1 and velocity < 0.0:
            velocity = -velocity_out

        return velocity

def setup(db: og.Database):
    pass

def cleanup(db: og.Database):
    pass

def compute(db: og.Database):
    # Get input 
    orientation = np.array(db.inputs.robot_quaternion) # quatd[4] (x, y, z, w)
    position = np.array(db.inputs.robot_position) # pointd[3] (x, y, z)
    dt = np.array(db.inputs.dt) #double
    
    # Calculate new pose
    linear_velocity = np.array(db.inputs.linear_velocity) # double[3]
    angular_velocity = np.array(db.inputs.angular_velocity) # double[3]

    quat = R.from_quat(orientation[:4])
    theta = quat.as_euler('xyz')[2]

    robot_pose = RobotPose(position[0], position[1], theta, dt)
    robot_pose.update_pose(
        vx=linear_velocity[0],
        vy=linear_velocity[1],
        vw=angular_velocity[2]
    )

    new_theta = R.from_euler('z', robot_pose.theta)
    out_theta = new_theta.as_quat()

    # Set output
    db.outputs.robot_position = [robot_pose.x, robot_pose.y, position[2]] # pointd[3] (x, y, z)
    db.outputs.robot_orientation = [out_theta[0], out_theta[1], out_theta[2], out_theta[3]] # quatd[4] (x, y, z, w)

    return True
