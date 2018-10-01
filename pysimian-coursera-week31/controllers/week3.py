#
# (c) PySimiam Team 2013
# 
# Contact person: Tim Fuchs <typograph@elec.ru>
#
# This class was implemented for the weekly programming excercises
# of the 'Control of Mobile Robots' course by Magnus Egerstedt.
#
from controller import Controller
import math
import numpy as np

class GoToGoal(Controller):
    """Go-to-goal steers the robot to a predefined position in the world."""
    def __init__(self, params):
        '''Initialize some variables'''
        
        Controller.__init__(self,params)
        self.heading_angle = 0

    def set_parameters(self, params):
        """Set PID values
        
        The params structure is expected to have in the `gains` field three
        parameters for the PID gains.
        
        :param params.gains.kp: Proportional gain
        :type params.gains.kp: float
        :param params.gains.ki: Integral gain
        :type params.gains.ki: float
        :param params.gains.kd: Differential gain
        :type params.gains.kd: float
        """
        self.kp = params.gains.kp
        self.ki = params.gains.ki
        self.kd = params.gains.kd

    def restart(self):
        #Week 3 Assignment Code:
        #Place any variables you would like to store here
        #You may use these variables for convenience
        self.E_k = 0 # Integrated error
        self.e_k_1 = 0 # Previous error calculation
        self.E = 0
        self.error_1 = 0
        
        #End Week 3 Assigment

    def get_heading_angle(self, state, t):
        """Get the heading angle in the world frame of reference."""
        
        #Insert Week 3 Assignment Code Here
        # Here is an example of how to get goal position
        # and robot pose data. Feel free to name them differently.
        
        # x_g, y_g = state.goal.x, state.goal.y

        return 0
        #End Week 3 Assigment        
    def execute(self, state, dt, t):
        """Calculate errors and steer the robot"""
     
        # This is the direction we want to go
        self.heading_angle = self.get_heading_angle(state, t)
        largo_del_auto = 0.5
        d = largo_del_auto
        x_r, y_r, theta = state.pose
        M  = np.array([np.cos(theta), -d*np.sin(theta), np.sin(theta), d*np.cos(theta)]).reshape(2,2)
        Mp = np.linalg.inv(M)
        xp = np.sin(t)
        yp = np.cos(t)
        X  = np.array([xp, yp]).reshape(-1,1)
        v, w = Mp.dot(X)

        # 1. Calculate simple proportional error
        # The direction is in the robot's frame of reference,
        # so the error is the direction.
        # Note that the error is automatically between pi and -pi.
        error = self.heading_angle

        # 2. Calculate integral error
        self.E += error*dt
        self.E = (self.E + math.pi)%(2*math.pi) - math.pi

        # 3. Calculate differential error
        dE = (error - self.error_1)/dt
        self.error_1 = error #updates the error_1 var

        # 4. Calculate desired omega
        w_ = w#self.kp*error + self.ki*self.E + self.kd*dE
        
        # The linear velocity is given to us:
        v_ = v#state.velocity.v

        return [v_, w_]