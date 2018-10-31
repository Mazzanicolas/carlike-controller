#
# (c) PySimiam Team 2014
#
# Contact person: John Witten <jon.witten@gmail.com>
#
# This class was implemented as a weekly programming excercise
# of the 'Control of Mobile Robots' course by Magnus Egerstedt.
#
from supervisor import Supervisor
from helpers import Struct
from pose import Pose
from math import pi, sin, cos, log1p, atan2
from simobject import Path
import numpy

class QuickBotSupervisor(Supervisor):
    """The QuickBotSupervisor inherits from the superclass 'supervisor.Supervisor'
       to implement detailed calculations for any inheriting QuickBot supervisor.
       Students are intended to inherit from this class when making their own supervisors.
       An example of implementation is the :class:`~quickbotdefaultsupervisor.QuickBotDefaultSupervisor` class
       in which this class is used to reduce noisy code interactions.

       Most importantly, the QuickBotSupervisor object implements the system functions
       necessary to operate a QuickBot, namely the uni2diff unicycle to differential
       motion model conversion, the Jacobian problem, and any other computationally complex interface.

       The UI may use the get_parameters function interface to create docker windows
       for real-time update of the PID parameters. This is an advanced implementation
       and is not required for students to properly implement their own supervisors."""

    ir_coeff = numpy.array([  1.16931064e+07,  -1.49425626e+07,
                              7.96904053e+06,  -2.28884314e+06,
                              3.80068213e+05,  -3.64435691e+04,
                              1.89558821e+03])

    def __init__(self, robot_pose, robot_info):
        """Initialize internal variables"""
        Supervisor.__init__(self, robot_pose, robot_info)

        # initialize memory registers
        self.left_ticks  = robot_info.wheels.left_ticks
        self.right_ticks = robot_info.wheels.right_ticks

        # Let's say the robot is that big:
        self.robot_size = robot_info.wheels.base_length

    def init_default_parameters(self):
        """Sets the default PID parameters, goal, and velocity"""
        p = Struct()
        p.goal = Struct()
        p.goal.x = 1.0
        p.goal.y = 1.0
        p.velocity = Struct()
        p.velocity.v = 0.2
        p.gains = Struct()
        p.gains.kp = 10.0
        p.gains.ki = 2.0
        p.gains.kd = 0.0

        self.parameters = p

    def get_ui_description(self,p = None):
        """Returns the UI description for the docker"""
        if p is None:
            p = self.parameters

        return [('goal', [('x',p.goal.x), ('y',p.goal.y)]),
                ('velocity', [('v',p.velocity.v)]),
                (('gains',"PID gains"), [
                    (('kp','Proportional gain'), p.gains.kp),
                    (('ki','Integral gain'), p.gains.ki),
                    (('kd','Differential gain'), p.gains.kd)])]

    def set_parameters(self,params):
        """Set param structure from docker"""
        self.parameters.goal = params.goal
        self.parameters.velocity = params.velocity
        self.parameters.gains = params.gains

    def _uni2diff(self,uni):
        """Convert between unicycle model to differential model"""
        (v,w) = uni

        summ = 2*v/self.robot.wheels.radius
        diff = self.robot.wheels.base_length*w/self.robot.wheels.radius

        vl = (summ-diff)/2
        vr = (summ+diff)/2

        return (vl,vr)

    def car2diff(self,uni):

        (v,w) = uni

        summ = 2*v/self.robot.wheels.radius
        diff = self.robot.wheels.base_length*w/self.robot.wheels.radius

        vl = (summ-diff)/2
        vr = (summ+diff)/2
        x, y, theta = self.robot.robot_pose.get_pose()

        distances = self.robot.robot_pose.get_info().ir_sensors.readings

        angle = atan2(self.parameters.goal.y - y, self.parameters.goal.x - x) - theta

        for distance in distances:
            if distance < 100:
                angle = pi/4

        return v, angle

    def get_ir_distances(self):
        """Converts the IR distance readings into a distance in meters"""

        return numpy.polyval(self.ir_coeff, self.robot.ir_sensors.readings)

    def estimate_pose(self):
        """Update self.pose_est using odometry"""

        # Get tick updates
        dtl = self.robot.wheels.left_ticks - self.left_ticks
        dtr = self.robot.wheels.right_ticks - self.right_ticks

        # Save the wheel encoder ticks for the next estimate
        self.left_ticks += dtl
        self.right_ticks += dtr

        x, y, theta = self.pose_est

        R = self.robot.wheels.radius
        L = self.robot.wheels.base_length
        m_per_tick = (2*pi*R)/self.robot.wheels.ticks_per_rev

        # distance travelled by left wheel
        dl = dtl*m_per_tick
        # distance travelled by right wheel
        dr = dtr*m_per_tick

        theta_dt = (dr-dl)/L
        theta_mid = theta + theta_dt/2
        dst = (dr+dl)/2
        x_dt = dst*cos(theta_mid)
        y_dt = dst*sin(theta_mid)

        theta_new = theta + theta_dt
        x_new = x + x_dt
        y_new = y + y_dt

        return Pose(x_new, y_new, (theta_new + pi)%(2*pi)-pi)

    def get_controller_state(self):
        return self.parameters

    def execute(self, robot_info, dt):
        """Inherit default supervisor procedures and return unicycle model output (x, y, theta)"""
        output = Supervisor.execute(self, robot_info, dt)
        return self.car2diff(output)

    def draw_background(self, renderer):
        """Draw a circular goal"""
        renderer.set_pose(Pose(self.parameters.goal.x, self.parameters.goal.y))
        renderer.set_pen(0)
        renderer.set_brush(self.robot_color)
        for r in numpy.arange(self.robot_size/2,0,-0.01):
            renderer.draw_ellipse(0,0,r,r)