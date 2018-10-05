#
# (c) PySimiam Team 2014
#
# Contact person: Tim Fuchs <typograph@elec.ru>
#
# This class was implemented as a weekly programming excercise
# of the 'Control of Mobile Robots' course by Magnus Egerstedt.
#
import numpy as np
from pose import Pose
from sensor import ProximitySensor
from robot import Robot
from math import ceil, exp, sin, cos, tan, pi, atan2
from helpers import Struct

class QuickBot_IRSensor(ProximitySensor):
    """Inherits from the proximity sensor class. Performs calculations specific to the khepera3 for its characterized proximity sensors"""
    
    ir_coeff = np.array([  8.56495710e-18,  -3.02930608e-14,   4.43025017e-11,
                          -3.49052288e-08,   1.61452174e-05,  -4.44025236e-03,
                           6.74137385e-01])
    
    def __init__(self,pose,robot):
        # values copied from SimIAm    
        ProximitySensor.__init__(self, pose, robot, (0.04, 0.3, np.radians(6)))

    def distance_to_value(self,dst):
        """Returns the distance calculation from the distance readings of the proximity sensors""" 
        
        if dst < self.rmin :
            return 917
        elif dst > self.rmax:
            return 133
        else:
            return np.polyval(self.ir_coeff,dst)

class QuickBot(Robot):
    """Inherts for the simobject--->robot class for behavior specific to the Khepera3""" 
    def __init__(self, pose, color = 0xFFFFFF):
        Robot.__init__(self, pose, color)
        self.size = 0.1
        self.wheels_angle = 0
        self.front_wheels_ang_vel = 0

        # create shape
        self._shapes = Struct()
        
        self._shapes.base_plate = np.array([[ 0.0335, 0.0534, 1],
                                            [ 0.0429, 0.0534, 1],
                                            [ 0.0639, 0.0334, 1],
                                            [ 0.0686, 0.0000, 1],
                                            [ 0.0639,-0.0334, 1],
                                            [ 0.0429,-0.0534, 1],
                                            [ 0.0335,-0.0534, 1],
                                            [-0.0465,-0.0534, 1],
                                            [-0.0815,-0.0534, 1],
                                            [-0.1112,-0.0387, 1],
                                            [-0.1112, 0.0387, 1],
                                            [-0.0815, 0.0534, 1],
                                            [-0.0465, 0.0534, 1]])
                         
        self._shapes.bbb = np.array([[-0.0914,-0.0406, 1],
                                    [-0.0944,-0.0376, 1],
                                    [-0.0944, 0.0376, 1],
                                    [-0.0914, 0.0406, 1],
                                    [-0.0429, 0.0406, 1],
                                    [-0.0399, 0.0376, 1],
                                    [-0.0399,-0.0376, 1],
                                    [-0.0429,-0.0406, 1]])
                    
        self._shapes.bbb_rail_l = np.array([[-0.0429, -0.0356,1],
                                            [-0.0429, 0.0233,1],
                                            [-0.0479, 0.0233,1],
                                            [-0.0479,-0.0356,1]])
                          
        self._shapes.bbb_rail_r = np.array([[-0.0914,-0.0356,1],
                                            [-0.0914, 0.0233,1],
                                            [-0.0864, 0.0233,1],
                                            [-0.0864,-0.0356,1]])
                          
        self._shapes.bbb_eth = np.array([[-0.0579, 0.0436, 1],
                                         [-0.0579, 0.0226, 1],
                                         [-0.0739, 0.0226, 1],
                                         [-0.0739, 0.0436, 1]])
                       
        self._shapes.left_wheel = np.array([[ 0.0254, 0.0595, 1],
                                            [ 0.0254, 0.0335, 1],
                                            [-0.0384, 0.0335, 1],
                                            [-0.0384, 0.0595, 1]])
                          
        self._shapes.left_wheel_ol = np.array([[ 0.0254, 0.0595, 1],
                                               [ 0.0254, 0.0335, 1],
                                               [-0.0384, 0.0335, 1],
                                               [-0.0384, 0.0595, 1]])
            
        self._shapes.right_wheel_ol = np.array([[ 0.0254,-0.0595, 1],
                                                [ 0.0254,-0.0335, 1],
                                                [-0.0384,-0.0335, 1],
                                                [-0.0384,-0.0595, 1]])
                         
        self._shapes.right_wheel = np.array([[ 0.0254,-0.0595, 1],
                                             [ 0.0254,-0.0335, 1],
                                             [-0.0384,-0.0335, 1],
                                             [-0.0384,-0.0595, 1]])
                         
        self._shapes.ir_1 = np.array([[-0.0732, 0.0534, 1],
                                      [-0.0732, 0.0634, 1],
                                      [-0.0432, 0.0634, 1],
                                      [-0.0432, 0.0534, 1]])
                    
        self._shapes.ir_2 = np.array([[ 0.0643, 0.0214, 1],
                                      [ 0.0714, 0.0285, 1],
                                      [ 0.0502, 0.0497, 1],
                                      [ 0.0431, 0.0426, 1]])
                    
        self._shapes.ir_3 = np.array([[ 0.0636,-0.0042, 1],
                                      [ 0.0636, 0.0258, 1],
                                      [ 0.0736, 0.0258, 1],
                                      [ 0.0736,-0.0042, 1]])
                    
        self._shapes.ir_4 = np.array([[ 0.0643,-0.0214, 1],
                                      [ 0.0714,-0.0285, 1],
                                      [ 0.0502,-0.0497, 1],
                                      [ 0.0431,-0.0426, 1]])
                    
        self._shapes.ir_5 = np.array([[-0.0732,-0.0534, 1],
                                      [-0.0732,-0.0634, 1],
                                      [-0.0432,-0.0634, 1],
                                      [-0.0432,-0.0534, 1]])

        self._shapes.bbb_usb = np.array([[-0.0824,-0.0418, 1],
                                         [-0.0694,-0.0418, 1],
                                         [-0.0694,-0.0278, 1],
                                         [-0.0824,-0.0278, 1]])
        
        # create IR sensors
        self.ir_sensors = []
              
        ir_sensor_poses = [
                          Pose(-0.0474, 0.0534, np.radians(90)),
                          Pose( 0.0613, 0.0244, np.radians(45)),
                          Pose( 0.0636, 0.0, np.radians(0)),
                          Pose( 0.0461,-0.0396, np.radians(-45)),
                          Pose(-0.0690,-0.0534, np.radians(-90))
                          ]                          
                           
        for pose in ir_sensor_poses:
            self.ir_sensors.append(QuickBot_IRSensor(pose,self))
                                
        # initialize motion
        self.ang_velocity = (0.0,0.0)

        self.info = Struct()
        self.info.robot_pose = self
        self.info.wheels = Struct()
        # these were the original parameters
        self.info.wheels.radius = 0.0325
        self.info.wheels.base_length = 0.09925
        self.info.wheels.ticks_per_rev = 16.0
        self.info.wheels.left_ticks = 0
        self.info.wheels.right_ticks = 0
        
        self.info.wheels.max_velocity = 8.377
        
        self.left_revolutions = 0.0
        self.right_revolutions = 0.0
        
        self.info.ir_sensors = Struct()
        self.info.ir_sensors.poses = ir_sensor_poses
        self.info.ir_sensors.readings = None
        self.info.ir_sensors.rmax = 0.2
        self.info.ir_sensors.rmin = 0.02

    def draw(self,r):
        r.set_pose(self.get_pose())
        r.set_pen(0)
        r.set_brush(0)
        r.draw_polygon(self._shapes.ir_1)
        r.draw_polygon(self._shapes.ir_2)
        r.draw_polygon(self._shapes.ir_3)
        r.draw_polygon(self._shapes.ir_4)
        r.draw_polygon(self._shapes.ir_5)
        
        r.draw_polygon(self._shapes.left_wheel)
        r.draw_polygon(self._shapes.right_wheel)
        
        r.set_pen(0x01000000)
        r.set_brush(self.get_color())
        r.draw_polygon(self._shapes.base_plate)

        r.set_pen(0x10000000)
        r.set_brush(None)
        r.draw_polygon(self._shapes.left_wheel)
        r.draw_polygon(self._shapes.right_wheel)        
        
        r.set_pen(None)
        r.set_brush(0x333333)
        r.draw_polygon(self._shapes.bbb)
        r.set_brush(0)
        r.draw_polygon(self._shapes.bbb_rail_l)
        r.draw_polygon(self._shapes.bbb_rail_r)
        r.set_brush(0xb2b2b2)
        r.draw_polygon(self._shapes.bbb_eth)
        r.draw_polygon(self._shapes.bbb_usb)
        
    def get_envelope(self):
        return self._shapes.base_plate
    
    def move(self,dt):
        # There's no need to use the integrator - these equations have a solution        
        #(vl, vr) = self.get_wheel_speeds()
        # self.wheels_angle += self.front_wheels_ang_vel * dt
        (v,w) = self.diff2car((1, self.wheels_angle))
        x, y, theta = self.get_pose()
        if w == 0:
            x += v*cos(theta)*dt
            y += v*sin(theta)*dt
        else:
            dtheta = w*dt
            x += self.xd(v, theta, dt)#2*v/w*cos(theta + dtheta/2)*sin(dtheta/2)
            y += self.yd(v, theta, dt)#2*v/w*sin(theta + dtheta/2)*sin(dtheta/2)
            theta += self.thetad(v, self.wheels_angle, dt) #dtheta
        
        self.set_pose(Pose(x, y, (theta + pi)%(2*pi) - pi))
        self.left_revolutions += dt/2/pi
        self.right_revolutions += dt/2/pi
        self.info.wheels.left_ticks  = int(self.left_revolutions*self.info.wheels.ticks_per_rev)
        self.info.wheels.right_ticks = int(self.right_revolutions*self.info.wheels.ticks_per_rev)
        
    def xd(self, u1, theta, dt):
        return u1*cos(theta)*dt
    
    def yd(self, u1, theta, dt):
        return u1*sin(theta)*dt

    def thetad(self, u1, fi, dt):
        return (u1/self.size)*tan(fi)*dt
    
    def adjust_wheels(self, dest_point):
        x, y, theta = self.get_pose()
        angle = atan2(dest_point[1]-y,dest_point[0]-x)-theta
        return angle
        
    
    def get_el_punto(self):#renombrar
        x, y, theta = self.get_pose()
        distancia =  self.size / tan(self.wheels_angle)
        x2 = x - (self.size/2)*cos(theta) - (distancia*cos(theta-(pi/2)))
        y2 = y - (self.size/2)*sin(theta) - (distancia*sin(theta-(pi/2)))
        
        return distancia

    def get_info(self):
        self.info.ir_sensors.readings = [sensor.reading() for sensor in self.ir_sensors]
        return self.info
    
    def set_inputs(self,inputs):
        self.set_wheel_speeds(inputs)

    def diff2car(self, diff):
        (back_wheels_v, alpha) = diff  
        if alpha == 0: 
            w = 0
        else:
            distancia = self.get_el_punto()
            w = (1/distancia) * self.info.wheels.radius/self.info.wheels.base_length;
        v = back_wheels_v
        return (v,w)  

    def set_wheel_speeds(self,*args):#angle

        if len(args) == 2:
            (back_wheels_v, target_alpha) = args
        else:
            (back_wheels_v, target_alpha) = args[0]
        
        #left_ms  = clump(back_wheels_v, -self.info.wheels.max_velocity,
         #                     self.info.wheels.max_velocity)
        angle = self.clump(target_alpha, (-pi/2), (pi/2))
        self.front_wheels_ang_vel = 2 * (target_alpha - angle)
        self.wheels_angle = angle

    def clump(self, value, lower_bound, upper_bound):
        return max(lower_bound, min(value, upper_bound))

    def diff2uni(self,diff):
        (vl,vr) = diff
        v = (vl+vr) * self.info.wheels.radius/2;
        w = (vr-vl) * self.info.wheels.radius/self.info.wheels.base_length;
        return (v,w)
    
    def get_wheel_speeds(self):
        return self.ang_velocity
    
    def _set_wheel_speeds(self,*args):
        if len(args) == 2:
            (vl, vr) = args
        else:
            (vl, vr) = args[0]
            
        left_ms  = max(-self.info.wheels.max_velocity, min(self.info.wheels.max_velocity, vl))
        right_ms = max(-self.info.wheels.max_velocity, min(self.info.wheels.max_velocity, vr))

        self.ang_velocity = (left_ms, right_ms)

    def get_external_sensors(self):
        return self.ir_sensors

    def draw_sensors(self,renderer):
        """Draw the sensors that this robot has"""
        for sensor in self.ir_sensors:
            sensor.draw(renderer)
            
    def update_sensors(self):
        for sensor in self.ir_sensors:
            sensor.update_distance()
    
if __name__ == "__main__":
    # JP limits
    #v = max(min(v,0.314),-0.3148);
    #w = max(min(w,2.276),-2.2763);
    # Real limits
    k = QuickBot(Pose(0,0,0))
    k.set_wheel_speeds(1000,1000)
    print(k.diff2uni(k.get_wheel_speeds()))
    k.set_wheel_speeds(1000,-1000)
    print(k.diff2uni(k.get_wheel_speeds()))
    # 0.341 and 7.7
