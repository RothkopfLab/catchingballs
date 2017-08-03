import viz
import vizact

import numpy as np
from vis_env import VisualObject
import hardware
from viz_utils import cos_between, angle_between

z1 = np.linspace(-3,0,300)
z2 = np.linspace(0,5,300)
z =  np.append(z1[:-1], z2)
#x_path = np.append(-1.2*np.arctan(1.8*(z1+3))[:-1], 1.2*np.arctan(1.8*(z2-4)))
#x_path = np.append(-1.8*np.tanh(1.5*(z1+3))[:-1], 1.8*np.tanh(1.5*(z2-3)))
x_path = np.append(-1.8*np.tanh(1.5*(z1+3))[:-1], 1.82*np.tanh(1*(z2-2.5)))


class Ball(VisualObject):
    """
    A Ball is a sphere with additional logic needed for the ball catching experiment
    """

    def __init__(self, room, size, position, color):
        """
        Initialize VisualObject, set up variables needed for contrast modes
        :param room: parent room
        :param size: radius
        :param position: initial position
        :param color: color (rgb)
        """
        VisualObject.__init__(self, room=room, shape='sphere', size=size, position=position, color=color)

        self.initial_dist = 0
        self.initial_pos = np.array(position)
        self.updateInitialDist()

        self.contrast_action = None
        self.setContrastMode('constant')


        self.bearing = 0


        #self.elevation = 0
        #self.elevation_vel = 0

    def setPosition(self, position):

        self.bearing = 0
        self.updateInitialDist()

        super(Ball, self).setPosition(position)

    def launch_fly(self, speed, target):
        """
        Launch the ball with some speed towards a target. Velocity needed is computed accordingly.
        :param speed: speed in m/s
        :param target: position to land on
        :return: None
        """
        # compute angle between direction of launch and x axis / z axis
        x_axis = np.array([1, 0, 0])
        direction = np.array(target) - np.array(self.position)
        theta_x = angle_between(direction, x_axis)
        theta_z = theta_x + np.pi / 2

        # compute velocities in x and z directions
        v_x = speed * np.cos(theta_x)
        v_z = speed * np.cos(theta_z)

        # compute time needed to travel in x direction
        t = direction[0] / v_x

        # compute velocity needed to be at correct y location after the same amount of time
        s_y = direction[1] + self.size
        v_y = 9.8 * t / 2 - s_y

        # maxHeight =  -v_y**2/(2*-9.8)
        # launch ball
        self.setVelocity(velocity=[v_x, v_y, v_z])

    def launch_straight(self, speed, direction):
        direction = np.array(direction)
        velocity = speed * direction / np.linalg.norm(direction)
        self.setVelocity(velocity.tolist())

    def updateInitialDist(self):
        """
        Update the initial distance from ball to MainView
        :return: None
        """
        pos = np.array(self.vis_node.getPosition())
        self.initial_pos = pos
        view_pos = np.array(viz.MainView.getPosition())
        self.initial_dist = np.sqrt(np.sum((pos - view_pos) ** 2))

    def setContrastMode(self, mode, min=0.02):
        """
        Setup an onupdate action that updates the contrast in the specified mode
        :param mode: lin_dist, inv_dist, exp_dist, cos2angle, cos2gaze, bearing_vel, elevation_acc
        :param min: minimum contrast
        :return: None
        """
        self.updateInitialDist()

        self.contrast_mode = mode

        if self.contrast_action:
            self.contrast_action.remove()
        self.contrast_action = vizact.onupdate(viz.PRIORITY_DEFAULT, self.updateContrast, mode, min)

    def updateContrast(self, mode, min=0.01):
        """
        Update the contrast in the specified mode (called by setContrastMode)
        :param mode: lin_dist, inv_dist, exp_dist, cos2angle, cos2gaze, bearing_vel, elevation_acc
        :param min: minimum contrast
        :return: None
        """

        pos = np.array(self.vis_node.getPosition())
        view_pos = np.array(viz.MainView.getPosition())
        view_ori = np.array(viz.MainView.getMatrix().getForward())

        if hardware.eye_tracker:
            gaze_dir = hardware.eye_tracker.getGazeDirection()
        else:
            gaze_dir = view_ori

        dist = np.sqrt(np.sum((pos - view_pos) ** 2))
        rel_dist = abs(self.initial_dist - dist) / self.initial_dist

        ball_vec = (pos - view_pos)

        # ball_vel = np.array(self.getVelocity())

        dt = viz.getFrameElapsed()
        
        # self.vis_velocity = np.mean(np.diff(self.vis_angle_array)) / dt
        # print vis_velocity

        # bearing angle
        bearing = np.rad2deg(np.arctan2(ball_vec[0], ball_vec[2]))
        
        # print self.bearing_velocity
        
        c = .3
        previous_bearing = self.bearing
        self.bearing = c * bearing + (1 - c) * previous_bearing



        # elevation angle
        # elevation = np.tan(np.arcsin(ball_vec.dot(np.array([0, 1, 0])) / np.linalg.norm(ball_vec)))

        if mode == 'lin_dist':
            light_o = min + (1 - min) * rel_dist

        elif mode == 'inv_dist':
            light_o = 1 - (1 - min) * rel_dist

        elif mode == 'exp_dist':
            light_o = min + ((1 - min) - (1 - min) * np.exp(-rel_dist))

        elif mode == 'cos2angle':
            dir_angle = cos_between(view_ori, (pos - view_pos))
            light_o = min + (1 - min) * (1 - dir_angle ** 2)

        elif mode == 'cos2gaze':
            dir_angle = cos_between(gaze_dir, (pos - view_pos))
            light_o = min + (1 - min) * (1 - dir_angle ** 2)

        elif mode == 'bearing_vel':
            # light_o = min + max * (1 - 1.0 / (1 + np.abs(bearing_vel)))
            # light_o = min + (1 - min) * (1 - np.exp(- 2 * np.abs(self.bearing_velocity)))


            #bearing_velocity = \
            #savgol_filter(np.array(self.bearing_array), window_length=9, polyorder=2, deriv=1, delta=dt)[4]
            bearing_velocity = (self.bearing - previous_bearing) / dt
            light_o = (1 / (1 + np.exp(- 2 * (np.abs(bearing_velocity) - 6))))
        # print light_o
        
        elif mode == 'bearing_dist':
            
            #bearing_velocity = \
            #savgol_filter(np.array(self.bearing_array), window_length=9, polyorder=2, deriv=1, delta=dt)[4]
            #bearing_velocity = (np.array(self.bearing_array[-5:]).mean() - np.array(self.bearing_array[-6:-1]).mean()) / dt
            bearing_velocity = (self.bearing - previous_bearing) / dt
            light_o = (1 / (1 + np.exp(- 1.5 * (np.abs(bearing_velocity) - 7)))) * (min + (1 - min) / self.initial_dist * dist)
            
            
        elif mode == 'tan_pos':
            x = x_path
            if self.initial_pos[0] > 0:
                x = -x_path
                
                
            posx = view_pos[0]
            posz = view_pos[2]
            
            dist = np.sqrt(np.min((posz-z)**2 + (posx-x)**2))
    
                
            light_o = 1 - (1 / (1 + np.exp(- 15 * (dist - 0.4))))
            
        elif mode == 'constant_dark':
            light_o = 0.2


        

        # constant or invalid mode
        else:
            light_o = 1.0

        # print 'light_o: '+str(light_o)
        # self.visNode.alpha(light_o)

        # TODO: do we really want 0.8? make this variable via config
        self.setColor([0.8 * light_o] * 3)
