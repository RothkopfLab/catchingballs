import json

import numpy as np
import pandas as pd
import viz
import vizact
import viztask

import hardware
import network
from stimuli import Ball
from vis_env import Room, BaseballGlove
from viz_utils import is_near, faces

# header for data files
HEADER = ','.join(["time", "viewpos_x", "viewpos_y", "viewpos_z", "viewdir_x", "viewdir_y", "viewdir_z",
                   "gazedir_x", "gazedir_y", "gazedir_z", "ballpos_x", "ballpos_y", "ballpos_z", "ball_color",
                   "glovepos_x", "glovepos_y", "glovepos_z", "gloveori_x", "gloveori_y", "gloveori_z"]) + '\n'


class Experiment(viz.EventClass):
    """
    Represents a ball catching experiment
    """

    # Experiment inherits from viz.EventClass in order to implement its own callbacks (e.g. for key presses)
    def __init__(self, subject, start_from_trial=0):
        """
        Setup data path, room, ball, baseball glove and trial list. Initialize events and actions.
        :param subject: subject nr
        :param start_from_trial: number of first trial (used when experiment crashed)
        """
        # call superclass constructor
        viz.EventClass.__init__(self)

        # create data path
        self.data_dir = 'data/{}/'.format(subject)

        # read experiment info (config dict)
        with open(self.data_dir + 'exp_info.txt') as info_file:
            config = json.load(info_file)

        # setup hardware
        hardware.setup(config['hardware'])

        # initialize visual environment
        print '\nSetting up visual environment...'
        self.room = Room(config)
        self.config = config['experiment']

        # add ball
        ball_size = self.config['ball_size']
        ball_initial_pos = [0, ball_size, 36.22]
        self.ball = Ball(room=self.room, size=ball_size, position=ball_initial_pos, color=[1, 1, 1])
        self.ball.setAlpha(0)

        self.glove = None
        self.catch_sound = None
        # add glove
        if self.config['glove']:
            self.glove = BaseballGlove(room=self.room, size=self.config['glove_radius'], marker_num=1,
                                       hand=config['experiment']['hand'])
            self.catch_sound = viz.addAudio('res/sounds/applause.wav')

        # initialize trial list
        self.trials = pd.read_csv(self.data_dir + 'trials.csv').to_dict('records')
        self.trial_num = start_from_trial - 1
        self.current_trial = None
        self.current_block = None

        self.message_panel = None
        if hardware.hmd:
            # HMD message panel for showing messages between blocks and at the end of the experiment
            self.message_panel = hardware.hmd.addMessagePanel('You did it!')
            self.message_panel.visible(False)

        # key-press events (for starting the experiment)
        self.callback(viz.KEYDOWN_EVENT, self.on_key_down)

        # handle collisions (register when glove touches ball)
        vizact.onupdate(0, self.handle_collisions)

        # play sound if participant leaves allowed area
        self.allowed_area_action = vizact.ontimer(0.7, self.check_allowed_area)

    def check_allowed_area(self):
        pos = viz.MainView.getPosition()
        if self.glove:
            glove_pos = self.glove.getPosition()
        else:
            glove_pos = [0,0,0]

        if (abs(pos[0]) > self.config['allowed_area_x'] or abs(pos[2]) > self.config[
            'allowed_area_z'] or abs(glove_pos[0]) > self.config['allowed_area_x'] or abs(glove_pos[2]) > self.config[
            'allowed_area_z']) and self.trial_num >= 0:
            viz.playSound('res/sounds/warning.wav')

    def handle_collisions(self):
        """
        Logic for handling collisions (currently only between ball and glove)
        :return: None
        """
        physenv = self.room.physenv

        if not physenv.collision_detected:
            return

        # floor = self.room.floor.phys_node
        ball = self.ball.phys_node
        glove = self.glove.phys_node if self.glove else None

        for collision in physenv.collision_list_idx_phys_nodes:
            node1, node2 = collision
            if node1 == ball and node2 == glove or node2 == ball and node1 == glove:
                # either make ball stick to the glove
                # or let it pass through (and make it invisble)
                # currently it bounces off the glove and then turns invisible
                self.catch_sound.play()
                self.ball.setAlpha(0)
                if self.current_trial and self.current_trial.has_started:
                    self.current_trial.end_trial()

    def next_trial(self):
        """
        Start the next trial if there are trials left in the list.
        :return: None
        """
        try:
            self.trial_num += 1
            trial_info = self.trials[self.trial_num]

            # from trial 0 on
            if self.current_trial:
                if self.trials[self.trial_num - 1]['block'] != trial_info['block']:
                    self.current_block = trial_info['block']
                    self.message_panel.setText('Next block!')
                    if self.trials[self.trial_num - 1]['block'] == 'training':
                        self.message_panel.setText('Training finished!')
                    self.message_panel.visible(True)
                    # how do we make it disappear again?
            else:
                # if this is the first block
                self.current_block = trial_info['block']
            self.current_trial = Trial(trial_num=self.trial_num, trial_info=trial_info, ball=self.ball,
                                       glove=self.glove, data_dir=self.data_dir, parent=self)

        # end experiment if there are no more trials in list
        except IndexError:
            self.end_experiment()

    def start(self):
        """
        Start the first trial
        :return: None
        """
        # remove the callback that starts the experiment
        self.callback(viz.KEYDOWN_EVENT, None)
        # start the first trial
        self.next_trial()

    def end_experiment(self):
        self.current_trial = None
        self.callback(viz.KEYDOWN_EVENT, 0)
        self.allowed_area_action.remove()
        self.message_panel.visible(True)
        self.message_panel.setText('You did it!')

        print '\nExperiment finished!'
        network.send_message('\nExperiment finished!')

    def on_key_down(self, key):
        # launch ball
        # BALL IS NOW LAUNCHED AUTOMATICALLY
        # if key == self.config['key_launch_ball']:
        #    if self.trial_num >= 0:
        #        self.current_trial.launch_ball()

        # set next trial
        if key == self.config['key_start']:
            self.start()


class Trial(object):
    """
    A single trial in an Experiment. Contains logic for starting trial, launching ball,
    collecting data and ending the trial.
    """

    def __init__(self, trial_num, trial_info, ball, glove, data_dir, parent):
        """Setup trial info and actions.
        :param trial_num: # of trial in Experiment
        :param trial_info: dict containing trial info
        :param ball: the ball used in the Experiment
        :param glove: a BaseballGlove object
        :param data_dir: directory to save data log file
        :param parent: parent Experiment
        """
        self.ball = ball
        self.glove = glove

        self.parent = parent

        self.data_filename = data_dir + "/trial" + str(trial_num) + "data.txt"
        # data file is only initialized when ball is launched
        self.data_file = None

        # set up logging
        self.log_action = vizact.onupdate(0, self.log_data)
        self.log_action.setEnabled(False)
        self.timer = 0

        # get trial info
        self.trial_info = trial_info
        self.start_pos = eval(trial_info['start_pos'])
        self.is_fly_ball = trial_info['fly_ball']
        speed = trial_info['speed']

        self.ball_dir = eval(trial_info['ball_dir'])
        self.ball_pos = eval(trial_info['ball_pos'])

        if trial_info['mirror']:
            # change position 
            # change direction
            self.ball_dir[0] = -self.ball_dir[0]
            self.ball_pos[0] = -self.ball_pos[0]

        if self.is_fly_ball:
            self.parent.room.physenv.world.setGravity([0, -9.8, 0])
        else:
            self.parent.room.physenv.world.setGravity([0, 0, 0])

        # reset ball
        self.ball.setPosition(self.ball_pos)
        self.ball.setVelocity([0, 0, 0])
        self.ball.setAlpha(0)
        self.ball.setContrastMode(trial_info['contrast_mode'])

        # print info for next trial
        info_texts = ['\nTrial ' + str(trial_num), 'Fly ball: ' + str(self.is_fly_ball), 'Speed ' + str(speed),
                      'Contrast: ' + str(trial_info['contrast_mode']), 'Ball pos: ' + str(self.ball_pos),
                      'Ball direction: ' + str(self.ball_dir), 'Mirrored: ' + str(trial_info['mirror'])]
        
        network.send_message('')
        for text in info_texts:
            print text
            network.send_message(text)

        pole_pos = list(self.ball_pos)
        pole_pos[1] = 0
        # set up poles (for start position and position to be faced)
        self.start_pole = viz.addChild('pole.wrl')
        self.start_pole.setPosition(self.start_pos)
        # TODO: make margin flexible?
        # 0.24 is pole size
        # 0.5 / 0.24 ~= 2 -> scale pole up to 2
        self.start_pole.setScale([2, 0.35, 2])
        self.start_pole.color(viz.BLUE)
        self.start_pole.alpha(0.5)
        self.ball_pole = viz.addChild('pole.wrl')
        self.ball_pole.setPosition(pole_pos)
        self.launch_pole = self.ball_pole.copy()
        self.launch_pole.setPosition(pole_pos)
        self.launch_pole.color(viz.RED)
        self.launch_pole.setScale([1.1, 0, 1.1])
        self.start_timer = 0

        self.countdown_sound = viz.addAudio('res/sounds/beeps.wav')

        # set up pre-launch phase
        self.await_action = vizact.onupdate(0, self.await_launch)

        self.await_end_task = None
        self.has_started = False

        self.timer = 0

    def await_launch(self):
        """
        Count timer up to 3s if participant stands at start location and faces ball location. Launch ball after
        successful 3s. Reset trial if participant fails.
        :return: None
        """
        subject_pos = viz.MainView.getPosition()
        view_dir = viz.MainView.getMatrix().getForward()
        if is_near(pos=subject_pos, target=self.start_pos, margin=0.5):
            if self.parent.message_panel:
                self.parent.message_panel.visible(False)
            self.start_pole.visible(viz.OFF)
            if faces(pos=subject_pos, direction=view_dir, target=self.launch_pole.getPosition(), margin=5):
                self.countdown_sound.play()
                self.start_timer += viz.getFrameElapsed()
                self.launch_pole.setScale([1.1, self.start_timer / 3.0, 1.1])
                if self.start_timer >= 3.0:
                    self.ball_pole.visible(viz.OFF)
                    self.launch_pole.visible(viz.OFF)
                    self.await_action.setEnabled(False)
                    self.launch_ball()
            else:
                self.reset()
        else:
            self.reset()

    def reset(self):
        """
        Reset trial (called if participant leaves start location or stops looking at the pole)
        :return: None
        """
        self.countdown_sound.stop()
        self.start_pole.visible(viz.ON)
        self.ball_pole.visible(viz.ON)
        self.start_timer = 0

    def launch_ball(self):
        """
        Launch the ball and start logging data. Schedule next trial.
        :return: None
        """
        # initialize new data file
        # viz.playSound('res/sounds/beeps.wav')
        # yield viztask.waitTime(3.41)

        self.data_file = open(self.data_filename, 'w')
        self.data_file.write(HEADER)

        self.ball.setAlpha(1)

        # reset timer
        self.timer = 0

        # start logging data
        self.log_action.setEnabled(True)

        # launch the ball
        #self.ball.setPosition(self.ball_pos)

        if self.is_fly_ball:
            self.ball.launch_fly(speed=self.trial_info['speed'], target=self.ball_dir)
        else:
            self.ball.launch_straight(speed=self.trial_info['speed'], direction=self.ball_dir)
        # TODO: maybe calculate trial duration and call end_trial with wait_time argument

        self.has_started = True
        self.await_end_task = viztask.schedule(self.await_end_trial())

    def log_data(self):
        """
        Gather data from viewpoint, ball, glove, eye tracker and write to fail
        :return: None
        """
        # update timer
        self.timer += viz.getFrameElapsed()

        #####################################
        # collect data ######################
        #####################################
        # time
        data = [self.timer]
        # viewpos xyz
        data += viz.MainView.getPosition()
        # view direction xyz
        data += viz.MainView.getMatrix().getForward()
        # gaze direction xyz
        if hardware.eye_tracker:
            gaze_vec = hardware.eye_tracker.getGazeDirection()
        else:
            gaze_vec = [np.nan, np.nan, np.nan]
        data += gaze_vec
        # ball position xyz
        data += self.ball.vis_node.getPosition()
        # ball color
        data.append(self.ball.getColor()[0])
        # glove position and orientation (if any)
        if self.glove:
            data += self.glove.vis_node.getPosition()
            data += self.glove.ori_tracker.getMatrix().getForward()
        else:
            data += [np.nan, np.nan, np.nan]
            data += [np.nan, np.nan, np.nan]

        ######################################
        ######################################

        data_string = ','.join([str(x) for x in data])

        self.data_file.write(data_string + "\n")

    def await_end_trial(self, wait_time=10):
        """
        End trial after wait_time seconds. 
        :param wait_time: seconds to wait
        :return: None
        """
        yield viztask.waitTime(wait_time)
        # viz.playSound('res/sounds/beeps.wav')
        # yield viztask.waitTime(3.41)
        self.end_trial()

    def end_trial(self):
        self.await_end_task.kill()
        self.log_action.setEnabled(False)
        self.data_file.close()
        self.start_pole.remove()
        self.ball_pole.remove()
        self.launch_pole.remove()
        self.parent.next_trial()
