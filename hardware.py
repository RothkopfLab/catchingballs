import viz
# import oculus_08 as oculus
import oculus
import smi
import ppt
import vizcam

eye_tracker = None
hmd = None
head_tracker = None
link = None

config = None


class EyeTracker(smi.iViewHMD):
    """
    Inherits from smi.iViewHMD and implements additional methods
    """

    def __init__(self):
        """
        Call constructor of superclass
        """
        smi.iViewHMD.__init__(self)

    def getGazeDirection(self):
        """
        Compute gaze direction from current gaze matrix
        :return: gaze direction
        """
        gaze_mat = self.getLastGazeMatrix()
        gaze_mat.postMult(viz.MainView.getMatrix())
        return gaze_mat.getForward()


def setup(conf):
    """
    Set up all the hardware used in the ball catching experiment
    :param conf: config['hardware'] entry of full config dict
    :return: None
    """
    global config
    global hmd
    global head_tracker
    global eye_tracker
    global link

    config = conf

    print '\nSetting up hardware modules...'

    # configure Oculus Rift
    if config['use_hmd']:
        hmd = oculus.Rift()
        hmd.setMonoMirror(True)
        hmd.getSensor().reset()

        # setup position tracking (WorldViz PPTX)
        if config['use_ppt']:
            head_tracker = ppt.add_tracker(0)

            link = ppt.link(tracker=head_tracker, ori=hmd.getSensor(), target=viz.MainView,
                            pre_trans=[-0.05, -0.05, 0.02])

            head_tracker = head_tracker
        # no ppt
        else:

            link = viz.link(hmd.getSensor(), viz.MainView, mask=viz.LINK_ORI)
            link.setOffset([0, 1.8, 0])
            viz.MainView.setPosition([0, 1.8, -3])

        # setup eye tracker
        if config['eye_tracking']:
            eye_tracker = EyeTracker()

    # configure screen setup
    else:
        viz.MainView.setPosition(0, 0, -4, viz.REL_LOCAL)
        viz.setOption('viz.fullscreen.monitor', 2)
        keyboard_cam = vizcam.KeyboardCamera()

    link = link

    # keys for hardware control
    viz.callback(viz.KEYDOWN_EVENT, on_key_down)


def on_key_down(key):
    """
    Add key bindings for resetting the HMD's sensor, calibrating and validating the eye tracker
    :param key: the key currently pressed
    :return: None
    """
    # reset hmd orientation
    if key == config['key_reset_hmd']:
        hmd.getSensor().reset()
    # calibrate eye tracker
    if key == config['key_calibrate']:
        # TODO: change back to 5 point calibration
        eye_tracker.calibrate(type=smi.CALIBRATION_3_POINT)
    # quantitative validation
    if key == config['key_validate']:
        eye_tracker.quantitativeValidation()
    if key == 'p':
        gaze_mat = eye_tracker.getLastGazeMatrix()
        gaze_mat.postMult(viz.MainView.getMatrix())
        print gaze_mat.getForward()
        print viz.MainView.getMatrix().getForward()
