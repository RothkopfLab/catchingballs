from vis_env import BaseballGlove, Room
from configobj import ConfigObj
from validate import Validator
import vizcam

config = ConfigObj('config/test_glove_presenter.ini', configspec='config/default_straight.ini')

validator = Validator()
val = config.validate(validator)

room = Room(config)
glove = BaseballGlove(room=room, size=1.0, marker_num=1)

keyboard_cam = vizcam.KeyboardCamera()
viz.go()
