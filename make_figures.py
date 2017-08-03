from config_parser import Config
import json
import viz
import vizact
import numpy as np
from vis_env import Room, BaseballGlove
from stimuli import Ball

config_filename = 'config/home.py'

config = Config(config_filename)
config.save_experiment_info(path='figures/')

# read experiment info (config dict)
with open('figures/exp_info.txt') as info_file:
	config = json.load(info_file)

room = Room(config)

#ball = Ball(room, size=0.15, position=[0,1.8,4], color=[1, 1, 1])
#ball.setContrastMode('constant_dark')
glove = viz.addChild('pole.wrl')


def rotate_glove():
	direction = glove.getMatrix().getForward()
	print direction
	print glove.getQuat()
	#direction[0] = -direction[0]
	#glove.lookAt(direction, viz.REL_LOCAL)
	#print glove.getMatrix().getForward()
	
vizact.onkeydown(' ', rotate_glove)

viz.go()