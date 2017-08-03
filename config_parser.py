import json
import pandas as pd
from configobj import ConfigObj
from validate import Validator
import itertools
import numpy as np


class Config(object):
    def __init__(self, config_filename):
        # load config
        self.config = ConfigObj(config_filename, configspec='config/default.ini')

        validator = Validator()
        self.config.validate(validator)

    def save_trial_list(self, path):
        blocks = self.config['blocks']
        trials = []
        for block_name, block in blocks.iteritems():
            contrast_mode = block['contrast_mode']
            ball_positions = block['ball_start']
            ball_directions = block['ball_direction']
            speeds = block['speeds']
            condition_idx = ball_positions.keys()

            # all combinations of start_pos
            combinations = list(itertools.product(ball_positions.keys(), speeds, [True, False]))

            # repeat and shuffle
            for i in range(int(block['repetitions'])):

                np.random.shuffle(combinations)
                np.random.shuffle(combinations)

                # append to trial list
                for trial in combinations:
                    ball_idx, speed, mirror = trial
                    ball_pos = ball_positions[ball_idx]
                    ball_dir = ball_directions[ball_idx]
                    trials.append(
                        {"contrast_mode": contrast_mode, "ball_dir": [float(x) for x in ball_dir], "mirror": mirror,
                         "ball_pos": [float(x) for x in ball_pos],
                         "fly_ball": bool(int(block['fly_balls'])), "start_pos": self.config['experiment']['start_pos'],
                         "speed": float(speed),
                         "block": block_name})

        # save in subject's data folder
        df = pd.DataFrame(trials)

        df.to_csv(path + 'trials.csv')

    def save_experiment_info(self, path, glove_hand='right'):
        # add glove_hand to config for logging (because we only know this from our GUI)
        self.config['experiment']['hand'] = glove_hand

        # dump to file
        with open(path + 'exp_info.txt', 'w') as info_file:
            json.dump(self.config, info_file)
