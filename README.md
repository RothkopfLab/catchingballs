# Catching balls
This repo provides a virtual reality environment for a ball catching experiment. It is based on WorldViz Vizard 5.6, a Virtual Reality development platform. Higher versions of Vizard might not work with this code base. Beside WorldViz Vizard 5.6, the following Python
packages need to be installed for Vizard’s Python environment:
• numpy
• pandas
• PyODE
• ConfigObj
In addition to this, an up-to-date version of the Oculus Rift Runtime is required. In order for the the eye tracking part of the experiment to work, you also need version 1.5 of the SMI Eye Tracking HMD Upgrade software.

For details about the hardware components and the experimental setup, please consider reading my [Bachelor Thesis](https://github.com/dominikstrb/catchingballs/blob/master/thesis.pdf).

In order to start the experiment, simply run `main.py`. This script will start a GUI (see `gui.py`), which enables you to load a config file that determines the hardware settings as well the experimental conditions. Example config files (used in my thesis) can be found [here](https://github.com/dominikstrb/catchingballs/blob/master/config). Pressing the "Start" button in the GUI creates an experiment info file in the subdirectory `data/{subject_nr}/`. When the experiment is launched, this file is used to intialize the hardware, the visual environment and the experiment. All the magic then happens in `èxperiment.py`.
