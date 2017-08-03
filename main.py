from Tkinter import Tk
from gui import ExperimentUI
from experiment import Experiment
import wifi


def main():
    if wifi.connect_to_network('Banana_WiFi'):
        print '\nSuccessfully connected to Banana_WiFi'

    root = Tk()
    ui = ExperimentUI(root)
    ui.pack()
    root.mainloop()

    subject_number = ui.subject_number.get()
    hand = ui.hand.get()
    config_filename = ui.config_filename.get()
    initial_trial = ui.initial_trial.get()

    print '\nSubject ' + subject_number
    print 'Dominant hand: ' + hand
    print 'Starting from trial: ' + str(initial_trial)

    # start experiment
    exp = Experiment(subject=subject_number, start_from_trial=initial_trial)

    # limit frame rate to 60Hz (since PPT and eye tracking data come at 60Hz)
    viz.setOption('viz.max_frame_rate', '60')
    viz.go()


if __name__ == "__main__":
    main()
