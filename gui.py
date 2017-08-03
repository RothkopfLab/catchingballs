from Tkinter import *
import ttk
import tkFileDialog
import os
import glob
import sys
from config_parser import Config


class ExperimentUI(Frame):
    def __init__(self, root):
        Frame.__init__(self, root)

        self.root = root

        self.subject_number = StringVar()
        self.hand = StringVar()
        self.config_filename = StringVar()
        self.initial_trial = IntVar()

        Label(self, text='Subject number').grid(row=1, column=0, padx=5, pady=5)
        self.subject_number_input = Entry(self, textvariable=self.subject_number, width=30)
        self.subject_number_input.grid(row=1, column=1, padx=5, pady=5)
        self.subject_number_input.focus()

        Label(self, text='Glove hand').grid(row=2, column=0, padx=5, pady=5)
        self.hand.set('right')
        hand_menu = OptionMenu(self, self.hand, 'left', 'right')
        hand_menu.grid(row=2, column=1, padx=5, pady=5)

        Label(self, text='Config file').grid(row=3, column=0, padx=5, pady=5)
        self.config_filename_input = Entry(self, textvariable=self.config_filename, width=30)
        self.config_filename_input.grid(row=3, column=1, padx=5, pady=5)
        self.config_filename_input.insert(0, 'config\\experiment_1.ini')
        self.config_filename_input.config(state=DISABLED)
        Button(self, text='Choose file', command=self.askopenfilename).grid(row=3, column=2, padx=5, pady=5)
        # define options for opening or saving a file

        ttk.Separator(self, orient=HORIZONTAL).grid(row=4, columnspan=3, sticky="ew")

        Button(self, text='Create trial list', command=self.create_experiment_info).grid(row=5, column=0, padx=5,
                                                                                         pady=5)

        Button(self, text='Start (from trial)...', command=self.start_experiment).grid(row=5, column=1, padx=5, pady=5)
        self.initial_trial_input = Entry(self, textvariable=self.initial_trial, width=10).grid(row=5, column=2, padx=5,
                                                                                               pady=5, sticky=W)

        Button(self, text='Cancel', command=self.cancel).grid(row=6, column=2, padx=5, pady=5)

        self.file_opt = options = {}
        options['defaultextension'] = '.ini'
        options['filetypes'] = [('all files', '.*'), ('config files', '.ini')]
        options['initialdir'] = 'config\\'
        options['initialfile'] = 'experiment_1.ini'
        options['parent'] = root
        options['title'] = 'Choose a config file'

        self.root.bind('<Return>', lambda e: self.start_experiment())

    def create_experiment_info(self):
        # create subject data directory
        data_dir = 'data/{}/'.format(self.subject_number.get())
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)
        else:
            files = glob.glob(data_dir + '*')
            [os.remove(f) for f in files]

        # create config object
        config = Config(config_filename=self.config_filename.get())

        # create trial list and save to csv
        config.save_trial_list(path=data_dir)

        print "Trial list created as {}.".format(data_dir + 'trials.csv')

        # create experiment info and save to txt
        config.save_experiment_info(glove_hand=self.hand.get(), path=data_dir)

        print "Experiment info created as {}.".format(data_dir + 'exp_info.txt')

    def askopenfilename(self):

        """Returns an opened file in read mode.
        This time the dialog just returns a filename and the file is opened by your own code.
        """

        # get filename
        filename = tkFileDialog.askopenfilename(**self.file_opt)

        self.config_filename_input.configure(state=NORMAL)
        self.config_filename_input.delete(0, "end")
        self.config_filename_input.insert(0, os.path.relpath(filename))
        self.config_filename_input.config(state=DISABLED)

    def start_experiment(self):
        subj = self.subject_number.get()
        if subj:
            if os.path.isfile('data/{}/trials.csv'.format(subj)) and os.path.isfile(
                    'data/{}/exp_info.txt'.format(subj)):
                self.root.destroy()
            else:
                print 'Please create the trial list first!'
        else:
            print '\nPlease enter a subject number (or name)!'

    def cancel(self):
        sys.exit('Experiment cancelled')


# for testing purposes
if __name__ == '__main__':
    root = Tk()
    ui = ExperimentUI(root)
    ui.pack()
    root.mainloop()
