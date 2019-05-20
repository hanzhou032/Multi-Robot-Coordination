#! /usr/bin/env python

# Load Larger LSTM network and generate text
import sys
import numpy as np
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Dropout
from keras.layers import LSTM
from keras.callbacks import ModelCheckpoint
from keras.utils import np_utils
import math
import tensorflow as tf
import rospy
from sequential_single_item_auction.msg import SetofTasks, NewTasks, LSTMTasks

from keras.backend import clear_session


class LSTM_node():
    def __init__(self):
        # Initialise the LSTM node used for predicting tasks
        rospy.init_node("LSTM_node")
        clear_session()

        # Initialise several lists
        self.task_input=[]
        self.NN_input=[]
        self.predvY = []
        self.predY = []
        self.pred_prob = []

        # Set up the initial subscriber and publishers
        self.source_sot = rospy.Subscriber("/set_of_tasks", SetofTasks, self.ini_pattern)
        self.source_nt = rospy.Subscriber("/new_tasks", NewTasks, self.add_pattern)
        self.pub_LSTM_tasks = rospy.Publisher("/LSTM_tasks", LSTMTasks, queue_size=1, latch=True)

        # default parameter settings
        self.window_size = 3
        self.num_task = 7
        pred_length = 1

        # define the LSTM model
        self.model = Sequential()
        self.model.add(LSTM(256, input_shape=(self.window_size, self.num_task), return_sequences=True))
        self.model.add(Dropout(0.2))
        self.model.add(LSTM(256))
        self.model.add(Dropout(0.2))
        self.model.add(Dense(self.num_task, activation='softmax'))

        # load the selected trained weight file and compile
        # filename = "weights-new-task-16-1.6835-bigger.hdf5"
        # filename = "weights-new-task-40-1.4430-bigger.hdf5"
        # filename = "weights-new-task-18-1.6345-bigger.hdf5"
        # filename = "weights-new-task-26-1.5879-bigger.hdf5"
        # filename = "weights-new-task-60-0.2842-bigger.hdf5"
        # filename = "weights-new-task-60-0.2842-bigger.hdf5"
        filename = "weights-batchsize1ws3-task-20-1.6041-bigger.hdf5"

        self.model.load_weights(filename)
        self.model.compile(loss='categorical_crossentropy', optimizer='adam')
        print("\nDone.")

        # Set up the default graph for tensorflow backend in multiple threads situation
        self.graph = tf.get_default_graph()

        rospy.spin()

    # callback function used to initialise the beginning sequence of tasks
    def ini_pattern(self, msg):
        self.task_input=msg.tasks[:]
        #print "The initial task sequence is:", self.task_input
    # callback function used to add new task into the input of NN and triggers the LSTM prediction

    def add_pattern(self, msg):
        if msg.new_flag:
            self.task_input.append(msg.tasks[0])
            #print "Updated task sequence is:", self.task_input
            self.LSTM_predict()

    # Transformation between task strings and NN readable ints
    def char_to_int(self, char):
        #print char
        #print char[12:-2]
        return int(char[12:-2]) - 1  # F ("WayPoint1") or F ("WayPoint14")

    def int_to_char(self, value):
        task_name = 'F ("WayPoint' + str(value + 1)+'")'
        return task_name

    # Generation of the LSTM predicted task with its corresponding probability
    def gen_LSTM_tasks(self):
        self.my_LSTM_tasks = LSTMTasks()
        #print self.predvY
        #print self.pred_prob
        self.my_LSTM_tasks.tasks = self.predvY[:]
        self.my_LSTM_tasks.prob = self.pred_prob[:]
        #print type(self.predvY)
        #print self.predvY[-1]
        #print type(self.predvY[-1])
        self.my_LSTM_tasks.tasks = []
        self.my_LSTM_tasks.prob = []
        self.my_LSTM_tasks.tasks.append(self.predvY[-2:])
        self.my_LSTM_tasks.prob.append(self.pred_prob[-2:])

    # Prepare the datatype and format used for feeding to the input of NN
    def NN_input_prepare(self):
        validx=[]
        #print self.NN_input
        validx.append([self.char_to_int(task) for task in self.NN_input])
        #print validx
        validx = validx[0][-self.window_size:]
        #print validx
        #print type(validx)
        self.NN_input=np_utils.to_categorical(validx, num_classes=self.num_task)
        # I should make the category before extracting the latest sequence of tasks
        #print self.NN_input

    # The minimum starting sequence for the LSTM network is 10
    def LSTM_predict(self):
        self.input_dim = len(self.task_input)
        if self.input_dim >= self.window_size:
            with self.graph.as_default():  # This solves the tensorflow multithread problem
                self.NN_input = self.task_input[:]
                self.NN_input_prepare()
                x = np.reshape(self.NN_input, (1, self.window_size, 7))
                # reshape X to be [samples, time steps, features]
                x = x / float(self.num_task-1)
                #print x
                prediction = self.model.predict(x, verbose=0)
                index = np.argmax(prediction)
                temp_prediction = prediction
                second_max = np.partition(temp_prediction, -2)[-2]
                second_index = list(prediction[0]).index(second_max)
                self.pred_prob.append(float(prediction[0][index]))
                self.pred_prob.append(float(prediction[0][second_index]))
                result = self.int_to_char(index)
                second_result = self.int_to_char(second_index)
                self.predvY.append(result)
                self.predvY.append(second_result)
                self.predY.append(index)
                self.predY.append(second_index)
                # sys.stdout.write(result)
                #print result
                self.gen_LSTM_tasks()
                rospy.loginfo("The most recent prediction is %s", self.my_LSTM_tasks.tasks)
                self.pub_LSTM_tasks.publish(self.my_LSTM_tasks)


if __name__ == '__main__':
    lstm=LSTM_node()
