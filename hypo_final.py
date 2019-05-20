#! /usr/bin/env python

import rospy
import random
import sys
from pandas import read_csv

from sequential_single_item_auction.msg import LSTMTasks, HypoRoundResult, HypoBids
from strands_executive_msgs.srv import GetGuaranteesForCoSafeTask, GetGuaranteesForCoSafeTaskResponse
from strands_executive_msgs.msg import MdpDomainSpec
from strands_navigation_msgs.msg import TopologicalMap


class Hypo_Auctioneer():
    def __init__(self):
        rospy.init_node("SSIA_hypo_auctioneer")

        self.bid_robot_task_list = []
        self.num_bids = 0
        self.num_robots = 2
        self.num_of_tasks = 2
        self.num_reply = 0
        self.hypo_tasks = []
        self.hypo_prob = []

        # Initialise the subscriber and publisher used in the auctioneer
        self.source_LSTM_tasks = rospy.Subscriber("/LSTM_tasks", LSTMTasks, self.assign_hypo_tasks)
        self.source_bid = rospy.Subscriber("/hypo_bids", HypoBids, self.check_hypo_bids)
        self.pub_hypo_round_result = rospy.Publisher("/hypo_round_result", HypoRoundResult, queue_size=1, latch=True)

        rospy.spin()

    def assign_hypo_tasks(self, msg):
        self.hypo_tasks = msg.tasks[:]
        self.hypo_prob = msg.prob[:]

    # Whenever the auctioneer receives a new bid, it checks whether the number is equal to the number of robots
    def check_hypo_bids(self, msg):  # update the finish_cond for checking whether robots have done the tasks or not.
        self.num_bids += 1
        self.bid_robot_task_list.append((msg.bid, msg.robot_id, msg.task))  # Create a list of tuples
        #rospy.loginfo("hypo bids received is %s, robot is %s, task is %s", msg.bid, msg.robot_id, msg.task)
        if self.num_bids == self.num_robots:
            # check infinity bids
            self.not_all_inf_flag = True
            # self.check_inf_bids() # Turn off the ini bid method, maybe used for Safety in future case
            self.auction_step()

    def check_inf_bids(self):
        self.not_all_inf_flag = False
        for i in range(len(self.bid_robot_task_list)):
            # print(self.bid_robot_task_list[i][0])
            if self.bid_robot_task_list[i][0] < sys.maxint:
                self.not_all_inf_flag = True

    # Find the lowest bid and assign corresponding task to that robot
    def auction_step(self):
        index = self.bid_robot_task_list.index(min(self.bid_robot_task_list))
        self.bid = self.bid_robot_task_list[index][0]
        self.robot_id = self.bid_robot_task_list[index][1]
        self.task = self.bid_robot_task_list[index][2]
        self.num_bids = 0
        self.bid_robot_task_list = []
        if self.not_all_inf_flag:
            self.gen_round_result()
        else:
            self.gen_non_round_result()

        self.pub_hypo_round_result.publish(self.my_round_result)
        rospy.loginfo("The hypo round result shows the hypo task %s is allocated to the robot %s"
                      , self.my_round_result.task, self.my_round_result.robot_id)

    # Generate the round result message
    def gen_round_result(self):
        self.my_round_result = HypoRoundResult()
        self.my_round_result.robot_id = self.robot_id
        self.my_round_result.task = self.task

    def gen_non_round_result(self):
        self.my_round_result = HypoRoundResult()
        self.my_round_result.robot_id = "-1"
        # which means this task is not actually allocated to anyone of the robot, however we need to
        # take care about the case when it receives complete set of tasks because it will not be completed forever
        self.my_round_result.task = self.task


if __name__ == '__main__':
    hypo_auctioneer = Hypo_Auctioneer()
