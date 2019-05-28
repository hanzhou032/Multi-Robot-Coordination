#! /usr/bin/env python

import rospy
import random
import numpy as np
from sequential_single_item_auction.msg import BidsCA, SetofTasks, RoundResultCA
from strands_executive_msgs.srv import GetGuaranteesForCoSafeTask, GetGuaranteesForCoSafeTaskResponse
from strands_executive_msgs.msg import MdpDomainSpec
from strands_navigation_msgs.msg import TopologicalMap

class Auctioneer():
    # set up two publishers(SetofTasks, RoundResult) and one subscriber(Bids)
    def __init__(self):
        rospy.init_node("SSIA_auctioneer")
        self.finish_cond = False
        self.bid_robot_task_list=[]
        self.num_bids=0
        self.num_robots=2
        self.num_of_tasks=8

        self.robot_id_array=[]
        self.task_array=[]

        self.pub_set_of_tasks = rospy.Publisher("/set_of_tasks", SetofTasks, queue_size=1)
        self.source_topo=rospy.Subscriber("/topological_map", TopologicalMap, self.top_map_sub)
        self.source_bid = rospy.Subscriber("/bids_and_tasks", BidsCA, self.check_num_bids)
        self.pub_round_result = rospy.Publisher("/round_result", RoundResultCA, queue_size=1)
        self.top_nodes = []
        while self.top_nodes == []:
            rospy.sleep(1)
        self.gen_rd_set_of_tasks()
        if self.my_set_of_tasks.tasks!=[]: #Running multiple rounds of auctions until the list of tasks becomes empty.
            self.pub_set_of_tasks.publish(self.my_set_of_tasks)
            #rospy.loginfo("The random generated set of task is : %s", self.my_set_of_tasks.tasks)
        rospy.spin()

    def top_map_sub(self, msg):
        self.top_nodes = [node.name for node in msg.nodes]

    def check_num_bids(self,msg):            # update the finish_cond for checking whether robots have done the tasks or not.
        self.task_length=len(msg.tasks)
        if self.num_bids==0:
            self.bid_array=np.zeros((self.num_robots,self.task_length))
        temp_bids=list(msg.bids)
        temp_tasks=list(msg.tasks)

        if self.my_set_of_tasks.tasks!=[]:
            self.num_bids+=1
            if self.num_bids==2:
                temp_bids.reverse()
            else:
                self.task_array=temp_tasks[:]
            for i in range(self.task_length):
                self.bid_array[self.num_bids-1,i]=temp_bids[i]
            #print self.bid_array
            self.robot_id_array.append(msg.robot_id)
            #rospy.loginfo("bids received is %s, robot is %s, task is %s", msg.bids, msg.robot_id, msg.tasks)
            if self.num_bids==self.num_robots:
                self.auction_step()
                self.finish_cond = True
                self.num_bids=0

    def auction_step(self):
        sum_bid_array=self.bid_array.sum(axis=0)
        index = sum_bid_array.argmin(axis=0)
        for id in range(len(self.robot_id_array)):
            self.robot_id=self.robot_id_array[id]
            if id ==0:
                self.task=self.task_array[index]
            else:
                self.task=self.task_array[-index-1]
            self.gen_round_result()
            self.pub_round_result.publish(self.my_round_result)
            #rospy.loginfo(
            #    "The round result shows the task %s is allocated to the robot %s"
             #                  , self.my_round_result.tasks, self.my_round_result.robot_id)

    def gen_round_result(self):
        self.my_round_result=RoundResultCA()
        self.my_round_result.robot_id=self.robot_id
        self.my_round_result.tasks=self.task

    def gen_rd_set_of_tasks(self):
        self.my_set_of_tasks=SetofTasks()
        while len(self.my_set_of_tasks.tasks) < self.num_of_tasks:
            self.random_waypoint=self.random_wp_spec()
            if self.random_waypoint in self.my_set_of_tasks.tasks:
                self.random_waypoint=self.random_wp_spec()
            else:
                self.my_set_of_tasks.tasks.append(self.random_waypoint)

    def random_wp_spec(self):
        return 'F ("' + random.choice(self.top_nodes) + '")'

if __name__ == '__main__':
    auctioneer=Auctioneer()
