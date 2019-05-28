#! /usr/bin/env python

import rospy
import random
import numpy as np
from sequential_single_item_auction.msg import Bids, SetofTasks, RoundResult
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
        self.num_of_tasks=9

        self.pub_set_of_tasks = rospy.Publisher("/set_of_tasks", SetofTasks, queue_size=1)
        self.source_topo=rospy.Subscriber("/topological_map", TopologicalMap, self.top_map_sub)
        self.source_bid = rospy.Subscriber("/bids_and_tasks", Bids, self.check_num_bids)
        self.pub_round_result = rospy.Publisher("/round_result", RoundResult, queue_size=1)
        self.top_nodes = []
        while self.top_nodes == []:
            rospy.sleep(1)

        self.gen_rd_set_of_tasks()
        if len(self.my_set_of_tasks.tasks)==self.num_of_tasks:
            self.global_tasks = self.my_set_of_tasks.tasks

        if self.my_set_of_tasks.tasks!=[]: #Running multiple rounds of auctions until the list of tasks becomes empty.
            self.pub_set_of_tasks.publish(self.my_set_of_tasks)
            rospy.loginfo("The random generated set of task is : %s", self.my_set_of_tasks.tasks)

        rospy.spin()

    def top_map_sub(self, msg):
        self.top_nodes = [node.name for node in msg.nodes]

    def check_num_bids(self,msg):            # update the finish_cond for checking whether robots have done the tasks or not.
        self.bid_robot_task_array=np.zeros((self.num_robots,self.num_of_tasks))
        if self.my_set_of_tasks.tasks!=[]:
            self.num_bids+=1
            self.bid_robot_task_array[int(msg.robot_id[6]),self.global_tasks.index(msg.task)]=msg.bid
            rospy.loginfo("bids received is %s, robot is %s, task is %s", msg.bid, msg.robot_id, msg.task)
            if self.num_bids==self.num_of_tasks*self.num_robots:
                self.auction_step()
                self.finish_cond = True

    def auction_step(self): #I cannot do robot_1 with robot_2 because of my way to deal with rhe index. which should be fixed in the future version.
        index_array=self.bid_robot_task_array.argmin(axis=0)
        for i in range(len(index_array)):
            self.robot_id="robot_"+str(index_array[i])
            self.task=self.global_tasks[0]
            self.gen_round_result()
            self.pub_round_result.publish(self.my_round_result)
            if self.my_round_result.task in self.my_set_of_tasks.tasks:
                 self.my_set_of_tasks.tasks.remove(self.my_round_result.task)
            rospy.loginfo(
                "The round result shows the task %s is allocated to the robot %s, and my current set of task is %s"
                               , self.my_round_result.task, self.my_round_result.robot_id, self.my_set_of_tasks)

    def gen_round_result(self):
        self.my_round_result=RoundResult()
        self.my_round_result.robot_id=self.robot_id
        self.my_round_result.task=self.task

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
