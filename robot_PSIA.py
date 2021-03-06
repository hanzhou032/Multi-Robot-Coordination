#! /usr/bin/env python

import rospy #File name without capital
import sys

from sequential_single_item_auction.msg import Bids, SetofTasks, RoundResult
from std_msgs.msg import String  #Create a topic of current node
from strands_executive_msgs.msg import MdpDomainSpec
from strands_executive_msgs.srv import GetGuaranteesForCoSafeTask, GetGuaranteesForCoSafeTaskResponse

class Robot():
    # Set up two subsribers(SetofTasks, RoundResult) and one publisher(Bids)
    def __init__(self, robot_id):
        rospy.init_node("SSIA_robot"+str(robot_id))
        self.robot_id=robot_id
        print "I AM ROBOT ", rospy.get_namespace()

        self.robot_schedule = []
        self.robot_spec=""
        self.source_wp = rospy.Subscriber("current_node",String,self.get_waypoint) #rostopic type robot0/current_node
        self.source_sot = rospy.Subscriber("/set_of_tasks",SetofTasks,self.assign_tasks) #/ means no namespace for each one of the robot
        self.pub_bid = rospy.Publisher("/bids_and_tasks",Bids,queue_size=1)
        self.source_rr = rospy.Subscriber("/round_result",RoundResult,self.add_task)
        rospy.wait_for_service('mdp_plan_exec/get_guarantees_for_co_safe_task')
        self.call_MDP_service = rospy.ServiceProxy('mdp_plan_exec/get_guarantees_for_co_safe_task',
                                              GetGuaranteesForCoSafeTask)
        rospy.spin()

    def assign_tasks(self,msg): # Instead writing do_bid as the callback function, I change it to assign_tasks().
        self.current_global_tasks=msg.tasks
        #rospy.loginfo("received global task is %s", self.current_global_tasks)
        time=rospy.Time.now()
        rospy.loginfo("the start bidding time is %s", time)
        self.do_bid()

    def do_bid(self):
        for task in self.current_global_tasks:
            time_now = rospy.get_rostime()
            spec=MdpDomainSpec()
            spec.ltl_task=task
            waypoint = self.current_waypoint  # Obtain the initial waypoint
            cost = self.call_MDP_service(waypoint,spec,time_now) # Receive the expected cost value from the MDP service
            self.best_bid = cost.expected_time.to_nsec()
            self.best_task = task
            self.gen_my_bid()
            self.pub_bid.publish(self.my_bid)

    def gen_my_bid(self):
        self.my_bid = Bids()
        self.my_bid.robot_id = self.robot_id
        self.my_bid.bid = self.best_bid
        self.my_bid.task = self.best_task
        # rospy.loginfo("my best bid is %s, and my best task is %s",self.my_bid.bid, self.my_bid.task)

    def add_task(self,msg):
        if robot_id == msg.robot_id: # Checks that the robot is the one specified by the winner task
            self.extend_task(msg.task)
            #rospy.loginfo("the robot: %s assigned task is %s",self.robot_id,self.robot_schedule)
        if msg.task in self.current_global_tasks:
            self.current_global_tasks.remove(msg.task)
        #rospy.loginfo("remaining current global tasks are:%s", self.current_global_tasks)
        time = rospy.Time.now()
        rospy.loginfo('the complete time is at %s', time)

    def extend_task(self,new_task):
        self.robot_schedule.append(new_task) # Extend the assigned list of task to robots
        if len(self.robot_spec)==0:
            self.robot_spec += "(" + new_task +")"
        else:
            self.robot_spec+="& (" + new_task +")"

    def get_waypoint(self,msg):
        self.current_waypoint=msg.data
        #rospy.loginfo("my current waypoint is : %s", self.current_waypoint)

if __name__ == '__main__':
    filtered_argv = rospy.myargv(argv=sys.argv)
    print filtered_argv
    robot_id = filtered_argv[1]
    robot=Robot(robot_id)
