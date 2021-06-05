#!/usr/bin/python3
# -*- coding: utf-8 -*-
import time
import csv
import sys
import rospy
from sensor_msgs.msg import JointState

# /Left_Hip_Joint_Effort 
# std_msgs/Float64

# 创建csv文件

class Joint_State_Generator():

    def __init__(self):
        self.file_global_path = sys.path[0].replace("scripts","data/")
        self.joint_state_sub  = rospy.Subscriber("lower_limb/joint_states" ,data_class=JointState ,callback=self.cb ,queue_size=100)
        self.flag = True
        self.start_time = None

    def create_csv(self):
        Position_head = ['时间','踝关节位置','小腿关节位置','大腿关节位置','髋关节位置']
        Position_path = self.file_global_path+"position.csv"
        with open(Position_path,'w') as f:
            csv_write = csv.writer(f)
            csv_write.writerow(Position_head)
        f.close()

        Effort_head = ['时间','踝关节力矩','小腿关节力矩','大腿关节力矩','髋关节力矩']
        Effort_path = self.file_global_path+"effort.csv"
        with open(Effort_path,'w') as f:
            self.csv_write = csv.writer(f)
            self.csv_write.writerow(Effort_head)
        f.close()
    
    def cb(self,joint_state_msg):
        if self.flag:
            self.start_time = time.time()
            self.flag = False

        end_time = time.time()
        state_list = []
        state_list.append(end_time-self.start_time)
        state_list.append(round(joint_state_msg.position[0],2))
        state_list.append(round(joint_state_msg.position[1],2))
        state_list.append(round(joint_state_msg.position[2],2))
        state_list.append(round(joint_state_msg.position[3],2))
        Position_path = self.file_global_path+"position.csv"
        with open(Position_path,'a+') as f:
            csv_write = csv.writer(f)
            csv_write.writerow(state_list)
        f.close()

        state_list = []
        state_list.append(end_time-self.start_time)
        state_list.append(round(joint_state_msg.effort[0],2))
        state_list.append(round(joint_state_msg.effort[1],2))
        state_list.append(round(joint_state_msg.effort[2],2))
        state_list.append(round(joint_state_msg.effort[3],2))
        Effort_path = self.file_global_path+"effort.csv"
        with open(Effort_path,'a+') as f:
            self.csv_write = csv.writer(f)
            self.csv_write.writerow(state_list)
        f.close()
    def main(self):
        rospy.spin()
if __name__=="__main__":
    rospy.init_node("joint_state_record")
    joint_state_generator = Joint_State_Generator()
    joint_state_generator.create_csv()
    joint_state_generator.main()