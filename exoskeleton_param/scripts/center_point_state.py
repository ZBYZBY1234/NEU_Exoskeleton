#!/usr/bin/python3
# -*- coding: utf-8 -*-
import time
import csv
import sys
import rospy
from std_msgs.msg import Float64

class Center_Point_State_Generator():

    def __init__(self):
        self.file_global_path = sys.path[0].replace("scripts","data/")
        self.joint_state_sub  = rospy.Subscriber("center_point" ,data_class=Float64 ,callback=self.cb ,queue_size=100)
        self.flag = True
        self.start_time = None

    def create_csv(self):
        center_point_head = ['时间','重心位置']
        center_point_path = self.file_global_path+"center_point.csv"
        with open(center_point_path,'w') as f:
            csv_write = csv.writer(f)
            csv_write.writerow(center_point_head)
        f.close()
    
    def cb(self,center_state_msg):
        if self.flag:
            self.start_time = time.time()
            self.flag = False

        end_time = time.time()
        state_list = []
        state_list.append(end_time-self.start_time)
        state_list.append(round(center_state_msg.data,2))

        center_point_path = self.file_global_path+"center_point.csv"
        with open(center_point_path,'a+') as f:
            csv_write = csv.writer(f)
            csv_write.writerow(state_list)
        f.close()

    def main(self):
        rospy.spin()
if __name__=="__main__":
    rospy.init_node("center_point_state_record")
    center_point_state_generator = Center_Point_State_Generator()
    center_point_state_generator.create_csv()
    center_point_state_generator.main()