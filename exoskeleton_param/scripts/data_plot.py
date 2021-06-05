#!/usr/bin/python3
# -*- coding: utf-8 -*-
import time
import csv
import sys
import matplotlib.pyplot as plt


class Data_Plot(object):

    def __init__(self):
        self.global_file_path = sys.path[0].replace("scripts","data/")
        print(self.global_file_path)
        self.time  = []
        self.L_A_P = []
        self.L_C_P = []
        self.L_T_P = []
        self.L_H_P = []
        self.center_point = []

    def read_csv_file(self, file_name, class_name):
        data_file_path = self.global_file_path+file_name
        csv_file       = open(data_file_path,'r')
        reader         = csv.reader(csv_file)

        for item in reader:
            if reader.line_num == 1:
                continue
            self.time.append(float(item[0]))
            self.L_A_P.append(float(item[1]))
            self.L_C_P.append(float(item[2]))
            self.L_T_P.append(float(item[3]))
            self.L_H_P.append(float(item[4]))

        plt.plot(self.time, self.L_A_P, label="踝关节"+class_name)
        plt.plot(self.time, self.L_C_P, label="小腿关节"+class_name)
        plt.plot(self.time, self.L_T_P, label="大腿关节"+class_name)
        plt.plot(self.time, self.L_H_P, label="髋关节"+class_name)
        plt.ylabel(class_name)
        plt.xlabel("时间")
        plt.title("关节"+class_name)
        plt.show()

    def read_center_file(self):
        data_file_path = self.global_file_path+"center_point.csv"
        csv_file       = open(data_file_path,'r')
        reader         = csv.reader(csv_file)

        for item in reader:
            if reader.line_num == 1:
                self.time.clear()
                continue
            self.time.append(float(item[0]))
            self.center_point.append(float(item[1]))
        print(len(self.time))
        print(len(self.center_point))
        plt.plot(self.time, self.center_point, label="中心位置")
        plt.ylabel("中心位置")
        plt.xlabel("时间")
        plt.title("中心位置")
        plt.show()
if __name__=="__main__":
    data_plot = Data_Plot()
    data_plot.read_csv_file("position.csv","角度")
    data_plot.read_csv_file("effort.csv","力矩")
    data_plot.read_center_file()

