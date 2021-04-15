import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import csv
import os
import sys
topic = 'Sensor'

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.filedir = os.getcwd();
        self.file_path = self.filedir+\
        '/src/pattern_recognition/csv_file/2021_4_17.csv'

        self.csv_head = ['time',
                    'Angle_Thigh',
                    'Angle_Calf',
                    'Presure_Front',
                    'Presure_Middle',
                    'Presure_Back']
        self.mode = 'new'

        self.pre_csv_file(self.file_path,
                        self.csv_head,
                        self.mode)
        self.file = open(self.file_path,'a+',newline='')

    def listener_callback(self, msg):
        self.get_logger().info('I heard and write')
        csv_write = csv.writer(self.file)
        num_list = [msg.data[0],msg.data[1],msg.data[2],
                    msg.data[3],msg.data[4],msg.data[5],
                    msg.data[6],msg.data[7],msg.data[8]]
        csv_write.writerow(num_list)


    def pre_csv_file(self,file_path, csv_head = [], mode='new'):
        '''
        csv file creation
        '''
        def create_csv(csv_head = [], path=file_path):
            with open(path, 'w', newline='') as f:
                csv_write = csv.writer(f)
                if csv_head!=[]:
                    csv_write.writerow(csv_head)
        if mode == 'new':
            if os.path.isfile(file_path):
                os.remove(file_path)
            create_csv(csv_head, file_path)
        elif mode == 'add':
            if os.path.isfile(file_path) == False:
                print('原文件缺失，自动新建')
                create_csv(csv_head, file_path)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()