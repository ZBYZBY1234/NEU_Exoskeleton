import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import numpy as np
import matplotlib.pyplot as plt

import random

import numpy as np
import math
#--------------------------------LSTM Module Class--------------------------------#
def sigmoid(x): 
    return 1. / (1 + np.exp(-x))

def sigmoid_derivative(values): 
    return values*(1-values)

def tanh_derivative(values): 
    return 1. - values ** 2

# createst uniform random array w/ values in [a,b) and shape args
def rand_arr(a, b, *args): 
    np.random.seed(0)
    return np.random.rand(*args) * (b - a) + a

class LstmParam:
    def __init__(self, mem_cell_ct, x_dim):
        self.mem_cell_ct = mem_cell_ct
        self.x_dim = x_dim
        concat_len = x_dim + mem_cell_ct
        # weight matrices
        self.wg = rand_arr(-0.1, 0.1, mem_cell_ct, concat_len)
        self.wi = rand_arr(-0.1, 0.1, mem_cell_ct, concat_len) 
        self.wf = rand_arr(-0.1, 0.1, mem_cell_ct, concat_len)
        self.wo = rand_arr(-0.1, 0.1, mem_cell_ct, concat_len)
        # bias terms
        self.bg = rand_arr(-0.1, 0.1, mem_cell_ct) 
        self.bi = rand_arr(-0.1, 0.1, mem_cell_ct) 
        self.bf = rand_arr(-0.1, 0.1, mem_cell_ct) 
        self.bo = rand_arr(-0.1, 0.1, mem_cell_ct) 
        # diffs (derivative of loss function w.r.t. all parameters)
        self.wg_diff = np.zeros((mem_cell_ct, concat_len)) 
        self.wi_diff = np.zeros((mem_cell_ct, concat_len)) 
        self.wf_diff = np.zeros((mem_cell_ct, concat_len)) 
        self.wo_diff = np.zeros((mem_cell_ct, concat_len)) 
        self.bg_diff = np.zeros(mem_cell_ct) 
        self.bi_diff = np.zeros(mem_cell_ct) 
        self.bf_diff = np.zeros(mem_cell_ct) 
        self.bo_diff = np.zeros(mem_cell_ct) 

    def apply_diff(self, lr = 1):
        self.wg -= lr * self.wg_diff
        self.wi -= lr * self.wi_diff
        self.wf -= lr * self.wf_diff
        self.wo -= lr * self.wo_diff
        self.bg -= lr * self.bg_diff
        self.bi -= lr * self.bi_diff
        self.bf -= lr * self.bf_diff
        self.bo -= lr * self.bo_diff
        # reset diffs to zero
        self.wg_diff = np.zeros_like(self.wg)
        self.wi_diff = np.zeros_like(self.wi) 
        self.wf_diff = np.zeros_like(self.wf) 
        self.wo_diff = np.zeros_like(self.wo) 
        self.bg_diff = np.zeros_like(self.bg)
        self.bi_diff = np.zeros_like(self.bi) 
        self.bf_diff = np.zeros_like(self.bf) 
        self.bo_diff = np.zeros_like(self.bo) 

class LstmState:
    def __init__(self, mem_cell_ct, x_dim):
        self.g = np.zeros(mem_cell_ct)
        self.i = np.zeros(mem_cell_ct)
        self.f = np.zeros(mem_cell_ct)
        self.o = np.zeros(mem_cell_ct)
        self.s = np.zeros(mem_cell_ct)
        self.h = np.zeros(mem_cell_ct)
        self.bottom_diff_h = np.zeros_like(self.h)
        self.bottom_diff_s = np.zeros_like(self.s)
    
class LstmNode:
    def __init__(self, lstm_param, lstm_state):
        # store reference to parameters and to activations
        self.state = lstm_state
        self.param = lstm_param
        # non-recurrent input concatenated with recurrent input
        self.xc = None

    def bottom_data_is(self, x, s_prev = None, h_prev = None):
        # if this is the first lstm node in the network
        if s_prev is None: s_prev = np.zeros_like(self.state.s)
        if h_prev is None: h_prev = np.zeros_like(self.state.h)
        # save data for use in backprop
        self.s_prev = s_prev
        self.h_prev = h_prev

        # concatenate x(t) and h(t-1)
        xc = np.hstack((x,  h_prev))
        self.state.g = np.tanh(np.dot(self.param.wg, xc) + self.param.bg)
        self.state.i = sigmoid(np.dot(self.param.wi, xc) + self.param.bi)
        self.state.f = sigmoid(np.dot(self.param.wf, xc) + self.param.bf)
        self.state.o = sigmoid(np.dot(self.param.wo, xc) + self.param.bo)
        self.state.s = self.state.g * self.state.i + s_prev * self.state.f
        self.state.h = self.state.s * self.state.o

        self.xc = xc
    
    def top_diff_is(self, top_diff_h, top_diff_s):
        # notice that top_diff_s is carried along the constant error carousel
        ds = self.state.o * top_diff_h + top_diff_s
        do = self.state.s * top_diff_h
        di = self.state.g * ds
        dg = self.state.i * ds
        df = self.s_prev * ds

        # diffs w.r.t. vector inside sigma / tanh function
        di_input = sigmoid_derivative(self.state.i) * di 
        df_input = sigmoid_derivative(self.state.f) * df 
        do_input = sigmoid_derivative(self.state.o) * do 
        dg_input = tanh_derivative(self.state.g) * dg

        # diffs w.r.t. inputs
        self.param.wi_diff += np.outer(di_input, self.xc)
        self.param.wf_diff += np.outer(df_input, self.xc)
        self.param.wo_diff += np.outer(do_input, self.xc)
        self.param.wg_diff += np.outer(dg_input, self.xc)
        self.param.bi_diff += di_input
        self.param.bf_diff += df_input       
        self.param.bo_diff += do_input
        self.param.bg_diff += dg_input       

        # compute bottom diff
        dxc = np.zeros_like(self.xc)
        dxc += np.dot(self.param.wi.T, di_input)
        dxc += np.dot(self.param.wf.T, df_input)
        dxc += np.dot(self.param.wo.T, do_input)
        dxc += np.dot(self.param.wg.T, dg_input)

        # save bottom diffs
        self.state.bottom_diff_s = ds * self.state.f
        self.state.bottom_diff_h = dxc[self.param.x_dim:]

class LstmNetwork():
    def __init__(self, lstm_param):
        self.lstm_param = lstm_param
        self.lstm_node_list = []
        # input sequence
        self.x_list = []

    def y_list_is(self, y_list, loss_layer):
        """
        Updates diffs by setting target sequence 
        with corresponding loss layer. 
        Will *NOT* update parameters.  To update parameters,
        call self.lstm_param.apply_diff()
        """
        assert len(y_list) == len(self.x_list)
        idx = len(self.x_list) - 1
        # first node only gets diffs from label ...
        loss = loss_layer.loss(self.lstm_node_list[idx].state.h, y_list[idx])
        diff_h = loss_layer.bottom_diff(self.lstm_node_list[idx].state.h, y_list[idx])
        # here s is not affecting loss due to h(t+1), hence we set equal to zero
        diff_s = np.zeros(self.lstm_param.mem_cell_ct)
        self.lstm_node_list[idx].top_diff_is(diff_h, diff_s)
        idx -= 1

        ### ... following nodes also get diffs from next nodes, hence we add diffs to diff_h
        ### we also propagate error along constant error carousel using diff_s
        while idx >= 0:
            loss += loss_layer.loss(self.lstm_node_list[idx].state.h, y_list[idx])
            diff_h = loss_layer.bottom_diff(self.lstm_node_list[idx].state.h, y_list[idx])
            diff_h += self.lstm_node_list[idx + 1].state.bottom_diff_h
            diff_s = self.lstm_node_list[idx + 1].state.bottom_diff_s
            self.lstm_node_list[idx].top_diff_is(diff_h, diff_s)
            idx -= 1 

        return loss

    def x_list_clear(self):
        self.x_list = []

    def x_list_add(self, x):
        self.x_list.append(x)
        if len(self.x_list) > len(self.lstm_node_list):
            # need to add new lstm node, create new state mem
            lstm_state = LstmState(self.lstm_param.mem_cell_ct, self.lstm_param.x_dim)
            self.lstm_node_list.append(LstmNode(self.lstm_param, lstm_state))

        # get index of most recent x input
        idx = len(self.x_list) - 1
        if idx == 0:
            # no recurrent inputs yet
            self.lstm_node_list[idx].bottom_data_is(x)
        else:
            s_prev = self.lstm_node_list[idx - 1].state.s
            h_prev = self.lstm_node_list[idx - 1].state.h
            self.lstm_node_list[idx].bottom_data_is(x, s_prev, h_prev)
#--------------------------------LSTM Module Class--------------------------------#


#迭代训练次数
M = 200
class ToyLossLayer:
    """
    Computes square loss with first element of hidden layer array.
    """
    @classmethod
    def loss(self, pred, label):
        return (pred[0] - label) ** 2

    @classmethod
    def bottom_diff(self, pred, label):
        diff = np.zeros_like(pred)
        diff[0] = 2 * (pred[0] - label)
        return diff

class LSTM_Subscriber(Node):

    def __init__(self):

        np.random.seed(0)

        np.random.seed(0)
        train_y = [[1,0],[0,1]]
        x_dim = 8
        train_x = [np.random.random(x_dim) for _ in range(len(train_y))]

        self.lstm_net   = []
        self.N          = 2
        self.LSTM_Train(train_x,train_y, self.N)
        self.gait_times = 0

        super().__init__('LSTM_Subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'Sensor_Gait',
            self.gait_data_callback,
            10
        )
        self.subscription


    def gait_data_callback(self,msg):
        data = msg.data
        for i in range(len(self.lstm_net)):
            self.self.lstm_net[i].x_list_add(data)

        for k in range(len(self.self.lstm_net)):

            print("y_pred = [" +
                    ", ".join(["% 2.5f" % self.self.lstm_net[k].lstm_node_list[ind].state.h[0] for ind in range(self.N)]) +
                    "]", end=", ")
            self.self.lstm_net[k].x_list_clear()
    '''
    LSTM_NN: 输入数据集: train_x, train_y 当前时刻LSTM个数对应train_y的维度: N
    '''
    def LSTM_Train(self,train_x,train_y, N):
        '''
        x_dim:          输入数据及train_x的横向维度
        mem_cell_ct:    数据链保存数据量最大个数
        M:              迭代训练次数
        '''
        mem_cell_ct = 200
        x_dim = len(train_x[0])


        lstm_param = []
        y_list     = []

        # 对N个LSTM模块进行初始化
        for i in range(N):
            lstm_param_i = LstmParam(mem_cell_ct, x_dim)
            lstm_param.append(lstm_param_i)

            lstm_net_i   = LstmNetwork(lstm_param_i)
            self.lstm_net.append(lstm_net_i)

        # 对训练数据集train_y进行按照对应的LSTM进行分块
        for i in range(N):
            y_i = []
            for j in range(len(train_y)):
                y_i.append(train_y[j][i])
            y_list.append(y_i)

        # 迭代训练
        loss_1 = []
        loss_2 = []
        loss = [loss_1,loss_2]
        for i in range(M):
            print("iter", "%2s" % str(i), end=": \n")

            # 为每个LSTM模块添加训练输入数据集train_x
            for ind in range(len(train_x)):
                for j in range(len(self.lstm_net)):
                    self.lstm_net[j].x_list_add(train_x[ind])

            for k in range(len(self.lstm_net)):

                print("y_pred = [" +
                        ", ".join(["% 2.5f" % self.lstm_net[k].lstm_node_list[ind].state.h[0] for ind in range(len(y_list[k]))]) +
                        "]", end=", ")
                loss_i = self.lstm_net[k].y_list_is(y_list[k], ToyLossLayer)
                print("loss_","%2s"%str(k),": ", "%.3e" % loss_i)
                loss[k].append(loss_i)

                lstm_param[k].apply_diff(lr=0.1)
                self.lstm_net[k].x_list_clear()

        # 可视化 Visualization
        x = []
        for i in range(M):
            x.append(i)
        # 创建图像布局对象fig
        fig = plt.figure(figsize = (12, 6))
        # 原图
        plot1 = fig.add_subplot(121)
        plot1.plot(x,loss[0],label="loss")
        plot1.legend()
        plot1.set_title('LSTM 1 Loss')
        plot1.set_xlabel('Iteration/time')
        plot1.set_ylabel('Loss')

        plot2 = fig.add_subplot(122)
        plot2.plot(x,loss[1],label="loss")
        plot2.legend()
        plot2.set_title('LSTM 2 Loss')
        plot2.set_xlabel('Iteration/time')
        plot2.set_ylabel('Loss')

        plt.legend()
        plt.show()

def main(args=None):

    np.random.seed(0)
    y_list = [[1,0],[0,1]]
    x_dim = 8
    input_val_arr = [np.random.random(x_dim) for _ in range(len(y_list))]

    # loss = LSTM_NN(input_val_arr,y_list, 2)



    rclpy.init(args=args)

    lstm_subscriber = LSTM_Subscriber()

    rclpy.spin(lstm_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lstm_subscriber.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()