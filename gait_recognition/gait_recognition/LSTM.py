import numpy as np
from gait_recognition.Activators import *
from gait_recognition.Error_Func import *

class Unit:

    '''----------------注意----------------'''
    # Unit 表示为一层神经网络，每个神经元通过列表list，向前向后计算以及时间计数times实现
    # 这里面包涵两类list：
    #                     1、向前传播各个门的输出list + h & c（共6个）；
    #                     2、反向传播各个门的误差项list + 返回误差项h（共5个）。
    # 假设：x的输入向量长度为 n ；其他输出的向量大小均为 t
    def __init__(self, input_x_size, state_size, learning_rate):
        # 设置LSTM中各个门的激活函数和状态c的激活函数
        self.f_acti, self.i_acti, self.ct_acti, self.o_acti, self.c_acti \
            = self.set_init_activator(Sigmoid(), Sigmoid(), Tanh(), Sigmoid(), Tanh())
        # 设置输入和输出数据的长度
        self.input_x_size = input_x_size  # n维
        self.state_size = state_size  # t维
        # 设置学习率
        self.learning_rate = learning_rate
        # 初始化各个门的权重w 和偏执项b
        self.f_wh, self.f_wx, self.f_b = self.__init_weight_and_bias()
        self.i_wh, self.i_wx, self.i_b = self.__init_weight_and_bias()
        self.ct_wh, self.ct_wx, self.ct_b = self.__init_weight_and_bias()
        self.o_wh, self.o_wx, self.o_b = self.__init_weight_and_bias()
        # 初始化当前的时间层（默认为0）
        self.times = 0
        self.h_list, self.c_list, self.f_list, self.i_list, self.ct_list, self.o_list = self.__init_state_list()


    def __init_weight_and_bias(self):
        """
        计算各种门的 初始权重w 和 偏执项b
        """
        w_h = np.random.uniform(-1e-4, 1e-4, (self.state_size, self.state_size))  # t*t维
        w_x = np.random.uniform(-1e-4, 1e-4, (self.state_size, self.input_x_size))  # t*n 维
        b = np.zeros(self.state_size)  # t 维
        return w_h, w_x, b

    def __init_state_list(self):
        """
        初始化保存4个门各自状态的向量（这里先写出来，感觉后面需要修改）
        """
        c = []
        h = []
        f = []
        i = []
        o = []
        ct = []
        # 这里在列表里加一层0，感觉是为了后面for循环防止溢出的作用，全是 t 维
        c.append(np.zeros(self.state_size))
        h.append(np.zeros(self.state_size))
        f.append(np.zeros(self.state_size))
        i.append(np.zeros(self.state_size))
        o.append(np.zeros(self.state_size))
        ct.append(np.zeros(self.state_size))
        return c, h, f, i, o, ct

    def set_init_activator(self, f, i, ct, o, c):
        """
        初始化激活函数，传入的为激活函数的类，如：Sigmoid()，RELU()...
        """
        return f, i, ct, o, c

    '''----------------向前传播计算----------------'''
    def forward(self, input_x):
        """
        计算各个门的结果和输出h、c的结果，并将结果储存入list
        """
        self.times += 1  # 向前计算一次，时间层+1
        f = self.gate(input_x, self.f_wh, self.f_wx, self.f_b, self.f_acti)
        i = self.gate(input_x, self.i_wh, self.i_wx, self.i_b, self.i_acti)
        ct = self.gate(input_x, self.ct_wh, self.ct_wx, self.ct_b, self.ct_acti)
        o = self.gate(input_x, self.o_wh, self.o_wx, self.o_b, self.o_acti)
        c = f * self.c_list[self.times - 1] + i * ct
        h = o * self.c_acti.forward(c)
        # 将计算结果存储到列表里
        self.c_list.append(c)
        self.h_list.append(h)
        self.f_list.append(f)
        self.i_list.append(i)
        self.ct_list.append(ct)
        self.o_list.append(o)
        return h

    def gate(self, x, wh, wx, b, activator_f):
        """
        向前传播——计算各个门的输出
        """
        h = np.array(self.h_list[self.times - 1])
        combo_h_x = list(h)+x
        combo_w_for_hx = np.column_stack((wh, wx))  # 向量列合并用column_stack，行合并用row_stack
        net = np.dot(combo_w_for_hx, combo_h_x) + b
        return activator_f.forward(net)

    '''----------------反向传播计算----------------'''

    def backward(self, x, delta_h):
        """计算sigema和 梯度"""
        self.calc_delta(delta_h)
        self.calc_gradient(x)

    def calc_delta(self, delta_h):
        # 初始化各个时刻的误差项
        self.delta_h_list = self.__init_delta_list()  # 输出误差项
        self.delta_o_list = self.__init_delta_list()  # 输出门误差项
        self.delta_i_list = self.__init_delta_list()  # 输入门误差项
        self.delta_f_list = self.__init_delta_list()  # 遗忘门误差项
        self.delta_ct_list = self.__init_delta_list()  # 即时输出误差项
        # 初始化各个门的权重w和偏执项b的梯度矩阵
        self.f_wh_grad, self.f_wx_grad, self.f_b_grad = self.__init_weight_and_bias_gradient()
        self.i_wh_grad, self.i_wx_grad, self.i_b_grad = self.__init_weight_and_bias_gradient()
        self.ct_wh_grad, self.ct_wx_grad, self.ct_b_grad = self.__init_weight_and_bias_gradient()
        self.o_wh_grad, self.o_wx_grad, self.o_b_grad = self.__init_weight_and_bias_gradient()

        # 保存从上一层传递下来的当前时刻误差
        self.delta_h_list[-1] = delta_h

        # 迭代计算每个时刻的误差项
        for t in range(self.times, 0, -1):
            self.calc_delta_t(t)

    def __init_delta_list(self):
        """初始化误差项"""
        # 误差项是指每一个神经元返回的误差
        delta_list = []
        for i in range(self.times + 1):
            # 为什么要+1？ 因为最后一层的误差项是直接由E计算的，需要存储上一层的误差项因此要+1
            delta_list.append(np.zeros(self.state_size))  # 这个误差项包含每一层的误差
        return delta_list

    def calc_delta_t(self, t):
        """
        根据k时刻的delta_h，计算k时刻的delta_f、delta_i、delta_o、delta_ct，以及k-1时刻的delta_h
        """
        # 获得k时刻向前计算的值
        f = self.f_list[t]
        i = self.i_list[t]
        ct = self.ct_list[t]
        o = self.o_list[t]
        c = self.c_list[t]
        c_last = self.c_list[t - 1]
        tanh_c = self.c_acti.forward(c)
        h = self.delta_h_list[t]  # 这一项是上一层传递过来的误差项，即sita

        # 计算每一个门的delta，都是t 维的向量
        delta_f = h * o * (1 - tanh_c ** 2) * c_last * self.f_acti.backward(f)
        delta_i = h * o * (1 - tanh_c ** 2) * ct * self.i_acti.backward(i)
        delta_ct = h * o * (1 - tanh_c ** 2) * i * self.ct_acti.backward(ct)
        delta_o = h * tanh_c * self.o_acti.backward(o)
        delta_h_last = (
            np.dot(delta_f, self.f_wh) +
            np.dot(delta_i, self.i_wh) +
            np.dot(delta_ct, self.ct_wh) +
            np.dot(delta_o, self.o_wh)
        ).transpose()

        # 保存全部的delta值
        self.delta_h_list[t - 1] = delta_h_last
        self.delta_f_list[t] = delta_f
        self.delta_i_list[t] = delta_i
        self.delta_o_list[t] = delta_o
        self.delta_ct_list[t] = delta_ct

    def calc_gradient(self, x):
        # 计算对上一次输出h的权重梯度，时间层上的梯度
        for t in range(self.times, 0, -1):
            # 计算各个时刻的梯度
            f_wh_grad, f_b_grad, i_wh_grad, i_b_grad, \
            ct_wh_grad, ct_b_grad, o_wh_grad, o_b_grad = self.calc_gradient_timelayer(t)

            # 实际梯度是各个时刻梯度之和
            self.f_wh_grad += f_wh_grad
            self.f_b_grad += f_b_grad
            self.i_wh_grad += i_wh_grad
            self.i_b_grad += i_b_grad
            self.ct_wh_grad += ct_wh_grad
            self.ct_b_grad += ct_b_grad
            self.o_wh_grad += o_wh_grad
            self.o_b_grad += o_b_grad

        # 为了将2维向量和3维向量生成2*3维矩阵，又需要reshape，非常麻烦，感觉是numpy的问题，没有将array和matrix封装好
        xt = np.array(x).reshape((1, self.input_x_size))
        self.f_wx_grad = np.dot(np.array(self.delta_f_list[-1]).reshape((self.state_size, 1)), xt)
        self.i_wx_grad = np.dot(np.array(self.delta_i_list[-1]).reshape((self.state_size, 1)), xt)
        self.ct_wx_grad = np.dot(np.array(self.delta_ct_list[-1]).reshape((self.state_size, 1)), xt)
        self.o_wx_grad = np.dot(np.array(self.delta_o_list[-1]).reshape((self.state_size, 1)), xt)
        # 这里为什么是-1，因为每一层权重共享，所以，wx的梯度只要计算最后一个就够了

    # 这个方法中，原文存在错误，现在这个应该是对的
    # 这里需要疯狂的 reshape ，估计是numpy的问题，非常麻烦
    def calc_gradient_timelayer(self, t):
        """
        计算对于时间层w的梯度
        """
        h_last = np.array(self.h_list[t - 1]).reshape((1, self.state_size))
        f_wh_grad = np.dot(self.delta_f_list[t].reshape((self.state_size, 1)), h_last)
        f_b_grad = self.delta_f_list[t].reshape((1, self.state_size))
        i_wh_grad = np.dot(self.delta_i_list[t].reshape((self.state_size, 1)), h_last)
        i_b_grad = self.delta_i_list[t].reshape((1, self.state_size))
        ct_wh_grad = np.dot(self.delta_ct_list[t].reshape((self.state_size, 1)), h_last)
        ct_b_grad = self.delta_ct_list[t].reshape((1, self.state_size))
        o_wh_grad = np.dot(self.delta_o_list[t].reshape((self.state_size, 1)), h_last)
        o_b_grad = self.delta_o_list[t].reshape((1, self.state_size))
        return f_wh_grad, f_b_grad, i_wh_grad, i_b_grad, \
               ct_wh_grad, ct_b_grad, o_wh_grad, o_b_grad

    def __init_weight_and_bias_gradient(self):
        """
        初始化权重w和偏执项b梯度系数矩阵
        """
        wh = np.zeros((self.state_size, self.state_size))
        wx = np.zeros((self.state_size, self.input_x_size))
        b = np.zeros((1, self.state_size))
        return wh, wx, b

    def update(self):
        """
        按照梯度下降，更新权重
        """
        self.f_wh -= self.learning_rate * self.f_wh_grad
        self.f_wx -= self.learning_rate * self.f_wx_grad
        self.f_b -= self.learning_rate * self.f_b_grad
        self.i_wh -= self.learning_rate * self.i_wh_grad
        self.i_wx -= self.learning_rate * self.i_wx_grad
        self.i_b -= self.learning_rate * self.i_b_grad
        self.o_wh -= self.learning_rate * self.o_wh_grad
        self.o_wx -= self.learning_rate * self.o_wx_grad
        self.o_b -= self.learning_rate * self.o_b_grad
        self.ct_wh -= self.learning_rate * self.ct_wh_grad
        self.ct_wx -= self.learning_rate * self.ct_wx_grad
        self.ct_b -= self.learning_rate * self.ct_b_grad

    '''----------梯度检验的方法----------'''
    # 首先将权重和偏执项都设置成 1 方便观察
    def check_initial_list(self):
        self.h_list, self.c_list, self.f_list, self.i_list, self.ct_list, self.o_list = self.__init_state_list()
        self.times = 0

    def check_Ew_sita_list(self, sita, command, row, column):

        if command == 'f':
            self.reset_Ew_sita(self.f_wh, sita, row, column)
        elif command == 'i':
            self.reset_Ew_sita(self.i_wh, sita, row, column)
        elif command == 'ct':
            self.reset_Ew_sita(self.ct_wh, sita, row, column)
        elif command == 'o':
            self.reset_Ew_sita(self.o_wh, sita, row, column)

        self.h_list, self.c_list, self.f_list, self.i_list, self.ct_list, self.o_list = self.__init_state_list()
        self.times = 0

    def reset_Ew_sita(self, w_h, sita, row, column):
        """
        """
        for i in range(len(w_h)):
            for j in range(len(w_h[0])):
                if j ==column and i == row:
                    w_h[i][j] = w_h[i][j]+sita
                else:
                    pass

class WeightCheck:

    def weight_check(self):
        x = [[1,2,3],[2,3,4]]
        y = [1,2]

        u = Unit(3,2,0.1)
        u.forward(x[0])
        u.forward(x[1])

        E = SquareError().normal(u.h_list[-1], y)
        print('_____________________前向计算结果_____________________')
        print(E)
        delta_h = SquareError().delta(u.h_list[-1], y)
        print(delta_h)
        u.backward(x[1], delta_h[1], Identity_Activator())
        list_grad = [u.Wfh_grad]
        print('_____________________反向计算结果_____________________')
        print(list_grad)
        print(np.linalg.det(list_grad))

        sita = 0.0001
        # print(u.f_wh)
        u.check_Ew_sita_list(sita, 'f', 0, 0)
        # print(u.f_wh)
        u.forward(x[0])  # 向前走两步
        u.forward(x[1])  # 向前走两步
        h = u.h_list[-1]
        E1 = SquareError().normal(h, y)  # 计算行列式的值

        u.check_Ew_sita_list(-sita*2, 'f', 0, 0) # 因为前面加了一个sita，所以这里要减去2个sita
        u.forward(x[0])  # 向前走两步
        u.forward(x[1])  # 向前走两步
        h = u.h_list[-1]
        E2 = SquareError().normal(h, y)
        list_appro = (E1 - E2)/(2 * sita)
        print('_____________________检查结果_____________________')
        print(list_appro)
        print(np.sqrt(np.dot(list_appro, list_appro)))#计算向量的模

def data_set():
    x = [   np.array([[1], [2], [3]]),
            np.array([[2], [3], [4]])
        ]
    d = np.array([[1], [2]])
    return x, d

def main(args=None):
    # l = LSTM_Layer(3,2,1e-3)
    # x, d = data_set()
    # l.forward(x[0])
    # l.forward(x[1])
    # l.backward(x[1], d, Identity_Activator())

    wc = WeightCheck()
    wc.weight_check()

if __name__ == '__main__':
    main()
