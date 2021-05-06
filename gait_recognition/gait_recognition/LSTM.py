import numpy as np
from gait_recognition.Activators import *

class LSTM_Layer(object):

    def __init__(   self,
                    input_width,
                    state_width,
                    learning_rate
                ):

        self.input_width   = input_width
        self.state_width   = state_width
        self.learning_rate = learning_rate

        self.gate_activator = Sigmoid_Activator()
        self.output_activator = Tanh_Activator()

        self.times = 0

        self.f_wh, self.f_wx

        #各个时刻的单元状态向量c
        self.c_list = self.__init__state_list()
        # 各个时刻的输出向量h
        self.h_list = self.__init__state_list()
        # 各个时刻的遗忘门 f
        self.f_list = self.__init__state_list()
        # 各个时刻的输入门 i
        self.i_list = self.__init__state_list()
        # 各个时刻的输出门 o
        self.o_list = self.__init__state_list()
        # 各个时刻的即时状态 c~
        self.ct_list = self.__init__state_list()

        # 遗忘门权重矩阵 Wfh, Wfx, 偏置项 bf
        self.Wfh, self.Wfx, self.bf = (
            self.__init__weight_and_bias()
        )
        # 输入门权重矩阵 Wih, Wix, 偏置项 bi
        self.Wih, self.Wix, self.bi = (
            self.__init__weight_and_bias()
        )
        # 输出门权重矩阵 Woh, Wox, 偏置项 bo
        self.Woh, self.Wox, self.bo = (
            self.__init__weight_and_bias()
        )
        # 单元状态矩阵 Wch, Wcx, 偏置项 bc
        self.Wch, self.Wcx, self.bc = (
            self.__init__weight_and_bias()
        )
    def __init__weight_and_bias(self):
        """
        计算各种门的 初始权重w 和 偏置项b
        """
        w_h = np.random.uniform(-1e-4, 1e-4, (self.state_width, self.state_width))
        w_x = np.random.uniform(-1e-4, 1e-4, (self.state_width, self.state_width))
        b   = np.zeros(self.state_width)

        return w_h, w_x, b

    def __init__state_list(self):
        """
        初始化保存状态的向量
        """
        state_vec_list = []
        state_vec_list.append(np.zeros(
            (self.state_width,1)
        ))
        return state_vec_list

    def forward(self, x):
        self.times += 1

        # 遗忘门
        forget_gate = self.calc_gate(
            x, self.Wfx, self.Wfh,
            self.bf, self.gate_activator
        )
        self.f_list.append(forget_gate)

        # 输入门
        input_gate  = self.calc_gate(
            x, self.Wix, self.Wih,
            self.bi, self.gate_activator
        )
        self.i_list.append(input_gate)

        # 输出门
        output_gate = self.calc_gate(
            x, self.Wox, self.Woh,
            self.bo, self.gate_activator
        )
        self.o_list.append(output_gate)

        # 即时状态
        cell_t      = self.calc_gate(
            x, self.Wcx, self.Wch,
            self.bc, self.output_activator
        )
        self.ct_list.append(cell_t)

        # 单元状态
        cell        = (forget_gate * self.c_list[self.times - 1]
        + ig*cell_t)
        self.c_list.append(cell)

        # 输出
        h           = output_gate * self.output_activator.forward(cell)
        self.h_list.append(h)

    def calc_gate(self, x, Wx, Wh, b, activator):
        """
        计算门
        """
        h    = self.h_list[self.times - 1]          # 上一次的LSTM输出
        net  = np.dot(Wh, h) + np.dot(Wx, x) + b
        gate = activator.forward(net)               #activator 在初始化中定义为Sigmoid函数
        return gate

def data_set():
    x = [   np.array([[1], [2], [3]]),
            np.array([[2], [3], [4]])
        ]
    d = np.array([[1], [2]])
    return x, d

def main(args=None):
    l = LSTM_Layer(3,2,1e-3)
    x, d = data_set()
    l.forward(x[0])
    l.forward(x[1])

if __name__ == '__main__':
    main()
