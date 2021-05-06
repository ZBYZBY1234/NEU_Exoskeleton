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
        w_x = np.random.uniform(-1e-4, 1e-4, (self.state_width, self.input_width))
        b   = np.zeros((self.state_width,1))

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
        # 公式(1)
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
        + input_gate*cell_t)
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


    def backward(self, x, delta_h, activator):
        """
        实现LSTM训练算法
        """
        self.calc_delta(delta_h, activator)
        self.calc_gradient(x)

    def update(self):
        '''
        按照梯度下降，更新权重
        '''
        self.Wfh -= self.learning_rate * self.Whf_grad
        self.Wfx -= self.learning_rate * self.Whx_grad
        self.bf -= self.learning_rate * self.bf_grad
        self.Wih -= self.learning_rate * self.Whi_grad
        self.Wix -= self.learning_rate * self.Whi_grad
        self.bi -= self.learning_rate * self.bi_grad
        self.Woh -= self.learning_rate * self.Wof_grad
        self.Wox -= self.learning_rate * self.Wox_grad
        self.bo -= self.learning_rate * self.bo_grad
        self.Wch -= self.learning_rate * self.Wcf_grad
        self.Wcx -= self.learning_rate * self.Wcx_grad
        self.bc -= self.learning_rate * self.bc_grad

    def calc_delta(self, delta_h, activator):
        # 初始化各个时刻的误差项
        # 输出误差项
        self.delta_h_list  = self.__init__delta()
        # 输出门误差项
        self.delta_o_list  = self.__init__delta()
        # 输入门误差项
        self.delta_i_list  = self.__init__delta()
        # 遗忘门误差项
        self.delta_f_list  = self.__init__delta()
        # 即时输出误差项
        self.delta_ct_list = self.__init__delta()

        # 保存从上一层传递下来的当前时刻的误差项
        self.delta_h_list[-1] = delta_h

        # 迭代计算每个时刻的误差项
        for k in range(self.times, 0, -1):
            self.calc_delta_k(k)

    def __init__delta(self):
        """
        初始化误差项
        """
        delta_list = []
        for i in range(self.times + 1):
            delta_list.append(np.zeros(
                (self.state_width, 1)
            ))
        return delta_list

    def calc_delta_k(self, k):
        """
        根据k时刻的delta_h, 计算k时刻的delta_f、
        delta_i、delta_o、delta_ct，以及k-1时刻的delta_h
        """

        # 获得k时刻前向计算的值大小
        input_gate  = self.i_list[k]
        output_gate = self.o_list[k]
        forget_gate = self.f_list[k]
        cell_t      = self.ct_list[k]
        cell        = self.c_list[k]

        cell_prev   = self.c_list[k-1]
        tanh_cell   = self.output_activator.forward(cell)
        delta_k     = self.delta_h_list[k]

        # 根据公式9计算delta_o
        delta_output = (delta_k * tanh_cell *
                self.gate_activator.backward(output_gate))
        delta_forget = (delta_k * output_gate *
                (1 - tanh_cell * tanh_cell) * cell_prev *
                self.gate_activator.backward(forget_gate))
        delta_input  = (delta_k * output_gate *
                (1 - tanh_cell * tanh_cell) * cell_t *
                self.gate_activator.backward(input_gate))
        delta_cell_t = (delta_k * output_gate *
                (1 - tanh_cell * tanh_cell) * input_gate *
                self.output_activator.backward(cell_t))
        delta_h_prev = (
            np.dot(delta_output.transpose(), self.Woh) +
            np.dot(delta_input.transpose(), self.Wih) +
            np.dot(delta_forget.transpose(), self.Wfh) +
            np.dot(delta_cell_t.transpose(), self.Wch)
        ).transpose()

        # 保存全部delta值大小
        self.delta_h_list[k-1] = delta_h_prev
        self.delta_f_list[k]   = delta_forget
        self.delta_i_list[k]   = delta_input
        self.delta_o_list[k]   = delta_output
        self.delta_ct_list[k]  = delta_cell_t

    def calc_gradient(self, x):
        # 初始化遗忘门权重梯度矩阵和偏置项
        self.Wfh_grad, self.Wfx_grad, self.bf_grad = (
            self.init_weight_gradient_mat())
        # 初始化输入门权重梯度矩阵和偏置项
        self.Wih_grad, self.Wix_grad, self.bi_grad = (
            self.init_weight_gradient_mat())
        # 初始化输出门权重梯度矩阵和偏置项
        self.Woh_grad, self.Wox_grad, self.bo_grad = (
            self.init_weight_gradient_mat())
        # 初始化单元状态权重梯度矩阵和偏置项
        self.Wch_grad, self.Wcx_grad, self.bc_grad = (
            self.init_weight_gradient_mat())

        # 计算对上一次输出h的权重梯度
        for t in range(self.times, 0, -1):
            # 计算各个时刻的梯度
            (Wfh_grad, bf_grad,
            Wih_grad, bi_grad,
            Woh_grad, bo_grad,
            Wch_grad, bc_grad) = (
                self.calc_gradient_t(t))
            # 实际梯度是各时刻梯度之和
            self.Wfh_grad += Wfh_grad
            self.bf_grad += bf_grad
            self.Wih_grad += Wih_grad
            self.bi_grad += bi_grad
            self.Woh_grad += Woh_grad
            self.bo_grad += bo_grad
            self.Wch_grad += Wch_grad
            self.bc_grad += bc_grad

        # 计算对本次输入x的权重梯度
        xt = x.transpose()
        self.Wfx_grad = np.dot(self.delta_f_list[-1], xt)
        self.Wix_grad = np.dot(self.delta_i_list[-1], xt)
        self.Wox_grad = np.dot(self.delta_o_list[-1], xt)
        self.Wcx_grad = np.dot(self.delta_ct_list[-1], xt)

    def init_weight_gradient_mat(self):
        '''
        初始化权重矩阵
        '''
        Wh_grad = np.zeros((self.state_width,
            self.state_width))
        Wx_grad = np.zeros((self.state_width,
            self.input_width))
        b_grad = np.zeros((self.state_width, 1))
        return Wh_grad, Wx_grad, b_grad
    def calc_gradient_t(self, t):
        '''
        计算每个时刻t权重的梯度
        '''
        h_prev = self.h_list[t-1].transpose()
        Wfh_grad = np.dot(self.delta_f_list[t], h_prev)
        bf_grad = self.delta_f_list[t]
        Wih_grad = np.dot(self.delta_i_list[t], h_prev)
        bi_grad = self.delta_f_list[t]
        Woh_grad = np.dot(self.delta_o_list[t], h_prev)
        bo_grad = self.delta_f_list[t]
        Wch_grad = np.dot(self.delta_ct_list[t], h_prev)
        bc_grad = self.delta_ct_list[t]
        return Wfh_grad, bf_grad, Wih_grad, bi_grad, \
                Woh_grad, bo_grad, Wch_grad, bc_grad
    def reset_state(self):
        # 当前时刻初始化为t0
        self.times = 0
        # 各个时刻的单元状态向量c
        self.c_list = self.__init__state_list()
        # 各个时刻的输出向量h
        self.h_list = self.__init__state_list()
        # 各个时刻的遗忘门f
        self.f_list = self.__init__state_list()
        # 各个时刻的输入门i
        self.i_list = self.__init__state_list()
        # 各个时刻的输出门o
        self.o_list = self.__init__state_list()
        # 各个时刻的即时状态c~
        self.ct_list = self.__init__state_list()

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
    l.backward(x[1], d, Identity_Activator())

if __name__ == '__main__':
    main()
