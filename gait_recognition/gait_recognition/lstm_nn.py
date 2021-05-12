import numpy as np

from lstm_method import LstmParam, LstmNetwork


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

def LSTM_NN(train_x,train_y, N):
    mem_cell_ct = 200
    x_dim = len(train_x[0])

    lstm_param = []
    lstm_net   = []
    y_list     = []
    for i in range(N):
        lstm_param_i = LstmParam(mem_cell_ct, x_dim)
        lstm_param.append(lstm_param_i)

        lstm_net_i   = LstmNetwork(lstm_param_i)
        lstm_net.append(lstm_net_i)
    for i in range(len(train_y[0])):
        y_i = []
        for j in range(len(train_y)):
            y_i.append(train_y[j][i])
        y_list.append(y_i)
    print(train_x)
    print(y_list)
    M = 10
    for i in range(M):
        print("iter", "%2s" % str(i), end=": ")

        for ind in range(len(train_x)):
            for j in range(len(lstm_net)):
                lstm_net[j].x_list_add(train_x[ind])

        for k in range(len(lstm_net)):

            print("y_pred = [" +
                    ", ".join(["% 2.5f" % lstm_net[k].lstm_node_list[ind].state.h[0] for ind in range(len(y_list[k]))]) +
                    "]", end=", ")
            loss_i = lstm_net[k].y_list_is(y_list[k], ToyLossLayer)
            print("loss_i:", "%.3e" % loss_i)

            lstm_param[k].apply_diff(lr=0.1)
            lstm_net[k].x_list_clear()

        # print("y_pred = [" +
        #         ", ".join(["% 2.5f" % lstm_net_2.lstm_node_list[ind].state.h[0] for ind in range(len(y_2_list))]) +
        #         "]", end=", ")
        # loss_2 = lstm_net_2.y_list_is(y_2_list, ToyLossLayer)
        # print("loss_2:", "%.3e" % loss_2)

        # lstm_param_1.apply_diff(lr=0.1)
        # lstm_param_2.apply_diff(lr=0.1)
        # lstm_net_1.x_list_clear()
        # lstm_net_2.x_list_clear()

def main():
    # learns to repeat simple sequence from random inputs
    np.random.seed(0)

    # parameters for input data dimension and lstm cell count
    mem_cell_ct = 200
    # xdim: 输入维度
    x_dim = 18

    # LSTM PARAM
    lstm_param_1 = LstmParam(mem_cell_ct, x_dim)
    lstm_param_2 = LstmParam(mem_cell_ct, x_dim)

    # LSTM NET
    lstm_net_1 = LstmNetwork(lstm_param_1)
    lstm_net_2 = LstmNetwork(lstm_param_2)

    # 训练数据集
    # 训练输出 output
    y_1_list = [-0.5, 0.2]
    y_2_list = [0.5,-0.2]

    # 输入列表 input
    input_val_arr = [np.random.random(x_dim) for _ in y_1_list]
    print(len(input_val_arr))

    # 训练迭代次数 N
    N = 50
    for cur_iter in range(N):
        print("iter", "%2s" % str(cur_iter), end=": ")
        for ind in range(len(y_1_list)):
            lstm_net_1.x_list_add(input_val_arr[ind])
            lstm_net_2.x_list_add(input_val_arr[ind])

        print("y_pred = [" +
                ", ".join(["% 2.5f" % lstm_net_1.lstm_node_list[ind].state.h[0] for ind in range(len(y_1_list))]) +
                "]", end=", ")
        loss_1 = lstm_net_1.y_list_is(y_1_list, ToyLossLayer)
        print("loss_1:", "%.3e" % loss_1)

        print("y_pred = [" +
                ", ".join(["% 2.5f" % lstm_net_2.lstm_node_list[ind].state.h[0] for ind in range(len(y_2_list))]) +
                "]", end=", ")
        loss_2 = lstm_net_2.y_list_is(y_2_list, ToyLossLayer)
        print("loss_2:", "%.3e" % loss_2)

        lstm_param_1.apply_diff(lr=0.1)
        lstm_param_2.apply_diff(lr=0.1)
        lstm_net_1.x_list_clear()
        lstm_net_2.x_list_clear()

def test():
    np.random.seed(0)
    y_list = [[0.1,0.2],[0.3,0.4]]
    x_dim = 8
    input_val_arr = [np.random.random(x_dim) for _ in range(len(y_list))]

    LSTM_NN(input_val_arr,y_list, 2)
if __name__ == "__main__":
    # main()
    test()