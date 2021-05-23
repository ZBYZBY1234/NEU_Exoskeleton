import numpy as np
import matplotlib.pyplot as plt
from lstm_method import LstmParam, LstmNetwork
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

'''
LSTM_NN: 输入数据集: train_x, train_y 当前时刻LSTM个数对应train_y的维度: N
'''
def LSTM_NN(train_x,train_y, N):
    '''
    x_dim:          输入数据及train_x的横向维度
    mem_cell_ct:    数据链保存数据量最大个数
    M:              迭代训练次数
    '''
    mem_cell_ct = 200
    x_dim = len(train_x[0])


    lstm_param = []
    lstm_net   = []
    y_list     = []

    # 对N个LSTM模块进行初始化
    for i in range(N):
        lstm_param_i = LstmParam(mem_cell_ct, x_dim)
        lstm_param.append(lstm_param_i)

        lstm_net_i   = LstmNetwork(lstm_param_i)
        lstm_net.append(lstm_net_i)

    # 对训练数据集train_y进行按照对应的LSTM进行分块
    for i in range(N):
        y_i = []
        for j in range(len(train_y)):
            y_i.append(train_y[j][i])
        y_list.append(y_i)

    # 迭代训练
    loss_1 = []
    loss_2 = []
    loss_3 = []
    loss_4 = []
    loss_5 = []
    loss_6 = []

    loss = [loss_1,loss_2,loss_3,loss_4,loss_5,loss_6]

    for i in range(M):
        print("iter", "%2s" % str(i), end=": \n")

        # 为每个LSTM模块添加训练输入数据集train_x
        for ind in range(len(train_x)):
            for j in range(len(lstm_net)):
                lstm_net[j].x_list_add(train_x[ind])

        for k in range(len(lstm_net)):

            print("y_pred = [" +
                    ", ".join(["% 2.5f" % lstm_net[k].lstm_node_list[ind].state.h[0] for ind in range(len(y_list[k]))]) +
                    "]", end=", ")
            loss_i = lstm_net[k].y_list_is(y_list[k], ToyLossLayer)
            print("loss_","%2s"%str(k),": ", "%.3e" % loss_i)
            loss[k].append(loss_i)

            lstm_param[k].apply_diff(lr=0.1)
            lstm_net[k].x_list_clear()
    return loss

def test():
    np.random.seed(0)
    y_list = [
                [0.1,0.9,0.8,0.2,0.3,0.4],
                [0.2,0.5,0.2,0.4,0.5,0.6],
                [0.3,0.6,0.4,0.3,0.2,0.2],
                [0.4,0.4,0.6,0.8,0.4,0.1],
                [0.5,0.2,0.5,0.6,0.5,0.3],
                [0.6,0.3,0.9,0.5,0.2,0.4]
            ]
    x_dim = 8
    input_val_arr = [np.random.random(x_dim) for _ in range(len(y_list))]

    loss = LSTM_NN(input_val_arr,y_list, 6)
    print(loss)
    x = []
    for i in range(M):
        x.append(i)

    # 创建图像布局对象fig
    fig = plt.figure(figsize = (12, 6))
    # 原图
    plot1 = fig.add_subplot(231)
    plot1.plot(x,loss[0],label="loss")
    plot1.legend()
    plot1.set_title('LSTM 1 Loss')
    plot1.set_xlabel('Iteration/time')
    plot1.set_ylabel('Loss')

    plot2 = fig.add_subplot(232)
    plot2.plot(x,loss[1],label="loss")
    plot2.legend()
    plot2.set_title('LSTM 2 Loss')
    plot2.set_xlabel('Iteration/time')
    plot2.set_ylabel('Loss')

    plot3 = fig.add_subplot(233)
    plot3.plot(x,loss[2],label="loss")
    plot3.legend()
    plot3.set_title('LSTM 3 Loss')
    plot3.set_xlabel('Iteration/time')
    plot3.set_ylabel('Loss')

    plot4 = fig.add_subplot(234)
    plot4.plot(x,loss[3],label="loss")
    plot4.legend()
    plot4.set_title('LSTM 3 Loss')
    plot4.set_xlabel('Iteration/time')
    plot4.set_ylabel('Loss')

    plot5 = fig.add_subplot(235)
    plot5.plot(x,loss[3],label="loss")
    plot5.legend()
    plot5.set_title('LSTM 3 Loss')
    plot5.set_xlabel('Iteration/time')
    plot5.set_ylabel('Loss')

    plot6 = fig.add_subplot(236)
    plot6.plot(x,loss[3],label="loss")
    plot6.legend()
    plot6.set_title('LSTM 3 Loss')
    plot6.set_xlabel('Iteration/time')
    plot6.set_ylabel('Loss')

    plt.legend()
    plt.show()

if __name__ == "__main__":
    # main()
    test()