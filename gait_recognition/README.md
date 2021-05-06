# LSTM

![Screenshot 2021-05-06 22:22:41](/home/hemingshan/Pictures/Screenshot 2021-05-06 22:22:41.png)
$$
f_t = \sigma(W_f*[h_{t-1},x_t]+b_f)\\\tag{1}
[W_f]*[h_{t-1},x_t]=W_{fh}h_{t-1}+W_{fx}x_t
$$

$$
i_t = \sigma(W_i*[h_{t-1},x_t]+b_i)\tag{2}
$$

$$
\hat{c}_t = tanh(W_c*[h_{t-1},x_t]+b_c)\tag{3}
$$

$$
c_t = f_t*c_{t-1}+i_t*\hat{c}_t \tag{4}
$$

$$
o_t = \sigma(W_o*[t_{t-1},x_t]+b_o)\tag{5}
$$

$$
h_t = o_t*tanh(c_t)\tag{6}
$$

  单腿用LSTM测试出摆动项和支撑项，用神经网络配合马尔科夫链来判断整体的步态类型，如跑动，走路，蹲起等。