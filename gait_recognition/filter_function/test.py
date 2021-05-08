import matplotlib.pyplot as plt
import numpy as np
plt.figure(figsize = (20,8))

s = np.random.normal(0,25,400)
plt.plot(s,label = '原始数据')

n = 7     #一次采样的次数
rank = []
for i in range(int(len(s)/n)):
    select_s = s[i*n:(i+1)*n]     #切片选取一次采样的个数
    mid_s = np.median(select_s)
    for j in range(n):       #将取到的值都赋值为中值，方便画图体现，实际中可以只要一次值
        rank.append(mid_s)

plt.plot(rank,label = '滤波后的数据')
plt.title('中值滤波算法')
plt.legend()
plt.show()
