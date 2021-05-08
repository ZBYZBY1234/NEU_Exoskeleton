# /* * @Author: Beal.MS
#    * @Date: 2021-05-08 13:21:58
#    * @Last Modified by:   Beal.MS
#    * @Last Modified time: 2021-05-08 13:21:58
#    * @Description: 常见滤波算法
# */
import numpy as np

#* 限幅滤波算法 *#
#* Limiting Filter *#
#* Input: 上一个量 Old_Value，当前量 Value， 滤波幅值 Variation_Range
#* Output: 滤波结果量
#* 优点: 能够有效克服因为偶然因粗引起的脉冲干扰
#* 缺点: 无法抑制周期性的干扰

def Limmiting_Filtering(Old_Value, Value, Variation_Range):
    if (abs(Value - Old_Value)>Variation_Range):
        return Old_Value
    else:
        return Value

#* 中位值滤波算法 *#
#* Median Filter *#
#* Input: 待滤波的数据 Value_List, 采样次数 Sample_Times
#* Output: 滤波结果量
#* 优点: 能够有效克服因偶然因素引起的波动干扰
#* 缺点: 对快速变化的不适宜

def Median_Filter(Value_List, Sample_Times):

    Filtered_Value = []

    for i in range(int(len(Value_List)/Sample_Times)):
        Select_Slide = Value_List[i*Sample_Times : (i+1)*Sample_Times]
        Median_Slide = np.median(Select_Slide)
        for j in range(Sample_Times):
            Filtered_Value.append(Median_Slide)

    # 当采样次数无法整除数据量长度时需要考虑剩下的部分
    if(len(Value_List)%Sample_Times != 0):
        Select_Slide = Value_List[(i+1)*Sample_Times:]
        Median_Slide = np.median(Select_Slide)
        for j in range(len(Value_List)%Sample_Times):
            Filtered_Value.append(Median_Slide)

    return Filtered_Value

#* 算术平均滤波算法 *#
#* Arithmetic Average Filter *#
#* Input: 待滤波的数据 Value_List, 采样次数 Sample_Times
#* Output: 滤波结果量
#* 优点: 适用于对一般具有随即干扰的信号进行滤波
#* 缺点: 对于测量速度较慢或要求数据计算速度较快的实时控制不适用

def Arithmetic_Average_Filter(Value_List, Sample_Times):

    Filtered_Value = []

    for i in range(int(len(Value_List)/Sample_Times)):
        Select_Slide = Value_List[i*Sample_Times : (i+1)*Sample_Times]
        Mean_Slide = np.mean(Select_Slide)
        for j in range(Sample_Times):
            Filtered_Value.append(Mean_Slide)

    # 当采样次数无法整除数据量长度时需要考虑剩下的部分
    if(len(Value_List)%Sample_Times != 0):
        Select_Slide = Value_List[(i+1)*Sample_Times:]
        Mean_Slide = np.mean(Select_Slide)
        for j in range(len(Value_List)%Sample_Times):
            Filtered_Value.append(Mean_Slide)

    return Filtered_Value


def Recursive_Average_Filter(Value_List, Sample_Times):

    Mean_List = []
    Select_Slide = Value_List[0:0+Sample_Times]
    mean = np.mean(Select_Slide)
    Select_Slide.insert(len(Select_Slide), Select_Slide[0]) #左移动一位
    Select_Slide.remove(Select_Slide[0])
    for i in range(Sample_Times):
        Mean_List.append(mean)

    for i in range(len(Value_List) - Sample_Times):
        Select_Slide.append(Value_List[i+Sample_Times])
        Select_Slide.insert(len(Select_Slide), Select_Slide[0])
        Select_Slide.remove(Select_Slide[0])
        Select_Slide.remove(Select_Slide[-1])
        mean = np.mean(Select_Slide)

        Mean_List.append(mean)

    return Mean_List