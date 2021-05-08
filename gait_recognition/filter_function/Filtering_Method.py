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