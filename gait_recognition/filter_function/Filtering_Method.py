# /* * @Author: Beal.MS
#    * @Date: 2021-05-08 13:21:58
#    * @Last Modified by:   Beal.MS
#    * @Last Modified time: 2021-05-08 13:21:58
#    * @Description: 限幅滤波算法
# */
def Limmiting_Filtering(Old_Value, Value, Variation_Range):
    if (abs(Value - Old_Value)>Variation_Range):
        return Old_Value
    else:
        return Value

