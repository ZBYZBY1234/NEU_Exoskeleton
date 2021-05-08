import Filtering_Method as fm
import csv
import matplotlib.pyplot as plt

csvFile = open('/home/hemingshan/exo_ws/src/sensor_module/csv_File/3.csv','r')
reader  = csv.reader(csvFile)

i = 0
x = []

L_F_P = []
L_M_P = []
L_B_P = []

L_F_P_Filter = []
L_M_P_Filter = []
L_B_P_Filter = []
for item in reader:
    if reader.line_num == 1:
        continue

    L_F_P.append(round(float(item[12]),2))
    L_M_P.append(round(float(item[13]),2))
    L_B_P.append(round(float(item[14]),2))

    if reader.line_num == 2:
        Old_Value = [round(float(item[12]),2),round(float(item[13]),2),round(float(item[14]),2)]
        L_F_P_Filter.append(round(float(item[12]),2))
        L_M_P_Filter.append(round(float(item[13]),2))
        L_B_P_Filter.append(round(float(item[14]),2))

        x.append(i)
        i = i+1
        continue

    else:
        Value = [round(float(item[12]),2),round(float(item[13]),2),round(float(item[14]),2)]

    L_F_P_Filter.append(fm.Limmiting_Filtering(Old_Value[0],Value[0],10))
    L_M_P_Filter.append(fm.Limmiting_Filtering(Old_Value[1],Value[1],10))
    L_B_P_Filter.append(fm.Limmiting_Filtering(Old_Value[2],Value[2],10))

    Old_Value[0] = Value[0]
    Old_Value[1] = Value[1]
    Old_Value[2] = Value[2]

    x.append(i)
    i = i+1
    if i == 500:
        break

# plt.plot(x,L_T_A_Feedbk,label="Left_Thigh_Angle_Feedbk")
# plt.plot(x,L_C_A_Feedbk,label="Left_Calf_Angle_Feedbk")
# plt.plot(x,R_T_A_Feedbk,label="Right_Thigh_Angle_Feedbk")
# plt.plot(x,R_C_A_Feedbk,label="Right_Calf_Angle_Feedbk")


plt.plot(x,L_F_P,label="Left_Front_Pressure")
# plt.plot(x,L_M_P,label="Left_Middle_Pressure")
# plt.plot(x,L_B_P,label="Left_Back_Pressure")

plt.plot(x,L_F_P_Filter,label="Left_Front_Pressure_Filter")
# plt.plot(x,L_M_P_Filter,label="Left_Middle_Pressure_Filter")
# plt.plot(x,L_B_P_Filter,label="Left_Back_Pressure_Filter")
# plt.plot(x,R_F_P,label="Left_Thigh_Angle_Feedbk")
# plt.plot(x,R_M_P,label="Left_Calf_Angle_Feedbk")
# plt.plot(x,R_B_P,label="Right_Thigh_Angle_Feedbk")

plt.legend()
plt.show()