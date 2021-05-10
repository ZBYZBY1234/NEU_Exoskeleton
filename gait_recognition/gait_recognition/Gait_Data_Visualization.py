import csv
import matplotlib.pyplot as plt
from gait_recognition import Limmiting_Filtering_Method

csvFile = open('/home/hemingshan/exo_ws/src/sensor_module/csv_File/6.csv','r')
reader  = csv.reader(csvFile)

i = 0
x = []
L_T_A_Feedbk = []
L_C_A_Feedbk = []
R_T_A_Feedbk = []
R_C_A_Feedbk = []

L_F_P = []
L_M_P = []
L_B_P = []

L_F_P_Filter = []
L_M_P_Filter = []
L_B_P_Filter = []

R_F_P = []
R_M_P = []
R_B_P = []

for item in reader:
    if reader.line_num == 1:
        continue


    # if float(item[12]) > 100:
    #     L_F_P.append(1)
    # else:
    #     L_F_P.append(0)
    # if float(item[13])>100:
    #     L_M_P.append(1)
    # else:
    #     L_M_P.append(0)
    # if float(item[14])>100:
    #     L_B_P.append(1)
    # else:
    #     L_B_P.append(0)
    L_F_P.append(round(float(item[12]),2))
    L_M_P.append(round(float(item[13]),2))
    L_B_P.append(round(float(item[14]),2))

    if reader.line_num == 2:
        Old_Value = [round(float(item[12]),2),round(float(item[13]),2),round(float(item[14]),2)]
        L_F_P_Filter.append(round(float(item[12]),2))
        L_M_P_Filter.append(round(float(item[13]),2))
        L_B_P_Filter.append(round(float(item[14]),2))
    # else:
    #     Value = [round(float(item[12]),2),round(float(item[13]),2),round(float(item[14]),2)]

    # L_F_P_Filter.append(Limmiting_Filtering(Old_Value[0],Value[0],20))
    # L_M_P_Filter.append(Limmiting_Filtering(Old_Value[1],Value[1],20))
    # L_B_P_Filter.append(Limmiting_Filtering(Old_Value[2],Value[2],20))

    x.append(i)
    L_T_A_Feedbk.append(round(float(item[0]),2))
    L_C_A_Feedbk.append(round(float(item[3])-float(item[0]),2))
    R_T_A_Feedbk.append(round(float(item[6]),2))
    R_C_A_Feedbk.append(round(float(item[9])-float(item[6]),2))

    # L_F_P.append(round(float(item[12]),2))
    # L_M_P.append(round(float(item[13]),2))
    # L_B_P.append(round(float(item[14]),2))

    R_F_P.append(round(float(item[15]),2))
    R_M_P.append(round(float(item[16]),2))
    R_B_P.append(round(float(item[17]),2))

    i = i+1
    if i == 1000:
        break

# plt.plot(x,L_T_A_Feedbk,label="Left_Thigh_Angle_Feedbk")
# plt.plot(x,L_C_A_Feedbk,label="Left_Calf_Angle_Feedbk")
# plt.plot(x,R_T_A_Feedbk,label="Right_Thigh_Angle_Feedbk")
# plt.plot(x,R_C_A_Feedbk,label="Right_Calf_Angle_Feedbk")


plt.plot(x,L_F_P,label="Left_Front_Pressure")
plt.plot(x,L_M_P,label="Left_Middle_Pressure")
plt.plot(x,L_B_P,label="Left_Back_Pressure")
# plt.plot(x,R_F_P,label="Left_Thigh_Angle_Feedbk")
# plt.plot(x,R_M_P,label="Left_Calf_Angle_Feedbk")
# plt.plot(x,R_B_P,label="Right_Thigh_Angle_Feedbk")

plt.legend()
plt.show()