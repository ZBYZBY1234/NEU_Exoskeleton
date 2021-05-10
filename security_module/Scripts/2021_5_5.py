import csv
import matplotlib.pyplot as plt

csvFile = open('/home/hemingshan/exo_ws/src/sensor_module/csv_File/3_5_km.csv','r')
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

R_F_P = []
R_M_P = []
R_B_P = []

for item in reader:
    if reader.line_num == 1:
        continue



    L_T_A_Feedbk.append(round(float(item[0]),2))
    L_C_A_Feedbk.append(round(float(item[3])-float(item[0]),2))
    R_T_A_Feedbk.append(round(float(item[6]),2))
    R_C_A_Feedbk.append(round(float(item[9])-float(item[6]),2))

    L_F_P.append(round(float(item[12]),2))
    L_M_P.append(round(float(item[13]),2))
    L_B_P.append(round(float(item[14]),2))

    R_F_P.append(round(float(item[15]),2))
    R_M_P.append(round(float(item[16]),2))
    R_B_P.append(round(float(item[17]),2))

    x.append(i)
    i = i+1
    if i == 5000:
        break

plt.plot(x,L_T_A_Feedbk,label="Left_Thigh_Angle_Feedbk")
plt.plot(x,L_C_A_Feedbk,label="Left_Calf_Angle_Feedbk")
plt.plot(x,R_T_A_Feedbk,label="Right_Thigh_Angle_Feedbk")
plt.plot(x,R_C_A_Feedbk,label="Right_Calf_Angle_Feedbk")


# plt.plot(x,L_F_P,label="L_F_P")
# plt.plot(x,L_M_P,label="L_M_P")
# plt.plot(x,L_B_P,label="L_B_P")
# plt.plot(x,R_F_P,label="R_F_P")
# plt.plot(x,R_M_P,label="R_M_P")
# plt.plot(x,R_B_P,label="R_B_P")

plt.legend()
plt.show()