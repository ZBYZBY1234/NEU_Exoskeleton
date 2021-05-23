import csv
import matplotlib.pyplot as plt

csvFile = open('/home/hemingshan/exo_ws/src/security_module/csv_File/2021_5_19.csv','r')
reader  = csv.reader(csvFile)

i = 0
x = []
L_T_A_Feedbk = []
L_C_A_Feedbk = []
R_T_A_Feedbk = []
R_C_A_Feedbk = []

L_T_A_Expect = []
L_C_A_Expect = []
R_T_A_Expect = []
R_C_A_Expect = []

L_T_Force = []
L_C_Force = []
R_T_Force = []
R_C_Force = []

L_T_A_Driver = []
L_C_A_Driver = []
R_T_A_Driver = []
R_C_A_Driver = []

for item in reader:
    if reader.line_num == 1:
        continue



    L_T_A_Feedbk.append(round(float(item[0]),2))
    L_C_A_Feedbk.append(round(float(item[3])-float(item[0]),2))
    R_T_A_Feedbk.append(round(float(item[6]),2))
    R_C_A_Feedbk.append(round(float(item[9])-float(item[6]),2))

    L_T_A_Expect.append(round(float(item[12]),2))
    L_C_A_Expect.append(round(float(item[15])-float(item[0]),2))
    R_T_A_Expect.append(round(float(item[18]),2))
    R_C_A_Expect.append(round(float(item[21])-float(item[18]),2))

    L_T_Force.append(round(float(item[24]),2))
    L_C_Force.append(round(float(item[25]),2))
    R_T_Force.append(round(float(item[26]),2))
    R_C_Force.append(round(float(item[27]),2))

    L_T_A_Driver.append(round(float(item[28]),2))
    L_C_A_Driver.append(round(float(item[29]),2))
    R_T_A_Driver.append(round(float(item[30]),2))
    R_C_A_Driver.append(round(float(item[31]),2))

    x.append(i)
    i = i+1
    # if i == 5000:
    #     break

# plt.plot(x,L_T_A_Feedbk,label="Left_Thigh_Angle_Feedbk")
# plt.plot(x,L_C_A_Feedbk,label="Left_Calf_Angle_Feedbk")
# plt.plot(x,R_T_A_Feedbk,label="Right_Thigh_Angle_Feedbk")
# plt.plot(x,R_C_A_Feedbk,label="Right_Calf_Angle_Feedbk")

# plt.plot(x,L_T_A_Expect,label="Left_Thigh_Angle_Expect")
# plt.plot(x,L_C_A_Expect,label="Left_Calf_Angle_Expect")
# plt.plot(x,R_T_A_Expect,label="Right_Thigh_Angle_Expect")
# plt.plot(x,R_C_A_Expect,label="Right_Calf_Angle_Expect")

plt.plot(x,L_T_Force,label="Left_Thigh_Force")
plt.plot(x,L_C_Force,label="Left_Calf_Force")
plt.plot(x,R_T_Force,label="Right_Thigh_Force")
plt.plot(x,R_C_Force,label="Right_Calf_Force")

# plt.plot(x,L_T_A_Driver,label="Left_Thigh_Angle_Driver")
# plt.plot(x,L_C_A_Driver,label="Left_Calf_Angle_Driver")
# plt.plot(x,R_T_A_Driver,label="Right_Thigh_Angle_Driver")
# plt.plot(x,R_C_A_Driver,label="Right_Calf_Angle_Driver")

plt.title("Force Plot")
plt.legend()
plt.show()