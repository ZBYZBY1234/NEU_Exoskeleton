import csv
import matplotlib.pyplot as plt

csvFile = open('security_module/csv_File/4.csv','r')
reader  = csv.reader(csvFile)

i = 0
x = []
L_T_A_Expect = []
L_C_A_Expect = []
R_T_A_Expect = []
R_C_A_Expect = []

L_T_A_Driver = []
L_C_A_Driver = []
R_T_A_Driver = []
R_C_A_Driver = []

L_T_A_Feedbk = []
L_C_A_Feedbk = []
R_T_A_Feedbk = []
R_C_A_Feedbk = []

for item in reader:
    if reader.line_num == 1:
        continue
    x.append(i)
    L_T_A_Feedbk.append(round(float(item[0]),2))
    L_C_A_Feedbk.append(round(float(item[3]),2))
    R_T_A_Feedbk.append(round(float(item[6]),2))
    R_C_A_Feedbk.append(round(float(item[9]),2))

    L_T_A_Expect.append(round(float(item[12]),2))
    L_C_A_Expect.append(round(float(item[15]),2))
    R_T_A_Expect.append(round(float(item[18]),2))
    R_C_A_Expect.append(round(float(item[21]),2))

    L_T_A_Driver.append(round(float(item[24]),2))
    L_C_A_Driver.append(round(float(item[25]),2))
    R_T_A_Driver.append(round(float(item[26]),2))
    R_C_A_Driver.append(round(float(item[27]),2))

    i = i+0.1
plt.plot(x,L_T_A_Expect,label="Left_Thigh_Angle_Expect")
plt.plot(x,R_T_A_Expect,label="Right_Thigh_Angle_Expect")
plt.plot(x,L_T_A_Feedbk,label="Left_Thigh_Angle_Feedback")
plt.plot(x,R_T_A_Feedbk,label="Right_Thigh_Angle_Feedback")

plt.legend()
plt.show()