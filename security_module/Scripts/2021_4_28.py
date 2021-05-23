import csv
import matplotlib.pyplot as plt

csvFile = open('security_module/csv_File/4.csv','r')
reader  = csv.reader(csvFile)

i = 0
x = []
L_T_A_Expect = []
R_T_A_Expect = []
L_T_A_Feedbk = []
R_T_A_Feedbk = []

for item in reader:
    if reader.line_num == 1:
        continue
    x.append(i)
    L_T_A_Expect.append(round(float(item[0]),2))
    R_T_A_Expect.append(round(float(item[1]),2))
    L_T_A_Feedbk.append(round(float(item[2]),2))
    R_T_A_Feedbk.append(round(float(item[3]),2))
    i = i+0.1
plt.plot(x,L_T_A_Expect,label="Left_Thigh_Angle_Expect")
plt.plot(x,R_T_A_Expect,label="Right_Thigh_Angle_Expect")
plt.plot(x,L_T_A_Feedbk,label="Left_Thigh_Angle_Feedback")
plt.plot(x,R_T_A_Feedbk,label="Right_Thigh_Angle_Feedback")
plt.title("Motor Following Experiment")
plt.xlabel("Time/s")
plt.ylabel("Angle/rad")
plt.legend()
plt.show()