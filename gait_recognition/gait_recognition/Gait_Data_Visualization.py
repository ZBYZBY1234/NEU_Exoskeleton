import csv
import matplotlib.pyplot as plt

csvFile = open('/home/hemingshan/exo_ws/src/sensor_module/csv_File/2_km.csv','r')
reader  = csv.reader(csvFile)

i = 0
x = []
L_T_A = []
L_C_A = []
R_T_A = []
R_C_A = []

for item in reader:
    if reader.line_num == 1:
        continue

    x.append(i)
    L_T_A.append(round(float(item[0]),2))
    L_C_A.append(round(float(item[3])-float(item[0]),2))
    R_T_A.append(round(float(item[6]),2))
    R_C_A.append(round(float(item[9])-float(item[6]),2))
    i = i+1
    if i == 10000:
        break

plt.plot(x,L_T_A,label="Left_Thigh_Angle_Feedbk")
plt.plot(x,L_C_A,label="Left_Calf_Angle_Feedbk")
plt.plot(x,R_T_A,label="Right_Thigh_Angle_Feedbk")
plt.plot(x,R_C_A,label="Right_Calf_Angle_Feedbk")

plt.legend()
plt.show()