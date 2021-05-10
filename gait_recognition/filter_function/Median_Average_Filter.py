import Filtering_Method as fm
import csv
import matplotlib.pyplot as plt

csvFile = open('/home/hemingshan/exo_ws/src/sensor_module/csv_File/2_km.csv','r')
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



    x.append(i)
    i = i+1
    if i == 500:
        break
L_F_P_Filter=fm.Median_Average_Filter(L_F_P,30)
L_M_P_Filter=fm.Median_Average_Filter(L_M_P,30)
L_B_P_Filter=fm.Median_Average_Filter(L_B_P,30)

plt.plot(x,L_F_P,label="Left_Front_Pressure")
# plt.plot(x,L_M_P,label="Left_Middle_Pressure")
# plt.plot(x,L_B_P,label="Left_Back_Pressure")

plt.plot(x,L_F_P_Filter,label="Left_Front_Pressure_Filter")
# plt.plot(x,L_M_P_Filter,label="Left_Middle_Pressure_Filter")
# plt.plot(x,L_B_P_Filter,label="Left_Back_Pressure_Filter")


plt.legend()
plt.show()