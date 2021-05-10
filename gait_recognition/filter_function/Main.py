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

    L_F_P_Filter.append(fm.Limmiting_Filtering(Old_Value[0],Value[0],40))
    L_M_P_Filter.append(fm.Limmiting_Filtering(Old_Value[1],Value[1],40))
    L_B_P_Filter.append(fm.Limmiting_Filtering(Old_Value[2],Value[2],40))

    Old_Value[0] = Value[0]
    Old_Value[1] = Value[1]
    Old_Value[2] = Value[2]

    x.append(i)
    i = i+1
    print(i)
    if i == 10000:
        break
# 创建图像布局对象fig
fig = plt.figure(figsize = (12, 6))
# 原图
plot1 = fig.add_subplot(231)
# plt.plot(x,L_F_P,label="Left_Front_Pressure")
plot1.plot(x,L_M_P,label="Left_Middle_Pressure")
# plt.plot(x,L_B_P,label="Left_Back_Pressure")

#一次滤波
plot1 = fig.add_subplot(232)
# plot2.plot(x,L_B_P_Filter,label="Left_Back_Pressure_Filter")
plot1.plot(x,L_F_P_Filter,label="Left_Front_Pressure_Filter")
# plot2.plot(x,L_M_P_Filter,label="Left_Middle_Pressure_Filter")

#二次滤波
L_F_P_Filter=fm.Median_Average_Filter(L_F_P_Filter,20)
L_M_P_Filter=fm.Median_Average_Filter(L_M_P_Filter,20)
L_B_P_Filter=fm.Median_Average_Filter(L_B_P_Filter,20)

plot1 = fig.add_subplot(233)
# plot2.plot(x,L_B_P_Filter,label="Left_Back_Pressure_Filter")
plot1.plot(x,L_F_P_Filter,label="Left_Front_Pressure_Filter")
# plot2.plot(x,L_M_P_Filter,label="Left_Middle_Pressure_Filter")

#三次滤波
L_F_P_Filter=fm.Recursive_Average_Filter(L_F_P_Filter,80)
L_M_P_Filter=fm.Recursive_Average_Filter(L_M_P_Filter,80)
L_B_P_Filter=fm.Recursive_Average_Filter(L_B_P_Filter,80)

plot1 = fig.add_subplot(234)
# plot2.plot(x,L_B_P_Filter,label="Left_Back_Pressure_Filter")
plot1.plot(x,L_F_P_Filter,label="Left_Front_Pressure_Filter")
# plot2.plot(x,L_M_P_Filter,label="Left_Middle_Pressure_Filter")

#四次滤波
L_F_P_Filter=fm.Median_Average_Filter(L_F_P_Filter,20)
L_M_P_Filter=fm.Median_Average_Filter(L_M_P_Filter,20)
L_B_P_Filter=fm.Median_Average_Filter(L_B_P_Filter,20)

plot1 = fig.add_subplot(235)
# plot2.plot(x,L_B_P_Filter,label="Left_Back_Pressure_Filter")
plot1.plot(x,L_F_P_Filter,label="Left_Front_Pressure_Filter")
# plot2.plot(x,L_M_P_Filter,label="Left_Middle_Pressure_Filter")

#五次滤波
L_F_P_Filter=fm.Recursive_Average_Filter(L_F_P_Filter,80)
L_M_P_Filter=fm.Recursive_Average_Filter(L_M_P_Filter,80)
L_B_P_Filter=fm.Recursive_Average_Filter(L_B_P_Filter,80)

plot1 = fig.add_subplot(236)
# plot2.plot(x,L_B_P_Filter,label="Left_Back_Pressure_Filter")
plot1.plot(x,L_F_P_Filter,label="Left_Front_Pressure_Filter")
# plot2.plot(x,L_M_P_Filter,label="Left_Middle_Pressure_Filter")





plt.legend()
plt.show()