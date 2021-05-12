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
plot1.plot(x,L_F_P,label="Left_Front_Pressure")
# plot1.plot(x,L_M_P,label="Left_Middle_Pressure")
# plt.plot(x,L_B_P,label="Left_Back_Pressure")
plot1.legend()
plot1.set_title('Original Plot')
plot1.set_xlabel('Time/s')
plot1.set_ylabel('Pressure')

#一次滤波
plot2 = fig.add_subplot(232)
# plot2.plot(x,L_B_P_Filter,label="Left_Back_Pressure_Filter")
plot2.plot(x,L_F_P_Filter,label="L_F_P_1_Filter")
# plot2.plot(x,L_M_P_Filter,label="Left_Middle_Pressure_Filter")
plot2.legend()
plot2.set_title('1st Filter Plot')
plot2.set_xlabel('Time/s')
plot2.set_ylabel('Pressure')

#二次滤波
L_F_P_Filter=fm.Median_Average_Filter(L_F_P_Filter,20)
L_M_P_Filter=fm.Median_Average_Filter(L_M_P_Filter,20)
L_B_P_Filter=fm.Median_Average_Filter(L_B_P_Filter,20)

plot3 = fig.add_subplot(233)
# plot2.plot(x,L_B_P_Filter,label="Left_Back_Pressure_Filter")
plot3.plot(x,L_F_P_Filter,label="L_F_P_2_Filer")
# plot2.plot(x,L_M_P_Filter,label="Left_Middle_Pressure_Filter")
plot3.legend()
plot3.set_title('2st Filter Plot')
plot3.set_xlabel('Time/s')
plot3.set_ylabel('Pressure')
#三次滤波
L_F_P_Filter=fm.Recursive_Average_Filter(L_F_P_Filter,80)
L_M_P_Filter=fm.Recursive_Average_Filter(L_M_P_Filter,80)
L_B_P_Filter=fm.Recursive_Average_Filter(L_B_P_Filter,80)

plot4 = fig.add_subplot(234)
# plot2.plot(x,L_B_P_Filter,label="Left_Back_Pressure_Filter")
plot4.plot(x,L_F_P_Filter,label="L_F_P_3_Filer")
# plot2.plot(x,L_M_P_Filter,label="Left_Middle_Pressure_Filter")
plot4.legend()
plot4.set_title('3st Filter Plot')
plot4.set_xlabel('Time/s')
plot4.set_ylabel('Pressure')
#四次滤波
L_F_P_Filter=fm.Median_Average_Filter(L_F_P_Filter,20)
L_M_P_Filter=fm.Median_Average_Filter(L_M_P_Filter,20)
L_B_P_Filter=fm.Median_Average_Filter(L_B_P_Filter,20)

plot5 = fig.add_subplot(235)
# plot2.plot(x,L_B_P_Filter,label="Left_Back_Pressure_Filter")
plot5.plot(x,L_F_P_Filter,label="L_F_P_4_Filer")
# plot2.plot(x,L_M_P_Filter,label="Left_Middle_Pressure_Filter")
plot5.legend()
plot5.set_title('4st Filter Plot')
plot5.set_xlabel('Time/s')
plot5.set_ylabel('Pressure')
#五次滤波
L_F_P_Filter=fm.Recursive_Average_Filter(L_F_P_Filter,80)
L_M_P_Filter=fm.Recursive_Average_Filter(L_M_P_Filter,80)
L_B_P_Filter=fm.Recursive_Average_Filter(L_B_P_Filter,80)

plot6 = fig.add_subplot(236)
# plot2.plot(x,L_B_P_Filter,label="Left_Back_Pressure_Filter")
plot6.plot(x,L_F_P_Filter,label="L_F_P_5_Filter")
# plot2.plot(x,L_M_P_Filter,label="Left_Middle_Pressure_Filter")
plot6.legend()
plot6.set_title('5st Filter Plot')
plot6.set_xlabel('Time/s')
plot6.set_ylabel('Pressure')


plt.legend()
plt.show()