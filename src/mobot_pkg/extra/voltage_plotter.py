import matplotlib.pyplot as plt
import numpy as np

timeVsvoltages = []
timeList = []
voltList = []
time0 = 0
# with open("src/mobot_pkg/data/voltage_data.txt", 'r', encoding="utf-8") as rf:
#     voltageData = rf.readlines()
#     cnt = 0
#     for volts in voltageData:
#         volts_ = volts.strip().split(",")
#         t, v = int(volts_[0])//1000000000, float(volts_[1])
#         if cnt == 0:
#             time0 = t
#         else:
#             t = t - time0
        
        
#         if t not in timeList:
#             if cnt == 0:
#                 time0 = t
            
#             timeList.append(t)
#             voltList.append(v)
#             timeVsvoltages.append([t, v])
#             cnt += 1
#             print(t, v)
            

data = np.loadtxt("src/mobot_pkg/data/voltage_dataset.txt")
timeList = data[:, 0]
voltList = data[:, 1]

plt.plot(timeList, voltList) 
plt.xlabel('Time (sec)') 
plt.ylabel('Voltage (volt)')  
plt.xticks( np.arange(0, 8500, 1000))
plt.yticks( np.arange(11.1, 12.6, .1))
plt.title(f'Battery Discharging', size = 20)
plt.grid(True, which='both')
plt.show()   

# np.savetxt("src/mobot_pkg/data/voltage_dataset.txt", data)

# # naming the x axis  
# plt.xlabel('Time (sec)')  
# # naming the y axis  
# plt.ylabel('Voltage (volt)')  
    
# # giving a title to my graph  
# plt.title('Robot operation time graph')  
    
# # function to show the plot  

# np.savetxt("src/mobot_pkg/data/voltage_data_np.txt", data)


# print(timeVsvoltages)


