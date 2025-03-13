import numpy as np
lines = []

filename = "datalog2024-11-07_11-37-50"

with open(filename+".txt") as file:
    lines = file.readlines()
    file.close()

GPSUpdates = []
IMUUpdates = []

IMUData = []
GPSData = []

counts = []

count = 0
for line in lines:
    if "IMU UPDATE TIME: " in line:
        IMUUpdates.append(line[17:])
        # count+=1
    elif "GPS Refresh Rate: " in line:
        GPSUpdates.append(line[19:])
        # counts.append(count)
        # count = 0

# print(counts)
# print(np.mean(counts))
# print(np.std(counts))
with open(filename+"_imu.csv", "w") as file:
    file.writelines(IMUUpdates)
    file.close()

with open(filename+"_gps.csv", "w") as file:
    file.writelines(GPSUpdates)
    file.close()
    
print(GPSUpdates)