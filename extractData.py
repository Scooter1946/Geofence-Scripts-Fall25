
lines = []

filename = "datalog2025-02-13_11-33-22"

with open(filename+".txt") as file:
    lines = file.readlines()
    file.close()

GPSUpdates = []
IMUUpdates = []
for line in lines:
    if "IMU UPDATE TIME: " in line:
        IMUUpdates.append(line[17:])
    elif "GPS UPDATE TIME: " in line:
        GPSUpdates.append(line[17:])

with open(filename+"_imu.csv", "w") as file:
    file.writelines(IMUUpdates)
    file.close()

with open(filename+"_gps.csv", "w") as file:
    file.writelines(GPSUpdates)
    file.close()