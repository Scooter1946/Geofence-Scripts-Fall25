import numpy as np
lines = []

filename = "datalog2025-04-17_11-20-15"

with open(filename+".txt") as file:
    lines = file.readlines()
    file.close()

GPSUpdates = []
IMUUpdates = []

IMUData = []
GPSData = []

counts = []

lat = []
long = []
velX = []
velY = []



count = 0
csv = ""
for line in lines:
    if "IMU UPDATE TIME: " in line:
        IMUUpdates.append(line[line.index(": ")+2:])
        csv += f"{line[line.index(": ")+2:-1]}\n"
        # count+=1
    elif "GPS UPDATE TIME: " in line:
        GPSUpdates.append(line[line.index(": ")+2:])
        # counts.append(count)
        # count = 0
    elif "Latitude:" in line:
        ind = line.index("Latitude: ")+10
        lat.append(line[ind:ind+13]+"\n")
        csv += f"{line[ind:ind+13]}, "
    if "Longitude:" in line:
        ind = line.index("Longitude: ")+11
        long.append(line[ind:ind+13]+"\n")
        csv += f"{line[ind:ind+13]}, "
    elif "Velocity X:" in line:
        velX.append(line[line.index(": ")+2:])
        csv += f"{line[line.index(": ")+2:-1]}, "
    elif "Velocity Y:" in line:
        velY.append(line[line.index(": ")+2:])
        csv += f"{line[line.index(": ")+2:-1]}, "
    

# print(counts)
# print(np.mean(counts))
# print(np.std(counts))
with open(filename+"_imu.csv", "w") as file:
    file.writelines(IMUUpdates)
    file.close()

with open(filename+"_lat.csv", "w") as file:
    file.writelines(lat)
    file.close()

with open(filename+"_long.csv", "w") as file:
    file.writelines(long)
    file.close()
    
with open(filename+"_velX.csv", "w") as file:
    file.writelines(velX)
    file.close()
    
with open(filename+"_velY.csv", "w") as file:
    file.writelines(velY)
    file.close()

with open(filename+"_gps.csv", "w") as file:
    file.writelines(GPSUpdates)
    file.close()
    
with open(filename+"_all.csv", "w") as file:
    file.write(csv)
    file.close()
print("done")