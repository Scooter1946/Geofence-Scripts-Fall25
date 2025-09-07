# EPICS EVEI
# 
# This script is used to create plots of data that only uses GPS.
# IT SHOULD BE IGNORED, unless you want to test out the old code.
# 'plot_imu_data.py' plots graphs of data with IMU, so YOU SHOULD
# USE THAT INSTEAD.

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import math as m

def Distance(x1, y1, x2, y2):
	distance = (((y2 - y1) ** 2) + ((x2 - x1) ** 2))
	distance = m.sqrt(distance)
	return distance

csv_data = pd.read_csv(sys.argv[1], keep_default_na = False, skiprows=1)
csv_data = csv_data.drop_duplicates()
data = csv_data.to_numpy()
print(data)

#for i in range(0, data[:,0].size()):


lat = np.array(data[:,0]) * 100000
long = np.array(data[:,1]) * 100000
ref_rate = np.array(data[:,2])

distance_arr = []
for i in range(1, lat.size):
	distance_arr.append(round(Distance(long[i-1], lat[i-1], long[i], lat[i]), 2))
	plt.plot(long[i-1:i+1],lat[i-1:i+1], "x")
	plt.text(x=np.mean(long[i-1:i+1]), y=np.mean(lat[i-1:i+1]), s=distance_arr[-1])


print(distance_arr)


plt.xlim(np.min(long) - 10, np.max(long) + 10)
plt.ylim(np.min(lat) - 10, np.max(lat) + 10)
#plt.xticks(range(int(min(long)), int(max(long)+1)))
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("Longitude vs Latitude")
plt.show()


