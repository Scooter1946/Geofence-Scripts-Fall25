# EPICS EVEI
# 
# This script is used to create plots of data that only uses GPS.
# IT SHOULD BE IGNORED, unless you want to test out the old code.
# 'plot_imu_data.py' plots graphs of data with IMU, so YOU SHOULD
# USE THAT INSTEAD.

'''
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import math as m
import re

def Distance(x1, y1, x2, y2):
	distance = (((y2 - y1) ** 2) + ((x2 - x1) ** 2))
	distance = m.sqrt(distance)
	return distance

if len(sys.argv) < 2:
    filename = "/Users/ishaansingh/Documents/Geofence-Scripts-Fall25-1/data_analysis/datalog2024-11-07_11-37-50.txt"
    print(f"No file provided, using default: {filename}")
else:
    filename = sys.argv[1]


csv_data = pd.read_csv(filename, keep_default_na = False, skiprows=1)
csv_data = csv_data.drop_duplicates()
data = csv_data.to_numpy()
print(data)

#for i in range(0, data[:,0].size()):


lat = []
long = []

for row in data.flatten():
    if "Latitude" in row and "Longitude" in row:
        match = re.search(r"Latitude:\s*([-0-9.]+)\s+Longitude:\s*([-0-9.]+)", row)
        if match:
            lat.append(float(match.group(1)))
            long.append(float(match.group(2)))

lat = np.array(lat)
long = np.array(long)

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
'''
# EPICS EVEI
# Old GPS-only plotting script

import sys
import re
import math as m
import numpy as np
import matplotlib.pyplot as plt

# ---- Config: default file if no CLI arg ----
if len(sys.argv) < 2:
    filename = "/Users/ishaansingh/Documents/Geofence-Scripts-Fall25-1/data_analysis/datalog2025-03-13_11-49-49.txt"
    print(f"No file provided, using default: {filename}")
else:
    filename = sys.argv[1]

# ---- Read file as raw lines (robust for logs) ----
with open(filename, "r") as f:
    lines = [line.strip() for line in f if line.strip()]

# ---- Parse "Latitude: ...   Longitude: ..." lines ----
pat = re.compile(r"Latitude:\s*([-+]?\d*\.?\d+)\s+Longitude:\s*([-+]?\d*\.?\d+)")
lat, lon = [], []
for line in lines:
    mobj = pat.search(line)
    if mobj:
        lat.append(float(mobj.group(1)))
        lon.append(float(mobj.group(2)))

lat = np.array(lat)
lon = np.array(lon)

if lat.size < 2:
    print("No GPS coordinate pairs found. Check file format or regex.")
    sys.exit(1)

print("Parsed sample (first 5):", list(zip(lat[:5], lon[:5])))

# ---- Haversine distance in meters ----
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0  # meters
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    dphi = np.radians(lat2 - lat1)
    dlmb = np.radians(lon2 - lon1)
    a = np.sin(dphi/2.0)**2 + np.cos(phi1)*np.cos(phi2)*np.sin(dlmb/2.0)**2
    return 2.0 * R * np.arcsin(np.sqrt(a))

# Compute segment distances (meters)
dists_m = [haversine(lat[i-1], lon[i-1], lat[i], lon[i]) for i in range(1, lat.size)]
print("First 10 segment distances (m):", [round(d, 2) for d in dists_m[:10]])

# ---- Plot path ----
plt.figure()
plt.plot(lon, lat, marker='.', linestyle='-')

# Label every ~5% of segments to avoid clutter
step = max(1, lat.size // 20)
for i in range(1, lat.size, step):
    midx = (lon[i-1] + lon[i]) / 2.0
    midy = (lat[i-1] + lat[i]) / 2.0
    plt.text(midx, midy, f"{dists_m[i-1]:.1f} m", fontsize=8)

plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("Longitude vs Latitude (GPS Path)")

# Tight, sensible bounds with equal aspect
pad_x = (lon.max() - lon.min()) * 0.05 or 1e-5
pad_y = (lat.max() - lat.min()) * 0.05 or 1e-5
plt.xlim(lon.min() - pad_x, lon.max() + pad_x)
plt.ylim(lat.min() - pad_y, lat.max() + pad_y)
plt.gca().set_aspect('equal', adjustable='box')
plt.tight_layout()
plt.show()

