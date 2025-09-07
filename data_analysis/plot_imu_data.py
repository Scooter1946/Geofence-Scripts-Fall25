# EPICS EVEI
#
# This python script is used to create plots of data that used IMU.
# You need to run it with the CSV file that was created with 'coords_to_csv.sh'
# (which must be run with the '--imu' flag). It is run like so:
# 
# python plot_imu_data.py <csvfile>
#
# where 'csvfile' is the name of the CSV file that was created using 'coords_to_csv.sh'.
#
# For questions, comments or concerns, reach out to Aadhavan Srinivasan (srini193@purdue.edu).

# import matplotlib.pyplot as plt
# import numpy as np
# import pandas as pd
# import sys
# import math as m

import re
import csv
import matplotlib.pyplot as plt

input_file = input("Input File: ")

def extract_gps_data(input_file, output_file):
    # Regular expression pattern to match latitude and longitude
    pattern = re.compile(r'Latitude:\s*([0-9.]+)\s*Longitude:\s*([0-9.]+)')
    
    gps_data = []
    
    with open(input_file, 'r') as file:
        for line in file:
            match = pattern.search(line)
            if match:
                latitude, longitude = match.groups()
                if float(latitude) and float(longitude) > 1.0:
                    gps_data.append((float(latitude), -1 * float(longitude)))
    
    # Write to CSV file
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Latitude', 'Longitude'])
        writer.writerows(gps_data)
    
    print(f"Extracted {len(gps_data)} GPS points and saved to {output_file}")
    
    return gps_data

def plot_gps_data(gps_data):
    latitudes, longitudes = zip(*gps_data)
    plt.figure(figsize=(8,6))
    plt.scatter(longitudes, latitudes)
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('GPS + IMU Data Plot')
    plt.grid()
    plt.show()

# Example usage
gps_data = extract_gps_data(input_file, "test1_2025.csv")
plot_gps_data(gps_data)



#def Distance(x1, y1, x2, y2):
#	distance = (((y2 - y1) ** 2) + ((x2 - x1) ** 2))
#	distance = m.sqrt(distance)
#	return distance

# Read the file like a regular text file
# with open(sys.argv[1]) as input_file:
# 	lines = input_file.readlines()

# # Find the locations of the delimiters
# sections = [i for i, line in enumerate(lines) if "<NEW>" in line]

# # Create a pandas dataframe for each section
# for i in range(len(sections)):
#     if (i == len(sections)-1):
#         df = pd.DataFrame(lines[sections[i]+1:len(lines)]) 	# For the last line, read until the end of the file
#     else:
#         df = pd.DataFrame(lines[sections[i]+1:sections[i+1]])

#     data = df.to_numpy()
#     print(data)

# #    Separate the comma-separated values into their own array elements
#     data = np.array([s[0].split(',') for s in data])

# #    Convert the array elements from strings to floats, and scale them
#     lat = np.array(data[:,0]).astype(float) * 100000
#     long = np.array(data[:,1]).astype(float) * 100000

#     for i in range(0, lat.size):
#         if (i == 0 or i == lat.size-1):
#                 plt.plot(long[i],lat[i], "or")
#         else:
#                 plt.plot(long[i], lat[i], "bx")

# plt.xlabel("Longitude")
# plt.ylabel("Latitude")
# plt.title("Longitude vs Latitude")
# plt.show()


# quit()


# csv_data = pd.read_csv(sys.argv[1], keep_default_na = False, header=None)
# data = csv_data.to_numpy()
# print(data)

# #for i in range(0, data[:,0].size()):

# lat = np.array(data[:,0]) * 100000
# long = np.array(data[:,1]) * 100000

# ref_rate = np.array(data[:,2])

# distance_arr = []
# for i in range(0, lat.size):
# 	distance_arr.append(round(Distance(long[i-1], lat[i-1], long[i], lat[i]), 2))
# 	plt.plot(long[i-1:i+1],lat[i-1:i+1], "x")
# #	plt.text(x=np.mean(long[i-1:i+1]), y=np.mean(lat[i-1:i+1]), s=distance_arr[-1])


# #print(distance_arr)


# plt.xlim(np.min(long) - 10, np.max(long) + 10)
# plt.ylim(np.min(lat) - 10, np.max(lat) + 10)
# #plt.xticks(range(int(min(long)), int(max(long)+1)))
# plt.xlabel("Longitude")
# plt.ylabel("Latitude")
# plt.title("Longitude vs Latitude")
# plt.show()


