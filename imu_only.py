import time
import busio
import board
import adafruit_bno055
import supervisor

import math

import machine
import select
import sys

def dataReceive():
    poll_obj = select.poll()
    # Register sys.stdin (standard input) for monitoring read events with priority 1
    poll_obj.register(sys.stdin,1)
    print("listening...")
    while True:
        if poll_obj.poll(0):
            value = input().strip()
            # Sometimes Windows sends an extra (or missing) newline - ignore them
            if value == "":
                continue
            print("Raw Input: {}".format(value))
            break

    # Temporary Geofence For testing purposes - Use computer-end-code.py for real Geofence Values
    #value = "OUTER, 40.39123253146508, -86.82833099365233, 40.365601883498044, -86.97458648681639, 40.42417189314546, -87.00067901611327, 40.468065962237894, -86.85236358642577, 40.428353515662465, -86.79743194580077, INNER"

    coordinateList = value.split(", ")
    innerBegin = coordinateList.index("INNER")

    outerList = [coordinateList[idx] for idx in range(1, innerBegin)]
    innerList = [coordinateList[idx] for idx in range(innerBegin + 1, len(coordinateList))]

    outerList = [float(i) for i in outerList]
    outerList = [(outerList[i], outerList[i+1]) for i in range(0, len(outerList)-1,2)] # Groups the latitudes and longitudes together

    innerList = [float(i) for i in innerList]
    innerList = [(innerList[i], innerList[i+1]) for i in range(0, len(innerList)-1,2)] # Groups the latitudes and longitudes together

    return outerList, innerList



def imu_update(latAvg, longAvg, time_interval):
    earth_radius = 6371000  # Earth's radius in meters

    velocity_x = 0
    velocity_y = 0

    imu_acceleration_x, imu_acceleration_y, imu_acceleration_z = sensor.linear_acceleration
    

    #print("Accelerations")
    #print(f"Acceleration X: {imu_acceleration_x:.10f}   Acceleration Y: {imu_acceleration_y:.10f}")
    #print("Sensor Linear Accelearion")
    #print(sensor.linear_acceleration)

    # Velocity Estimation
    velocity_x += imu_acceleration_x * time_interval
    velocity_y += imu_acceleration_y * time_interval

    #print("VELOCITIES")
    #print(f"Velocity X: {velocity_x:.10f}   Velocity Y: {velocity_y:.10f}") 

    # Position Estimation
    latitude_change = (velocity_x * time_interval)
    longitude_change = (velocity_y * time_interval)
    # latitude_change = ((velocity_x * time_interval) / earth_radius) * (180 / math.pi)
    # longitude_change = ((velocity_y * time_interval) / earth_radius) * (180 / math.pi) / math.cos(latAvg * (math.pi / 180))
    
    
    #print("LAT AVG/LONGAVG")
    #print(f"Latitude: {latAvg:.10f}   Longitude: {longAvg:.10f}")

    #print("CHANGES")
    #print(f"Latitude Change: {latitude_change:.10f}   Longitude Change: {longitude_change:.10f}")  

    # Update latitude and longitude
    newlatAvg = latAvg + latitude_change
    newlongAvg = longAvg + longitude_change

    # print(f'''
    #       IMU UPDATE\n
    #       Latitude: {newlatAvg:.10f}   Longitude: {newlongAvg:.10f}\n
    #       IMU DATA: {sensor.linear_acceleration}\n
    #       ''')
    #print("IMU Refresh Rate: ", float(endTime - startTime))

    return newlatAvg, newlongAvg


if __name__ == '__main__':
    
    outerPolygon, innerPolygon = dataReceive()
    latitude_avg,longitude_avg = 0,0
    i2c = busio.I2C(board.GP15, board.GP14, frequency=400000)       # Initializes I2C for the IMU
    sensor = adafruit_bno055.BNO055_I2C(i2c)                        # Initializes IMU
    imu_start_time = time.ticks_ms()
    while True:
        imu_end_time = time.time_ns()/1_000_000_000.0
        latitude_avg, longitude_avg = imu_update(latitude_avg, longitude_avg, imu_end_time-imu_start_time)
        imu_start_time = time.time_ns()/1_000_000_000.0
        print(f'''
IMU UPDATE\n
Latitude: {latitude_avg:.10f}   Longitude: {longitude_avg:.10f}\n
IMU DATA: {sensor.linear_acceleration}\n
IMU UPDATE TIME: {imu_end_time-imu_start_time}\n
              ''')
        

