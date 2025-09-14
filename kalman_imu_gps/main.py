import time
import busio
import board
import adafruit_bno055
import supervisor

import math

import machine
import select
import sys
import kalman_filter

import os


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def get_latitude(str_array, index):
    latDeg = float(str_array[index][0: 2])
    latMin = float(str_array[index][2: 10]) / 60
    latitude = (float(latDeg) + float(latMin))
    if str_array[index + 1] == "S":
        latitude = -latitude
    return '%f' % latitude


# Gets Longitude
def get_longitude(str_array, index2):
    longDeg = float(str_array[index2][1: 3])
    longMin = float(str_array[index2][3: 11]) / 60
    longitude = (float(longDeg) + float(longMin))
    if str_array[index2 + 1] == "E":
        longitude = -longitude
    return '%f' % longitude


# To determine if the coordinate/function q lies on the segment pr
def onSegment(p: tuple, q: tuple, r: tuple) -> bool:
    if ((q[0] <= max(p[0], r[0])) &
            (q[0] >= min(p[0], r[0])) &
            (q[1] <= max(p[1], r[1])) &
            (q[1] >= min(p[1], r[1]))):
        return True
    return False


# Finding orientation
def orientation(p: tuple, q: tuple, r: tuple) -> int:
    val = (((q[1] - p[1]) * (r[0] - q[0])) - ((q[0] - p[0]) * (r[1] - q[1])))  # calculating slope

    if (val > 0):
        return 1  # positive slope is clockwise orientation
    elif (val < 0):
        return 2  # negative slope is counterclockwise orientation
    else:
        return 0  # collinear orientation


# Determine if line segment p1q1 and p2q2 intersects
def doIntersect(p1, q1, p2, q2):
    # looking for orientation
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General Case: If the orientations are different, they intersect
    if (o1 != o2) and (o3 != o4):
        return True

    # Special Case (collinear): If x and y projection intersects, they intersect
    if (o1 == 0) and (onSegment(p1, p2, q1)):  # p1, q1 and p2 are collinear and p2 lies on segment p1q1
        return True

    if (o2 == 0) and (onSegment(p1, q2, q1)):  # p1, q1 and q2 are collinear and q2 lies on segment p1q1
        return True

    if (o3 == 0) and (onSegment(p2, p1, q2)):  # p2, q2 and p1 are collinear and p1 lies on segment p2q2
        return True

    if (o4 == 0) and (onSegment(p2, q1, q2)):  # p2, q2 and q1 are collinear and q1 lies on segment p2q2
        return True

    return False


# Determine if the point p lies within the polygon
def is_within_polygon(points: list, p: list) -> bool:
    n = len(points)
    if n < 3:  # there must be at least 3 points/vertices in a polygon
        return False

    extreme = (float('inf'), p[1])  # Create a point for line segment from p to infinite

    decrease = 0  # To calculate number of points where y-coordinate of the polygon is equal to y-coordinate of the point
    count = i = 0

    while True:
        next = (i + 1) % n

        if (points[i][1] == p[1]):
            decrease += 1

        if (doIntersect(points[i], points[next], p, extreme)):
            if orientation(points[i], p, points[next]) == 0:
                return onSegment(points[i], p, points[next])

            count += 1

        i = next

        if (i == 0):
            break

        count -= decrease

    return (count % 2 == 1)


# Initializes GPS
def initialize_gps():
    uart = busio.UART(baudrate=9600, tx=board.GP12, rx=board.GP13)
    print(uart)
    return uart


# Turns on the LCD
def initialize_lcd(backlight_red, backlight_green, backlight_blue):
    # lcd_uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))         # This line specifically should be changed to CircuitPython
    lcd_uart = busio.UART(board.GP4, board.GP5, baudrate=9600)
    lcd_uart.write(b'|')  # write 5 bytes
    lcd_uart.write(b'\x18')  # write 5 bytes
    lcd_uart.write(b'\x08')  # contrast
    lcd_uart.write(b'|')  # Put LCD into setting mode
    lcd_uart.write(b'\x2B')  # Set green backlight amount to 0%
    lcd_uart.write(backlight_red.to_bytes(1, 'big'))  # Set green backlight amount to 0%
    lcd_uart.write(backlight_green.to_bytes(1, 'big'))  # Set green backlight amount to 0%
    lcd_uart.write(backlight_blue.to_bytes(1, 'big'))  # Set blue backlight amount to 0%
    lcd_uart.write(b'|')  # Setting character
    lcd_uart.write(b'-')  # Clear display
    return lcd_uart


def dataReceive():
    poll_obj = select.poll()
    # Register sys.stdin (standard input) for monitoring read events with priority 1
    poll_obj.register(sys.stdin, 1)
    print("listening...")
    with open("coordinates.csv", 'r') as f:
        lines = f.readlines()
        lines = [item.replace("\n", "") for item in lines]
        value = ",".join(lines)
        print(value)
    '''
    while True:
        if poll_obj.poll(0):
            value = input().strip()
            # Sometimes Windows sends an extra (or missing) newline - ignore them
            if value == "":
                continue
            print("Raw Input: {}".format(value))
            break
    '''
    # Temporary Geofence For testing purposes - Use computer-end-code.py for real Geofence Values
    # value = "OUTER, 40.39123253146508, -86.82833099365233, 40.365601883498044, -86.97458648681639, 40.42417189314546, -87.00067901611327, 40.468065962237894, -86.85236358642577, 40.428353515662465, -86.79743194580077, INNER"

    coordinateList = value.split(",")
    innerBegin = coordinateList.index("INNER")

    outerList = [coordinateList[idx] for idx in range(1, innerBegin)]
    innerList = [coordinateList[idx] for idx in range(innerBegin + 1, len(coordinateList))]

    outerList = [float(i) for i in outerList]
    outerList = [(outerList[i], outerList[i + 1]) for i in
                 range(0, len(outerList) - 1, 2)]  # Groups the latitudes and longitudes together

    innerList = [float(i) for i in innerList]
    innerList = [(innerList[i], innerList[i + 1]) for i in
                 range(0, len(innerList) - 1, 2)]  # Groups the latitudes and longitudes together

    return outerList, innerList


# Gets Current Location
def get_gps_location(gps_uart):
    latitude_LL = 0
    longitude_LL = 0
    latitude_GA = 0
    longitude_GA = 0
    latDivisor = 1
    lonDivisor = 1

    while ((latitude_LL == 0 and longitude_LL == 0) and (latitude_GA == 0 and longitude_GA == 0)):

        time.sleep(0.03)
        str_array = gps_uart.readline()
        # print(str_array)

        if str_array is None:
            continue
        try:

            str_array = str_array.decode("utf-8")  # Decodes GPS input
            time.sleep(0.03)
            str_array = str_array.split(",")
            # print(str_array)                            # Prints GPS Output

            if str_array[0] is '$GPGLL':
                # print("in GPGLL")
                # lcd_uart.write("in GNGLL")
                latitude_LL = get_latitude(str_array, 1)
                longitude_LL = get_longitude(str_array, 3)
                # print("in GPGLL2: Latitude: ", latitude + "  Longitude: ", longitude)
                # lcd_uart.write("in GNGLL2")

            elif str_array[0] is '$GPGGA':
                # print("in GPGGA")
                # lcd_uart.write("in GNGGA")
                latitude_GA = get_latitude(str_array, 2)
                longitude_GA = get_longitude(str_array, 4)
                # print("in GPGGA2: Latitude: ", latitude  + "  Longitude: ", longitude)
        except (ValueError, IndexError):
            lcd_uart.write(b"Error                           ")  # For 16x2 LCD
            print("valueError: Likely no signal from being inside, no GPS antenna connected, or a broken wire")

    if (latitude_LL != 0 and latitude_GA != 0):
        latDivisor = 2

    if (longitude_LL != 0 and longitude_GA != 0):
        lonDivisor = 2

    latitude_avg = (float(latitude_LL) + float(latitude_GA)) / latDivisor
    longitude_avg = (float(longitude_LL) + float(longitude_GA)) / lonDivisor

    # print("LatIN: " + str(latitude_avg) + " LongIN: " + str(longitude_avg))
    return latitude_avg, longitude_avg


def imu_update(latAvg, longAvg, time_interval, velocity_x, velocity_y):
    earth_radius = 6378137.0  # Earth's radius in meters

    imu_acceleration_x, imu_acceleration_y, imu_acceleration_z = sensor.linear_acceleration

    # print("Accelerations")
    # print(f"Acceleration X: {imu_acceleration_x:.10f}   Acceleration Y: {imu_acceleration_y:.10f}")
    # print("Sensor Linear Accelearion")
    # print(sensor.linear_acceleration)

    # Velocity Estimation
    velocity_x += imu_acceleration_x * time_interval
    velocity_y += imu_acceleration_y * time_interval

    # print("VELOCITIES")
    # print(f"Velocity X: {velocity_x:.10f}   Velocity Y: {velocity_y:.10f}")

    # Position Estimation
    latitude_change = ((velocity_x * time_interval) / earth_radius) * (180 / math.pi)
    longitude_change = ((velocity_y * time_interval) / earth_radius) * (180 / math.pi) / math.cos(math.radians(latAvg))

    # print("LAT AVG/LONGAVG")
    # print(f"Latitude: {latAvg:.10f}   Longitude: {longAvg:.10f}")

    # print("CHANGES")
    # print(f"Latitude Change: {latitude_change:.10f}   Longitude Change: {longitude_change:.10f}")

    # Update latitude and longitude
    newlatAvg = latAvg + latitude_change
    newlongAvg = longAvg + longitude_change

    # endTime = time.ticks_ms()
    # print(f'''
    #       IMU UPDATE\n
    #       Latitude: {newlatAvg:.10f}   Longitude: {newlongAvg:.10f}\n
    #       IMU DATA: {sensor.linear_acceleration}\n
    #       ''')
    # print("IMU Refresh Rate: ", float(endTime - startTime))

    return newlatAvg, newlongAvg, velocity_x, velocity_y


# example main function that implements EKF (does not include
if __name__ == '__main__':
    # imports polygons
    outerPolygon, innerPolygon = dataReceive()
    # initializes devices
    i2c = busio.I2C(board.GP15, board.GP14, frequency=100)  # Initializes I2C for the IMU
    sensor = adafruit_bno055.BNO055_I2C(i2c)  # Initializes IMU

    last_val = 0xFFFF

    gps_uart = initialize_gps()  # Initializes GPS
    lcd_uart = initialize_lcd(backlight_red=255, backlight_green=1, backlight_blue=255)

    lcd_uart.write(b"Connecting to GPS...            ")  # For 16x2 LCD
    # time.sleep(1.5) - Can add back in to display message for readability on LCD screen. The GPS sensor needs a few seconds to connect usually anyways.

    # Example polygon for testing

    '''
    outerPolygon = [
    (40.430484, 86.915721),
    (40.430454, 86.915769),
    (40.430806, 86.916144),
    (40.430835, 86.916097)
    ]

    innerPolygon = [
    (40.430484, 86.915721),
    (40.430454, 86.915769),
    (40.430806, 86.916144),
    (40.430835, 86.916097)
    ]
    '''

    # CHANGE IMU SETTINGS HERE
    imu_update_points = 10  # This value can be further optimized. If set to zero, there will be no IMU points (only GPS points).
    imu_time_interval = 0.1  # This value can be further optimized. See IMU BNO055 documentation for minimum refresh rate.

    # Initalize the GPS position and time trackers
    velocity_x = 0
    velocity_y = 0

    latitude_avg, longitude_avg = 0, 0
    latitude_avg, longitude_avg = get_gps_location(gps_uart)
    gps_start_time, imu_start_time = time.ticks_ms(), time.ticks_ms()
    latitude_LL = 0
    longitude_LL = 0
    latitude_GA = 0
    longitude_GA = 0
    latDivisor = 1
    lonDivisor = 1

    # Initial state: [lat, lon, theta, vx, vy]
    x0 = [latitude_avg, longitude_avg, 0, 0, 0]

    # Initial covariance
    P0 = [[1e-4, 0, 0, 0, 0],
          [0, 1e-4, 0, 0, 0],
          [0, 0, 1e-2, 0, 0],
          [0, 0, 0, 1e-2, 0],
          [0, 0, 0, 0, 1e-2]]
    print(os.listdir())

    ekf = kalman_filter.ExtendedKalmanFilter(x0, P0)

    # Larger values for velocity → allow for some drift.
    # Heading (theta) has intermediate noise → account for small gyro errors.
    R = [[1e-4, 0, 0, 0, 0],
         [0, 1e-4, 0, 0, 0],
         [0, 0, 1e-3, 0, 0],
         [0, 0, 0, 1e-2, 0],
         [0, 0, 0, 0, 1e-2]]

    # GPS measurement noise - uncertainty in the GPS reading
    # Smaller values → GPS is very accurate, EKF leans more on GPS updates.
    # Larger values → GPS is noisy and EKF trusts IMU prediction more.
    # NOTE: in this case, since the kart will be used on a mostly flat track/parking lot, use smaller values
    Q = [[5e-5, 0],
         [0, 5e-5]]

    # Main loop
    while True:
        # --- GPS reading ---
        str_array = gps_uart.readline()
        gps_available = False
        if str_array is not None:
            try:
                str_array = str_array.decode("utf-8").split(",")
                latitude_LL = longitude_LL = latitude_GA = longitude_GA = 0

                if str_array[0] == '$GPGLL':
                    latitude_LL = get_latitude(str_array, 1)
                    longitude_LL = get_longitude(str_array, 3)
                    # print("in GPGLL2: Latitude: ", latitude + "  Longitude: ", longitude)
                elif str_array[0] == '$GPGGA':
                    latitude_GA = get_latitude(str_array, 2)
                    longitude_GA = get_longitude(str_array, 4)
                    # print("in GPGGA2: Latitude: ", latitude  + "  Longitude: ", longitude)

                if (latitude_LL != 0 or latitude_GA != 0) and (longitude_LL != 0 or longitude_GA != 0):
                    latDivisor = 2 if latitude_LL != 0 and latitude_GA != 0 else 1
                    lonDivisor = 2 if longitude_LL != 0 and longitude_GA != 0 else 1
                    latitude_avg = (float(latitude_LL) + float(latitude_GA)) / latDivisor
                    longitude_avg = (float(longitude_LL) + float(longitude_GA)) / lonDivisor
                    gps_available = True
                    print(f'''
GPS UPDATE\n
Latitude: {latitude_avg:.10f}   Longitude: {longitude_avg:.10f}\n
Raw Data: {str_array}\n
GPS UPDATE TIME: {time.ticks_ms() - gps_start_time}\n
                            ''')

            except (ValueError, IndexError):
                lcd_uart.write(b"Error No Signal                 ")
                print("GPS signal error or disconnected.")

        # --- IMU reading ---
        dt = (time.ticks_ms() - imu_start_time) / 1000
        imu_start_time = time.ticks_ms()

        ax, ay = sensor.linear_acceleration[0], sensor.linear_acceleration[1]
        omega = sensor.gyro[2] if hasattr(sensor, 'gyro') else 0

        # --- EKF Prediction (IMU) ---
        ekf.propagate(u=(ax, ay, omega), dt=dt, R=R)

        # --- EKF Update (GPS) ---
        if gps_available:
            ekf.update(z=[latitude_avg, longitude_avg], Q=Q)

        # --- Get filtered state ---
        lat_f, lon_f, theta, vx, vy = ekf.state()

        # --- Print / debug ---
        print(f'''
FILTERED UPDATE
Latitude: {lat_f:.10f}   Longitude: {lon_f:.10f}
Velocity: vx={vx:.3f}, vy={vy:.3f}   Heading: {theta:.3f} rad
IMU Accel: ax={ax:.3f}, ay={ay:.3f}, omega={omega:.3f}
        ''')

        if is_within_polygon(outerPolygon, (float(lat_f), float(lon_f))) is True and is_within_polygon(innerPolygon,
                                                                                                       (float(lat_f),
                                                                                                        float(lon_f))) is False:
            print("IN")
        else:
            print("OUT")

