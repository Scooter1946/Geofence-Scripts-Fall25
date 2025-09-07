# How imu_gps_update_parallel.py works

Objects 
-
point - (X.Y) coords

Data Input:
- 
dataReceive() - Uses select.poll() to wait for input from the console (stdin).
Receives a formatted geofence string (with "OUTER" and "INNER" polygons).
Splits the coordinates, groups them into (lat, lon) pairs.
Returns two polygon lists: outerList and innerList.

Hardware Initializations:
-
initialize_gps() - Creates a UART object on pins GP12 (TX) and GP13 (RX) for GPS.

initialize_lcd(backlight_red, backlight_green, backlight_blue) - Creates a UART object on pins GP4 (TX) and GP5 (RX) for LCD control.
Returns a handle for later writing text.

Receiving Data from GPS:
-
get_gps_location(gps_uart) -
Continuously reads from GPS until it has both \$GPGLL and \$GPGGA data.
Converts both into decimal lat/long, then averages them if both available.
Returns (latitude_avg, longitude_avg).

GPS Parsing:
-
get_latitude(str_array, index) - parse lattitude from \$GPGGA or \$GPGLL and 
converts from degrees and minutes into decimal degrees (pos = north)

get_longitude(str_array, index2) - Converts from degrees/minutes to decimal degrees.
Returns negative if the hemisphere is E (East).

Receiving Data from IMU + Predicting positions
-
imu_update(latAvg, longAvg, time_interval, velocity_x, velocity_y) -
Reads linear acceleration (x, y, z) from the BNO055 IMU.
Integrates acceleration → velocity.
Integrates velocity → position change (lat/long shift).
Updates and returns new (lat, long, velocity_x, velocity_y).

Kart Position:
-
onSegment(p, q, r) - Checks if point q lies on the line segment from p to r.

orientation(p, q, r) - Determines the orientation of the triplet (p, q, r)

doIntersect(p1, q1, p2, q2) - Determines if two line segments (p1, q1) and (p2, q2) intersect.
Uses orientation and onSegment

is_within_polygon(points, p) - Implements the ray-casting algorithm.
Shoots a horizontal ray from point p to infinity and counts intersections with polygon edges.
Odd = inside, Even = outside.  

Program Flow
-
1. Gets geofence polygons from PC (dataReceive).
2. Initializes IMU over I2C, GPS over UART, and LCD.
3. Displays “Connecting to GPS...” on LCD.
4. Waits until a GPS fix is obtained, averages GLL + GGA readings.
5. Main Loop
   1. Reads GPS data (when available) → updates position.
   2. Calls imu_update() between GPS updates to refine the position estimate.
   3. Prints GPS/IMU updates with timestamps.
   4. TODO: check if within boundaries using is_within_polygon

## TODO
- [ ] Implement is_within_polygon in main
  - Only check after GPS signal or GPS + IMU?
  - [ ] Signal to kart (cut throttle / brake) if not within polygon 
- [ ] Reimplement imu_update to use kalman filter
  - Check example in kalman_filter.py
  - https://sparxeng.com/blog/software/imu-signal-processing-with-kalman-filter#:~:text=The%20Kalman%20Filter%20is%20a,reality%2C%20and%20many%20other%20fields
  - https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf
  - https://github.com/motokimura/kalman_filter_with_kitti/blob/master/src/kalman_filters/extended_kalman_filter.py
    - Important: check units and orientation to ensure that calculations are correct
- [ ] Add documentation for functions


