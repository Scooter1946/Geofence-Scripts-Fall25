import time
from datetime import date
import serial # pip install pyserial
import csv

coordinateFilename = "coordinates"
# Options: armsSide, baseball, volley, road
datetime = str(date.today()) + '_' + time.strftime("%H-%M-%S", time.localtime())

def initialize():
    ser = 0
    while ser == 0:
        try:
            ser = serial.Serial('COM3', 9600)  # change to serial port the pi is connected to
        except:
            print("Busy Port: Try closing Raspberry Pi IDE(Thonny) or change \"COM\" parameter on line 14")
            time.sleep(0.3) 
            pass
    return ser

# Writes variable x to the PI
def serialWrite(x, ser):
    time.sleep(1) # Required to give code time to upload, may need to test with increasing/decreasing it
    cmd = str(x)
    cmd = cmd + '\r'
    print(f"Sending Command: [{cmd}]")
    ser.write(cmd.encode()) # Encodes cmd to transfer to pico
    time.sleep(0.05)
    
    reply = b''

    for _ in range(len(cmd)):
        a = ser.read() # Read the loopback chars and ignore

    coordinateTransferred = False
    while True:
        while True:
            a = ser.read()

            if a== b'\r':
                break

            else:
                reply += a
            #time.sleep(0.001) This time.sleep can be added back for readability while live-testing; however it could delay GPS/IMU points from showing up in the logfile
        if coordinateTransferred == False:
            #print(f"\nReply was: {reply}")
            print("\nConnection Confirmed")
            coordinateTransferred = True
        if coordinateTransferred:
            reply = str(reply)
            print(f"{reply[4:len(reply)-1]}\n")
            with open(f'datalog{datetime}.txt', 'a') as f:
                f.write(f'{reply[4:len(reply)-1]}\n')
        reply = b''

    ser.close()

# Turns coordinates.csv file into one string
def get_coordinates():
  filename = open(f'{coordinateFilename}.csv', 'r')
  data = list(csv.reader(filename, delimiter = ","))
  filename.close()

  # Puts coordinates into one row array
  new_data = []
  for row in range(len(data)):
    for col in range(len(data[row])):
      if (data[row][col] == "OUTER" or data[row][col] == "INNER"):
        new_data.append(data[row][col])
      else:
        new_data.append(str(abs(float(data[row][col]))))

  big_string = ', '.join(new_data) # Turns coordinates.csv file into one long string to make data transfer easier
  # Could transfer coordinates one at a time using a loop if that turns out to be easier
  return(big_string)

coordinate_data = get_coordinates() # Gets coordinates from coordinates.csv and changes the format
serialWrite(coordinate_data, initialize()) # Transmits data to raspberry pi and returns raspberry pi response
