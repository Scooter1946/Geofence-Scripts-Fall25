# Geofence Scripts

Computer-End(Python) - Geofence-Scripts -> computer-end-code.py

Pi-End(MicroPython) - Geofence Scripts -> imu_update.py (pi-end-code.py is the older version that is now deprecated)

imu_update.py has everything you need to run the geofence (Instructions in Transition Document and also important comments at the top of the file).

To Test: 
1. Download Thonny and install required libraries if they aren't already installed.
2. Download Visual Studio(or other preferred IDE such as IDLE) for running the computer-end code. As far as we know you cannot run both programs from Thonny.
3. Use website to create coordinates.csv file, you can change the name but make sure you change this in the computer end code on line 5.
4. Put coordinates.csv file in the same folder as the computer-end code.
5. Download pySerial library. Make sure you dont download "serial" library as this is completely different. If you do download serial, delete this otherwise you will have issues (Will give you a "busy port" error).
6. Through Thonny figure out which port the Raspberry Pi Pico is connected to (found under "Tools" bar).
7. Change "COM#" argument for serial.Serial() inside of the initialize function of the computer-end-code to the connected port to the COM # port the Pi is connected to found in step 6.
8. Run the pi-end-code in Thonny first.
9. Close out Thonny once you see "listening..." printed, otherwise the port will be busy and data won't transfer.
10. Next run the computer-end-code on a different IDE than Thonny (Visual Studio, IDLE, etc).
11. Should print "Connection Confirmed" in the terminal as well as any other print statements if successful. Our code will also log any data in a timestamped text file in the same folder as the computer-end-code on your computer.
12. To split the text file into csv's you can run the extractData.py file and change the filename in the first line to the file you want to split.


Troubleshooting:
"Busy Port: Try closing Raspberry Pi IDE(Thonny)", then that means something is interupting the port the Pi is connected to. This would be caused if you kept Thonny open while running the computer end code. You could also have other serial monitors or serial data printers such as the one in the Arduino IDE that is preventing you from transferring data. As mentioned in step 5, you could also be using the "serial" library, rather than the "pyserial" library. Make sure you delete serial libary if you have that installed. You could also have the wrong COM port selected in the computer end code as discussed in step 7.

"ValueError: Likely no signal from being inside, no GPS antenna connected, or a broken wire" comes from the impu_update.py code. It is a ValueError, and the most likely culprits are being somewhere where the GPS can't get satellite data, such as too close to a building or inside. It could also indicate a problem with one of the connections, most likely the GPS connection. Make sure there are no cut or disconnected wires and the GPS antenna is firmly connected to the GPS module on the geofence. It could also be that the GPS needs a few seconds to connect to satellites when first turned on (usually five seconds at worst).

Contact raghav21@purdue.edu for any questions
