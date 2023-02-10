import serial
import pynmea2

ser = serial.Serial("/dev/serial0", 9600, timeout=0.5)

while True:
    data = ser.readline().decode("utf-8")
    if data.startswith("$GPGGA"):
        msg = pynmea2.parse(data)
        print("Latitude: " + str(msg.latitude) + " Longitude: " + str(msg.longitude))