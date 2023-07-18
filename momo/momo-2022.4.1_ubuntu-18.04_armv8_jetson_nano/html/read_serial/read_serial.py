import serial
import time

def read_from_serial_port():
    # Specify the correct port and baudrate
    ser = serial.Serial('/dev/pts/7', 9600)

    try:
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                print("Received data: ", data)
                time.sleep(0.01)  # Just to prevent overwhelming the CPU
    except Exception as e:
        print("Error reading from serial port: ", e)
    finally:
        ser.close()

# Start reading data from the serial port
read_from_serial_port()
