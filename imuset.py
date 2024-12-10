import serial
import time

# Serial port configuration
SERIAL_PORT = "/dev/serial1"  # Replace with your serial port
DEFAULT_BAUD_RATE = 9600     # Start with the default baud rate
NEW_BAUD_RATE = 115200       # Desired baud rate
PACKET_HEADER = b'\x55'      # Example header byte
PACKET_MIN_SIZE = 11         # Minimum packet size


# Command to set the new baud rate (example for 115200bps)
# Adjust the command according to the IMU documentation
set_baud_rate_command = b'\xFF\xAA\x02\x08'  # 0x08 represents 115200bps

try:
    # Step 1: Open serial port at the default baud rate
    ser = serial.Serial(SERIAL_PORT, DEFAULT_BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {DEFAULT_BAUD_RATE} baud.")

    # Step 2: Send the command to set the baud rate
    print("Sending baud rate configuration command...")
    ser.write(set_baud_rate_command)
    time.sleep(1)  # Wait for the command to be processed
    ser.close()    # Close the connection to apply changes

    # Step 3: Open serial port at the new baud rate
    ser = serial.Serial(SERIAL_PORT, NEW_BAUD_RATE, timeout=1)
    print(f"Reconnected to {SERIAL_PORT} at {NEW_BAUD_RATE} baud.")

    # Step 4: Read data to verify the new baud rate
    print("Reading data from IMU at new baud rate...")
    while True:
        data = ser.read(11)  # Read 11 bytes (packet size)
        if data:
            print(data.hex())  # Print raw data in hex format
except Exception as e:
    print(f"Error: {e}")
finally:
    ser.close()
    print("Serial port closed.")
