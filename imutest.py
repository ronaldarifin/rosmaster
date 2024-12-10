import serial
import struct

# Serial port configuration
SERIAL_PORT = "/dev/serial1"  # Replace with the correct serial port
BAUD_RATE = 115200  # Adjust based on the IMU's documentation
PACKET_SIZE = 11  # Typical packet size for HWT901B
HEADER = 0x55  # Header byte indicating the start of a packet

def initialize_serial():
    """
    Initializes the serial port for communication.
    """
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        return ser
    except Exception as e:
        print(f"Error initializing serial port: {e}")
        exit()

def parse_packet(packet):
    """
    Parses a single packet and extracts data based on the protocol.
    """
    try:
        if packet[0] == HEADER:
            # Extract data type and sensor values
            data_type = packet[1]
            ax, ay, az = struct.unpack('<hhh', packet[2:8])  # Adjust format as per protocol
            checksum = packet[-1]

            # Verify checksum (sum of all bytes except checksum itself)
            calculated_checksum = sum(packet[:-1]) & 0xFF
            if checksum != calculated_checksum:
                print(f"Checksum mismatch: calculated={calculated_checksum}, received={checksum}")
                return None

            # Interpret data based on type
            if data_type == 0x51:  # Accelerometer data
                ax_scaled = ax / 32768.0 * 16  # Convert to g-force
                ay_scaled = ay / 32768.0 * 16
                az_scaled = az / 32768.0 * 16
                return f"Accel: ax={ax_scaled:.2f}g, ay={ay_scaled:.2f}g, az={az_scaled:.2f}g"
            elif data_type == 0x52:  # Gyroscope data
                gx = ax / 32768.0 * 2000  # Convert to degrees/sec
                gy = ay / 32768.0 * 2000
                gz = az / 32768.0 * 2000
                return f"Gyro: gx={gx:.2f}°/s, gy={gy:.2f}°/s, gz={gz:.2f}°/s"
            elif data_type == 0x53:  # Angle data
                roll = ax / 32768.0 * 180  # Convert to degrees
                pitch = ay / 32768.0 * 180
                yaw = az / 32768.0 * 180
                return f"Angle: roll={roll:.2f}°, pitch={pitch:.2f}°, yaw={yaw:.2f}°"
            else:
                return f"Unknown data type: {data_type:#x}"
    except Exception as e:
        print(f"Error parsing packet: {e}")
        return None

def read_and_parse_data(ser):
    """
    Reads data from the serial port and parses valid packets.
    """
    buffer = b''
    try:
        while True:
            buffer += ser.read(1024)  # Read chunks of data
            while len(buffer) >= PACKET_SIZE:
                # Process one packet at a time
                packet = buffer[:PACKET_SIZE]
                buffer = buffer[PACKET_SIZE:]  # Remove processed packet

                # Parse and print packet data
                result = parse_packet(packet)
                if result:
                    print(result)
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error during data reading: {e}")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    # Step 1: Initialize the serial connection
    ser = initialize_serial()

    # Step 2: Read and parse data
    read_and_parse_data(ser)
