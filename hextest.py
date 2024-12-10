import struct

data = "98e60660fee686601878787e1e9e6666989e780666e0e6009ee6fe669e1e1e600066e666fefefefefee0fee6fe6666667ee69e6000e680006618fe1e069e7e7e66e69e66fefefefefe1e069e069866866698e60660fee686601878787e1e9e6666989e780666e0e6009ee6fe669e1e1e600066e666fefefefefee07ee0fee6f866667e66"

# Check if all characters are valid hex digits
is_valid_hex = all(c in "0123456789abcdefABCDEF" for c in data)
print("Is valid hexadecimal data:", is_valid_hex)

if is_valid_hex:
    # Convert hex string to bytes
    byte_data = bytes.fromhex(data)

    PACKET_SIZE = 11  # Example size, adjust as per the documentation
    HEADER = 0x55  # Example header byte, adjust as needed

    # Split data into packets and process
    for i in range(0, len(byte_data), PACKET_SIZE):
        packet = byte_data[i:i + PACKET_SIZE]
        if len(packet) < PACKET_SIZE:
            break  # Skip incomplete packets
        if packet[0] == HEADER:
            # Extract data (adjust fields based on your protocol)
            ax, ay, az = struct.unpack('<hhh', packet[2:8])  # Example unpacking
            print(f"Accel: ax={ax}, ay={ay}, az={az}")
        else:
            print(f"Invalid packet header: {packet[0]:#x}")
