import struct

# Example hex data
hex_data = (
    "86fefee69efe9e1866667866f8e606e060e61ef89e1878783f9e0066669886609e"
    "1e66009898fe661e021ef80066e666fefefe7efef87e06f8fe7866667e667e1800"
    # Truncated for brevity...
)

# Convert hex string to bytes
byte_data = bytes.fromhex(hex_data)

# Packet details (example values, adjust based on documentation)
PACKET_HEADER = b'\x55'  # Example header byte
PACKET_SIZE = 11         # Size of each packet
PACKET_TYPES = {
    b'\x51': "Accelerometer",
    b'\x52': "Gyroscope",
    b'\x53': "Angle",
}  # Example data types

def parse_packet(packet):
    """Parse a single packet."""
    try:
        header = packet[0:1]
        data_type = packet[1:2]
        if header != PACKET_HEADER:
            return None  # Not a valid packet
        if data_type not in PACKET_TYPES:
            return None  # Unknown data type
        # Unpack the payload (e.g., accelerometer data)
        ax, ay, az, checksum = struct.unpack('<hhhB', packet[2:])
        ax = ax / 32768.0 * 16  # Scale based on documentation
        ay = ay / 32768.0 * 16
        az = az / 32768.0 * 16
        return f"{PACKET_TYPES[data_type]}: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}"
    except Exception as e:
        return f"Error parsing packet: {e}"

# Parse all packets
offset = 0
while offset + PACKET_SIZE <= len(byte_data):
    packet = byte_data[offset:offset + PACKET_SIZE]
    result = parse_packet(packet)
    if result:
        print(result)
    offset += PACKET_SIZE
