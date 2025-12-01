#!/usr/bin/env python3
"""
BNO055 Attitude Display - Receives roll and pitch data via UART
Reads raw degree data from COM21 and displays roll/pitch values with sign
Data format: [0xAA, Roll_LSB, Roll_MSB, Pitch_LSB, Pitch_MSB, 0x55]
"""

import serial
import struct
import time

# Configuration
SERIAL_PORT = 'COM21'
BAUD_RATE = 115200
START_BYTE = 0xAA
END_BYTE = 0x55

def bytes_to_signed_int16(lsb, msb):
    """Convert two bytes (LSB, MSB) to signed 16-bit integer"""
    # Combine bytes into 16-bit unsigned value
    value = (msb << 8) | lsb

    # Convert to signed (two's complement)
    if value >= 0x8000:
        value = value - 0x10000

    return value

def encode_orientation(roll_raw, pitch_raw, threshold=400):
    """
    Encode 9 orientations based on raw roll and pitch values

    Orientations:
        0 = Neutral (center)
        1 = Up (pitch positive, roll neutral)
        2 = Up-Right (pitch positive, roll positive)
        3 = Right (pitch neutral, roll positive)
        4 = Down-Right (pitch negative, roll positive)
        5 = Down (pitch negative, roll neutral)
        6 = Down-Left (pitch negative, roll negative)
        7 = Left (pitch neutral, roll negative)
        8 = Up-Left (pitch positive, roll negative)

    Args:
        roll_raw: Raw roll value (16-bit signed)
        pitch_raw: Raw pitch value (16-bit signed)
        threshold: Threshold for detecting tilt (default 400)

    Returns:
        orientation code (0-8)
    """
    # Determine roll direction
    if roll_raw > threshold:
        roll_dir = 1    # Right
    elif roll_raw < -threshold:
        roll_dir = -1   # Left
    else:
        roll_dir = 0    # Neutral

    # Determine pitch direction
    if pitch_raw > threshold:
        pitch_dir = 1   # Up
    elif pitch_raw < -threshold:
        pitch_dir = -1  # Down
    else:
        pitch_dir = 0   # Neutral

    # Encode orientation based on combination
    # Using a lookup table for clarity
    orientation_map = {
        (0, 0): 0,    # Neutral
        (0, 1): 1,    # Up
        (1, 1): 2,    # Up-Right
        (1, 0): 3,    # Right
        (1, -1): 4,   # Down-Right
        (0, -1): 5,   # Down
        (-1, -1): 6,  # Down-Left
        (-1, 0): 7,   # Left
        (-1, 1): 8    # Up-Left
    }

    return orientation_map.get((roll_dir, pitch_dir), 0)

def orientation_to_string(orientation):
    """Convert orientation code to readable string"""
    orientation_names = {
        0: "Neutral",
        1: "Up",
        2: "Up-Right",
        3: "Right",
        4: "Down-Right",
        5: "Down",
        6: "Down-Left",
        7: "Left",
        8: "Up-Left"
    }
    return orientation_names.get(orientation, "Unknown")

def main():
    print(f"Opening serial port {SERIAL_PORT} at {BAUD_RATE} baud...")

    try:
        # Open serial port
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )

        print(f"Connected to {SERIAL_PORT}")
        print("Waiting for data... (Press Ctrl+C to exit)")
        print("-" * 50)

        packet_count = 0

        while True:
            # Look for start byte
            byte = ser.read(1)
            if len(byte) == 0:
                continue

            if byte[0] == START_BYTE:
                # Read the next 5 bytes (Roll LSB, Roll MSB, Pitch LSB, Pitch MSB, End byte)
                data = ser.read(5)

                if len(data) == 5 and data[4] == END_BYTE:
                    # Extract bytes
                    roll_lsb = data[0]
                    roll_msb = data[1]
                    pitch_lsb = data[2]
                    pitch_msb = data[3]

                    # Convert to signed 16-bit integers
                    roll_raw = bytes_to_signed_int16(roll_lsb, roll_msb)
                    pitch_raw = bytes_to_signed_int16(pitch_lsb, pitch_msb)

                    # BNO055 Euler angles are in units of 1/16 degrees
                    # Divide by 16 to get degrees
                    roll_deg = roll_raw / 16.0
                    pitch_deg = pitch_raw / 16.0

                    # Encode orientation
                    orientation = encode_orientation(roll_raw, pitch_raw, threshold=400)
                    orientation_name = orientation_to_string(orientation)

                    # Display with sign and orientation
                    packet_count += 1
                    print(f"[{packet_count:05d}] Roll: {roll_deg:+7.2f}°  Pitch: {pitch_deg:+7.2f}°  "
                          f"(Raw: Roll={roll_raw:+6d}, Pitch={pitch_raw:+6d})  "
                          f"Orientation: {orientation} ({orientation_name})")

                else:
                    print(f"Warning: Invalid packet (length={len(data)}, end_byte={data[4] if len(data) == 5 else 'N/A'})")

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        print(f"\nMake sure:")
        print(f"  1. Your device is connected to {SERIAL_PORT}")
        print(f"  2. No other program is using {SERIAL_PORT}")
        print(f"  3. You have the correct permissions")

    except KeyboardInterrupt:
        print("\n\nExiting...")

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"Closed {SERIAL_PORT}")

if __name__ == "__main__":
    main()
