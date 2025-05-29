import socket
import struct
import time
import random
import math

def get_local_ip():
    try:
        # Create a temporary socket to get the local IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except:
        return "Could not determine local IP"

def decode_packet(data):
    if len(data) != 65:
        return f"Invalid packet length: {len(data)}"
    
    packet_type = data[0]
    payload = data[1:]
    
    try:
        if packet_type == 0:  # ServoCommand
            x = struct.unpack('<d', payload[0:8])[0]
            y = struct.unpack('<d', payload[8:16])[0]
            z = struct.unpack('<d', payload[16:24])[0]
            return f"ServoCommand: x={x:.3f}, y={y:.3f}, z={z:.3f}"
            
        elif packet_type == 1:  # TriggerCommand
            value = payload[0] != 0
            return f"TriggerCommand: {value}"
            
        elif packet_type == 2:  # EscCommand
            value = struct.unpack('<d', payload[0:8])[0]
            return f"EscCommand: {value:.3f}"
            
        elif packet_type == 3:  # TunePitch
            p = struct.unpack('<d', payload[0:8])[0]
            i = struct.unpack('<d', payload[8:16])[0]
            d = struct.unpack('<d', payload[16:24])[0]
            return f"TunePitch: p={p:.3f}, i={i:.3f}, d={d:.3f}"
            
        elif packet_type == 4:  # TuneYaw
            p = struct.unpack('<d', payload[0:8])[0]
            i = struct.unpack('<d', payload[8:16])[0]
            d = struct.unpack('<d', payload[16:24])[0]
            return f"TuneYaw: p={p:.3f}, i={i:.3f}, d={d:.3f}"
            
        elif packet_type == 5:  # Start
            return "Start"
            
        elif packet_type == 6:  # Stop
            return "Stop"
            
        elif packet_type == 7:  # SetAutonomous
            return "SetAutonomous"
            
        elif packet_type == 8:  # SetManual
            return "SetManual"
            
        elif packet_type == 9:  # SetAutoAim
            return "SetAutoAim"
            
        elif packet_type == 10:  # SetOffset
            x = struct.unpack('<d', payload[0:8])[0]
            y = struct.unpack('<d', payload[8:16])[0]
            z = struct.unpack('<d', payload[16:24])[0]
            return f"SetOffset: x={x:.3f}, y={y:.3f}, z={z:.3f}"
            
        elif packet_type == 11:  # SetPitchIntegralLimit
            limit = struct.unpack('<d', payload[0:8])[0]
            return f"SetPitchIntegralLimit: {limit:.3f}"
            
        elif packet_type == 12:  # SetYawIntegralLimit
            limit = struct.unpack('<d', payload[0:8])[0]
            return f"SetYawIntegralLimit: {limit:.3f}"
            
        elif packet_type == 13:  # SetLaunchThreshold
            eps = struct.unpack('<d', payload[0:8])[0]
            n = payload[8]
            return f"SetLaunchThreshold: eps={eps:.3f}, n={n}"
            
        elif packet_type == 17:  # Query
            return "Query"
            
        elif packet_type == 18:  # MotorOffset
            offset = struct.unpack('<d', payload[0:8])[0]
            return f"MotorOffset: {offset:.3f}"
            
        elif packet_type == 20:  # SetCutoffFrequency
            freq = struct.unpack('<d', payload[0:8])[0]
            return f"SetCutoffFrequency: {freq:.3f}"
            
        elif packet_type == 21:  # LaunchCounter
            counter = int(struct.unpack('<d', payload[0:8])[0])
            return f"LaunchCounter: {counter}"
            
        elif packet_type == 22:  # SetStopThrottle
            throttle = struct.unpack('<d', payload[0:8])[0]
            return f"SetStopThrottle: {throttle:.3f}"
            
        elif packet_type == 23:  # SetMaxConsecutiveNans
            max_nans = struct.unpack('<I', payload[0:4])[0]
            return f"SetMaxConsecutiveNans: {max_nans}"
            
        elif packet_type == 24:  # SetDefaultSpeed
            speed = struct.unpack('<d', payload[0:8])[0]
            return f"SetDefaultSpeed: {speed:.3f}"
            
        else:
            return f"Unknown packet type: {packet_type}"
            
    except Exception as e:
        return f"Error decoding packet: {str(e)}"

def create_bbox_packet(x1, y1, x2, y2):
    # Packet format: [type(1 byte) + payload(64 bytes)]
    packet = bytearray(65)
    
    # Set packet type to 14 (BboxPos)
    packet[0] = 14
    
    # Convert coordinates to little-endian doubles
    struct.pack_into('<d', packet, 1, x1)   # x1 at offset 1
    struct.pack_into('<d', packet, 9, y1)   # y1 at offset 9
    struct.pack_into('<d', packet, 17, x2)  # x2 at offset 17
    struct.pack_into('<d', packet, 25, y2)  # y2 at offset 25
    
    return packet

def create_set_manual_packet():
    # Packet format: [type(1 byte) + payload(64 bytes)]
    packet = bytearray(65)
    # Set packet type to 8 (setManual)
    packet[0] = 8
    return packet

def create_start_packet():
    # Packet format: [type(1 byte) + payload(64 bytes)]
    packet = bytearray(65)
    # Set packet type to 5 (start)
    packet[0] = 5
    return packet

def create_stop_packet():
    # Packet format: [type(1 byte) + payload(64 bytes)]
    packet = bytearray(65)
    # Set packet type to 6 (stop)
    packet[0] = 6
    return packet

def create_data_packet(type_value, data):
    # Create a 65-byte packet
    packet = bytearray(65)
    # First byte is type
    packet[0] = type_value
    
    if type_value == 19:  # FilteredBbox
        # Convert x, y coordinates to bytes (little endian)
        x_bytes = struct.pack('<d', data[0])  # x coordinate
        y_bytes = struct.pack('<d', data[1])  # y coordinate
        z_bytes = struct.pack('<d', 0.0)      # z coordinate (unused)
        
        # Copy coordinates into packet
        packet[1:9] = x_bytes    # x at offset 1
        packet[9:17] = y_bytes   # y at offset 9
        packet[17:25] = z_bytes  # z at offset 17
    
    return packet

def create_current_state_packet(mode, state):
    # Packet format: [type(1 byte) + payload(64 bytes)]
    packet = bytearray(65)
    # Set packet type to 25 (CurrentState)
    packet[0] = 25
    
    # Set mode and state
    packet[1] = mode  # 7 for autonomous, 8 for manual, 9 for autoAim
    packet[2] = state  # 5 for running, 6 for stopped
    
    # Fill rest of the packet with default values
    # launchCounter (UInt32)
    struct.pack_into('<I', packet, 3, 0)
    # maxConsecutiveNans (UInt32)
    struct.pack_into('<I', packet, 7, 0)
    # targetX (Double)
    struct.pack_into('<d', packet, 11, 0.0)
    # targetY (Double)
    struct.pack_into('<d', packet, 19, 0.0)
    # stopThrottle (Double)
    struct.pack_into('<d', packet, 27, 0.0)
    # motorOffset (Double)
    struct.pack_into('<d', packet, 35, 0.0)
    # defaultSpeed (Double)
    struct.pack_into('<d', packet, 43, 0.0)
    # cutoffFreq (Double)
    struct.pack_into('<d', packet, 51, 0.0)
    
    return packet

def main():
    # Server configuration
    HOST = '0.0.0.0'  # Listen on all available interfaces
    PORT = 12345
    
    # State variables
    target_x = 0.0
    target_y = 0.0
    max_consecutive_nans = 10
    stop_throttle = 0.0
    motor_offset = 0.0
    default_speed = 1.0
    cutoff_freq = 10.0
    launch_counter = 0
    
    # Create socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    
    print(f"Server listening on {HOST}:{PORT}")
    
    # Accept client connection
    client_socket, address = server_socket.accept()
    print(f"Connected to client at {address}")
    
    try:
        # Wait for Query response before sending CurrentState
        print("Waiting for Query response...")
        data = client_socket.recv(65)
        if data and data[0] == 17:  # Query response
            print("Received Query response, sending CurrentState packet")
            # Send CurrentState packet with manual mode and running state
            current_state_packet = create_current_state_packet(8, 5)  # 8 for manual, 5 for running
            # Update packet with current values
            struct.pack_into('<I', current_state_packet, 3, launch_counter)
            struct.pack_into('<I', current_state_packet, 7, max_consecutive_nans)
            struct.pack_into('<d', current_state_packet, 11, target_x)
            struct.pack_into('<d', current_state_packet, 19, target_y)
            struct.pack_into('<d', current_state_packet, 27, stop_throttle)
            struct.pack_into('<d', current_state_packet, 35, motor_offset)
            struct.pack_into('<d', current_state_packet, 43, default_speed)
            struct.pack_into('<d', current_state_packet, 51, cutoff_freq)
            client_socket.send(current_state_packet)
            print("Sent CurrentState packet")
        else:
            print("Received unexpected packet type")
            return

        packet_counter = 0  # Counter for tracking number of packets sent
        is_running = True
        current_mode = 8
        
        while True:
            # Check for incoming data
            try:
                client_socket.settimeout(0.001)  # Set a very short timeout
                data = client_socket.recv(65)
                if data:
                    decoded = decode_packet(data)
                    print(f"Received: {decoded}")
                    
                    # Handle SetOffset command
                    if data[0] == 10:  # SetOffset
                        target_x = struct.unpack('<d', data[1:9])[0]
                        target_y = struct.unpack('<d', data[9:17])[0]
                        print(f"Updated target offset: x={target_x:.3f}, y={target_y:.3f}")
                        continue
                    
                    # Handle SetMaxConsecutiveNans
                    elif data[0] == 23:
                        max_consecutive_nans = struct.unpack('<I', data[1:5])[0]
                        print(f"Updated max consecutive NANs: {max_consecutive_nans}")
                        continue
                    
                    # Handle SetStopThrottle
                    elif data[0] == 22:
                        stop_throttle = struct.unpack('<d', data[1:9])[0]
                        print(f"Updated stop throttle: {stop_throttle:.3f}")
                        continue
                    
                    # Handle MotorOffset
                    elif data[0] == 18:
                        motor_offset = struct.unpack('<d', data[1:9])[0]
                        print(f"Updated motor offset: {motor_offset:.3f}")
                        continue
                    
                    # Handle SetDefaultSpeed
                    elif data[0] == 24:
                        default_speed = struct.unpack('<d', data[1:9])[0]
                        print(f"Updated default speed: {default_speed:.3f}")
                        continue
                    
                    # Handle SetCutoffFrequency
                    elif data[0] == 20:
                        cutoff_freq = struct.unpack('<d', data[1:9])[0]
                        print(f"Updated cutoff frequency: {cutoff_freq:.3f}")
                        continue
                    
                    # Handle STOP command
                    elif data[0] == 6:  # Stop command
                        is_running = False
                        # print("Received STOP command, stopping data transmission")
                        # client_socket.send(create_stop_packet())  # Send stop response
                        continue
                    
                    # Handle START command
                    elif data[0] == 5:  # Start command
                        is_running = True
                        # print("Received START command, resuming data transmission")
                        # client_socket.send(create_start_packet())  # Send start response
                        continue

                    elif data[0] == 7:
                        current_mode = 7
                        continue

                    elif data[0] == 8:
                        current_mode = 8
                        continue
                    
                    elif data[0] == 9:
                        current_mode = 9
                        continue
                    
                    # Handle Query command
                    elif data[0] == 17:  # Query command
                        print("Received Query command, sending CurrentState packet")
                        # Send CurrentState packet with current mode and state
                        current_state_packet = create_current_state_packet(current_mode, 5 if is_running else 6)
                        # Update packet with current values
                        struct.pack_into('<I', current_state_packet, 3, launch_counter)
                        struct.pack_into('<I', current_state_packet, 7, max_consecutive_nans)
                        struct.pack_into('<d', current_state_packet, 11, target_x)
                        struct.pack_into('<d', current_state_packet, 19, target_y)
                        struct.pack_into('<d', current_state_packet, 27, stop_throttle)
                        struct.pack_into('<d', current_state_packet, 35, motor_offset)
                        struct.pack_into('<d', current_state_packet, 43, default_speed)
                        struct.pack_into('<d', current_state_packet, 51, cutoff_freq)
                        client_socket.send(current_state_packet)
                        continue
                    
            except socket.timeout:
                pass  # No data received, continue with sending
            except Exception as e:
                print(f"Error receiving data: {str(e)}")
                break
            
            # Only send data if running
            if is_running:
                # Calculate oscillating coordinates for the bounding box center
                t = time.time()
                
                # BboxPos: x fixed at 0.5, y oscillates -0.5 to 0.5
                bbox_center_x = 0.5 * math.sin(t)  # Fixed x position
                bbox_center_y = 0.2  # Oscillate between -0.5 and 0.5
                
                # FilteredBbox: x fixed at -0.3, y oscillates -0.3 to 0.3
                filtered_center_x = -0.3  # Fixed x position
                filtered_center_y = 0.4 * math.sin(t)  # Oscillate between -0.3 and 0.3
                
                # Fixed size for the bounding box
                width = 0.1
                height = 0.1
                
                # Calculate box corners for BboxPos
                if packet_counter % 50 <=20:  # Every 20th packet, send NaN values
                    x1 = float('nan')
                    y1 = float('nan')
                    x2 = float('nan')
                    y2 = float('nan')
                else:
                    x1 = bbox_center_x - width/2
                    y1 = bbox_center_y - height/2
                    x2 = bbox_center_x + width/2
                    y2 = bbox_center_y + height/2
                
                # Create and send bbox packet
                bbox_packet = create_bbox_packet(x1, y1, x2, y2)
                client_socket.sendall(bbox_packet)
                
                # Create and send filtered bbox packet (center coordinates)
                filtered_packet = create_data_packet(19, [filtered_center_x, filtered_center_y])
                client_socket.sendall(filtered_packet)
                
                # Increment packet counter
                packet_counter += 1
            
            # Sleep for a short time to control update rate
            time.sleep(0.01)  # 10 Hz update rate
            
    except KeyboardInterrupt:
        print("\nServer shutting down...")
    finally:
        client_socket.close()
        server_socket.close()

if __name__ == "__main__":
    main() 