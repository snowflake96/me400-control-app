import socket
import struct
import threading
import time
import random
import math
from typing import Dict, Optional

class TCPServer:
    def __init__(self, host: str = '0.0.0.0', port: int = 12345):
        self.host = host
        self.port = port
        self.server_socket = None
        self.clients: Dict[str, socket.socket] = {}
        self.running = True
        self.bbox_thread = None
        
    def start(self):
        """Start the TCP server and listen for connections."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.running = True
            print(f"Server started on {self.host}:{self.port}")
            
            # Start the bounding box update thread
            self.bbox_thread = threading.Thread(target=self.send_bbox_updates)
            self.bbox_thread.daemon = True
            self.bbox_thread.start()
            
            while self.running:
                try:
                    client_socket, address = self.server_socket.accept()
                    client_id = f"{address[0]}:{address[1]}"
                    self.clients[client_id] = client_socket
                    print(f"New client connected: {client_id}")
                    
                    # Start a new thread to handle this client
                    client_thread = threading.Thread(target=self.handle_client, args=(client_id,))
                    client_thread.daemon = True
                    client_thread.start()
                    
                except Exception as e:
                    if self.running:
                        print(f"Error accepting connection: {e}")
                    
        except Exception as e:
            print(f"Server error: {e}")
        finally:
            self.stop()
            
    def stop(self):
        """Stop the server and close all connections."""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
        for client_id, client_socket in self.clients.items():
            try:
                client_socket.close()
            except:
                pass
        self.clients.clear()
        print("Server stopped")
        
    def handle_client(self, client_id):
        """Handle individual client connections."""
        client_socket = self.clients[client_id]
        try:
            while self.running:
                # Read the packet type (first byte)
                packet_type = client_socket.recv(1)
                if not packet_type:
                    break
                
                packet_type = int.from_bytes(packet_type, byteorder='little')
                
                # Read the remaining 64 bytes of the packet
                data = client_socket.recv(64)
                if not data:
                    break
                
                # Handle different packet types
                if packet_type == 0:  # ServoCommand
                    if len(data) >= 8:  # Vector3 data
                        x, y, z = struct.unpack('<ddd', data[:24])
                        print(f"Received servo command - Vector3: x={x:.2f}, y={y:.2f}, z={z:.2f}")
                    else:
                        text = data.decode('utf-8').strip('\x00')
                        print(f"Received servo command: {text}")
                elif packet_type == 1:  # TriggerCommand
                    value = bool(data[0])
                    print(f"Received trigger command: {value}")
                elif packet_type == 2:  # EscCommand
                    if len(data) >= 8:
                        value = struct.unpack('<d', data[:8])[0]
                        print(f"Received ESC command: {value:.2f}")
                elif packet_type == 3:  # TunePitch
                    if len(data) >= 24:
                        p, i, d = struct.unpack('<ddd', data[:24])
                        print(f"Received pitch PID: P={p:.2f}, I={i:.2f}, D={d:.2f}")
                elif packet_type == 4:  # TuneYaw
                    if len(data) >= 24:
                        p, i, d = struct.unpack('<ddd', data[:24])
                        print(f"Received yaw PID: P={p:.2f}, I={i:.2f}, D={d:.2f}")
                elif packet_type == 5:  # Start
                    print("Received start command")
                elif packet_type == 6:  # Stop
                    print("Received stop command")
                elif packet_type == 7:  # SetAutonomous
                    print("Received set autonomous command")
                elif packet_type == 8:  # SetManual
                    print("Received set manual command")
                elif packet_type == 9:  # SetAutoAim
                    print("Received set auto aim command")
                elif packet_type == 10:  # SetOffset
                    if len(data) >= 24:
                        x, y, z = struct.unpack('<ddd', data[:24])
                        print(f"Received set offset: x={x:.2f}, y={y:.2f}, z={z:.2f}")
                elif packet_type == 11:  # SetPitchIntegralLimit
                    if len(data) >= 4:
                        value = struct.unpack('<f', data[:4])[0]
                        print(f"Received pitch integral limit: {value:.4f}")
                elif packet_type == 12:  # SetYawIntegralLimit
                    if len(data) >= 4:
                        value = struct.unpack('<f', data[:4])[0]
                        print(f"Received yaw integral limit: {value:.4f}")
                elif packet_type == 13:  # BboxPos
                    # BboxPos is only sent from server, not handled here
                    pass
                
        except Exception as e:
            print(f"Error handling client {client_id}: {e}")
        finally:
            if client_id in self.clients:
                del self.clients[client_id]
            try:
                client_socket.close()
            except:
                pass
            print(f"Client disconnected: {client_id}")
    
    def generate_random_bbox(self):
        # Box size 0.2 x 0.2
        box_size = 0.2
        half_size = box_size / 2
        # Start at center
        center_x = 0.0
        center_y = 0.0
        # Randomly choose direction: 0=+x, 1=-x, 2=+y, 3=-y
        direction = random.randint(0, 3)
        movement = random.uniform(0.05, 0.5)  # Move by 0.05 to 0.5 in the chosen direction
        if direction == 0:  # +x
            center_x += movement
        elif direction == 1:  # -x
            center_x -= movement
        elif direction == 2:  # +y
            center_y += movement
        elif direction == 3:  # -y
            center_y -= movement
        # Clamp to [-1+half_size, 1-half_size] to keep box in bounds
        center_x = max(-1 + half_size, min(1 - half_size, center_x))
        center_y = max(-1 + half_size, min(1 - half_size, center_y))
        x1 = center_x - half_size
        y1 = center_y - half_size
        x2 = center_x + half_size
        y2 = center_y + half_size
        return [x1, y1, x2, y2]
    
    def send_bbox_updates(self):
        """Continuously send bounding box updates to all clients"""
        start_time = time.time()
        while self.running:
            try:
                current_time = time.time()
                elapsed_time = current_time - start_time
                
                # Check if we're in the NaN sending period (1 second every 10 seconds)
                if (elapsed_time % 10) < 1:
                    # Send NaN values using struct.pack to ensure proper IEEE 754 format
                    bbox = [float('nan')] * 4
                else:
                    # Generate random bounding box
                    bbox = self.generate_random_bbox()
                
                # Create packet
                packet = bytearray(65)  # 65 bytes total
                packet[0] = 13  # BboxPos type (updated to match enum)
                
                # Pack the coordinates as doubles using struct.pack
                for i, value in enumerate(bbox):
                    struct.pack_into('<d', packet, 1 + i*8, value)
                
                # Send to all clients
                for client_id, client_socket in list(self.clients.items()):
                    try:
                        client_socket.send(packet)
                    except:
                        # Remove disconnected clients
                        del self.clients[client_id]
                
                # Wait before next update
                time.sleep(0.1)  # 10 Hz update rate
                
            except Exception as e:
                print(f"Error in bbox update thread: {e}")
                time.sleep(1)  # Wait before retrying

if __name__ == "__main__":
    server = TCPServer()
    try:
        server.start()
    except KeyboardInterrupt:
        print("\nShutting down server...")
        server.stop() 