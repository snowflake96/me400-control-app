import socket
import struct
import time
import threading
from enum import IntEnum
from typing import Union, Tuple, Dict
from datetime import datetime

class PacketType(IntEnum):
    Command = 0
    TunePitch = 1
    TuneYaw = 2
    Start = 3
    Stop = 4
    SetAutonomous = 5
    SetManual = 6
    Status = 7  # New packet type for status updates

class DataPacket:
    def __init__(self, data: bytes):
        if len(data) != 65:
            raise ValueError(f"Invalid packet size: {len(data)} bytes (expected 65)")
        
        self.type = PacketType(data[0])
        self.data = data[1:]
    
    def get_text(self) -> str:
        return self.data.decode('utf-8').rstrip('\x00')
    
    def get_integer(self) -> int:
        return struct.unpack('<i', self.data[:4])[0]
    
    def get_float(self) -> float:
        return struct.unpack('<f', self.data[:4])[0]
    
    def get_vector3(self) -> Tuple[float, float, float]:
        x, y, z = struct.unpack('<ddd', self.data[:24])
        return x, y, z

def create_response_packet(packet_type: PacketType, data: Union[str, int, float, Tuple[float, float, float]]) -> bytes:
    packet = bytearray([packet_type.value])
    
    if isinstance(data, str):
        # Pad or truncate string to 64 bytes
        text_bytes = data.encode('utf-8')
        if len(text_bytes) > 64:
            text_bytes = text_bytes[:64]
        else:
            text_bytes = text_bytes.ljust(64, b'\x00')
        packet.extend(text_bytes)
    
    elif isinstance(data, int):
        packet.extend(struct.pack('<i', data))
        packet.extend(b'\x00' * 60)  # Pad remaining bytes
    
    elif isinstance(data, float):
        packet.extend(struct.pack('<f', data))
        packet.extend(b'\x00' * 60)  # Pad remaining bytes
    
    elif isinstance(data, tuple) and len(data) == 3:
        packet.extend(struct.pack('<ddd', *data))
        packet.extend(b'\x00' * 40)  # Pad remaining bytes
    
    return bytes(packet)

class ClientHandler:
    def __init__(self, client_socket: socket.socket, address: Tuple[str, int]):
        self.client_socket = client_socket
        self.address = address
        self.is_running = True
        self.last_activity = time.time()
        self.status_thread = threading.Thread(target=self.send_status_updates)
        self.status_thread.daemon = True
        self.status_thread.start()
    
    def send_status_updates(self):
        while self.is_running:
            try:
                # Create status message with timestamp
                status_msg = f"Server Status: {datetime.now().strftime('%H:%M:%S')}"
                response = create_response_packet(PacketType.Status, status_msg)
                self.client_socket.sendall(response)
                time.sleep(1)  # Send status every second
            except:
                self.is_running = False
                break
    
    def handle(self):
        print(f"Connected to {self.address}")
        
        try:
            while self.is_running:
                # Receive exactly 65 bytes
                data = self.client_socket.recv(65)
                if not data:
                    break
                
                self.last_activity = time.time()
                
                try:
                    packet = DataPacket(data)
                    print(f"\nReceived packet type: {packet.type.name}")
                    
                    # Process the packet based on its type
                    if packet.type == PacketType.Command:
                        text = packet.get_text()
                        print(f"Command: {text}")
                        if text == "heartbeat":
                            # Echo back the heartbeat
                            response = create_response_packet(PacketType.Command, "heartbeat_ack")
                        else:
                            # Echo back the command
                            response = create_response_packet(PacketType.Command, f"Echo: {text}")
                    
                    elif packet.type == PacketType.TunePitch:
                        value = packet.get_float()
                        print(f"Tune Pitch: {value}")
                        response = create_response_packet(PacketType.TunePitch, value)
                    
                    elif packet.type == PacketType.TuneYaw:
                        value = packet.get_float()
                        print(f"Tune Yaw: {value}")
                        response = create_response_packet(PacketType.TuneYaw, value)
                    
                    elif packet.type == PacketType.Start:
                        print("Start command received")
                        response = create_response_packet(PacketType.Command, "Started")
                    
                    elif packet.type == PacketType.Stop:
                        print("Stop command received")
                        response = create_response_packet(PacketType.Command, "Stopped")
                    
                    elif packet.type == PacketType.SetAutonomous:
                        print("Set to Autonomous mode")
                        response = create_response_packet(PacketType.Command, "Autonomous mode activated")
                    
                    elif packet.type == PacketType.SetManual:
                        print("Set to Manual mode")
                        response = create_response_packet(PacketType.Command, "Manual mode activated")
                    
                    # Send the response
                    self.client_socket.sendall(response)
                    
                except ValueError as e:
                    print(f"Error processing packet: {e}")
                    continue
                
        except ConnectionResetError:
            print(f"Connection reset by {self.address}")
        except Exception as e:
            print(f"Error handling client {self.address}: {e}")
        finally:
            self.is_running = False
            print(f"Disconnected from {self.address}")
            self.client_socket.close()

def start_server(host: str = '0.0.0.0', port: int = 12345):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server_socket.bind((host, port))
        server_socket.listen(1)
        print(f"Server listening on {host}:{port}")
        
        while True:
            client_socket, address = server_socket.accept()
            client_handler = ClientHandler(client_socket, address)
            client_handler.handle()
            
    except KeyboardInterrupt:
        print("\nServer shutting down...")
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        server_socket.close()

if __name__ == "__main__":
    start_server() 