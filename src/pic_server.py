import socket
import numpy as np
import cv2
import os

def prepare_image_packets(image_path):
    """
    Prepare image for transmission in 4096-byte packets
    
    Args:
        image_path (str): Path to the image file
    
    Returns:
        list: Packets for transmission (each 4096 bytes)
    """
    # Read image
    image = cv2.imread(image_path)
    
    if image is None:
        raise FileNotFoundError(f"Could not read image at {image_path}")
    
    # Serialize image
    encoded_image = cv2.imencode('.png', image)[1]
    image_bytes = encoded_image.tobytes()
    
    # Prepare packets
    packets = []
    packet_data_size = 4095  # 1 byte for flag, 4095 for data
    
    for i in range(0, len(image_bytes), packet_data_size):
        # Determine if this is the last packet
        chunk = image_bytes[i:i+packet_data_size]
        is_last = b'10101010' if i + packet_data_size >= len(image_bytes) else b'0'
        
        # Prepare full 4096-byte packet
        packet = np.zeros(4096, dtype=np.uint8)
        packet[0] = is_last  # First byte is the flag
        packet[1:1+len(chunk)] = np.frombuffer(chunk, dtype=np.uint8)
        
        packets.append(packet)
    
    return packets

def start_server(host='127.0.0.1', port=65432, image_path=None):
    """
    Start socket server to transfer image
    """
    # Validate image path
    if not image_path:
        raise ValueError("Image path must be specified")
    
    if not os.path.exists(image_path):
        raise FileNotFoundError(f"Image file not found: {image_path}")
    
    # Prepare image packets
    packets = prepare_image_packets(image_path)
    
    # Create socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    
    print(f"Server listening on {host}:{port}")
    print(f"Serving image: {image_path}")
    print(f"Total packets: {len(packets)}")
    
    while True:
        # Wait for client connection
        conn, addr = server_socket.accept()
        print(f"Connected by {addr}")
        
        try:
            # Send packets
            for packet in packets:
                conn.send(packet.tobytes())
            
            print("Image transfer complete")
        
        except Exception as e:
            print(f"Error during transfer: {e}")
        
        finally:
            conn.close()

def main():
    # Define image path and server configuration here
    image_path = 'tmp/rp_conexiones.png'
    host = '127.0.0.1'
    port = 65432
    
    # Start server with configuration
    start_server(host=host, port=port, image_path=image_path)

if __name__ == "__main__":
    main()