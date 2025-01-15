import socket
import numpy as np
import cv2

def receive_image(host='127.0.0.1', port=65432):
    """
    Receive image from server
    
    Args:
        host (str): Server host
        port (int): Server port
    
    Returns:
        numpy.ndarray: Received image
    """
    # Create socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))
    
    try:
        # Receive packets
        image_bytes = b''
        while True:
            # Receive exactly 4096 bytes
            packet = client_socket.recv(4096)
            
            if not packet:
                break
            
            # Convert to numpy array
            np_packet = np.frombuffer(packet, dtype=np.uint8)
            
            # Check if this is the last packet (first byte)
            is_last = np_packet[0]
            
            # Append data (skip first byte)
            image_bytes += np_packet[1:].tobytes()
            
            # Break if last packet
            if is_last == b'10101010':
                break
        
        # Convert bytes to image
        nparr = np.frombuffer(image_bytes, np.uint8)
        decoded_image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        return decoded_image
    
    except Exception as e:
        print(f"Error receiving image: {e}")
        return None
    
    finally:
        client_socket.close()

def main():
    # Define server configuration here
    host = '127.0.0.1'
    port = 65432
    
    # Optional: Define output path for saving
    output_path = 'tmp/rp_recieved.png'
    
    # Receive image
    received_image = receive_image(host=host, port=port)
    
    if received_image is not None:
        # Optional: Save image
        if output_path:
            cv2.imwrite(output_path, received_image)
            print(f"Image saved to {output_path}")
        
        # Display image
        cv2.imshow('Received Image', received_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()