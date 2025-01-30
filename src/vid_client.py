import socket
import cv2
import numpy as np
import time
from threading import Thread
from queue import Queue, Empty
import logging
from red_pitaya import RedPitayaRx

class VideoClient:
    def __init__(self, host="rp-f09035.local", port=1001, bitstream="bitstreams/vlc_rx.bit", initial_buffer_size=100):

        self.frame_buffer = Queue(maxsize=500)
        self.initial_buffer_size = initial_buffer_size

        self.rp = RedPitayaRx(bitstream=bitstream, host=host, port=port)
        self.reg_count = 0

        # Used for logging
        self.frame_count = 0
        self.total_bytes_received = 0
        logging.basicConfig(filename='logs/client_log.log', level=logging.INFO, filemode="w")
        self.logger = logging.getLogger('VideoClient')

    def receive_frame(self) -> bool:
        """Receive and process incoming packets"""
        current_frame_data = None
 
        while True:
            # Read registers
            [data, reg0, reg1, reg2, reg3, self.reg_count, data_size, h_ready, h_error] = self.rp.read_vlc_rx(self.reg_count)

            # Check for errors
            if (h_error):
                self.logger.error(f"Header error on frame: {self.frame_count}")
                self.rp.reset()
                return False
            elif (data is None):
                self.logger.error(f"Data was None: {self.frame_count}")
                self.rp.reset()
                return False

            # Unpack registers
            packet_number = (reg3 >> 24) & 0b111111
            fps = (((reg2 >> 8) & 0b111) << 2) | (reg2 & 0b11)
            packets_in_frame = (((reg2 >> 24) & 0b111) << 3) | ((reg2 >> 16) & 0b111)
            last_packet = (fps == 0)
            
            # If its the first frame, load fps and packets in frame
            if (packet_number == 0 and current_frame_data is None and not last_packet):
                current_frame_data = data
                self.fps = fps
                self.packets_in_frame = packets_in_frame
                self.time_per_frame = 1/fps

            elif (current_frame_data is not None):
                if not last_packet:
                    current_frame_data = np.hstack([current_frame_data, data])

                else:
                    # Para que todos los paquetes tengan el mismo tamaño,
                    # El último paquete de un frame va a transmitir el tamaño del paquete completo,
                    # pero los 5 bits de FPS van a valer todos cero (para identificarlo)
                    # y los 6 bits de "frame_number" y "packet_in_frame" son 12 bits del tamaño real del paquete
                    last_packet_size = (packet_number << 6) | packets_in_frame
                    data = data[0:last_packet_size]

                    current_frame_data = np.hstack([current_frame_data, data])

                    frame = cv2.imdecode(current_frame_data, cv2.IMREAD_COLOR)
                    self.total_bytes_received += len(current_frame_data)
                    current_frame_data = None
                    self.frame_count += 1
                    
                    if (frame is not None):
                        self.frame_buffer.put(frame)
                    else:
                        self.logger.error(f"Invalid image on frame: {self.frame_count}")
                        self.rp.reset()
                        return False
                    
                    break
        return True

    def print_metrics(self):
        """Print reception metrics"""
        if (self.frame_count == 1):
            self.start_time = time.time()
        
        if (self.frame_count != 0):
            rx_time = time.time() - self.start_time
            bitrate = (self.total_bytes_received * 8) / rx_time
        else:
            bitrate = 0
            rx_time = 0
        
        print(f"\rBytes received: {self.total_bytes_received*1e-6:.2f} MB, ",
              f"Bitrate: {bitrate*1e-6:.2f} Mbps, "
              f"Frames received: {self.frame_count}, ",
              f"Rx time: {rx_time*1000:.1f} msec, ",
              f"Queue size: {self.frame_buffer.qsize()}", end="")

    def display_frames(self):
        """Optimized display loop with strict timing"""
        self.logger.info("Waiting for initial buffer fill...")

        cv2.namedWindow('Video client', cv2.WINDOW_NORMAL)
        cv2.moveWindow('Video client', 1100, 150)
        cv2.resizeWindow('Video client', 750, 750)

        # Wait for a little of the buffer to fill before starting to display
        print("Waiting for transmissions...")
        while (self.frame_buffer.qsize() < self.initial_buffer_size):
            time.sleep(1e-3)
        self.logger.info("Transmissions received")

        while True:
            try:
                frame = self.frame_buffer.get(timeout=10)
            except Empty:
                self.logger.warning("Empty frame_buffer queue. No more frames to display.")
                break

            previous_frame_time = time.time()
            try:
                cv2.imshow("Video client", frame)
            except Exception as e:
                self.logger.error(f"Error in display loop: {e}")
                continue

            if (self.time_per_frame > time.time() - previous_frame_time):
                time.sleep(abs(self.time_per_frame - (time.time() - previous_frame_time)))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

    def run(self):
        """Main client loop"""
        display_thread = Thread(target=self.display_frames)
        display_thread.start()
        
        # Exit the loop after failing "x" consecutive times to parse a frame,
        # after having received a frist successful frame
        tries = 0
        while (tries < 10):
            if (self.receive_frame() or self.frame_count == 0):
                tries = 0
            else:
                tries += 1

            self.print_metrics()

        print("\nWaiting to join display_thread...")
        self.logger.info("Waiting to join display_thread...")
        display_thread.join()

        print("\nPlayback complete!")

if __name__ == "__main__":
    HOST = "rp-f09168.local"
    PORT = 1001
    BITSTREAM = "bitstreams/vlc_rx.bit"
    INIT_BUFFER_SIZE = 10

    client = VideoClient(host=HOST, port=PORT, bitstream=BITSTREAM, initial_buffer_size=INIT_BUFFER_SIZE)
    client.run()