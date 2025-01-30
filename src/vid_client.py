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

        logging.basicConfig(filename='logs/client_log.log', level=logging.INFO, filemode="w")
        self.logger = logging.getLogger('VideoClient')

        self.rp = RedPitayaRx(bitstream=bitstream, host=host, port=port)

    def receive_frame_packets(self):
        """Receive and process incoming packets"""
        receiving_frame = False
        while True:
            time_start = time.time_ns()
            # Read registers
            [data, reg0, reg1, reg2, reg3, reg_count, data_size, h_ready, h_error] = self.rp.read_vlc_rx()

            if (reg0 != 0 and not h_error):
                packets_in_frame = (((reg2 >> 24) & 0b111) << 3) | ((reg2 >> 16) & 0b111)
                packet_number = (reg3 >> 24) & 0b111111
                fps = (((reg2 >> 8) & 0b111) << 2) | (reg2 & 0b11)
                self.time_per_frame = 1/fps

                # TODO Erase print
                print(f"Packet: {packet_number}/{packets_in_frame-1}")

                if (packet_number == 0 and not receiving_frame):
                    receiving_frame = True
                    current_frame_data = data
                elif (receiving_frame):
                    # TODO: missing frame handling
                    current_frame_data = np.hstack([current_frame_data, data])

                    if (packet_number == packets_in_frame - 1):
                        frame = cv2.imdecode(current_frame_data, cv2.IMREAD_COLOR)
                        current_frame_data = None
                        receiving_frame = False
                        if (frame is not None):
                            self.frame_buffer.put(frame)
                        else:
                            print("INVALID IMAGE")
                            self.logger.error("INVALID IMAGE")
                            self.rp.reset()

                time_end = time.time_ns()
                print(f"Time elapsed: {(time_end - time_start)*1e-6} [ms])")

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
        print("Transmissions received")

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
                print("Error showing frame")
                self.logger.error(f"Error in display loop: {e}")
                cv2.waitKey(10)
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

        self.receive_frame_packets()

        display_thread.join()

        print("\nPlayback complete!")

if __name__ == "__main__":
    HOST = "rp-f09168.local"
    PORT = 1001
    BITSTREAM = "bitstreams/vlc_rx.bit"
    INIT_BUFFER_SIZE = 10

    client = VideoClient(host=HOST, port=PORT, bitstream=BITSTREAM, initial_buffer_size=INIT_BUFFER_SIZE)
    client.run()