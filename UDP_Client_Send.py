import socket
import time

i=0

UDP_IP = "192.168.1.10"
UDP_PORT = 5001
MESSAGE = b"Hello, World!\n"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP
    
while True:    
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    print("Intento: " + str(i))
    i+=1
    time.sleep(6)
    