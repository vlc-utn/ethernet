from pyhubio import PyhubTCP
import numpy as np
import time
import threading
import os

def write_buffer_to_file(buffer, filename):
    
    mode = 'x' if not os.path.exists(filename) else 'w'
    
    with open(filename, mode) as f:
        
        for value in buffer:
            # Convert to binary string, remove '0b' prefix, and pad to 16 bits
            decimal_str = str(value)
            value = value & 0xFFFF	
            binary_str = format(value, '016b')
            
            # Write binary value followed by ',\n'
            f.write(f"{binary_str}, {decimal_str}\n")

def DAC_write(size, io):

	count = 0
	buffer_DAC = np.zeros(size*2, np.int16)
	
	send = np.zeros(1, np.uint32)

	msg = 'This is an example message used to test the transmitter. It is made large on purpose to test for a large message being transmitted\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0'

	msg_num = np.array([ord(char) for char in msg], dtype=np.int16)
	aux = np.zeros(size-len(msg_num), np.int16)
	msg_num = np.append(msg_num, aux)

	buffer_DAC[0::2] = msg_num

	reg0 = np.zeros(1, np.uint32)
	reg0[0] = 147
	reg1 = np.zeros(1, np.uint32)
	reg1[0] = 17

	time.sleep(3)

	while count < 8:  

		print('Sender: Envío vez n°',count,'\n')

		io.write(reg0, port=0, addr=8)
		io.write(reg1, port=0, addr=12)

		io.write(buffer_DAC, port=2, addr=0)
		count+=1
		
		send[0] = 7
		io.write(send, port=0, addr=4)				#new frame in
		time.sleep(0.1)
		send[0] = 0
		io.write(send, port=0, addr=4)

		time.sleep(4.5)

	print('Sender: Master send finished\n')

def ADC_read(size, io):

	data = np.zeros(size * 2, np.int16)
	count = 0
	base_filename = "./TX_tests/ADC_IN/ADC_input_v6_"
	status = np.zeros(1, np.uint32)

	while count < 8:

		status [0] = 0
		print('Reciever: Espero a llenar buffer vez n°',count,'\n')

		while status[0] < 12300:
			io.read(status, port=1, addr=0)			#Wait fifo buffer full
			time.sleep(0.1)
			#print('Reciever: Valor de status =', status ,'\n')

		io.read(data, port=2, addr=0)
		
		filename = f"{base_filename}{count}.txt"

		ch = data[0::2]
		
		write_buffer_to_file(buffer=ch, filename=filename)

		count+=1	

	print('Reciever: Slave read finished\n')

def TX_read(size, io):

	data = np.zeros(size * 2, np.int16)
	buffer = np.zeros(size, np.uint16)
	count = 0
	base_filename = "./TX_tests/FIFO_IN/txt_input_v6_"
	status = np.zeros(1, np.uint32)

	while count < 8:

		status [0] = 0

		print('TX: Espero a llenar buffer vez n° ', count, '\n')

		while status[0] < 12300:
			io.read(status, port=1, addr=4)			#Wait fifo buffer full
			time.sleep(0.1)
			#print('TX: Valor de status =', status ,'\n')

		io.read(data, port=3, addr=0)
		
		buffer = data[0::2]
		filename = f"{base_filename}{count}.txt"

		write_buffer_to_file(filename=filename, buffer=buffer)

		count+=1	

	print('TX: Slave read finished\n')

def main():
	
	#try:
	#	requests.get("http://rp-f09168.local/playground")
	#except requests.RequestException as e:
	#	print(f"Error accessing webpage: {e}")

	io = PyhubTCP("rp-f09168.local")

	io.start()

	io.program("TX_broadcast_v6.bit")

	io.socket.settimeout(10000)

	print("Programado\n")

	size = 16384				# Mantener igual que el resto del diseño de vivado siempre

	thread1 = threading.Thread(target=DAC_write, args=(size, io))
	thread2 = threading.Thread(target=ADC_read, args=(size, io))
	thread3 = threading.Thread(target=TX_read, args=(size, io))

	send = np.zeros(1, np.uint32)

	send[0] = 7
	io.write(send, port=0, addr=0)				#Reset
	time.sleep(0.1)
	send[0] = 0
	io.write(send, port=0, addr=0)

	time.sleep(0.1)

	thread1.start()
	thread2.start()
	thread3.start()

	thread1.join()
	thread2.join()
	thread3.join()

	print("Main program finished")

	io.stop()

if __name__ == "__main__":
    main()
