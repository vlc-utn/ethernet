from pyhubio import PyhubTCP
import numpy as np
import pylab as pl
import time
import threading
import queue

def gauss(x, A, x0, sigma):
    return A * np.exp(-((x - x0) ** 2) / (2 * sigma**2))


def DAC_write(size, io):

	count = 0
	buffer_DAC = np.zeros(size, np.int32)
	pulse = gauss(np.arange(0, size, 1), 1, 5000, 500)
	pulse = np.int32(np.floor(pulse * 8191 + 0.5))

	pins = np.zeros(1, np.uint32)

	buffer_DAC = pulse

	while count < 5:        
		
		print('\nSalida de interfaz master m00:\n\n', buffer_DAC[0:size:512], '\n\n')
		io.write(buffer_DAC, port=2, addr=0)
		count+=1
		pins[0] = 2**count
		io.write(pins, port=0, addr=4)
		time.sleep(5)

	print('Sender: Master send finished\n')
	pins[0] = 0
	io.write(pins, port=0, addr=4)

def ADC_read(size, io, buffer):

	data = np.zeros(size, np.int32)	
	count = 0

	while count < 5:

		io.edge(0, 1, positive=True, addr=0) 		#Reset fifo buffer
		status = np.zeros(1, np.uint16)

		while status[0] < size:
			time.sleep(0.1)
			io.read(status, port=1, addr=0)			#Wait fifo buffer full

		io.read(data, port=2, addr=0)
		
		buffer.put(data)

		print('\nEntrada de interfaz Slave S00:\n\n', data[0:size:512], '\n\n')

		count+=1		

	print('Slave read finished')

def main():
	
	io = PyhubTCP("rp-f09168.local")

	io.start()

	io.program("template_loopback_1.bit")

	print("Programado")

	buffer1 = queue.Queue()
	buffer2 = queue.Queue()

	size = 16384
	count = 0

	thread1 = threading.Thread(target=DAC_write, args=(size, io))
	thread2 = threading.Thread(target=ADC_read, args=(size, io, buffer1))

	thread1.start()
	thread2.start()

	active_threads = 2
	while active_threads > 0:
		for buffer in [buffer1, buffer2]:
			try:
				data = buffer.get(timeout=1)
				if data is None:
					active_threads -= 1
				else:
					
					pl.figure(figsize=[8, 4], dpi=150, constrained_layout=True)
					pl.plot(data)

					pl.xlabel("sample number")
					pl.ylabel("ADC units")

					pl.ylim(-9000, 9000)
					pl.grid()
					pl.show()

					if count < 4:
						count +=1
					else: 
						active_threads = 0
			
			except queue.Empty:
				pass

	thread1.join()
	thread2.join()

	print("Main program finished")

	io.stop()

if __name__ == "__main__":
    main()
