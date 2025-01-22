from red_pitaya import RedPitayaTx
from time import sleep
import numpy as np
from binary import *
import matplotlib.pyplot as plt

rp = RedPitayaTx(bitstream="bitstreams/vlc_tx.bit", host="rp-f09035.local")

### General functionality test
rp.test_tx( waveform_file="waveforms/waveform_sin_1mhz.mem",
            tx_input_file="mem_files/data_in_tx.mem",
            tx_output_file="mem_files/data_out_tx.mem",
            plot_periodic=True, plot_tx=True)

rp.test_tx_speed()


# # Writing a periodic waveform
#data_in = read_binary_file("waveforms/waveform_sin_10mhz_02.mem", np.int32, signed=True)

#data_in = read_binary_file("mem_files/data_out_reduced.mem", np.int32, signed=True)
#rp.write_periodic_waveform(data_in, plot=False)
#plt.plot(data_in)
#plt.show()

# # Writing a VLC frame
# data_in = read_binary_file("mem_files/data_in_tx.mem")
# regs = np.zeros(4, np.uint32)
# regs[0] = 147
# regs[1] = 17
# regs[2] = 65792
# regs[3] = 66063
# rp.write_vlc_tx(data_in, regs)
# print(data_in)
