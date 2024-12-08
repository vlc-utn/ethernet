from red_pitaya import RedPitayaTx
from time import sleep
import numpy as np
from binary import *
import matplotlib.pyplot as plt

rp = RedPitayaTx(bitstream="bitstreams/vlc_tx.bit")
rp.test_tx(waveform_file="waveforms/waveform_sin_1mhz.mem",
                     plot_periodic=True, plot_tx=True)

# Writing a periodic waveform
data_in = read_binary_file("waveforms/waveform_sin_1mhz.mem", np.int32, signed=True)
rp.write_periodic_waveform(data_in, plot=False)


# Writing a VLC frame
data_in = read_binary_file("mem_files/data_in_tx.mem")
regs = np.zeros(4, np.uint32)
regs[0] = 147
regs[1] = 17
regs[2] = 65792
regs[3] = 66063
rp.write_vlc_tx(data_in, regs)
