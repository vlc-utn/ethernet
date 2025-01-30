# Test transmission through cable
# Use OUT2

from red_pitaya import RedPitayaTx
from time import sleep
import numpy as np
from binary import *
import matplotlib.pyplot as plt

rp = RedPitayaTx(bitstream="bitstreams/vlc_tx.bit", host="rp-f09035.local")

# Writing a VLC frame
data_in = read_binary_file("mem_files/data_in_tx.mem")
regs = np.zeros(4, np.uint32)
regs[0] = 147
regs[1] = 17
regs[2] = 65792
regs[3] = 66063
rp.write_vlc_tx(data_in, regs)
print(data_in)
