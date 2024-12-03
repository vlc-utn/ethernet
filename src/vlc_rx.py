from red_pitaya import RedPitayaRx
from time import sleep
import numpy as np
from binary import *
import matplotlib.pyplot as plt

rp = RedPitayaRx(bitstream="bitstreams/vlc_rx.bit")
data_rx = rp.test_rx()

data_tx = read_binary_file("mem_files/data_out_rx.mem", dtype=np.int16, signed=True)

## Plotting
plt.subplot(3,1,1)
plt.plot(data_tx)

plt.subplot(3,1,2)
plt.plot(data_rx)

plt.subplot(3,1,3)
plt.plot(abs(data_tx - data_rx))

plt.show()
