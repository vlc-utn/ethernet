from red_pitaya import RedPitayaRx
from time import sleep
import numpy as np
from binary import *
import matplotlib.pyplot as plt

rp = RedPitayaRx(bitstream="bitstreams/vlc_rx.bit")
data_rx = rp.test_rx()
expected_out = read_binary_file("mem_files/data_out_rx.mem", dtype=np.uint8, signed=False)

## Plotting
plt.subplot(3,1,1)
plt.plot(expected_out)

plt.subplot(3,1,2)
plt.plot(data_rx)

plt.subplot(3,1,3)
plt.plot(abs(expected_out - data_rx))

plt.show()
