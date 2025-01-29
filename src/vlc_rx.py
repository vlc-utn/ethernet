from red_pitaya import RedPitayaRx
from time import sleep
import numpy as np
from binary import *
import matplotlib.pyplot as plt

rp = RedPitayaRx(bitstream="bitstreams/vlc_rx.bit")
data_rx = rp.test_rx(plot=False)
rp.test_rx_speed()
