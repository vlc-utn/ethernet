from red_pitaya import RedPitayaRx
from time import sleep
import numpy as np
from binary import *
import matplotlib.pyplot as plt

rp = RedPitayaRx(bitstream="bitstreams/vlc_rx.bit")
#data_rx = rp.test_rx(plot=True)
rp.test_rx_speed()

## Reading data
#[regs, data] = rp.read_vlc_rx(wait_for_ms=10000)