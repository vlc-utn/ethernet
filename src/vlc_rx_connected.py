# Test reception through cable
# Use IN2 port

from red_pitaya import RedPitayaRx
from time import sleep
import numpy as np
from binary import *
import matplotlib.pyplot as plt


rp = RedPitayaRx(bitstream="bitstreams/vlc_rx.bit", host="rp-f09168.local")

print("Waiting for a transmission to start")

reg_count = 0
h_error = False
while (not reg_count and not h_error):
    [reg0, reg1, reg2, reg3, reg_count, data_size, h_ready, h_error] = rp.read_status()
    sleep(10e-3)

if (h_error):
    print("Header Error!!")
    exit()

[data, reg0, reg1, reg2, reg3, reg_count, data_size, h_ready, h_error] = rp.read_vlc_rx()
print(reg0); print(reg1); print(reg2); print(reg3); print(data)

expected_reg0 = 147
expected_reg1 = 17
expected_reg2 = 65792
expected_reg3 = 66063
expected_data = read_binary_file("mem_files/data_in_tx.mem")[0:(reg0-reg1)]

assert(expected_reg0 == reg0)
assert(expected_reg1 == reg1)
assert(expected_reg2 == reg2)
assert(expected_reg3 == reg3)
assert(np.array_equal(data, expected_data))

print("Test successfull")
