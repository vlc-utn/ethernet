from enum import IntEnum

# Red Pitaya Leds
class Leds(IntEnum):
    POWER_ON = 0    # Power On LED
    TX_PERIODIC = 1
    VLC_TX = 2
    VLC_RX = 3
    HEADER_ERROR = 4
    HEADER_READY = 5
    LED6 = 6
    LED7 = 7

class CfgAddr(IntEnum):
    LEDS = 0                # Leds 0-7 of the board
    GENERAL_CONTROL = 1     # General control
    TX_CONTROL = 2          # VLC_TX control
    RX_CONTROL = 3          # VLC_RX control
    REG0_CFG = 4            # Reg0
    REG1_CFG = 8            # Reg1
    REG2_CFG = 12           # Reg2
    REG3_CFG = 16           # Reg3
    TX_PERIODIC_SIZE = 20   # Periodic Waveform Size

class StsAddr(IntEnum):
    TX_STS = 0              # Tx Status
    RX_STS = 1              # Rx Status
    RX_FIFO_REGS = 2        # Amount of registers that are waiting to be read
    FIFO_SIZE = 4           # Size of debug FIFOs
    REG0_STS = 8
    REG1_STS = 12
    REG2_STS = 16
    REG3_STS = 20

# IX allows to read s0x_axis, and write to b0x_bram and m0x_axis
class Ports(IntEnum):
    CONFIG = 0          #cfg_data
    STATUS = 1          #sts_data
    VLC_TX = 2          #I0 (s00_axis, b00_bram, m00_axis)
    VLC_RX = 3          #I1 (s01_axis, b01_bram, m01_axis)
    PERIODIC_TX = 4     #I2 (s02_axis, b02_bram, m02_axis)
    PORT_I3 = 5         #I3 (s03_axis, b03_bram, m03_axis)
    PORT_I4 = 6         #I4 (s04_axis, b04_bram, m04_axis)
    PORT_I5 = 7         #I5 (s05_axis, b05_bram, m05_axis)

TX_PERIODIC_WAVEFORM_SIZE = 4096