from pyhubio import PyhubTCP
import numpy as np
from binary import *
from time import sleep
from constants import *

class RedPitayaGeneric(PyhubTCP):
    def __init__(self, bitstream, host="10.42.0.172", port=1001):
        """Constructor

        Creates the socket, loads the "bitstream" in the FPGA memory and
        programs it, writes default values to control registers.
        """
        PyhubTCP.__init__(self, host, port)

        print("Trying to connect to Red Pitaya in address %s ...", host)
        self.start()

        print("Programming bitsteam file: %s ...", bitstream)
        self.program(bitstream)

        # Control registers
        self.led_status = np.zeros(1, np.uint8)
        self.tx_control = np.zeros(1, np.uint8)
        self.rx_control = np.zeros(1, np.uint8)
        self.general_control = np.zeros(1, np.uint8)
        self.general_control[0] = set_bit(self.general_control[0], 0) # Reset is nrst

        # Reset state is last, the order here is important
        self.write(self.led_status, port=Ports.CONFIG, addr=CfgAddr.LEDS)
        self.write(self.tx_control, port=Ports.CONFIG, addr=CfgAddr.TX_CONTROL)
        self.write(self.rx_control, port=Ports.CONFIG, addr=CfgAddr.RX_CONTROL)
        self.write(self.general_control, port=Ports.CONFIG, addr=CfgAddr.GENERAL_CONTROL)

        # Turn on Power On LED and reset
        self.write_led(Leds.POWER_ON, True)
        self.reset()
        print("Connection established!")

    def write_led(self, led: Leds, state: bool):
        """Turn on/off leds in Red Pitaya boards"""
        if(state):
            self.led_status[0] = set_bit(self.led_status[0], led)
        else:
            self.led_status[0] = clear_bit(self.led_status[0], led)

        self.write(self.led_status, port=Ports.CONFIG, addr=CfgAddr.LEDS)

    def write_registers(self, regs: np.ndarray):
        """Writes registers to config file"""
        self.write(regs, port=Ports.CONFIG, addr=CfgAddr.REG0_CFG)

    def read_status(self, address, dtype=np.uint8):
        """Read "dtype" bits of the status register and returns them.
            "dtype" can be "np.uint8, np.uint16 or np.uint32"
        """
        data = np.zeros(1, dtype)
        self.read(data, port=Ports.STATUS, addr=address)
        return data[0]

    def read_registers(self) -> np.ndarray:
        """Read registers"""
        regs = np.zeros(4, np.uint32)
        self.read(regs, port=Ports.STATUS, addr=StsAddr.REG0_STS)
        return regs

    def read_fifo_count(self) -> np.uint32:
        """Returns amount of values stored in the FIFO"""
        return self.read_status(StsAddr.FIFO_SIZE, dtype=np.uint32)

    def reset(self):
        """Resets IP cores"""
        # Turn of an on the power ON LED
        self.write_led(Leds.POWER_ON, False)
        self.general_control[0] = clear_bit(self.general_control[0], 0)
        self.write(self.general_control, port=Ports.CONFIG, addr=CfgAddr.GENERAL_CONTROL)
        sleep(0.01)
        self.general_control[0] = set_bit(self.general_control[0], 0)
        self.write(self.general_control, port=Ports.CONFIG, addr=CfgAddr.GENERAL_CONTROL)
        self.write_led(Leds.POWER_ON, True)

    def __del__(self):
        """Destructor

        Closes socket
        """
        self.stop()
        print("Connection closed to Red Pitaya!")


class RedPitayaTx(RedPitayaGeneric):
    def __init__(self, bitstream, host="10.42.0.172", port=1001):
        RedPitayaGeneric.__init__(self, bitstream, host, port)

    def write_vlc_tx(self, data: np.ndarray, regs: np.ndarray):
        """Writes a frame to the VLC_TX"""
        self.write_registers(regs)
        if(self.wait_for_msg_ready() == False):
            print("Error: msg_ready was not asserted")
            return

        self.write(data, port=Ports.VLC_TX, addr=0)
        self.start_tx()

    def enable_tx_fifo(self, state: bool):
        """Enable or disable Tx FIFO

        The Tx FIFO is used for debugging purposes. It stores all the valid
        outputs from the VLC_TX Ip Cores, and waits for them to be read from 
        the socket.
        """
        if (state == True):
            self.tx_control[0] = set_bit(self.tx_control[0], 1)
        else:
            self.tx_control[0] = clear_bit(self.tx_control[0], 1)

        self.write(self.tx_control, port=Ports.CONFIG, addr=CfgAddr.TX_CONTROL)

    def start_tx(self):
        """Signal that a new Tx message is ready to be sent

        Call this function after having loaded the data to be transmitted,
        so that the transmission can start.
        The new frame signal is like a pulse.
        """
        self.tx_control[0] = set_bit(self.tx_control[0], 0)
        self.write(self.tx_control, port=Ports.CONFIG, addr=CfgAddr.TX_CONTROL)
        sleep(0.01)
        self.tx_control[0] = clear_bit(self.tx_control[0], 0)
        self.write(self.tx_control, port=Ports.CONFIG, addr=CfgAddr.TX_CONTROL)

    def read_tx_fifo(self) -> np.ndarray:
        """Read TX FIFO

        This functions reads the data from the TX FIFO. Make sure to have
        enabled the FIFO, otherwise nothing will be read.
        """

        # Read the size of the FIFO, and that amount of words from it
        size = self.read_fifo_count()
        buffer = np.zeros(size, np.int32)
        self.read(buffer, port=Ports.VLC_TX, addr=0)

        # Data is in the form of 0x0000_xxxx, reinterpret as int16.
        buffer = buffer.astype(np.int16)

        return buffer

    def wait_for_msg_ready(self) -> bool:
        """Waits for the Tx to be ready to receive a new msg

        Waits up to 5 secs, returns "True" if a new msg can be sent.
        """
        i = 0
        while( not (self.read_status(address=StsAddr.TX_STS) & 0x1) or i == 500):
            sleep(0.01)
            i = i + 1

        return i!=500

    def test_tx(self):
        """Test the correct functionality of the Tx, before DAC"""

        print("Starting test of VLC_TX...")
        self.reset()

        # These values are the ones used for the test
        regs_tx = np.zeros(4, np.uint32)
        regs_tx[0] = 147
        regs_tx[1] = 17
        regs_tx[2] = 65792
        regs_tx[3] = 66063

        self.write_registers(regs_tx)
        regs_rx = self.read_registers()

        if(not np.array_equal(regs_tx, regs_rx)):
            print("Error while writing and reading registers")
            print("Tx registers:")
            print(regs_tx)
            print("Rx registers")
            print(regs_rx)
            return

        print("Registers correctly written!")

        data_tx = read_binary_file("mem_files/data_in_tx.mem", dtype=np.int16, signed=True)

        self.enable_tx_fifo(True)
        for _ in range(1):
            self.write_vlc_tx(data_tx, regs_tx)
            assert(self.wait_for_msg_ready())
            data_rx = self.read_tx_fifo()
            assert(np.array_equal(data_tx, data_rx))

        self.enable_tx_fifo(False)
        print("Test successful!")

        return data_rx

    def __del__(self):
        RedPitayaGeneric.__del__(self)


class RedPitayaRx(RedPitayaGeneric):
    def __init__(self, bitstream, host="10.42.0.172", port=1001):
        RedPitayaGeneric.__init__(self, bitstream, host, port)
        self.use_adc(True)

    def use_adc(self, input: bool):
        """Select Input "True" for ADC, "False" for master AXI.
        """
        if(input == False):
            self.rx_control[0] = set_bit(self.rx_control[0], 0)
        else:
            self.rx_control[0] = clear_bit(self.rx_control[0], 0)

        self.write(self.rx_control, port=Ports.CONFIG, addr=CfgAddr.RX_CONTROL)

    def read_vlc_rx(self) -> list:
        # Wait until the registers are safe to read (header_ready y no header_error)
        for i in range(10000):
            (header_ready, header_error) = self.read_header_status()
            if (header_error):
                print("Header was received with error. Resetting and exiting...")
                self.reset()
                return

            if (header_ready):
                break
            else:
                sleep(0.01)

        if (i==499):
            print("Timeout reached while waiting for read")
            return

        # Read registers, knowing that they are valid now
        regs = self.read_registers()
        self.header_ack()

        data = self.read_rx_fifo(regs[0] - regs[1])

        return [regs, data]


    def test_rx(self) -> np.ndarray:
        """Test Functionality of the RX
        """
        self.reset()

        print("Testing that the block doesn't trigger when idle...")
        (header_ready, header_error) = self.read_header_status()
        assert(header_ready == False and header_error == False)
        sleep(10)
        (header_ready, header_error) = self.read_header_status()
        assert(header_ready == False and header_error == False)
        print("Idle didn't trigger")

        # Write RF data
        for _ in range(5):
            data_rf = read_binary_file("mem_files/data_in_rx.mem", dtype=np.int16, signed=True)
            self.write(data_rf, port=Ports.VLC_RX, addr=0)
            sleep(1)
            self.use_adc(False)
            # Expected values
            expected_regs = np.zeros(4, np.uint32)
            expected_regs[0] = 105
            expected_regs[1] = 3
            expected_regs[2] = 65792
            expected_regs[3] = 66063
            expected_out = read_binary_file("mem_files/data_out_rx.mem", dtype=np.uint8, signed=False)

            print("Checking that the header was received without errors")
            sleep(1)
            (header_ready, header_error) = self.read_header_status()
            assert(header_error == False)
            assert(header_ready == True)

            print("Testing register values...")
            regs = self.read_registers()
            assert(np.array_equal(expected_regs, regs))
            print("Registers match!")

            print("Reading info...")
            [regs, data_rx] = self.read_vlc_rx()
            assert(np.array_equal(expected_out, data_rx))
            print("Data matches!")

            self.use_adc(True)
        print("Test Successful!")
        return data_rx

    def read_rx_fifo(self, size: np.uint32) -> np.ndarray:
        """Read RX values, returns uint8"""
        for i in range(500):
            if (self.read_fifo_count() == size):
                break

        if (i == 499):
            print("Timeout reached while waiting for the Fifo Rx")
            return

        data = np.zeros(size, np.uint32)
        self.read(data, port=Ports.VLC_RX, addr=0)
        data = data.astype(np.uint8)

        return data

    def read_header_status(self) -> tuple[bool, bool]:
        """Read header_ready and header_error

        If header_error == '1', the header was received with errors
        If header_ready == '1', then the registers have a valid value and can be read
        """
        data = np.zeros(1, np.uint8)
        data[0] = self.read_status(StsAddr.RX_STS)
        (header_ready, header_error) = (data[0] & 0x01, data[0] &0x02)
        return (header_ready, header_error)

    def header_ack(self):
        """Send a header ack after reading the registers of the current symbol"""
        self.rx_control[0] = set_bit(self.rx_control[0], 1)
        self.write(self.rx_control, port=Ports.CONFIG, addr=CfgAddr.RX_CONTROL)
        sleep(0.01)
        self.rx_control[0] = clear_bit(self.rx_control[0], 1)
        self.write(self.rx_control, port=Ports.CONFIG, addr=CfgAddr.RX_CONTROL)

    def __del__(self):
        RedPitayaGeneric.__del__(self)