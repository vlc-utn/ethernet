from pyhubio import PyhubTCP
import numpy as np
from binary import *
from time import sleep, time_ns
from constants import *
import matplotlib.pyplot as plt

class RedPitayaGeneric(PyhubTCP):
    def __init__(self, bitstream, host="10.42.0.172", port=1001):
        """Constructor

        Creates the socket, loads the "bitstream" in the FPGA memory and
        programs it, writes default values to control registers.
        """
        PyhubTCP.__init__(self, host, port)

        print(f"Trying to connect to Red Pitaya in address {host}:{port} ...")
        self.start()

        print(f"Programming bitsteam file: {bitstream} ...")
        self.program(bitstream)

        self.reset()
        print("Connection established!")

    def write_led(self, led: Leds, state: bool):
        """Turn on/off leds in Red Pitaya boards"""
        if(state):
            self.led_status[0] = set_bit(self.led_status[0], led)
        else:
            self.led_status[0] = clear_bit(self.led_status[0], led)

        self.write(self.led_status, port=Ports.CONFIG, addr=CfgAddr.LEDS)

    def write_registers(self, regs: np.ndarray, new_frame:bool=True):
        """Writes registers to config file, and toggles reg valid"""
        self.write(regs, port=Ports.CONFIG, addr=CfgAddr.REG0_CFG)

        # Toggle valid_reg bit, and set new_frame_in
        self.tx_control[0] = toggle_bit(self.tx_control[0], 4)
        if (new_frame):
            self.tx_control[0] = set_bit(self.tx_control[0], 0)

        self.write(self.tx_control, port=Ports.CONFIG, addr=CfgAddr.TX_CONTROL)

        if (new_frame):
            # Clear new_frame_in after writing the registers
            self.tx_control[0] = clear_bit(self.tx_control[0], 0)
            self.write(self.tx_control, port=Ports.CONFIG, addr=CfgAddr.TX_CONTROL)

    def read_status(self, address, dtype=np.uint8):
        """Read "dtype" bits of the status register and returns them.
            "dtype" can be "np.uint8, np.uint16 or np.uint32"
        """
        data = np.zeros(1, dtype)
        self.read(data, port=Ports.STATUS, addr=address)
        return data[0]

    def read_registers(self) -> np.ndarray:
        """Read registers"""
        # Signal to update registers from the FIFO (only applicable for RX)
        self.rx_control[0] = set_bit(self.rx_control[0], 1)
        self.write(self.rx_control, port=Ports.CONFIG, addr=CfgAddr.RX_CONTROL)
        self.rx_control[0] = clear_bit(self.rx_control[0], 1)
        self.write(self.rx_control, port=Ports.CONFIG, addr=CfgAddr.RX_CONTROL)

        # Read registers
        regs = np.zeros(4, np.uint32)
        self.read(regs, port=Ports.STATUS, addr=StsAddr.REG0_STS)
        return regs

    def read_register_count(self) -> np.uint8:
        reg_count = np.zeros(1, np.uint8)
        self.read(reg_count, port=Ports.STATUS, addr=StsAddr.TX_STS)
        reg_count[0] = (reg_count[0] & 0b01111110) >> 1
        return reg_count[0]

    def read_fifo_count(self) -> np.uint32:
        """Returns amount of values stored in the FIFO"""
        return self.read_status(StsAddr.FIFO_SIZE, dtype=np.uint32)

    def reset(self):
        """Resets IP cores"""

        # Set all control register to default values
        self.general_control = np.zeros(1, np.uint8)
        self.led_status = np.zeros(1, np.uint8)
        self.tx_control = np.zeros(1, np.uint8)
        self.rx_control = np.zeros(1, np.uint8)

        self.write(self.general_control, port=Ports.CONFIG, addr=CfgAddr.GENERAL_CONTROL)
        self.write(self.led_status, port=Ports.CONFIG, addr=CfgAddr.LEDS)
        self.write(self.tx_control, port=Ports.CONFIG, addr=CfgAddr.TX_CONTROL)
        self.write(self.rx_control, port=Ports.CONFIG, addr=CfgAddr.RX_CONTROL)

        # Wait a little, and then put the reset to high (nrst, reset is active low)
        sleep(0.01)
        self.general_control[0] = set_bit(self.general_control[0], 0)   # nRst
        self.general_control[0] = set_bit(self.general_control[0], 1)   # nRstRxFifo
        self.write(self.general_control, port=Ports.CONFIG, addr=CfgAddr.GENERAL_CONTROL)

        # Turn on Power On LED
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

    def write_vlc_tx(self, data: np.ndarray, regs: np.ndarray) -> None:
        """Writes a frame to the VLC_TX

        Word length of data written must be 32 bits.
        Return "True" if a new frame had to be sent
        """
        if (not (type(data[0]) == np.uint32 or type(data[0]) == np.int32)):
            raise Exception("Data written to the Red Pitaya must be of 32 bits")

        self.write(data, port=Ports.VLC_TX, addr=0)
        self.write_registers(regs)


    def use_ofdm(self, state: bool):
        """Enable or disable OFDM output

        If "enabled", the DAC is connected to the output of the OFDM. If 
        "disabled", the DAC is connected to the periodic signal.
        """
        if (state == False):
            self.tx_control[0] = set_bit(self.tx_control[0], 2)
        else:
            self.tx_control[0] = clear_bit(self.tx_control[0], 2)

        self.write(self.tx_control, port=Ports.CONFIG, addr=CfgAddr.TX_CONTROL)

    def use_two_times(self, state: bool):
        """Multiply the output of the VLC Tx by two

        If "enabled", the VLC TX output is shifted left by one bit.
        """
        if (state == True):
            self.tx_control[0] = set_bit(self.tx_control[0], 3)
        else:
            self.tx_control[0] = clear_bit(self.tx_control[0], 3)

        self.write(self.tx_control, port=Ports.CONFIG, addr=CfgAddr.TX_CONTROL)

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

    def wait_for_msg_ready(self, delay_ms:int = 5000) -> bool:
        """Waits for the Tx to be ready to receive a new msg

        Waits up to "delay_ms" milliseconds secs, returns "True" if a new msg can be sent.
        """
        i = 0
        while( not ((self.read_status(address=StsAddr.TX_STS) & 0x1) or (i == delay_ms))):
            sleep(0.001)
            i = i + 1

        return i != delay_ms

    def write_periodic_waveform(self, data: np.ndarray, plot:bool=False):
        """Writes a periodic waveform to the DAC

        Data written should be 32 bits long, but only the 16 LSB bits will be used.
        """

        if (len(data) > TX_PERIODIC_WAVEFORM_SIZE):
            raise Exception(f"Data of periodic waveform shouldn't be greater than {TX_PERIODIC_WAVEFORM_SIZE}")

        # Write length of the data to control register (must be len -1)
        length = np.zeros(1, np.uint16)
        length[0] = len(data) - 1
        self.write(length, port=Ports.CONFIG, addr=CfgAddr.TX_PERIODIC_SIZE)

        self.use_ofdm(False)
        self.write(data, port=Ports.PERIODIC_TX, addr=0)

        if (plot):
            # Enable Fifo, wait until enough values are processed, and then read
            self.enable_tx_fifo(True)
            sleep(1)
            self.enable_tx_fifo(False)
            data_fifo = self.read_tx_fifo()

            fig1 = plt.figure()
            plt.title("Periodic Waveform")

            plt.subplot(2,1,1)
            plt.plot(data)
            plt.xlabel("Samples")
            plt.ylabel("Reference signal")
            plt.grid(True)

            plt.subplot(2,1,2)
            plt.plot(data_fifo)
            plt.xlabel("Samples")
            plt.ylabel("Signal before DAC")
            plt.grid(True)

    def test_tx(self, waveform_file:str="waveforms/waveform_sin_1mhz.mem",
                tx_input_file:str="mem_files/data_in_tx.mem",
                tx_output_file:str="mem_files/data_out_tx.mem",
                plot_periodic:bool=False, plot_tx:bool=False):
        """Test the correct functionality of the Tx, before DAC"""

        self.reset()
        print("Starting test of VLC_TX...")

        print("Testing periodic waveform...")
        data_in = read_binary_file(waveform_file, np.int32, True)
        self.write_periodic_waveform(data_in, plot=plot_periodic)

        # Enable FIFO, store some samples, disable FIFO and read what's stored
        self.enable_tx_fifo(True)
        sleep(1)
        self.enable_tx_fifo(False)
        data_fifo = self.read_tx_fifo()

        # Test that the waveform matches periodically
        for i in range(0, len(data_in)):
            if(np.array_equal(data_in, data_fifo[i : len(data_in)+i : 1])):
                # Test that the data of the next period also matches
                if (np.array_equal(data_in, data_fifo[len(data_in)+i : 2*len(data_in) + i : 1])):
                    print("Periodic Waveform matches!")
                    break
        if (i == len(data_in) - 1):
            raise Exception("Periodic Waveform Failed!")

        # These register values are the ones used for the test
        self.reset()
        assert(self.read_register_count() == 0)     # No registers on reset

        regs_tx = np.zeros(4, np.uint32)
        regs_tx[0] = 147
        regs_tx[1] = 17
        regs_tx[2] = 65792
        regs_tx[3] = 66063

        self.write_registers(regs_tx, new_frame=False)
        regs_rx = self.read_registers()
        assert(np.array_equal(regs_tx, regs_rx))    # Registers correctly written
        assert(self.read_register_count() == 1)     # Register count updated

        self.write_registers(regs_tx, new_frame=False); assert(self.read_register_count() == 2)
        self.write_registers(regs_tx, new_frame=False); assert(self.read_register_count() == 3)

        self.reset()
        assert(self.read_register_count() == 0)
        print("Registers correctly written!")

        # The data sent to the IP Core must be an int32 or uint32
        print("Testing signal transmission...")
        data_in = read_binary_file(tx_input_file, dtype=np.uint32, signed=False)
        expected_out = read_binary_file(tx_output_file, dtype=np.int16, signed=True)

        # Send and read 5 times the same message
        self.enable_tx_fifo(True)
        for i in range(5):
            if (i > 2):
                # Test normal
                self.write_vlc_tx(data_in, regs_tx)
                assert(self.wait_for_msg_ready())
                data_out = self.read_tx_fifo()
                assert(np.array_equal(expected_out, data_out))
            else:
                # Test two times
                self.use_two_times(True)
                self.write_vlc_tx(data_in, regs_tx)
                assert(self.wait_for_msg_ready())
                data_out = self.read_tx_fifo()
                assert(np.array_equal(expected_out*2, data_out))
                self.use_two_times(False)

        self.enable_tx_fifo(False)
        if (plot_tx):
            fig2 = plt.figure()
            plt.title("VLC TX Waveform")
            plt.subplot(3,1,1)
            plt.plot(expected_out)
            plt.xlabel("Samples")
            plt.ylabel("Expected Output")
            plt.grid(True)

            plt.subplot(3,1,2)
            plt.plot(data_out)
            plt.xlabel("Samples")
            plt.ylabel("Output")
            plt.grid(True)

            plt.subplot(3,1,3)
            plt.plot(abs(expected_out - data_out))
            plt.xlabel("Samples")
            plt.ylabel("Error signal")
            plt.grid(True)

        print("Test successful!")
        plt.show()


    def test_tx_speed(self) -> None:
        """Test speed of writing the registers and data to the Red Pitaya"""
        self.reset()

        regs = np.zeros(4, np.uint32)
        regs[0] = 4011
        regs[1] = 0
        regs[2] = 65792
        regs[3] = 66063

        # Data sizes to test speed
        data_sizes = [105, 210, 504, 1008, 2016, 4011]

        print("Speed test for single transactions...")
        for size in data_sizes:
            data_in = np.random.randint(0, 255, size, dtype=np.uint32)
            regs[0] = size
            time_start = time_ns()
            self.write_vlc_tx(data_in, regs)
            time_end = time_ns()
            self.wait_for_msg_ready()
            print(f"Time elapsed with size {size}: {(time_end - time_start)*1e-6} [ms])")

        print("Speed test successful!")

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

    def read_vlc_rx(self, wait_for_ms=10000) -> list:
        """Read VLC RX

        Returns the registers and the data read, or an empty list in case of
        error
        """
        self.use_adc(True)

        if (not self.frames_waiting()):
            # Wait until the registers are safe to read (header_ready y no header_error)
            for i in range(wait_for_ms):
                if (self.read_header_error()):
                    print("Message was received with error")
                    self.reset()
                    return []
                elif (self.frames_waiting()):
                    break
                else:
                    sleep(0.001)

            if (i==wait_for_ms-1):
                print("Timeout reached while waiting for read")
                return []

        # Read registers, knowing that they are valid now
        regs = self.read_registers()
        data = self.read_rx_fifo(regs[0] - regs[1])
        return [regs, data]


    def test_rx(self, plot:bool=False) -> np.ndarray:
        """Test Functionality of the RX
        """
        self.reset()

        print("Testing that the block doesn't trigger when idle...")
        assert(self.read_header_error() == False)
        assert(not self.frames_waiting())
        sleep(2)
        assert(self.read_header_error() == False)
        assert(not self.frames_waiting())

        # Input
        data_rf = read_binary_file("mem_files/data_in_rx.mem", dtype=np.int32, signed=True)

        # Expected values
        expected_regs = np.zeros(4, np.uint32)
        expected_regs[0] = 105
        expected_regs[1] = 3
        expected_regs[2] = 65792
        expected_regs[3] = 66063
        expected_out = read_binary_file("mem_files/data_out_rx.mem", dtype=np.uint8, signed=False)

        for _ in range(5):
            print("Testing reception of known frame...")
            self.write(data_rf, port=Ports.VLC_RX, addr=0)

            # Sleep until the data is fully written
            sleep(0.1)

            # Once the FIFO is full, enable it's reading, and wait until the VLC_RX
            # processes the input
            self.use_adc(False)
            sleep(0.1)

            print("Checking that the header was received without errors...")
            assert(self.read_header_error() == False)
            assert(self.frames_waiting())

            print("Testing register values...")
            regs = self.read_registers()
            assert(np.array_equal(expected_regs, regs))
            print("Registers match!")

            print("Reading info...")
            [regs, data_rx] = self.read_vlc_rx()
            assert(np.array_equal(expected_out, data_rx))
            print("Data matches!")

        print("Test Successful!")

        if (plot):
            plt.subplot(3,1,1)
            plt.plot(expected_out)
            plt.title("Received signal")
            plt.xlabel("Samples")
            plt.ylabel("Expected output")
            plt.grid(True)

            plt.subplot(3,1,2)
            plt.plot(data_rx)
            plt.xlabel("Samples")
            plt.ylabel("Output")
            plt.grid(True)

            plt.subplot(3,1,3)
            plt.plot(abs(expected_out - data_rx))
            plt.xlabel("Samples")
            plt.ylabel("Error signal")
            plt.grid(True)

            plt.show()
        return data_rx

    def reset_rx_fifos(self):
        """Reset Rx Fifos"""
        self.general_control[0] = clear_bit(self.general_control[0], 1)   # nRstRxFifo
        self.write(self.general_control, port=Ports.CONFIG, addr=CfgAddr.GENERAL_CONTROL)
        self.general_control[0] = set_bit(self.general_control[0], 1)   # nRstRxFifo
        self.write(self.general_control, port=Ports.CONFIG, addr=CfgAddr.GENERAL_CONTROL)

    def read_rx_fifo(self, size: np.uint32, delay_ms=5000) -> np.ndarray:
        """Read RX values, returns uint8

        Waits until the "size" bytes are present in the fifo, and then reads
        them. If there fewer than "size" bytes, then raises an exception
        """
        i = 0
        while( not ((self.read_fifo_count() == size) or (i == delay_ms))):
            sleep(0.001)
            i = i + 1

        if (i == delay_ms):
            raise Exception("Timeout reached while waiting for the Fifo Rx")

        data = np.zeros(size, np.uint32)
        self.read(data, port=Ports.VLC_RX, addr=0)
        data = data.astype(np.uint8)

        return data

    def read_header_error(self) -> bool:
        """Read header_error

        If header_error == '1', the header was received with errors
        """
        data = np.zeros(1, np.uint8)
        data[0] = self.read_status(StsAddr.RX_STS)
        return data[0] & 0x02

    def frames_waiting(self) -> np.uint8:
        """Returns the amount of registers that are queued to be read"""
        return self.read_status(StsAddr.RX_FIFO_REGS)

    def __del__(self):
        RedPitayaGeneric.__del__(self)