import numpy as np

def bin2dec(binary_string: str, signed=True) -> int:
    """Convert a binary string to an integer

    If "signed", then two's complement is used. The binary string should
    look like "0011" or "1010".
    """
    # If MSB is 0, the number is positive, just convert normally
    if binary_string[0] == '0' or not signed:
        return int(binary_string, 2)

    # If MSB is 1, the number is negative in two's complement
    # Invert the bits and add 1 to get the positive equivalent
    inverted = ''.join('1' if bit == '0' else '0' for bit in binary_string)
    decimal_positive = int(inverted, 2) + 1
    return -decimal_positive

def set_bit(value, bit):
    return value | (1<<bit)

def clear_bit(value, bit):
    return value & ~(1<<bit)

def read_binary_file(filename: str, dtype=np.int32, signed=True) -> np.ndarray:
    """Read binary file and return data
    """
    # Get amount of lines in file
    with open(filename, 'r') as file:
        line_count = sum(1 for lines in file)

    data = np.zeros(line_count, dtype=dtype)
    with open(filename, "r") as file:
        i = 0
        for line in file:
            data[i] = bin2dec(line.strip(" \n,"), signed)
            i = i + 1

    return data.astype(dtype)