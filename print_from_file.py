import numpy as np
import matplotlib.pyplot as plt

def compare_files_v1(file1, file2):
    """
    Compare the decimal values from two files and plot the difference.
    
    Parameters:
    file1 (str): Path to the first input file
    file2 (str): Path to the second input file
    
    Returns:
    None
    """
    # Load data from the files
    with open(file1, 'r') as f1, open(file2, 'r') as f2:
        data1 = [int(line.split(',')[1].strip()) for line in f1]
        data2 = [int(line.split(',')[1].strip()) for line in f2]
    
    # Ensure both files have the same number of lines
    assert len(data1) == len(data2) == 16384, "Files must have 16384 lines each"

    for i in range(len(data2)):
        data2[i] = data2[i]*2

    # Calculate the difference between the decimal values
    diff = np.array(data1) - np.array(data2)
    
    # Plot the difference
    plt.figure(figsize=(12, 8))
    
    # Plot the data from the first file
    plt.subplot(3, 1, 1)
    plt.plot(data1, color='orange')
    plt.title("Data from ADC Input")
    plt.xlabel("Line Number")
    plt.ylabel("Value")
    
    # Plot the data from the second file
    plt.subplot(3, 1, 2)
    plt.plot(data2, color='red')
    plt.title("Data from TX FIFO")
    plt.xlabel("Line Number")
    plt.ylabel("Value")
    
    # Plot the difference
    plt.subplot(3, 1, 3)
    plt.plot(diff)
    plt.title("Difference Between Values")
    plt.xlabel("Line Number")
    plt.ylabel("Difference")
    
    plt.tight_layout()
    plt.show()

def binary_to_signed_int16(binary_str):
    """
    Convert 16-bit binary string to signed int16 value.
    """
    # Convert to integer first
    value = int(binary_str, 2)
    # If the first bit is 1, this is a negative number in two's complement
    if value & 0x8000:
        # Convert to negative number
        value -= 0x10000
    return np.int16(value)

def read_file1(filename):
    """
    Read binary data from file 1.
    Returns list of binary strings with cleaned data.
    """
    binary_data = []
    with open(filename, 'r') as f:
        for line in f:
            # Remove whitespace, commas and get only the binary part
            cleaned = line.strip().replace(',', '')
            if cleaned and all(c in '01' for c in cleaned) and len(cleaned) == 16:
                binary_data.append(cleaned)
    return binary_data

def read_file2(filename):
    """
    Read binary and decimal data from file 2.
    Returns list of tuples (binary_string, decimal_value).
    """
    data = []
    with open(filename, 'r') as f:
        for line in f:
            # Split on comma and clean up
            parts = line.strip().split(',')
            if len(parts) == 2:
                binary = parts[0].strip()
                decimal = int(parts[1].strip())
                if len(binary) == 16 and all(c in '01' for c in binary):
                    data.append((binary, decimal))
    return data

def find_matching_value(array1, array2, start_pos):
    """
    Find how many positions forward in array2 it takes to find a value matching array1[start_pos]
    
    Parameters:
    array1 (np.array): First array with source value
    array2 (np.array): Second array to search in
    start_pos (int): Starting position in array1
    
    Returns:
    tuple: (positions_moved, value_found)
        positions_moved: How many positions forward from start_pos it took to find the value
                        Returns -1 if value not found
        value_found: The value that was being searched for
    """
    # Check if start_pos is valid
    if start_pos < 0 or start_pos >= len(array1):
        raise ValueError(f"Start position {start_pos} is out of bounds for array of length {len(array1)}")
    
    # Get the value we're looking for
    target_value = array1[start_pos]
    
    # Search in array2 starting from start_pos
    for i in range(start_pos, len(array2)):
        if array2[i] == target_value:
            positions_moved = i - start_pos
            return positions_moved, target_value
    
    # If we didn't find the value
    return -1, target_value

def compare_and_plot_binary_files(file1_path, file2_path):
    """
    Compare and plot binary data from two files with different formats.
    
    Parameters:
    file1_path (str): Path to file 1
    file2_path (str): Path to file 2
    
    Returns:
    tuple: (array1, array2, difference_array)
    """
    # Read data from files
    file1_data = read_file1(file1_path)
    file2_data = read_file2(file2_path)
    
    # Convert file2 length as reference for array size
    array_size = len(file2_data)
    
    # Initialize arrays with zeros
    array1 = np.zeros(array_size, dtype=np.int16)
    array2 = np.zeros(array_size, dtype=np.int16)
    
    # Fill array1 with available data
    for i, binary_str in enumerate(file1_data):
        if i < array_size:  # Only process up to array_size
            array1[i] = binary_to_signed_int16(binary_str)
    
    # Fill array2 with data
    for i, (binary_str, decimal) in enumerate(file2_data):
        array2[i] = np.int16(decimal)
    

    for i in range(len(array2)):
        array2[i] = array2[i]*2

    #Roll para acomodar valores
    #array1 = np.roll(np.array(array1), 24)

    #Find the matching values
    #moves, target_value = find_matching_value(array1, array2, 11750)
    #print("Had to move", moves,'times to find value', target_value,'\n')

    #moves, target_value = find_matching_value(array1, array2, 12200)
    #print("Had to move", moves,'times to find value', target_value,'\n')

    #moves, target_value = find_matching_value(array1, array2, 11500)
    #print("Had to move", moves,'times to find value', target_value,'\n')

    # Calculate difference
    difference = array2 - array1

    # Create subplot figure
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))
    
    # Plot array1
    ax1.plot(array1, 'b')
    ax1.set_title('Data_out.mem Values')
    ax1.grid(True)
    
    # Plot array2
    ax2.plot(array2, 'r')
    ax2.set_title('FIFO in Values')
    ax2.grid(True)
    ax2.set_ylim(-5000, 5000)
    
    # Plot difference
    ax3.plot(difference, 'g')
    ax3.set_title('Difference between Files')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.show()

#compare_files_v1("./TX_tests/ADC_IN/ADC_input_v6_7.txt", "./TX_tests/FIFO_IN/txt_input_v6_7.txt")
#compare_files_v1("./TX_tests/ADC_IN/ADC_input_vB_1.txt", "./TX_tests/ADC_IN/ADC_input_1.txt")
#compare_files_v1("./TX_tests/FIFO_IN/txt_input_1.txt", "./TX_tests/FIFO_IN/txt_input_3.txt")
compare_and_plot_binary_files("data_out.mem", "./TX_tests/FIFO_IN/txt_input_v6_7.txt")