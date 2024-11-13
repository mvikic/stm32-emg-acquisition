import time
import numpy as np
import serial
import logging
import sys

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s',
                    handlers=[logging.StreamHandler(sys.stdout)])


def read_emg_data(port="COM7", baudrate=460800, duration=30, filename="emg_data_30s_all.txt"):
    emg_data = []
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            logging.info("Recording started")
            start_time = time.time()
            while (time.time() - start_time) < duration:
                line = ser.readline().decode("utf-8").strip()
                try:
                    value = float(line)
                    emg_data.append(value)
                except ValueError:
                    continue  # Ignore invalid lines
    except Exception as e:
        print(f"Error reading from EMG device: {e}")

    # Save data to file
    np.savetxt(filename, emg_data)
    print(f"Recorded {len(emg_data)} samples and saved to {filename}.")


if __name__ == "__main__":
    read_emg_data()
