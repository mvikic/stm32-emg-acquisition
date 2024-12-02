import pandas as pd


def process_emg_data(input_txt_file: str, output_csv_file: str, sampling_frequency: float, duration: float):
    """
    Process EMG data and export to CSV with Time and Amplitude columns.

    :param input_txt_file:  Path to the input text file with EMG samples.
    :param output_csv_file: Path to save the output CSV file.
    :param sampling_frequency: Sampling frequency in Hz.
    :param duration: Duration of the recording in seconds.
    """
    total_samples = int(sampling_frequency * duration)

    with open(input_txt_file, "r") as file:
        data = file.readlines()

    amplitude = [float(line.strip()) for line in data]

    if len(amplitude) != total_samples:
        print(
            f"Warning: Number of samples ({len(amplitude)}) does not match expected ({total_samples})."
        )

    # Create the time axis
    time = [i / sampling_frequency for i in range(len(amplitude))]

    # Create a DataFrame
    df = pd.DataFrame({"Time": time, "Amplitude": amplitude})

    # Export to CSV
    df.to_csv(output_csv_file, index=False)
    print(f"Data successfully exported to {output_csv_file}")


def main():
    # Parameters
    input_txt_file = "data/emg_data_15s_raw.txt"
    output_csv_file = "data/emg_data_15s_raw_time.csv"
    sampling_frequency = 1500
    duration = 15

    process_emg_data(input_txt_file, output_csv_file, sampling_frequency, duration)


if __name__ == "__main__":
    main()
