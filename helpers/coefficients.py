import numpy as np
from scipy.signal import firwin, freqz, lfilter
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import serial
import time

# Sampling frequency
fs = 1500  # Hz

# Band-stop filter parameters
notch_freq = 50  # Frequency to be removed (Hz)
bandwidth = 2  # Bandwidth around the notch frequency (Hz)

# Low-pass filter parameters
cutoff_freq = 500  # Cutoff frequency (Hz)

# Filter orders
num_taps_notch = 51  # Order of the notch filter
num_taps_lowpass = 51  # Order of the low-pass filter


# Function to record EMG data
def record_emg_data(duration=5, filename="emg_data.txt", port="COM7", baudrate=460800):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print("Recording EMG data...")
        time.sleep(2)  # Wait for the connection to stabilize
        with open(filename, "w") as f:
            start_time = time.time()
            while time.time() - start_time < duration:
                line = ser.readline().decode("utf-8").strip()
                if line:
                    f.write(f"{line}\n")
        print("EMG data recorded successfully.")
        ser.close()
    except Exception as e:
        print(f"Error recording EMG data: {e}")


# Function to read EMG data from file
def read_emg_data_from_file(filename="emg_data.txt"):
    try:
        return np.loadtxt(filename)
    except Exception as e:
        print(f"Error reading EMG data from file: {e}")
        return None


# Function to ask user for input
def ask_user_input(prompt):
    while True:
        user_input = input(f"{prompt} (type 'yes' or 'no'): ").strip().lower()
        if user_input in ["yes", "no"]:
            return user_input  # Return valid input
        print("Invalid input. Please type 'yes' or 'no'.")


# Design the band-stop filter
notch_coefficients = firwin(
    num_taps_notch,
    [notch_freq - bandwidth / 2, notch_freq + bandwidth / 2],
    fs=fs,
    pass_zero="bandstop",
)
# Design the low-pass filter
lowpass_coefficients = firwin(num_taps_lowpass, cutoff_freq / (0.5 * fs), fs=fs)
# Combined filter coefficients
combined_coefficients = np.convolve(notch_coefficients, lowpass_coefficients)
# Frequency response
w, h = freqz(combined_coefficients, worN=8000, fs=fs)
# Display filter coefficients
print("Band-Stop Filter Coefficients:", notch_coefficients)
print("Low-Pass Filter Coefficients:", lowpass_coefficients)
print("Combined Filter Coefficients:", combined_coefficients)
print("Combined Filter Order: ", combined_coefficients.size)


# Function to apply the FIR filter
def apply_filter(signal, coefficients):
    return lfilter(coefficients, 1.0, signal)


# Convert coefficients to C array format
def coefficients_to_c_array(coefficients, array_name):
    c_array = f"const float {array_name}[FILTER_ORDER] = {{\n"
    c_array += ", ".join(f"{coefficient:.10f}" for coefficient in coefficients)
    c_array += "\n};\n"
    return c_array


print(coefficients_to_c_array(combined_coefficients, "fir_coefficients"))


# Function to calculate spectrum
def calculate_spectrum(data, fs):
    n = len(data)
    f = np.fft.fftfreq(n, d=1 / fs)
    f = f[: n // 2]  # Take only positive frequencies
    p_xx = np.abs(np.fft.fft(data)) ** 2
    p_xx = p_xx[: n // 2]
    return f, p_xx


# Main execution
if __name__ == "__main__":
    user_decision = ask_user_input("Do you want to record a new EMG sample?")

    if user_decision == "yes":
        print("You have 3 seconds to prepare...")
        time.sleep(3)  # Countdown before starting the recording
        # Record EMG data
        record_emg_data(
            duration=5, filename="emg_data.txt", port="COM7", baudrate=460800
        )
    else:
        print("Using stored EMG data...")

    # Read EMG data from file
    emg_data = read_emg_data_from_file()
    if emg_data is None or len(emg_data) == 0:
        print("No valid data read from the file.")
    else:
        # Cut the first 50 samples
        emg_data = emg_data[20:]
        # Apply the filter to the EMG data
        filtered_data = apply_filter(emg_data, combined_coefficients)
        # Time axis for the original signal
        t = np.arange(len(emg_data)) / fs

        fig = make_subplots(
            rows=3,
            cols=1,
            subplot_titles=(
                "Frequency Response",
                "Signal Before and After Filtering",
                "Frequency Spectrum",
            ),
        )

        fig.add_trace(
            go.Scatter(
                x=w,
                y=abs(h),
                mode="lines",
                name="Frequency Response",
                line=dict(color="blue"),
            ),
            row=1,
            col=1,
        )

        fig.add_trace(
            go.Scatter(
                x=t,
                y=emg_data,
                mode="lines",
                name="Original Signal",
                line=dict(color="blue"),
            ),
            row=2,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=t,
                y=filtered_data,
                mode="lines",
                name="Filtered Signal",
                line=dict(color="red"),
            ),
            row=2,
            col=1,
        )

        f_raw, p_xx_raw = calculate_spectrum(emg_data, fs)
        f_filtered, p_xx_filtered = calculate_spectrum(filtered_data, fs)

        # Frequency spectrum plot (Raw and Filtered in one subplot)
        fig.add_trace(
            go.Scatter(
                x=f_raw,
                y=p_xx_raw,
                mode="lines",
                name="Raw Signal Spectrum",
                line=dict(color="lime", width=2),
            ),
            row=3,
            col=1,
        )

        fig.add_trace(
            go.Scatter(
                x=f_filtered,
                y=p_xx_filtered,
                mode="lines",
                name="Filtered Signal Spectrum",
                line=dict(color="magenta", width=2),
            ),
            row=3,
            col=1,
        )

        fig.update_layout(
            title=f"EMG Data Analysis - Filter order {combined_coefficients.size}",
            paper_bgcolor="rgb(30,30,30)",
            plot_bgcolor="rgb(50,50,50)",
            font=dict(color="white"),
        )

        # Update axes titles
        fig.update_xaxes(
            title_text="Frequency (Hz)", row=1, col=1, title_font=dict(color="white")
        )
        fig.update_yaxes(
            title_text="Gain", row=1, col=1, title_font=dict(color="white")
        )
        fig.update_xaxes(
            title_text="Time (s)", row=2, col=1, title_font=dict(color="white")
        )
        fig.update_yaxes(
            title_text="Amplitude", row=2, col=1, title_font=dict(color="white")
        )
        fig.update_xaxes(
            title_text="Frequency (Hz)", row=3, col=1, title_font=dict(color="white")
        )
        fig.update_yaxes(
            title_text="Power", row=3, col=1, title_font=dict(color="white")
        )

        fig.show()
