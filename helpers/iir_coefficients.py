import numpy as np
from scipy.signal import firwin, freqz, lfilter, iirnotch, butter, sosfilt
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import serial
import time

# Sampling frequency
fs = 1500  # Hz

# Band-stop filter parameters for FIR
notch_freq = 50  # Frequency to be removed (Hz)
bandwidth = 2  # Bandwidth around the notch frequency (Hz)

# Low-pass filter parameters for FIR
cutoff_freq = 500  # Cutoff frequency (Hz)

# Filter orders
num_taps_notch = 51  # Order of the notch filter
num_taps_lowpass = 51  # Order of the low-pass filter

# Design the FIR filters
notch_coefficients = firwin(
    num_taps_notch,
    [notch_freq - bandwidth / 2, notch_freq + bandwidth / 2],
    fs=fs,
    pass_zero="bandstop",
)
lowpass_coefficients = firwin(num_taps_lowpass, cutoff_freq / (0.5 * fs), fs=fs)
combined_coefficients = np.convolve(notch_coefficients, lowpass_coefficients)

# Design the IIR filter to attenuate frequencies below 30Hz and above 500Hz, with a notch at 50Hz
# Notch filter (IIR)
notch_b, notch_a = iirnotch(notch_freq / (fs / 2), Q=30)

# Bandpass filter (IIR)
sos_bandpass = butter(
    4, [30 / (fs / 2), 500 / (fs / 2)], btype="bandpass", output="sos"
)


# Function to record EMG data
def record_emg_data(duration=5, filename="data/emg_data.txt", port="COM7", baudrate=460800):
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
def read_emg_data_from_file(filename="data/emg_data.txt"):
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


# Function to apply the FIR filter
def apply_fir_filter(signal, coefficients):
    return lfilter(coefficients, 1.0, signal)


# Function to apply the IIR filter
def apply_iir_filter(signal):
    # Apply the notch filter
    signal = lfilter(notch_b, notch_a, signal)
    # Apply the bandpass filter
    return sosfilt(sos_bandpass, signal)


# Convert coefficients to C array format
def coefficients_to_c_array(coefficients, array_name):
    c_array = f"const float {array_name}[FILTER_ORDER] = {{\n"
    c_array += ", ".join(f"{coefficient:.10f}" for coefficient in coefficients)
    c_array += "\n};\n"
    return c_array


print(coefficients_to_c_array(combined_coefficients, "fir_coefficients"))
print(coefficients_to_c_array(notch_b, "b"))
print(coefficients_to_c_array(notch_a, "a"))
# print(coefficients_to_c_array(sos_bandpass, "a"))


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
        time.sleep(3)
        record_emg_data(
            duration=5, filename="data/emg_data.txt", port="COM7", baudrate=460800
        )
    else:
        print("Using stored EMG data...")

    emg_data = read_emg_data_from_file()
    if emg_data is None or len(emg_data) == 0:
        print("No valid data read from the file.")
    else:
        emg_data = emg_data[20:]

        # Apply FIR filter
        filtered_data_fir = apply_fir_filter(emg_data, combined_coefficients)

        # Apply IIR filter
        filtered_data_iir = apply_iir_filter(emg_data)

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

        # Plot the FIR filter frequency response
        w, h = freqz(combined_coefficients, worN=8000, fs=fs)
        fig.add_trace(
            go.Scatter(
                x=w,
                y=abs(h),
                mode="lines",
                name="FIR Frequency Response",
                line=dict(color="blue"),
            ),
            row=1,
            col=1,
        )

        # Plot the raw, FIR filtered, and IIR filtered signals
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
                y=filtered_data_fir,
                mode="lines",
                name="FIR Filtered Signal",
                line=dict(color="red"),
            ),
            row=2,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=t,
                y=filtered_data_iir,
                mode="lines",
                name="IIR Filtered Signal",
                line=dict(color="cyan"),  # Bright blue color for IIR filtered signal
            ),
            row=2,
            col=1,
        )

        # Frequency spectrum of the raw and filtered signals
        f_raw, p_xx_raw = calculate_spectrum(emg_data, fs)
        f_fir, p_xx_fir = calculate_spectrum(filtered_data_fir, fs)
        f_iir, p_xx_iir = calculate_spectrum(filtered_data_iir, fs)

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
                x=f_fir,
                y=p_xx_fir,
                mode="lines",
                name="FIR Filtered Signal Spectrum",
                line=dict(color="magenta", width=2),
            ),
            row=3,
            col=1,
        )

        fig.add_trace(
            go.Scatter(
                x=f_iir,
                y=p_xx_iir,
                mode="lines",
                name="IIR Filtered Signal Spectrum",
                line=dict(color="cyan", width=2),  # Bright blue for IIR spectrum
            ),
            row=3,
            col=1,
        )

        # Set plot layout and show
        fig.update_layout(
            title=f"EMG Data Analysis - Filter order {combined_coefficients.size}",
            paper_bgcolor="rgb(30,30,30)",
            plot_bgcolor="rgb(50,50,50)",
            font=dict(color="white"),
        )

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
