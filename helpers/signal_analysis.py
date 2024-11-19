import numpy as np
import pandas as pd
import plotly.graph_objects as go
import plotly.express as px
from scipy.signal import welch


def read_emg_data(file_path):
    """Reads EMG data from a CSV file."""
    try:
        # Read the CSV into a DataFrame
        data = pd.read_csv(file_path)
        time = data["Time (s)"].values
        raw_signal = data["Raw"]
        fir_signal = data["FIR"]
        iir_signal = data["IIR"]
        return time, raw_signal, fir_signal, iir_signal
    except Exception as e:
        print(f"Error reading file: {e}")
        return None, None, None, None


def calculate_rms(signal):
    """Calculate the RMS (Root Mean Square) of the signal."""
    return np.sqrt(np.mean(signal**2)) if len(signal) > 0 else np.nan


def plot_time_series(time, raw_data, fir_data, iir_data):
    """Plot raw, FIR, and IIR data in the time domain."""
    fig = go.Figure()

    fig.add_trace(
        go.Scatter(
            x=time, y=raw_data, mode="lines", name="Raw EMG", line=dict(color="red")
        )
    )
    fig.add_trace(
        go.Scatter(
            x=time,
            y=fir_data,
            mode="lines",
            name="FIR Filtered EMG",
            line=dict(color="blue"),
        )
    )
    fig.add_trace(
        go.Scatter(
            x=time,
            y=iir_data,
            mode="lines",
            name="IIR Filtered EMG",
            line=dict(color="orange"),
        )
    )

    fig.update_layout(
        title="Time-domain EMG Signals (Raw, FIR, IIR)",
        xaxis_title="Time (s)",
        yaxis_title="Amplitude",
        showlegend=True,
        template="plotly_dark",
        title_font=dict(color="white"),
        font=dict(color="white"),
        paper_bgcolor="black",
        plot_bgcolor="black",
        xaxis=dict(gridcolor="gray"),
        yaxis=dict(gridcolor="gray"),
    )
    fig.show()


def plot_frequency_spectrum(raw_data, fir_data, iir_data, fs):
    """Plot Power Spectral Density (PSD) of raw, FIR, and IIR signals."""
    f_raw, p_xx_raw = welch(raw_data, fs, nperseg=1024)
    f_fir, p_xx_fir = welch(fir_data, fs, nperseg=1024)
    f_iir, p_xx_iir = welch(iir_data, fs, nperseg=1024)

    fig = go.Figure()

    fig.add_trace(
        go.Scatter(
            x=f_raw, y=p_xx_raw, mode="lines", name="Raw EMG", line=dict(color="red")
        )
    )
    fig.add_trace(
        go.Scatter(
            x=f_fir,
            y=p_xx_fir,
            mode="lines",
            name="FIR Filtered EMG",
            line=dict(color="blue"),
        )
    )
    fig.add_trace(
        go.Scatter(
            x=f_iir,
            y=p_xx_iir,
            mode="lines",
            name="IIR Filtered EMG",
            line=dict(color="orange"),
        )
    )

    fig.update_layout(
        title="Frequency-domain Power Spectral Density (PSD) of EMG Signals",
        xaxis_title="Frequency (Hz)",
        yaxis_title="Power Spectral Density (dB/Hz)",
        showlegend=True,
        template="plotly_dark",
        title_font=dict(color="white"),
        font=dict(color="white"),
        paper_bgcolor="black",
        plot_bgcolor="black",
        xaxis=dict(gridcolor="gray"),
        yaxis=dict(gridcolor="gray"),
    )
    fig.show()


def plot_rms_comparison(rms_raw, rms_fir, rms_iir):
    """Plot comparison of RMS values for raw, FIR, and IIR signals."""
    labels = ["Raw", "FIR Filtered", "IIR Filtered"]
    rms_values = [rms_raw, rms_fir, rms_iir]

    fig = px.bar(
        x=labels,
        y=rms_values,
        labels={"x": "Signal Type", "y": "RMS Value"},
        title="RMS Value Comparison (Raw, FIR, IIR)",
    )
    fig.update_layout(
        template="plotly_dark",
        title_font=dict(color="white"),
        font=dict(color="white"),
        paper_bgcolor="black",
        plot_bgcolor="black",
        xaxis=dict(gridcolor="gray"),
        yaxis=dict(gridcolor="gray"),
    )
    fig.show()


def main(file_path):
    """Main function to execute the analysis."""
    # Step 1: Read the data from the file
    time, raw_signal, fir_signal, iir_signal = read_emg_data(file_path)
    if time is None:
        return

    # Sampling frequency based on time step
    fs = int(1 / (time[1] - time[0]))

    raw_signal_no_na = raw_signal.dropna().values  # Remove NaN values
    fir_signal_no_na = fir_signal.dropna().values  # Remove NaN values
    iir_signal_no_na = iir_signal.dropna().values  # Remove NaN values

    # Step 2: Perform signal analysis
    rms_raw = calculate_rms(raw_signal)
    rms_fir = calculate_rms(fir_signal)
    rms_iir = calculate_rms(iir_signal)

    print(f"RMS Value (Raw EMG): {rms_raw:.4f}")
    print(f"RMS Value (FIR Filtered EMG): {rms_fir:.4f}")
    print(f"RMS Value (IIR Filtered EMG): {rms_iir:.4f}")

    # Step 3: Create interactive plots
    plot_time_series(time, raw_signal, fir_signal, iir_signal)
    plot_frequency_spectrum(raw_signal_no_na, fir_signal_no_na, iir_signal_no_na, fs)
    plot_rms_comparison(rms_raw, rms_fir, rms_iir)


if __name__ == "__main__":
    file_path = "emg_data_segmented_corrected.csv"  # Path to your EMG data file
    main(file_path)
