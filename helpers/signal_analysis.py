import numpy as np
import pandas as pd
import plotly.graph_objects as go
import plotly.express as px
from scipy.signal import welch


def read_emg_data(recording_path):
    """
    # TODO: Add docstring
    :param recording_path:
    :return:
    """
    try:
        data = pd.read_csv(recording_path, index_col="Time")

        raw_signal = data["Raw"] * 1e3
        fir_signal = data["FIR"] * 1e3
        iir_signal = data["IIR"] * 1e3

        time = data.index.values

        return time, raw_signal, fir_signal, iir_signal
    except Exception as e:
        print(f"Error reading file: {e}")
        return None, None, None, None


def calculate_rms(signal):
    """
    # TODO: Add docstring
    :param signal:
    :return:
    """
    return np.sqrt(np.mean(signal ** 2)) if len(signal) > 0 else np.nan


def plot_time_series(raw_data, fir_data, iir_data):
    """
    # TODO: Add docstring
    :param raw_data:
    :param fir_data:
    :param iir_data:
    :return:
    """
    fig = go.Figure()

    hover_template = "<b>Time:</b> %{x:.3f} s<br><b>Amplitude:</b> %{y:.3} mV<br>"  # <extra>Custom Note</extra>"

    fig.add_trace(
        go.Scatter(
            x=raw_data.index,
            y=raw_data,
            mode="lines",
            name="Raw EMG",
            line=dict(color="red"),
            hovertemplate=hover_template,
        )
    )
    fig.add_trace(
        go.Scatter(
            x=fir_data.index,
            y=fir_data,
            mode="lines",
            name="FIR Filtered EMG",
            line=dict(color="blue"),
            hovertemplate=hover_template,
        )
    )
    fig.add_trace(
        go.Scatter(
            x=iir_data.index,
            y=iir_data,
            mode="lines",
            name="IIR Filtered EMG",
            line=dict(color="orange"),
            hovertemplate=hover_template,
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
    fig.show(
        config={
            "scrollZoom": True,
            "displaylogo": False,
            "modeBarButtonsToAdd": [
                "drawline",
                "drawopenpath",
                "eraseshape",
                "hovercompare",
                "hoverclosest",
            ],
            "modeBarButtonsToRemove": [
                "lasso2d",
                "zoomIn2d",
                "zoomOut2d",
            ],
        }
    )


def plot_frequency_spectrum(raw_data, fir_data, iir_data, fs):
    """
    # TODO: Add docstring
    :param raw_data:
    :param fir_data:
    :param iir_data:
    :param fs:
    :return:
    """
    f_raw, p_xx_raw = welch(raw_data, fs, nperseg=1024)
    f_fir, p_xx_fir = welch(fir_data, fs, nperseg=1024)
    f_iir, p_xx_iir = welch(iir_data, fs, nperseg=1024)

    fig = go.Figure()

    hover_template = "<b>Frequency:</b> %{x:.1f} Hz<br><b>PSD:</b> %{y} dB/Hz<br>"

    fig.add_trace(
        go.Scatter(
            x=f_raw,
            y=p_xx_raw,
            mode="lines",
            name="Raw EMG",
            line=dict(color="red"),
            hovertemplate=hover_template,
        )
    )
    fig.add_trace(
        go.Scatter(
            x=f_fir,
            y=p_xx_fir,
            mode="lines",
            name="FIR Filtered EMG",
            line=dict(color="blue"),
            hovertemplate=hover_template,
        )
    )
    fig.add_trace(
        go.Scatter(
            x=f_iir,
            y=p_xx_iir,
            mode="lines",
            name="IIR Filtered EMG",
            line=dict(color="orange"),
            hovertemplate=hover_template,
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
        yaxis=dict(gridcolor="gray", type="log"),
    )

    # Show the figure with a full set of analysis tools
    fig.show(
        config={
            "scrollZoom": True,
            "displaylogo": False,
            "modeBarButtonsToAdd": [
                "drawline",
                "drawopenpath",
                "eraseshape",
                "hovercompare",
                "hoverclosest",
            ],
            "modeBarButtonsToRemove": [
                "lasso2d",
                "zoomIn2d",
                "zoomOut2d",
            ],
        }
    )

    # fig.show()


def plot_rms_comparison(rms_raw, rms_fir, rms_iir):
    """
    # TODO: Add docstring
    :param rms_raw:
    :param rms_fir:
    :param rms_iir:
    :return:
    """
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


def main(recording_path):
    """
    # TODO: Add docstring
    :param recording_path:
    :return:
    """
    time, raw_signal, fir_signal, iir_signal = read_emg_data(recording_path)
    if time is None:
        return

    # Sampling frequency based on time step
    fs = int(1 / (time[1] - time[0]))

    raw_signal_no_nan = raw_signal.dropna()
    fir_signal_no_nan = fir_signal.dropna()
    iir_signal_no_nan = iir_signal.dropna()

    rms_raw = calculate_rms(raw_signal_no_nan)
    rms_fir = calculate_rms(fir_signal_no_nan)
    rms_iir = calculate_rms(iir_signal_no_nan)

    print(f"RMS Value (Raw EMG): {rms_raw:.4f}")
    print(f"RMS Value (FIR Filtered EMG): {rms_fir:.4f}")
    print(f"RMS Value (IIR Filtered EMG): {rms_iir:.4f}")

    plot_time_series(raw_signal_no_nan, fir_signal_no_nan, iir_signal_no_nan)
    plot_frequency_spectrum(raw_signal_no_nan, fir_signal_no_nan, iir_signal_no_nan, fs)
    plot_rms_comparison(rms_raw, rms_fir, rms_iir)


if __name__ == "__main__":
    file_path = "data/emg_data_segmented_corrected.csv"
    main(file_path)
