import sys
import argparse
from pathlib import Path
from typing import Optional, Union

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
from scipy.signal import welch


class EMGAnalyzer:
    def __init__(self, fs: Optional[int] = None, mode: Optional[str] = None) -> None:
        """
        Initialize the EMGAnalyzer with an optional sampling frequency.

        :param fs: Sampling frequency in Hz.
        :param mode: Analysis mode, either "single" or "multi".
        """
        self._fs: Optional[int] = fs
        self._mode: Optional[str] = mode
        self._signals: dict[str, pd.Series] = {}
        self.colors: dict[str, str] = {"Raw": "red", "FIR": "blue", "IIR": "orange"}

    @property
    def fs(self) -> Optional[int]:
        return self._fs

    @fs.setter
    def fs(self, value: int) -> None:
        if value <= 0:
            raise ValueError("Sampling frequency must be positive.")
        self._fs = value

    @property
    def mode(self) -> Optional[str]:
        return self._mode

    @mode.setter
    def mode(self, value: str) -> None:
        if value not in {"single", "multi"}:
            raise ValueError("Mode must be 'single' or 'multi'.")
        self._mode = value

    @property
    def signals(self) -> dict[str, pd.Series]:
        return self._signals

    def calculate_sampling_frequency(self, time_data: np.ndarray) -> None:
        """
        Calculate and set the sampling frequency from the signal's index.

        :param time_data: Array containing time indices.
        """
        self.fs = int(1 / (time_data[1] - time_data[0]))

    def read_data(self, file_paths: Union[Path, list[Path]]) -> None:
        """
        Read EMG data from either a single file or multiple files.

        :param file_paths: A single path for "single" mode, or a list of three paths for "multi" mode.
        """
        try:
            if self.mode == "single":
                data: pd.DataFrame = pd.read_csv(file_paths, index_col="Time")
                self._signals = {key: data[key] * 1e3 for key in ["Raw", "FIR", "IIR"]}
                if not self.fs:
                    self.calculate_sampling_frequency(time_data=data.index.to_numpy())

            elif self.mode == "multi":
                names: list[str] = ["Raw", "FIR", "IIR"]
                self._signals = {
                    name: pd.read_csv(path, index_col="Time")["Amplitude"].iloc[1:] * 1e3
                    for name, path in zip(names, file_paths)
                }
                if not self.fs:
                    self.calculate_sampling_frequency(time_data=self.signals["Raw"].index.to_numpy())

            else:
                raise ValueError("Mode must be 'single' or 'multi'.")

        except Exception as e:
            print(f"Error reading data: {e}")

    def calculate_rms(self) -> dict[str, float]:
        """
        Calculate the RMS values of all signals.

        :return: dictionary of RMS values.
        """
        return {
            name: float(np.sqrt(np.mean(signal.dropna() ** 2)))
            for name, signal in self.signals.items()
        }

    def plot_time_series(self) -> None:
        """
        Plot time-domain signals in either a single plot or multiple subplots.
        """
        hover_template: str = "<b>Time:</b> %{x:.3f} s<br><b>Amplitude:</b> %{y:.3} mV<br>"

        if self.mode == "single":
            fig: go.Figure = go.Figure()
            for name, signal in self.signals.items():
                signal_no_nan: pd.Series = signal.dropna()
                fig.add_trace(
                    go.Scatter(
                        x=signal_no_nan.index,
                        y=signal_no_nan,
                        mode="lines",
                        name=f"{name} EMG",
                        line=dict(color=self.colors[name]),
                        xaxis=dict(gridcolor="lightgray"),
                        yaxis=dict(gridcolor="lightgray"),
                        hovertemplate=hover_template,
                    )
                )

            fig.update_layout(
                title="Time-domain EMG Signals (Single recording)",
                xaxis_title="Time (s)",
                yaxis_title="Amplitude (mV)",
                template="plotly_white",
            )

        elif self.mode == "multi":
            num_signals: int = len(self.signals)
            fig: go.Figure = make_subplots(
                rows=num_signals,
                cols=1,
                shared_xaxes=True,
                subplot_titles=list(self.signals.keys()),
                vertical_spacing=0.1,
            )

            for i, (name, signal) in enumerate(self.signals.items(), start=1):
                fig.add_trace(
                    go.Scatter(
                        x=signal.index,
                        y=signal,
                        mode="lines",
                        name=f"{name} EMG",
                        line=dict(color=self.colors[name]),
                        hovertemplate=hover_template,
                    ),
                    row=i,
                    col=1,
                )

            fig.update_layout(
                title="Time-domain EMG Signals (Multiple recordings)",
                xaxis_title="Time (s)",
                template="plotly_white",
                # height=300 * num_signals,
            )

            for i, name in enumerate(self.signals.keys(), start=1):
                fig.update_xaxes(gridcolor="lightgray")
                fig.update_yaxes(title_text=f"{name} Amplitude (mV)", row=i, col=1, gridcolor="lightgray")

        else:
            raise ValueError("Invalid mode. Use 'single' or 'multi'.")

        fig.show(
            config={
                "scrollZoom": True,
                "displaylogo": False,
            }
        )

    def plot_frequency_spectrum(self) -> None:
        """
        Plot frequency-domain power spectral density (PSD) for all signals.
        """
        fig: go.Figure = go.Figure()
        hover_template: str = "<b>Frequency:</b> %{x:.1f} Hz<br><b>PSD:</b> %{y} dB/Hz<br>"

        for name, signal in self.signals.items():
            f: np.ndarray
            pxx: np.ndarray
            f, pxx = welch(signal.dropna(), self.fs, nperseg=1024)
            fig.add_trace(
                go.Scatter(
                    x=f,
                    y=pxx,
                    mode="lines",
                    name=f"{name} EMG",
                    line=dict(color=self.colors[name]),
                    hovertemplate=hover_template,
                )
            )

        fig.update_layout(
            title="Frequency-domain Power Spectral Density (PSD) of EMG Signals",
            xaxis_title="Frequency (Hz)",
            yaxis_title="Power Spectral Density (dB/Hz)",
            template="plotly_white",
            xaxis=dict(gridcolor="lightgray"),
            yaxis=dict(gridcolor="lightgray"),
        )

        fig.update_layout(
            updatemenus=[
                dict(
                    type="buttons",
                    showactive=True,
                    buttons=[
                        dict(
                            label="Linear Scale",
                            method="relayout",
                            args=[{"yaxis.type": "linear"}],
                        ),
                        dict(
                            label="Log Scale",
                            method="relayout",
                            args=[{"yaxis.type": "log"}],
                        ),
                    ],
                    x=0.85,
                    y=1.15,
                    xanchor="center",
                    yanchor="top",
                    bgcolor="white",
                    bordercolor="white",
                    font=dict(color="gray"),
                )
            ]
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

    def plot_rms_comparison(self) -> None:
        """
        Plot a bar chart comparing RMS values for all signals.
        """
        rms_values: dict[str, float] = self.calculate_rms()
        fig = px.bar(
            x=list(rms_values.keys()),
            y=list(rms_values.values()),
            labels={"x": "Signal Type", "y": "RMS Value"},
            title="RMS Value Comparison (Raw, FIR, IIR)",
        )
        fig.update_layout(template="plotly_dark")
        fig.show()

    def analyze(self) -> None:
        """
        Perform full analysis: time-series plot, frequency spectrum, and RMS comparison.
        """
        rms_values: dict[str, float] = self.calculate_rms()
        for name, rms in rms_values.items():
            print(f"RMS Value ({name} EMG): {rms:.4f}")

        self.plot_time_series()
        self.plot_frequency_spectrum()


def main() -> None:
    parser = argparse.ArgumentParser(description="EMG Signal Analysis Tool")
    parser.add_argument(
        "--single",
        type=Path,
        help="Path to a single CSV file containing all signals (Raw, FIR, IIR).",
    )
    parser.add_argument(
        "--multi",
        nargs=3,
        type=Path,
        metavar=("RAW", "FIR", "IIR"),
        help="Paths to three CSV files (RAW, FIR, IIR).",
    )
    args = parser.parse_args()

    if not (args.single or args.multi):
        parser.error("You must provide either --single or --multi arguments.")
        sys.exit(1)

    mode: str
    data: Union[Path, list[Path]]
    mode, data = ("single", args.single) if args.single else ("multi", args.multi)

    analyzer = EMGAnalyzer(mode=mode)
    analyzer.read_data(data)
    analyzer.analyze()


if __name__ == "__main__":
    main()
