import pandas as pd
import plotly.graph_objects as go


def plot_combined_emg(file_path):
    """Plots combined Raw and FIR segments with IIR as separate, all togglable."""
    try:
        # Read the CSV file
        data = pd.read_csv(file_path)
        time = data["Time (s)"].values
        amplitude = data["Amplitude"].values

        # Combine segments for Raw and FIR, with 24s cutoff
        raw_segment = (0, 9.358)
        fir_segment = (9.358, 16.39133)
        iir_segment = (16.39133, 24)

        fig = go.Figure()

        # Add combined Raw Signal trace
        mask = (time >= raw_segment[0]) & (time < raw_segment[1])
        fig.add_trace(
            go.Scatter(
                x=time[mask],
                y=amplitude[mask],
                mode="lines",
                name="Raw",
                line=dict(color="red"),
                legendgroup="Raw",
                showlegend=(
                    False if raw_segment[0] > 0 else True
                ),  # Only show legend once
            )
        )

        # Add combined FIR Filtered Signal trace
        mask = (time >= fir_segment[0]) & (time < fir_segment[1])
        fig.add_trace(
            go.Scatter(
                x=time[mask],
                y=amplitude[mask],
                mode="lines",
                name="FIR Filtered",
                line=dict(color="blue"),
                legendgroup="FIR",
                showlegend=(
                    False if fir_segment[0] > 9.358 else True
                ),  # Only show legend once
            )
        )

        # Add IIR Filtered Signal trace
        mask = (time >= iir_segment[0]) & (time < iir_segment[1])
        fig.add_trace(
            go.Scatter(
                x=time[mask],
                y=amplitude[mask],
                mode="lines",
                name="IIR Filtered",
                line=dict(color="orange"),
                legendgroup="IIR",
            )
        )

        # Update layout for toggling
        fig.update_layout(
            title="Segmented EMG Data",
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

    except Exception as e:
        print(f"Error processing the file: {e}")


if __name__ == "__main__":
    file_path = "emg_data_with_time.csv"  # Replace with the path to your CSV file
    plot_combined_emg(file_path)
