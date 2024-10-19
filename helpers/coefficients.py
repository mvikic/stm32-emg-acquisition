from scipy.signal import firwin, freqz, lfilter
import matplotlib.pyplot as plt
import numpy as np

# Sampling frequency
fs = 1000  # Hz

# Band-stop filter parameters
notch_freq = 50  # Frequency to be removed (Hz)
bandwidth = 2     # Bandwidth around the notch frequency (Hz)

# Low-pass filter parameters
cutoff_freq = 500  # Cutoff frequency (Hz)

# Filter orders
numtaps_notch = 13  # Order of the notch filter
numtaps_lowpass = 13  # Order of the low-pass filter

# Design the band-stop filter
notch_coefficients = firwin(numtaps_notch, [notch_freq - bandwidth/2, notch_freq + bandwidth/2],
                            fs=fs, pass_zero='bandstop')

# Design the low-pass filter
lowpass_coefficients = firwin(numtaps_lowpass, cutoff_freq / (0.5 * fs), fs=fs)

# Combined filter coefficients (convolution of the two filters)
combined_coefficients = np.convolve(notch_coefficients, lowpass_coefficients)

# Frequency response
w, h = freqz(combined_coefficients, worN=8000, fs=fs)

# Plot frequency response
plt.figure()
plt.plot(w,  abs(h), 'b')
plt.title('Combined Frequency Response')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Gain')
plt.grid()
plt.show()

# Display filter coefficients
print("Band-Stop Filter Coefficients:", notch_coefficients)
print("Low-Pass Filter Coefficients:", lowpass_coefficients)
print("Combined Filter Coefficients:", combined_coefficients)
print("Combined Filter Order: ", combined_coefficients.size)


def coefficients_to_c_array(coeffs, array_name):
    c_array = f"const float {array_name}[] = {{\n"
    c_array += ", ".join(f"{coeff:.6f}" for coeff in coeffs)
    c_array += "\n};\n"
    return c_array


# Example signal processing (use with your EMG data)
def apply_filter(signal, coefficients):
    return lfilter(coefficients, 1.0, signal)


print(coefficients_to_c_array(combined_coefficients, "fir_coefficients"))

# Create a sample signal for demonstration (e.g., a signal with noise)
t = np.arange(0, 1.0, 1.0 / fs)
signal = np.sin(2 * np.pi * 100 * t) + np.sin(2 * np.pi * 50 * t)  # Signal with 100 Hz and 50 Hz components
filtered_signal = apply_filter(signal, combined_coefficients)

# Plot sample signal
plt.figure()
plt.plot(t, signal, 'b', label='Original Signal')
plt.plot(t, filtered_signal, 'r', label='Filtered Signal')
plt.title('Signal Before and After Filtering')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.legend()
plt.grid()
plt.show()
