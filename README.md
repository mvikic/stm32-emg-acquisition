# STM32 EMG Acquisition and Filtering with FIR/IIR

This project implements an EMG (Electromyography) signal acquisition and filtering system using an STM32F407G-DISC1 microcontroller. The system applies FIR (Finite Impulse Response) and IIR (Infinite Impulse Response) filters to process the acquired EMG signals, eliminating power grid noise (50Hz) and filtering out frequencies above 500Hz.

## Table of Contents

- [Introduction](#introduction)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Filter Design](#filter-design)
- [Implementation Details](#implementation-details)
  - [Circular Buffer](#circular-buffer)
  - [FIR Filter Implementation](#fir-filter-implementation)
  - [Storing Coefficients in Flash Memory](#storing-coefficients-in-flash-memory)
- [How to Build and Run](#how-to-build-and-run)
- [Usage](#usage)
- [License](#license)

## Introduction

This project focuses on the acquisition and processing of EMG signals using the STM32F407G-DISC1 microcontroller. The primary goal is to acquire clean EMG signals by filtering out unwanted frequencies, specifically the 50Hz power grid noise and frequencies above 500Hz. The system uses FIR and IIR filters, with coefficients designed using Python's `scipy` library.

## Hardware Requirements

- **STM32F407G-DISC1** development board
- **MikroE EMG Click Module**
- **PC** for programming and debugging
- **USB Cable** for power and data transfer

## Software Requirements

- **IDE**: STM32CubeIDE (version X.X.X)
- **Toolchain**: GCC ARM Embedded
- **Library**: STM32 HAL (Hardware Abstraction Layer)
- **Python**: for filter design (`scipy` library)

## Filter Design

The FIR and IIR filters are designed using Python's `scipy` library. The design parameters are:

- **Sampling Frequency**: 2500 Hz
- **Band-Stop Filter**: Attenuates 50Hz (power grid noise)
- **Low-Pass Filter**: Attenuates frequencies above 500Hz

### Python Code for Filter Design

```python
import numpy as np
from scipy.signal import firwin, iirnotch, lfilter

# Sampling frequency and desired specs
fs = 2500.0  # Sampling frequency in Hz
f0 = 50.0  # Frequency to be removed from signal (Hz)
Q = 30.0  # Quality factor for the notch filter
cutoff = 500.0  # Low-pass cutoff frequency

# IIR notch filter for 50Hz
b_notch, a_notch = iirnotch(f0, Q, fs)

# FIR low-pass filter
numtaps = 101  # Filter order
b_lowpass = firwin(numtaps, cutoff / (0.5 * fs), window='hamming')

# Combine filters for final design
b_combined = np.convolve(b_lowpass, b_notch)
```

### Circular Buffer Initialization

```c
void CircularBuffer_Init(CircularBuffer* cb) {
    memset(cb->buffer, 0, sizeof(cb->buffer));
    cb->index = 0;
}
```


### FIR Filter Implementation

```c
float FIR_Filter(CircularBuffer* cb, float input) {
    float sum = 0.0f;

    // Update buffer with new input
    cb->buffer[cb->index] = input;
    cb->index = (cb->index + 1) % FILTER_ORDER;

    // Compute the FIR filter output
    for (int i = 0; i < FILTER_ORDER; i++) {
        int idx = (cb->index + i) % FILTER_ORDER; // Circular indexing
        sum += cb->buffer[idx] * fir_coefficients[i];
    }

    return sum;
}
```

