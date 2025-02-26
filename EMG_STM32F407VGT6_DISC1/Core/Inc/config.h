#ifndef CONFIG_H
#define CONFIG_H

// Define any constants or macros used across multiple files

// Timeout for UART transmission
#define TIMEOUT              100            // Transmit timeout in milliseconds

// Floating-point precision for conversions
#define FLOAT_PRECISION      100000         // Scale factor for floating-point conversions

// EMG Signal Parameters
#define EMG_MAX_VOLTAGE      0.0005f        // 5 mV (maximum EMG signal voltage)
#define REF_VOLTAGE          3.3f           // Reference voltage for ADC (adjust as needed, e.g., 3.3V)
#define ADC_MAX_VAL          4095.0f        // Maximum value for 12-bit ADC
#define DC_BIAS              1.15f          // DC bias in volts (adjust based on your setup)

// IIR Filter Configuration
#define N_A                  8              // Number of feedback (denominator) coefficients (a[])
#define N_B                  8              // Number of feedforward (numerator) coefficients (b[])

// FIR Filter Configuration
#define FILTER_ORDER         61             // Example filter order for FIR filter
#define BUFFER_SIZE          FILTER_ORDER   // Size of the buffer for FIR filter

// Debounce Configuration
#define DEBOUNCE_DELAY       50             // Debounce delay in milliseconds

#endif // CONFIG_H
