#ifndef CONFIG_H
#define CONFIG_H

// Define any constants or macros used across multiple files
#define TIMEOUT              100            // Transmit timeout
#define FLOAT_PRECISION      100000         // Precision for floating-point conversion

#define EMG_SIGNAL_MAX_VOLTAGE 0.0005f     // 5 mV (example maximum EMG signal voltage)
#define REF_VOLTAGE          5.0f           // Reference voltage for ADC (should be 3.3V)
#define ADC_MAX_VAL          4095.0f        // Maximum value for 12-bit ADC
#define DC_BIAS              1.83f          // DC bias in volts (should see which is new, ideally 1.65)
#define ALPHA                0.1f           // Smoothing factor for IIR filter
#define FILTER_ORDER         41              // Example filter order
#define BUFFER_SIZE          FILTER_ORDER    // Size of the buffer for FIR filter
#define DEBOUNCE_DELAY       50              // Debounce delay in milliseconds

#endif // CONFIG_H
