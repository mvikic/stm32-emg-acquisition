import sys
import logging
from queue import Queue
from PySide6.QtCore import QTimer, QIODeviceBase, QObject, Signal
from PySide6.QtSerialPort import QSerialPort
from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLabel
import pyqtgraph as pg


class SerialReader(QObject):
    # Signal emitted when new data is received
    data_received = Signal(float)

    def __init__(self, port, baudrate):
        super().__init__()
        self.serial_port = QSerialPort()
        self.serial_port.setPortName(port)
        self.serial_port.setBaudRate(baudrate)
        self.serial_port.readyRead.connect(self.read_serial_data)

        if not self.serial_port.open(QIODeviceBase.OpenModeFlag.ReadOnly):
            logging.error("Failed to open serial port!")
        else:
            logging.info(f"Opened serial port {port} at {baudrate} baud.")

    def read_serial_data(self):
        while self.serial_port.canReadLine():
            line = self.serial_port.readLine().data().decode('utf-8').strip()
            try:
                value = float(line)
                logging.debug(f"Received data: {value}")
                self.data_received.emit(value)
            except ValueError:
                logging.warning(f"Received invalid data: {line}")
                continue

    def close(self):
        self.serial_port.close()
        logging.info("Serial port closed.")


class SerialPlotter(QMainWindow):
    def __init__(self, reader, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.reader = reader
        self.reader.data_received.connect(self.on_data_received)

        self.data_queue = Queue()
        self.data = []

        # Set up the GUI
        self.setWindowTitle("EMG Data Plotter")
        self.setGeometry(100, 100, 800, 600)

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        layout = QVBoxLayout(self.central_widget)

        self.plot_widget = pg.PlotWidget()
        layout.addWidget(self.plot_widget)

        self.plot_data_line = self.plot_widget.plot([], [], pen=pg.mkPen(color='b', width=1))

        self.start_button = QPushButton("Start Plotting", self)
        self.start_button.clicked.connect(self.start_plotting)
        layout.addWidget(self.start_button)

        self.stop_button = QPushButton("Stop Plotting", self)
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_plotting)
        layout.addWidget(self.stop_button)

        self.status_label = QLabel("Status: Connected")
        layout.addWidget(self.status_label)

        # Timer to update plot
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)

        self.plotting = False

    def start_plotting(self):
        if not self.plotting:
            self.plotting = True
            self.timer.start(50)  # Update plot every 50 ms
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            logging.info("Plotting started.")

    def stop_plotting(self):
        self.plotting = False
        self.timer.stop()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        logging.info("Plotting stopped.")

    def on_data_received(self, value):
        """Handles new data received from the serial reader."""
        self.data_queue.put(value)
        logging.debug(f"Data put into queue: {value}")

    def update_plot(self):
        """Update the plot with new data from the queue."""
        # Process all data points available in the queue
        while not self.data_queue.empty():
            value = self.data_queue.get()
            self.data.append(value)

            # Limit the data to the last 500 points for plotting
            if len(self.data) > 4000:
                self.data.pop(0)

        if self.data:
            # Update the plot with new data
            self.plot_data_line.setData(range(len(self.data)), self.data)

            # Auto-scale the Y-axis based on the current data
            min_y = -0.002  # min(self.data)
            max_y = 0.002  # max(self.data)
            # min_y = min(self.data)
            # max_y = max(self.data)
            self.plot_widget.setYRange(min_y, max_y, padding=0.1)

            logging.debug(f"Plot updated with {len(self.data)} points.")

    # def update_plot(self):
    #     # Process all new data from the queue
    #     new_data = []
    #     while not self.data_queue.empty():
    #         new_data.append(self.data_queue.get())
    #
    #     if new_data:
    #         self.data.extend(new_data)
    #         self.data = self.data[-5000:]  # Keep only the last 500 data points
    #
    #         # Update the plot with new data
    #         self.plot_data_line.setData(range(len(self.data)), self.data)
    #
    #         # Auto-scale the Y-axis based on the current data
    #         min_y = min(self.data) if self.data else 0
    #         max_y = max(self.data) if self.data else 0
    #         self.plot_widget.setYRange(min_y, max_y, padding=0.1)
    #
    #         logging.debug(f"Plot updated with {len(new_data)} new points.")

    def closeEvent(self, event):
        self.reader.close()
        event.accept()
        logging.info("Application closed.")


def main():
    # Configure logging to display INFO and DEBUG level messages to the console
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s',
                        handlers=[logging.StreamHandler(sys.stdout)])

    app = QApplication(sys.argv)

    # Create the SerialReader and SerialPlotter
    # reader = SerialReader(port='COM7', baudrate=115200)
    reader = SerialReader(port='COM7', baudrate=460800)
    window = SerialPlotter(reader)
    window.show()

    try:
        sys.exit(app.exec())
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")


if __name__ == "__main__":
    main()
