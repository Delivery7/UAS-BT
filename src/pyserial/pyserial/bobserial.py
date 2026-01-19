import serial
import time

class ArduinoSerial:
    def __init__(self, port, baudrate, timeout):
        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)

    def write(self, data):
        """Write data to Arduino."""
        if self.arduino.is_open:  # Check if the serial port is open
            try:
                self.arduino.write(bytes(data, 'utf-8'))
                time.sleep(0.05)  # Wait for the Arduino to process the data
            except serial.SerialException as e:
                print(f"Error writing to serial: {e}")
        else:
            print("Serial port is not open.")

    def read(self):
        """Read data from Arduino."""
        if self.arduino.is_open:  # Check if the serial port is open
            try:
                data = self.arduino.readline()  # Read a line from the serial
                return data.decode('utf-8').strip()  # Decode and strip whitespace/newlines
            except serial.SerialException as e:
                print(f"Error reading from serial: {e}")
                return None
        else:
            print("Serial port is not open.")
            return None

    def close(self):
        """Close the serial port."""
        if self.arduino.is_open:
            self.arduino.close()
            print("Serial port closed.")
        else:
            print("Serial port is already closed.")
