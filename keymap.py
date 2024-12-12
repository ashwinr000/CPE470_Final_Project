import serial
from pynput.keyboard import Controller, Key

# Configure the serial port
SERIAL_PORT = 'COM9'  # Replace with your serial port
BAUD_RATE = 9600  # Replace with the baud rate used by your device

# Mapping of input words to arrow keys
COMMANDS = {
    "up": Key.up,
    "down": Key.down,
    "left": Key.left,
    "right": Key.right
}

def main():
    # Initialize the serial connection
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    # Initialize the keyboard controller
    keyboard = Controller()

    try:
        while True:
            # Read a line from the serial console
            line = ser.readline().decode('utf-8').strip()
            if line:
                print(f"Received: {line}")
                # Check if the input matches a command
                if line in COMMANDS:
                    # Simulate the corresponding arrow key press
                    key = COMMANDS[line]
                    print(f"Pressing: {line}")
                    keyboard.press(key)
                    keyboard.release(key)
                else:
                    print(f"Unknown command: {line}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()