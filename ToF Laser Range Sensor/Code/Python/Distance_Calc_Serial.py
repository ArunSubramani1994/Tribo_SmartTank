import serial
import serial.tools.list_ports
import struct
import re

def find_esp32_port():
    """Find the COM port for ESP32"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # ESP32 usually shows up as CP210x or CH340
        if "CP210" in port.description or "CH340" in port.description:
            return port.device
    return None

def create_gauge(value, width=50, max_value=500):
    """Create an ASCII gauge visualization"""
    value = min(value, max_value)
    filled_length = int(width * value / max_value)
    bar = '█' * filled_length + '░' * (width - filled_length)
    return f"[{bar}] {value:.1f} cm"

def main():
    # Find ESP32 port automatically
    port = find_esp32_port()
    if port is None:
        print("ESP32 not found! Please check the connection.")
        return
        
    baud_rate = 115200  # ESP32's default baud rate

    try:
        with serial.Serial(port, baudrate=baud_rate, timeout=1) as ser:
            print(f"Connected to ESP32 on {port} at {baud_rate} baud...")
            print("Distance Gauge (0-500 cm)")
            print("-" * 60)

            # Pattern to match distance values from ESP32's output
            pattern = r"Distance: (\d+\.\d+) cm"

            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    match = re.search(pattern, line)
                    if match:
                        distance = float(match.group(1))
                        gauge = create_gauge(distance)
                        print('\033[F\033[K' + gauge)
    
    except OSError as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
    except ModuleNotFoundError as e:
        print("The 'pyserial' module is not installed. Please install it using 'pip install pyserial'")

if __name__ == "__main__":
    main()
