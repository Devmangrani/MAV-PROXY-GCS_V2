import os
import random
import time

GREEN = '\033[92m'
RED = '\033[91m'
WHITE = '\033[97m'
RESET = '\033[0m'


def generate_random_number(min_val, max_val, decimal_places=3):
    return round(random.uniform(min_val, max_val), decimal_places)


def generate_random_color():
    return random.choice([RED, GREEN, WHITE])


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def print_header():
    print("╔════════════╦════════════╦════════════╦════════════╦════════════╗")
    print(f"║    CHI     ║    NCL     ║    CGV     ║    NFR     ║  NFI       ║")
    print("╠════════════╬════════════╬════════════╬════════════╬════════════╬")


def print_row(row):
    print(row)
    print("╠════════════╬════════════╬════════════╬════════════╬════════════╣")


def simulate_connection():
    connection_strings = [
        "Initializing multispectral camera connection...",
        "Detecting available ports...",
        "Port /dev/ttyUSB0 found. Attempting connection...",
        "Establishing serial connection at 115200 baud...",
        "Camera firmware version: v2.3.1",
        "Checking sensor calibration...",
        "Calibration data loaded successfully.",
        "Initializing spectral bands: Red, Green, Blue, NIR, RedEdge",
        "Setting exposure time to 10ms...",
        "Configuring gain settings...",
        "Enabling auto-exposure mode...",
        "Synchronizing internal clock with GPS time...",
        "Warming up sensors... Please wait.",
        "All systems nominal. Camera ready for operation."
    ]

    for line in connection_strings:
        print(line)
        time.sleep(1.5)

    print("Finalising connections to camera")
    time.sleep(10)
    print("\nConnection established. Initializing data stream...\n")
    input("Press Enter to start data collection...")
    time.sleep(1)


def generate_table():
    rows = []
    max_rows = 10

    try:
        while True:
            clear_screen()
            print_header()

            row_data = [
                generate_random_number(0, 100),
                generate_random_number(10, 25),
                generate_random_number(0, 1),
                generate_random_number(0, 100),
                generate_random_number(0, 100)
            ]

            row_str = "║║"
            for i, value in enumerate(row_data):
                color = generate_random_color()
                if i == 4:  # NFI column
                    row_str += f" {color} {value:^6.3f}{RESET}  ║║  "
                else:
                    row_str += f" {color} {value:^7.3f}{RESET} ║║ "

            row_str += f" {GREEN}OK{RESET} "

            rows.append(row_str)
            if len(rows) > max_rows:
                rows.pop(0)

            for row in rows:
                print_row(row)

            print("╚════════════╩════════════╩════════════╩════════════╩════════════╝")

            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nProgram terminated")


def main():
    simulate_connection()
    generate_table()


if __name__ == "__main__":
    main()
