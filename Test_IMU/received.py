import serial
import re
import sys

# 配置串口参数（根据实际情况修改 COM 端口和波特率）
SERIAL_PORT = 'COM6'      # Windows 示例，Linux 下如 '/dev/ttyUSB0'
BAUDRATE = 115200

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUDRATE} baud.")
    except serial.SerialException as e:
        print(f"Failed to open serial port: {e}")
        sys.exit(1)

    # 正则匹配格式: Acc: num, num, num | Gyro: num, num, num
    pattern = re.compile(
        r'Acc:\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+)\s*\|\s*Gyro:\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+)'
    )

    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            match = pattern.search(line)
            if match:
                ax, ay, az, gx, gy, gz = map(float, match.groups())
                print(f"Accel (g): x={ax:6.3f}, y={ay:6.3f}, z={az:6.3f}  |  Gyro (dps): x={gx:7.2f}, y={gy:7.2f}, z={gz:7.2f}")
            else:
                # 如果格式不匹配，直接打印原始行（便于调试）
                print(f"[RAW] {line}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()