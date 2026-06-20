import serial, time
from protocol import pwm_servo_one
port='/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B32012490-if00'
frames=[
    pwm_servo_one(1, 900, time_ms=700),
    pwm_servo_one(1, 2100, time_ms=900),
]
ser=serial.Serial(port, 1000000, timeout=0, write_timeout=0.5)
try:
    while True:
        for frame in frames:
            ser.write(frame)
            ser.flush()
            time.sleep(1.0)
finally:
    try:
        ser.write(pwm_servo_one(1, 1500, time_ms=700))
        ser.flush()
    except Exception:
        pass
    ser.close()
