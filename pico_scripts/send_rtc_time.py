import machine, time

i2c = machine.I2C(0, sda=machine.Pin(0), scl=machine.Pin(1), freq=100000)
SLAVE_ADDR   = 0x13
CMD_SET_RTC  = 103

def set_stm32_rtc():
    # grab local time: (year,mon,day,hour,min,sec,weekday,yday)
    tm = time.localtime()
    year,mon,day,hour,minute,sec,wday,_ = tm
    # pack as big-endian: year_hi, year_lo, mon, day, weekday, hour, minute, sec
    payload = bytearray([
    CMD_SET_RTC,
    (year >> 8) & 0xFF,
    year & 0xFF,
    mon,
    day,
    wday+1,
    hour,
    minute,
    sec
    ])

#     # send command then payload
#     i2c.writeto(SLAVE_ADDR, bytes([CMD_SET_RTC]))
#     time.sleep_ms(10)   # allow STM to arm reception
    i2c.writeto(SLAVE_ADDR, payload)

    print("RTC set to %04d-%02d-%02d %02d:%02d:%02d" %
          (year,mon,day,hour,minute,sec))

# Example invocation
set_stm32_rtc()
