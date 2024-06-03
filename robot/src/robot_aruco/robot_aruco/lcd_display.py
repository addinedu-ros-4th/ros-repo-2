import RPi.GPIO as GPIO
import time
import smbus2 as smbus

class LCDDisplay:
    def __init__(self):
        self.lcd_addr = 0x27

        self.bus = smbus.SMBus(1)
        self.initialize(self.lcd_addr, 1)

    # LCD initialize
    def initialize(self, addr, bl):
        self.lcd_addr = addr
        self.blen = bl
        try:
            self.send_command(0x33) # Must initialize to 8-line mode at first
            time.sleep(0.005)
            self.send_command(0x32) # Then initialize to 4-line mode
            time.sleep(0.005)
            self.send_command(0x28) # 2 Lines & 5*7 dots
            time.sleep(0.005)
            self.send_command(0x0C) # Enable display without cursor
            time.sleep(0.005)
            self.send_command(0x01) # Clear Screen
            self.bus.write_byte(self.lcd_addr, 0x08)
        except:
            return False
        else:
            return True
        
    def send_command(self, cmd):
        buf = cmd & 0xF0
        buf |= 0x04
        self.write_word(self.lcd_addr, buf)
        time.sleep(0.002)
        buf &= 0xFB
        self.write_word(self.lcd_addr, buf)
        buf = (cmd & 0x0F) << 4
        buf |= 0x04
        self.write_word(self.lcd_addr, buf)
        time.sleep(0.002)
        buf &= 0xFB
        self.write_word(self.lcd_addr, buf)

    def write_word(self, addr, data):
        temp = data
        if self.blen == 1:
            temp |= 0x08
        else:
            temp &= 0xF7
        self.bus.write_byte(addr, temp)

    def write(self, x, y, str):
        if x < 0:
            x = 0
        if x > 15:
            x = 15
        if y < 0:
            y = 0
        if y > 1:
            y = 1
        addr = 0x80 + 0x40 * y + x
        self.send_command(addr)
        for chr in str:
            self.send_data(ord(chr))

    def send_data(self, data):
        buf = data & 0xF0
        buf |= 0x05
        self.write_word(self.lcd_addr, buf)
        time.sleep(0.002)
        buf &= 0xFB
        self.write_word(self.lcd_addr, buf)
        buf = (data & 0x0F) << 4
        buf |= 0x05
        self.write_word(self.lcd_addr, buf)
        time.sleep(0.002)
        buf &= 0xFB
        self.write_word(self.lcd_addr, buf)
