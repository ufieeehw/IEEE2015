import serial
import time

class Tester(object):
    def __init__(self, port):
        self.serial = serial.Serial(port=port)

    def _read_loop(self):
        while(1):
            self.serial.write(chr(0xC0))
            self.serial.write(chr(0x02))
            self.serial.write('aa')
            print 'Reading:', ord(self.serial.read())
            time.sleep(0.5)

    def _write_loop(self):
        pass

if __name__ == '__main__':
    tester = Tester('/dev/COM1')
    tester._read_loop()