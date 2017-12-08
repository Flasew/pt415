# Serial interface to Agilent TPS vacuum pump for monitoring pressure
# Author: Weiyang Wang (Previous code available from Alex Zahn)
# Date : Dec 8, 2017
from __future__ import print_function

import sys
from sys import stderr
from datetime import datetime
import serial
import threading
import binascii
import time
import struct
import signal

# Constants for message parsing, etc.
STX = '\x02'
CMD = '\x80'
CR = '\x0D'
DATA_WRITE = '\x61'
DATA_READ = '\x63'
ESC = '\x07'
ESC_STX = '\x30'
ESC_CR = '\x31'
ESC_ESC = '\x32'

# Debug mode. Will be VERY verbose if enabled
DEBUG = False

# Enable print to stderr
def eprint(*args, **kwargs):
    print(*args, file=stderr, **kwargs)

# debug print
def dprint(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)

# PT415 class. Actually a controller of the CRYOMECH CP-1000 or 2800 compressor.
class PT415:
    def __init__(self, port, baudrate=115200, devaddr=16, timeout=1, retries=3):
        """ Initializer. Initializes a PT415 controller connected to @port, with
        specified parameters.

        Arguments: 
            port {str} - serial port address (under /dev) that the pump is 
                         connected to.
            baudrate {int} - baudrate of the serial port. Need to match the 
                             setting of the pump.
            devaddr {int} - device address of the pump, if using RS485 mode.
                            for RS232 connection this number need to be set 
                            to 16 (0x10).
            timeout {float} - serial port time out.
            retries {int} - retries before reporting a command failed. Sometimes
                            a command could fail to be send on the first trail
                            but will succeed on the second, this number specifies
                            how many time should the program try before finally
                            report fail.
            """
        if devaddr < 16 or devaddr > 254:
            raise ValueError("Invalid device address")

        self.addr = struct.pack('B', devaddr)
        self.port = port
        self.ser = serial.Serial(port,
                baudrate,
                timeout=timeout,
                parity=serial.PARITY_NONE,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE)
        if not self.ser.isOpen():
            self.ser.open()
        self.retries = retries

        self.log_sig = False # for stop log signaling 

    def set_signal(self):
        """For signal (SIGINT, etc.) handling
        """
        eprint("Signal caught, ending log...")
        self.log_sig = True


    def _readline(self):
        """Read a line from self.ser until a CR character is hit"""

        eol = b'\r'
        leneol = len(eol)
        line = bytearray()
        while True:
            c = self.ser.read(1)
            if c:
                line += c
                if line[-leneol:] == eol:
                    break
            else:
                break
        return bytes(line)

    def runtime(self):
        """Returns the runtime of the pump controller."""
        return self.readvar('\x45\x4C',0)

    def status(self):
        """Returns the status of the pump controller."""
        return self.readvar('\x5F\x95',0)

    def start(self):
        """Start the pump controller immediately."""
        return self.writevar('\xD5\x01',0,1)

    def stop(self):
        """Stop the pump controller immediately."""
        return self.writevar('\xC5\x98',0,1)

    def checksum(self,msg):
        """Returns the checksum of a message
        Arguments:
            msg {str} - message string (contains special characters)"""
        cksum = sum([ord(x) for x in msg])
        cksum0 = ((cksum & 0xF0) >> 4) + 0x30
        cksum1 = (cksum & 0x0F) + 0x30
        return chr(cksum0)+chr(cksum1)

    def makePacket(self,dhash,index,val=None):
        """Make a message packet from the given parameter that the pump controller
        would accept. 
        Arguments:
            dhash {int} - hash code representing the command. See CP2800's manual
                          for all available hash codes.
            index {int} - index of the command. See manual for more information.
            val {int} - if the variable is to be written into the controller, 
                        this is the value to write. See manual for more detail.
        """
        msg = STX + self.addr + CMD
        if val is None:
            msgtype = DATA_READ
        else:
            msgtype = DATA_WRITE
        msg += msgtype
        payload = dhash + chr(index)
        if val is not None:
            payload += struct.pack('>I',val)
        table = {STX : ESC_STX, CR : ESC_CR, ESC : ESC_ESC}
        for i in range(len(payload)):
            if payload[i] in table:
                msg += ESC
                msg += table[payload[i]]
            else:
                msg += payload[i]
        cksum = self.checksum(self.addr+CMD+msgtype+payload)
        msg += cksum
        msg += CR
        return msg

    def rendermsg(self,msg):
        """Convert a message to a string of it's hex representation."""
        return ' '.join(['%02x'%ord(x) for x in msg])

    def deflatePacket(self,msg,reading=True):
        """Decode a returned packet. Runs checksum to check if the message if
        valid. If the original command were to read information from the controller
        this function will return the value read assume it's valid; if the
        original command were to write some value, this function will return 0 
        on success.
        Arguments:
            msg {str} - message to be decoded
            reading {bool} - if the original message was to read some value.
        """
        if len(msg) < 6:
            raise IOError("deflatePacket: packet too short!")

        begin = msg[:3]
        middle = msg[3:-3]
        end = msg[-3:]

        dprint('begin: ', self.rendermsg(begin))
        dprint('middle: ', self.rendermsg(middle))
        dprint('end: ', self.rendermsg(end))

        middle = middle.replace(ESC+ESC_STX,STX)
        middle = middle.replace(ESC+ESC_CR,CR)
        middle = middle.replace(ESC+ESC_ESC,ESC)

        msg = begin+middle+end

        cksum = self.checksum(msg[1:-3])
        if cksum[0] != msg[-3] or cksum[1] != msg[-2]:
            eprint("checksum: ",self.rendermsg(cksum))
            raise IOError("Bad checksum")
        if reading:
            data = struct.unpack('>I',msg[-7:-3])[0]
            return data
        else:
            return 0

    def readvar(self,code,index):
        """Wrapper of sending a reading command and return the value read.
        Arguments:
            code, index {int} - hash code and index of the command. See manual
                                for more details.
        """
        retries = self.retries

        msg = self.makePacket(code,index)
        self.ser.flushInput()

        while retries > 0:
            if self.ser.write(msg) != len(msg):
                eprint("Error sending message on trail {:d}."\
                    .format(self.retries-retries+1))
                continue

            ret = self._readline()
            dprint(self.rendermsg(ret))
            try:
                val = self.deflatePacket(ret)
                return val
            except IOError:
                eprint("Error receiving response on trail {:d}."\
                    .format(self.retries-retries+1))
                retries -= 1
                continue

        raise IOError("ERROR: All trails failed for readvar.")


    def writevar(self,code,index,val):
        """Wrapper of sending a writing command.
        Arguments:
            code, index {int} - hash code and index of the command. See manual
                                for more details.
        """
        retries = self.retries

        msg = self.makePacket(code,index,val)
        self.ser.flushInput()

        while retries > 0:
            if self.ser.write(msg) != len(msg):
                eprint("Error sending message on trail {:d}."\
                    .format(self.retries-retries+1))
                continue

            ret = self._readline()
            dprint(self.rendermsg(ret))
            try:
                val = self.deflatePacket(ret, reading = False)
                return val
            except IOError:
                eprint("Error receiving response on trail {:d}."\
                    .format(self.retries-retries+1))
                retries -= 1
                continue

        raise IOError("ERROR: All trails failed for writevar.")

    def log(self, interval=600, flag='w', filename=None):
        """Write logs into a file named @filename, with @interval seconds between
        each two entries

        If filename is None, a new file will be created with filename 
        "yyyymmddhhmmss.log", all number represent a time unit.

        if the flag argument is set to "a", the logger would append entries
        instead of overwrite. In this case filename should be specified,
        otherwise the behavior will be the same as "w".

        Arguments: 
            interval {number} - interval between two logs
            flag {char(str)} - flag of opening the file. Could be 'a' or 'w'.
                          With 'a', if the file specified by @filename already 
                          exist, logs will be appended to this file.
                          Otherwise, the function will create a new file and 
                          start writing. 
            filename {str} - file that the log will be write to. If is none,
                             will create a new file with name "yyyymmddhhmmss.log"
                             with the current time.
        """

        flag=flag.strip().lower()
        if flag not in ['a', 'w']:
            raise ValueError("Unsupported file opening flag!")

        if filename is None:
            filename = "{:s}.log".format(datetime.now().strftime("%Y%m%d%H%M%S"))

        eprint("Logging into file {:s}{:s}".format(filename, 
            "..." if flag == 'w' else ' with appending mode...'))

        with open(filename, flag) as f:
            if f.tell() == 0:
                f.write("Time, Unix Time, Elapsed time (min), Input H2O Temp (C), Output H2O Temp (C), Helium Temp (C), Oil Temp (C), High Side Pressure (PSIA), Low Side Pressure (PSIA)\n")
                f.flush()

            # signal handling.
            # stores the original signals
            original_sigint = signal.getsignal(signal.SIGINT)
            original_sighup = signal.getsignal(signal.SIGHUP)
            original_sigterm = signal.getsignal(signal.SIGTERM)

            # set the new signal handlers
            signal.signal(signal.SIGINT, lambda s, f: self.set_signal())
            signal.signal(signal.SIGHUP, lambda s, f: self.set_signal())
            signal.signal(signal.SIGTERM, lambda s, f: self.set_signal())

            while not self.log_sig:
                try:
                    now = time.strftime("\"%d %b %Y %H:%M:%S\"", time.localtime())
                    unix_now = int(time.time())

                    # elapsed time: minutes
                    elapsed = self.readvar('\x45\x4c', 0)

                    # input water temperature: converted from 0.1 degree C to degree C
                    inH2O_t = self.readvar('\x0d\x8f', 0)/10.0

                    # output water temperature: degree C
                    outH2O_t = self.readvar('\x0d\x8f', 1)/10.0

                    # helium temperature: degree C
                    he_t = self.readvar('\x0d\x8f', 2)/10.0

                    # Oil temperature: degree C
                    oil_t = self.readvar('\x0d\x8f', 3)/10.0

                    # High side pressure: PSIA
                    hi_p = self.readvar('\xaa\x50', 0)/10.0

                    # Low side pressure: PSIA
                    lo_p = self.readvar('\xaa\x50', 1)/10.0

                    # Diode volt: micro volts
                    # diode_v = self.readvar('\x8e\xea',0)
                    #
                    # # diode temperature: K
                    # # this is a more problematic one since two numbers are used
                    # diode_t_l, diode_t_h = self.readvar('\x58\x13', 0), self.readvar('\x58\x13', 1)
                    # diode_t = (diode_t_h << 8 + diode_t_l)/100

                    logstr = "{:s}, {:d}, {:d}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}\n".\
                            format(now, unix_now, elapsed, inH2O_t, outH2O_t, he_t, oil_t, hi_p, lo_p)
                    eprint(logstr, end='')
                    f.write(logstr)
                    f.flush()

                    time.sleep(interval)

                except IOError:
                    eprint("WARNING: status reading failed for a parameter. Retrying...")
                    time.sleep(1)
                    continue
            ### END WHILE 
            
            # restore the original handlers
            signal.signal(signal.SIGINT, original_sigint)
            signal.signal(signal.SIGHUP, original_sighup)
            signal.signal(signal.SIGTERM, original_sigterm)
            self.log_sig = False
            f.flush()
