"""
A module to read World View flight telemetry data from the Future Engineers TechRise flight simulator.
Data is read over UART (RX pin on most CircuitPython boards).
Defines a Simulator class with properties to determine when a new packet is available.
Helper property functions are included to convert from telemetry format to usable values.

World View data format:
    * Transmitted via serial connection: 9600 baud, 8N1
    * 38-byte data packet, little endian
    * 1Hz telemetry packets

Data fields:
    Header (2):
        | uint8[2]  *Sync Word*, 0x71 [0x0F, 0xA7, 0xF0, 0xFF], 0xFF = data packet, others are events
    Payload (32):
        | float32   *Latitude*, in degrees
        | float32   *Longitude*, in degrees
        | float32   *Altitude*, in meters
        | float32   *Speed*, in meters/second
        | float32   *Heading*, in degrees
        | float32   *VelocityD*, in meters/second
        | float32   *Pressure*, in pascals
        | float32   *Temperature*, in celsius
    Timestamp (2):
        | uint16    *Timestamp*, elapsed time, [12..15] hours, [6..11] minutes, [0..5] seconds
    CRC (2):
        | uint16    *CRC*, checksum

"""

# Written by Mark DeLoura and Arnie Martin for Future Engineers
# Last Updated: 2/12/2023

import time
import board
import busio
import digitalio
import binascii
import struct

# Internal constants for serial communication
_BUFFER_SIZE = const(1024)
_BUFFER_SIZE_MASK = const(0x3ff)
_RECEIVER_BUFFER_SIZE = const(512)
_PACKET_SIZE = const(38)
_DEFAULT_BAUD_RATE = const(9600)
_UART_TIMEOUT = 0.1

# Packet start bytes and state
_SYNCBYTE1 = const(0x71)
_SYNCBYTE2CMDA = const(0x0F)
_SYNCBYTE2CMDB = const(0xA7)
_SYNCBYTE2CMDC = const(0xF0)
_SYNCBYTE2DATA = const(0xFF)
_MODE_FINDING_SYNCBYTE1 = const(1)
_MODE_FINDING_SYNCBYTE2 = const(2)
_MODE_FINDING_PAYLOAD   = const(3)

# Default PBF and GO pins
_DEFAULT_PBF_PIN = board.D2
_DEFAULT_GO_PIN  = board.D3

# Flight status constants
STATUS_NUM_EVENTS = const(5)
STATUS_UNKNOWN = const(0)
STATUS_INITIALIZING = const(1)
STATUS_LAUNCHING = const(2)
STATUS_FLOATING = const(3)
STATUS_TERMINATING = const(4)

class Simulator:
    """Simulator class - Conducts World View flight telemetry processing

       | :py:meth:`Simulator.update` - Processes incoming flight telemetry data
       | :py:attr:`Simulator.new_data` - True if new data has been received
       | :py:attr:`Simulator.streaming` - True if data is streaming
       | :py:attr:`Simulator.pbf` - Value of Pull-Before-Flight Header, True = removed
       | :py:attr:`Simulator.go` - Value of the GO LED, True = lit
       | :py:attr:`Simulator.data` - Value of the new full data packet as a bytearray

    Use .data to acquire all telemetry fields of the current packet.
    To acquire individual data fields from the current packet, use the helper properties detailed below.

    """

    def __init__(self, pbf_pin=_DEFAULT_PBF_PIN, go_pin=_DEFAULT_GO_PIN):
        """Initializes the simulator

        :param pbf_pin is a board.XX pin connected to the PBF pin on PIB J5
        :param go_pin is a board.XX pin connected to the GO pin on PIB J5
        """

        # declare private variables
        self._new_data = False
        self._data = None
        self._streaming = False
        self._stream_timeout = 0
        self._uart_pre = None
        self._pbf_pin = pbf_pin
        self._go_pin = go_pin
        self._pbf_state = False

        self._buffer = bytearray(_BUFFER_SIZE)
        self._mv = memoryview(self._buffer)

        self._curr_packet_buffer = bytearray(_PACKET_SIZE)
        self._curr_packet_buffer_mv = memoryview(self._curr_packet_buffer)

        self._return_buffer = bytearray(_PACKET_SIZE)

        self._is_command_packet = False
        self._flight_status = STATUS_UNKNOWN

        self._mode = _MODE_FINDING_SYNCBYTE1
        self._save_start = 0
        self._save_end = 0
        self._packet_start = 0
        self._packet_end = 0
        self._packet_bytes_found = 0

        # set the onboard UART pin
        self._uart = busio.UART(board.TX, board.RX,
                          baudrate=_DEFAULT_BAUD_RATE, timeout=_UART_TIMEOUT,
                          receiver_buffer_size=_RECEIVER_BUFFER_SIZE)

        # setup the pbf_pin and go_pin
        if self._pbf_pin is not _DEFAULT_PBF_PIN:
            print("Pull Before Flight header active on", self._pbf_pin)
        self._pbf = digitalio.DigitalInOut(self._pbf_pin)
        self._pbf.direction = digitalio.Direction.INPUT

        if self._go_pin is not _DEFAULT_GO_PIN:
            print("Go LED active on", self._go_pin)
        self._go = digitalio.DigitalInOut(self._go_pin)
        self._go.direction = digitalio.Direction.OUTPUT

        # print out the starting state of the PBF header
        self._pbf_state = self._pbf.value
        if self._pbf_state:
            print("Pull Before Flight header removed")
        else:
            print("Pull Before Flight header inserted")

    def _set_flight_status(self, eventbyte):
        """Internal helper function for processing event command packets"""

        if eventbyte == _SYNCBYTE2CMDA:
            self._flight_status = STATUS_LAUNCHING
        elif eventbyte == _SYNCBYTE2CMDB:
            self._flight_status= STATUS_FLOATING
        elif eventbyte == _SYNCBYTE2CMDC:
            self._flight_status = STATUS_TERMINATING

    def update(self):
        """Processes incoming flight telemetry data"""

        # Check for removal of PBF header
        #   True = removed, so new value is True, stored state is False
        if self._pbf.value and not self._pbf_state:
            # flash the GO led a few times
            # go_pin will be set according to the state of the pbf_pin when TRsim.update() is called
            print("Pull Before Flight header has been removed")
            for x in range(5):
                self._go.value = not self._go.value
                time.sleep(0.5)
        self._pbf_state = self._pbf.value

        # Light Go LED if the PBF header is inserted
        if self._pbf_state:
            # it is not, so turn on the go led
            self._go.value = True
        else:
            # it must be so turn off the go led
            self._go.value = False

        # Process waiting telemetry bytes
        bytes_waiting = self._uart.in_waiting
        if bytes_waiting > 0:
            first_packets = 0
            is_wrapped_load = False
            is_wrapped_packet = False

            # Grab self vars to improve performance
            mv = self._mv
            curr_mv = self._curr_packet_buffer_mv
            uart = self._uart
            mode = self._mode
            save_start = self._save_start
            save_end = self._save_end
            packet_start = self._packet_start
            packet_end = self._packet_end
            packet_bytes_found = self._packet_bytes_found

            # Copy waiting bytes to ring buffer, from saveStart to saveEnd
            #   Wrap ring buffer if load goes past end of buffer
            save_end = save_start + bytes_waiting
            is_wrapped_load = (save_end >= _BUFFER_SIZE)
            if is_wrapped_load:
                first_packets = _BUFFER_SIZE - save_start
                mv[save_start:_BUFFER_SIZE] = uart.read(first_packets)
                mv[0:save_end & _BUFFER_SIZE_MASK] = \
                    uart.read(bytes_waiting - first_packets)
            else:
                mv[save_start:save_end] = uart.read(bytes_waiting)

            # Process new bytes one by one
            #   Advance through modes based on data:
            #     MODE_FINDING_SYNCBYTE1 - looking for 0x71 packet start
            #     MODE_FINDING_SYNCBYTE2 - looking for 0xFF, second data packet byte
            #     MODE_FINDING_PAYLOAD   - looking for rest of 38 byte packet
            #   When full packet found, copy to currMV. Deal with wrap if necessary.
            save_range = range(save_start, save_end)
            for index in save_range:
                wrapped_index = index & _BUFFER_SIZE_MASK

                if mode == _MODE_FINDING_SYNCBYTE1:
                    if mv[wrapped_index] == _SYNCBYTE1:
                        mode = _MODE_FINDING_SYNCBYTE2
                        packet_start = wrapped_index

                elif mode == _MODE_FINDING_SYNCBYTE2:
                    if mv[wrapped_index] == _SYNCBYTE2DATA:
                        mode = _MODE_FINDING_PAYLOAD
                        self._is_command_packet = False
                        packet_bytes_found = 2
                    elif (mv[wrapped_index] == _SYNCBYTE2CMDA) or \
                        (mv[wrapped_index] == _SYNCBYTE2CMDB) or \
                        (mv[wrapped_index] == _SYNCBYTE2CMDC):
                        mode = _MODE_FINDING_PAYLOAD
                        self._is_command_packet = True
                        packet_bytes_found = 2
                    else: # if SYNCBYTE2 is not immediately after SYNCBYTE1, start over
                        mode = _MODE_FINDING_SYNCBYTE1

                elif mode == _MODE_FINDING_PAYLOAD:
                    packet_bytes_found = packet_bytes_found + 1
                    if packet_bytes_found == _PACKET_SIZE:     # Full packet found
                        packet_end = wrapped_index
                        is_wrapped_packet = ((packet_end+1) -
                            _PACKET_SIZE < 0)
                        if is_wrapped_packet:
                            first_packets = _BUFFER_SIZE - packet_start
                            curr_mv[0:first_packets] = \
                                mv[packet_start:_BUFFER_SIZE]
                            curr_mv[first_packets:_PACKET_SIZE] = \
                                mv[0:packet_end+1]
                        else:
                            curr_mv[0:_PACKET_SIZE] = \
                                mv[packet_start:packet_end+1]

                        # We have a full packet
                        #   If it's a command packet, ignore payload and set event
                        #   If it's a data packet, set _new_data
                        #     If time == 1, set flight status to INITIALIZING
                        if self._is_command_packet:
                            self._set_flight_status(curr_mv[1])
                        else:
                            self._new_data = True
                            if self.time == 1:
                                self._flight_status = STATUS_INITIALIZING

                        self._stream_timeout = time.monotonic() + 1.5
                        mode = _MODE_FINDING_SYNCBYTE1
                else:
                    raise Exception('UART data state machine error')

            save_start = save_end & _BUFFER_SIZE_MASK

            # Save state back to self
            self._mode = mode
            self._save_start = save_start
            self._save_end = save_end
            self._packet_start = packet_start
            self._packet_end = packet_end
            self._packet_bytes_found = packet_bytes_found

    @property
    def new_data(self) -> bool:
        """Returns True if new data has been received"""

        if self._new_data:
            return True
        else:
            return False

    @property
    def streaming(self) -> bool:
        """Returns True if data is streaming, and recieve timeout has not been exceeded"""

        if time.monotonic() <= self._stream_timeout:
            return True
        else:
            return False

    @property
    def pbf(self) -> bool:
        """Returns value of the Pull-Before-Flight Header, True if Header removed"""

        if self._pbf_pin is not None:
            return self._pbf.value
        else:
            return None

    @property
    def go(self) -> bool:
        """Returns value of the GO LED"""

        if self._go_pin is not None:
            return self._go.value
        else:
            return None

    @go.setter
    def go(self, val:bool):
        """Sets GO pin

        :param val: Value for GO LED
        :type val: bool
        """

        if self._go_pin is not None:
            if isinstance(val, bool):
                self._go.value = val
            else:
                raise Exception(".go value must be True/False")
        else:
            raise Exception("Go pin must be declared in TRsim initializer")

    @property
    def data(self) -> bytearray:
        """Returns the new full data packet as a bytearray"""

        if self._new_data:
            self._new_data = False
            self._return_buffer[:] = self._curr_packet_buffer
            return self._return_buffer
        else:
            return None

    @property
    def status(self) -> int:
        """Returns flight status, as a constant int"""

        if self.streaming:
            return self._flight_status
        else:
            return None

    @property
    def time(self) -> int:
        """Returns timestamp, in seconds"""

        if self.streaming:
            intval = ((self._curr_packet_buffer_mv[35]<<8)| \
                        (self._curr_packet_buffer_mv[34]))
            retval = ((intval >> 12) & 0xf) * 3600 + \
                        ((intval >> 6) & 0x3f) * 60 + \
                        (intval & 0x3f)
            return retval
        else:
            return None

    @property
    def latitude(self) -> float:
        """Returns latitude, in degrees"""

        if self.streaming:
            retval = struct.unpack('<f', self._curr_packet_buffer_mv[2:6])[0]
            return retval
        else:
            return None

    @property
    def longitude(self) -> float:
        """Returns longitude, in degrees"""

        if self.streaming:
            retval = struct.unpack('<f', self._curr_packet_buffer_mv[6:10])[0]
            return retval
        else:
            return None

    @property
    def altitude(self) -> float:
        """Returns altitude, in meters"""

        if self.streaming:
            retval = struct.unpack('<f', self._curr_packet_buffer_mv[10:14])[0]
            return retval
        else:
            return None

    @property
    def speed(self) -> float:
        """Returns speed, signed, in meters/second"""

        if self.streaming:
            retval = struct.unpack('<f', self._curr_packet_buffer_mv[14:18])[0]
            return retval
        else:
            return None

    @property
    def heading(self) -> float:
        """Returns heading, signed, in degrees"""

        if self.streaming:
            retval = struct.unpack('<f', self._curr_packet_buffer_mv[18:22])[0]
            return retval
        else:
            return None

    @property
    def velocity_down(self) -> float:
        """Returns velocity down, signed, in meters/second"""

        if self.streaming:
            retval = struct.unpack('<f', self._curr_packet_buffer_mv[22:26])[0]
            return retval
        else:
            return None

    @property
    def pressure(self) -> float:
        """Returns pressure, in pascals"""

        if self.streaming:
            retval = struct.unpack('<f', self._curr_packet_buffer_mv[26:30])[0]
            return retval
        else:
            return None

    @property
    def temperature(self) -> float:
        """Returns temperature, in celsius"""

        if self.streaming:
            retval = struct.unpack('<f', self._curr_packet_buffer_mv[30:34])[0]
            return retval
        else:
            return None

    def print_current_packet(self):
        """Prints current packet information to serial"""
        print(binascii.hexlify(self._return_buffer))
        print('  time', self.time)
        print('  latitude', '%.3f' % self.latitude)
        print('  longitude', '%.3f' % self.longitude)
        print('  altitude', '%.3f' % self.altitude)
        print('  speed', '%.3f' % self.speed)
        print('  heading', '%.3f' % self.heading)
        print('  velocity down', '%.3f' % self.velocity_down)
        print('  pressure', '%.3f' % self.pressure)
        print('  temperature', '%.3f' % self.temperature)
