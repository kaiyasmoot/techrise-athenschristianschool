
import time
import board
import busio
import digitalio
import storage
import adafruit_sdcard
import array
import math
from digitalio import DigitalInOut, Direction
from audiocore import RawSample
import adafruit_mcp9808
import trsim_worldview
import neopixel

try:
    from audioio import AudioOut
except ImportError:
    try:
        from audiopwmio import PWMAudioOut as AudioOut
    except ImportError:
        pass  # not always supported by every board!

print("Running the Audio Experiment by the TechRise Team at Athens Christian School.")

# Set up Neopixel hardware constants and object for the M4's on-board Neopixel
NEOPIXEL_PIN = board.NEOPIXEL
NEOPIXEL_COUNT = 1
NEOPIXEL_BRIGHTNESS = 0.2
pixels = neopixel.NeoPixel(NEOPIXEL_PIN, NEOPIXEL_COUNT, brightness=NEOPIXEL_BRIGHTNESS)

# Set up some basic color constants for use later
COLOR_RED = const(0xff0000)
COLOR_GREEN = const(0x00ff00)
COLOR_BLUE = const(0x0000ff)
COLOR_YELLOW = const(0xffff00)
COLOR_MAGENTA = const(0xff00ff)
COLOR_CYAN = const(0x00ffff)
COLOR_BLACK = const(0x000000)
COLOR_GRAY = const(0x7f7f7f)
COLOR_WHITE = const(0xffffff)

# Set up flight status event Neopixel colors index
status_colors = [None] * trsim_worldview.STATUS_NUM_EVENTS
status_colors[trsim_worldview.STATUS_UNKNOWN] = COLOR_GRAY
status_colors[trsim_worldview.STATUS_INITIALIZING] = COLOR_YELLOW
status_colors[trsim_worldview.STATUS_LAUNCHING] = COLOR_GREEN
status_colors[trsim_worldview.STATUS_FLOATING] = COLOR_CYAN
status_colors[trsim_worldview.STATUS_TERMINATING] = COLOR_BLUE


# Set up simulator data library
#   By default, PBF pin = 2, GO pin = 3. If your wiring is different, see
#   documentation for how to change pins using this function call.
TRsim = trsim_worldview.Simulator()

# Initialize flight status
curr_status = trsim_worldview.STATUS_UNKNOWN
prev_status = curr_status

# Display flight status (unknown) on Neopixel
pixels.fill(status_colors[curr_status])

movieCount = 0  # Movie counter for data
tone_volume = 1  # Increase this to increase the volume of the tone.
frequency = 329  # Set this to the Hz of the tone you want to generate.
length = 8000 // frequency
sine_wave = array.array("H", [0] * length)
for i in range(length):
    sine_wave[i] = \
        int((1 + math.sin(math.pi * 2 * i / length)) * tone_volume * (2 ** 15 - 1))
audio = AudioOut(board.A0)

sine_wave_sample = RawSample(sine_wave)
audio.play(sine_wave_sample, loop=True)

Camera = DigitalInOut(board.D12)
Camera.direction = Direction.OUTPUT

i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)


# Setup the SD CARD
# Use any pin that is not taken by SPI
SD_CS = board.D10
# Connect to the card and mount the filesystem.
spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
cs = digitalio.DigitalInOut(SD_CS)
sdcard = adafruit_sdcard.SDCard(spi, cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")


mcp = adafruit_mcp9808.MCP9808(i2c)
timeStart = time.time()

startTime = time.time()
RecordInterval = 10
RecordLength = 10
RecordingFlag = False
timer = time.time()

while True:

    # Check if the PBF header is closed (False). If it is, light Neopixel red.
    #   If it is open (True), we are free to do in-flight work!
    if TRsim.pbf is False:
        # Light the neopixel red to highlight that the PBF is inserted
        pixels.fill(COLOR_RED)
    else:
        # PBF header is open, we are flying and can do some work!

        # TRsim.streaming will be True while valid data is incoming
        # If data is paused for more than 1.5 seconds it will become False
        if TRsim.streaming:
            # TRsim.new_data will be True after a new packet arrives
            # When TRsim.data is called TRsim.new_data will become False
            if TRsim.new_data:
                # Got a new telemetry packet, let's count it!
                num_packets += 1

                # Grab new data - NOTE this sets new_data to False
                data = TRsim.data

                # You can add code here that needs to execute each time new
                #   telemetry data is received.

                # Get current flight status and check to see if it has changed
                curr_status = TRsim.status
                # See if the status has changed by comparing with previous value
                if curr_status != prev_status:
                    # Is it has changed, save the current status to prev_status
                    prev_status = curr_status
                    # Since the event changed, print something to indicate change.
                    # You can initiate activity for your payload in this
                    #   section, since it will only execute on status change.
                    #   However, when running the simulator, please note that
                    #   this section may execute again if you pause and unpause
                    #   the simulator.
                    if curr_status == trsim_worldview.STATUS_INITIALIZING:
                        print("We are initializing")
                    elif curr_status == trsim_worldview.STATUS_LAUNCHING:
                        print("We are launching")
                    elif curr_status == trsim_worldview.STATUS_FLOATING:
                        print("We are floating")
                        if not RecordingFlag and time.time()-timer > RecordInterval:

                            tempC = mcp.temperature

                            print(
                                TRsim.time + ",", \
                                TRsim.latitude + ",", \
                                TRsim.logitude + ",", \
                                TRsim.altitude + ",", \
                                TRsim.speed + ",", \
                                TRsim.heading + ",", \
                                TRsim.velocity_down + ",", \
                                TRsim.pressure + ",", \
                                TRsim.temperature)
                            FileName = "/sd/Movie" + str(movieCount) + ".txt"
                            with open(FileName, "a") as f:
                                f.write("Time" + "," + "Temperature" + "," + "Pressure\n")
                            with open(FileName, "a") as f:
                                f.write("{},{},{}\n".format(time.time()-timeStart, tempC, 1))

                            Camera.value = True
                            print("CAM ON")
                            timer = time.time()
                            RecordingFlag = True
                            movieCount = movieCount + 1

                        if RecordingFlag and time.time()-timer > RecordLength:

                            with open(FileName, "a") as f:
                                f.write("{},{},{}\n".format(time.time()-timeStart, tempC, TRsim.pressure))

                            Camera.value = False
                            print("CAM OFF")
                            RecordingFlag = False
                            timer = time.time()
                    elif curr_status == trsim_worldview.STATUS_TERMINATING:
                        print("We are terminating")
                    # Indicate the new event with a color from the status list
                    pixels.fill(status_colors[curr_status])

                # Print every 100th packet to verify data
                if (num_packets % 100) == 1:
                    TRsim.print_current_packet()

