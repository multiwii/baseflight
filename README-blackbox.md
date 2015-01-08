# Blackbox flight data recorder

![Rendered flight log frame](http://i.imgur.com/FBphB8c.jpg)

## Introduction
This is a modified version of Baseflight for the Naze32 which adds a flight data recorder function ("Blackbox"). Flight
data information is transmitted over the Naze32's serial port on every Baseflight loop iteration to an external logging
device to be recorded.

Please download the .hex firmware file for the Naze32 on the "releases" tab at the top of the page.

After your flight, you can process the resulting logs on your computer to either turn them into CSV (comma-separated
values) or render your flight log as a video using the tools `blackbox_decode` and `blackbox_render`. Those tools can be
found in this repository:

https://github.com/cleanflight/blackbox-tools

## Logged data
The blackbox records flight data on every iteration of the flight control loop. It records the current time in
microseconds, P, I and D corrections for each axis, your RC command stick positions (after applying expo curves),
gyroscope data, accelerometer data (after your configured low-pass filtering), barometer readings, 3-axis magnetometer
readings, raw VBAT measurements, and the command being sent to each motor speed controller. This is all stored without
any approximation or loss of precision, so even quite subtle problems should be detectable from the fight data log.

Currently, the blackbox attempts to log GPS data whenever new GPS data is available, but this has not been tested yet.
The CSV decoder and video renderer do not yet show any of the GPS data (though this will be added). If you have a working
GPS, please send in your logs so I can get the decoding implemented.

The data rate for my quadcopter using a looptime of 2400 is about 10.25kB/s. This allows about 18 days of flight logs
to fit on a 16GB MicroSD card, which ought to be enough for anybody :).

## Supported configurations

The maximum data rate for the flight log is fairly restricted, so anything that increases the load can cause the flight
log to drop frames and contain errors.

The Blackbox was developed and tested on a quadcopter. It has also been tested on a tricopter. It should work on
hexacopters or octocopters, but as they transmit more information to the flight log (due to having more motors), the 
number of dropped frames may increase. The `blackbox_render` tool only supports tri and quadcopters (please send me 
flight logs from other craft, and I can add support for them!)

Baseflight's `looptime` setting will decide how many times per second an update is saved to the flight log. The
software was developed on a craft with a looptime of 2400. Any looptime smaller than this will put more strain on the
data rate. The default looptime on Baseflight is 3500. If you're using a looptime of 2000 or smaller, you will probably
need to reduce the sampling rate in the Blackbox settings, see the later section on configuring the Blackbox feature for
details.

## Hardware

The blackbox software is designed to be used with an [OpenLog serial data logger][] and a microSDHC card. You need a
little prep to get the OpenLog ready for use, so here are the details:

### Firmware

The OpenLog ships with standard OpenLog 3 firmware installed. However, in order to reduce the number of dropped frames,
it should be reflashed with the [OpenLog Light firmware][] or the special [OpenLog Blackbox firmware][] . The Blackbox
variant of the firmware ensures that the OpenLog is running at the correct baud-rate and does away for the need for a 
`CONFIG.TXT` file to set up the OpenLog.

You can find the Blackbox version of the OpenLog firmware [here](https://github.com/cleanflight/blackbox-firmware), 
along with instructions for installing it onto your OpenLog.

[OpenLog serial data logger]: https://www.sparkfun.com/products/9530
[OpenLog Light firmware]: https://github.com/sparkfun/OpenLog/tree/master/firmware/OpenLog_v3_Light
[OpenLog Blackbox firmware]: https://github.com/cleanflight/blackbox-firmware

### Serial port
Connect the "TX" pin from the two-pin TX/RX header on the center of the Naze32 to the OpenLog's "RXI" pin. Don't
connect the Naze32's RX pin to the OpenLog.

The OpenLog accepts input power voltages from 3.3 to 12V, so if you're powering the Naze32 with something like 5 volts
from a BEC, you can connect the VCC and GND pins on the OpenLog to one of the Naze32's spare motor headers in order to
power it.

### microSDHC

Your choice of microSDHC card is very important to the performance of the system. The OpenLog relies on being able to
make many small writes to the card with minimal delay, which not every card is good at. A faster SD-card speed rating is
not a guarantee of better performance.

#### microSDHC cards known to have poor performance

 - Generic 4GB Class 4 microSDHC card - the rate of missing frames is about 1%, and is concentrated around the most
   interesting parts of the log!

#### microSDHC cards known to have good performance

 - Transcend 16GB Class 10 UHS-I microSDHC (typical error rate < 0.1%)
 - Sandisk Extreme 16GB Class 10 UHS-I microSDHC (typical error rate < 0.1%)

You should format any card you use with the [SD Association's special formatting tool][] , as it will give the OpenLog
the best chance of writing at high speed. You must format it with either FAT, or with FAT32 (recommended).

[SD Association's special formatting tool]: https://www.sdcard.org/downloads/formatter_4/

### OpenLog configuration

This section applies only if you are using the OpenLog or OpenLog Light original firmware on the OpenLog. If you flashed
it with the special OpenLog Blackbox firmware, you don't need to configure it further.

Power up the OpenLog with a microSD card inside, wait 10 seconds or so, then power it down and plug the microSD card
into your computer. You should find a "CONFIG.TXT" file on the card. Edit it in a text editor to set the first number
(baud) to 115200. Set esc# to 0, mode to 0, and echo to 0. Save the file and put the card back into your OpenLog, it
should use those settings from now on.

If your OpenLog didn't write a CONFIG.TXT file, create a CONFIG.TXT file with these contents and store it in the root
of the MicroSD card:

```
115200,26,0,0,1,0,1
baud,escape,esc#,mode,verb,echo,ignoreRX
```

### Protection

The OpenLog can be wrapped in black electrical tape or heat-shrink in order to insulate it from conductive frames (like
carbon fiber), but this makes its status LEDs impossible to see. I recommend wrapping it with some clear heatshrink
tubing instead.

![OpenLog installed](http://i.imgur.com/jYyZ0oC.jpg "OpenLog installed in my frame with double-sided tape, SDCard slot pointing outward")

## Installation of firmware
Before installing the new firmware onto your Naze32, back up your configuration: Connect to your flight controller
using the [Baseflight Configurator][], open up the CLI tab and enter "dump" into the box at the bottom and press enter. Copy
all of the text that results and paste it into a text document somewhere for safe-keeping.

Click the disconnect button, then on the main page choose the Firmware Flasher option. Tick the box for "Full Chip
Erase" (warning, this will erase all your settings!). Click the "Load firmware (local)" button, and select the .hex
file you downloaded from the releases tab above. Click the "Flash Firmware" button and wait for it to complete.

Now you need to reload your configuration: Go to the CLI tab and paste in the dump that you saved earlier and press
enter, it should execute and restore your settings.

Before you leave the CLI tab, enable the Blackbox feature by typing in `feature BLACKBOX` and pressing enter. Leave the
CLI tab and your configuration should be saved and the flight controller will reboot. You're ready to go!

If you ever need to disable the Blackbox (say, for example, to switch to using the serial port for an OSD instead), you
can either reflash the stock Baseflight firmware using the Configurator, or you can just turn off the Blackbox feature
by entering `feature -BLACKBOX` on the CLI tab.

[Baseflight Configurator]: https://chrome.google.com/webstore/detail/baseflight-configurator/mppkgnedeapfejgfimkdoninnofofigk?hl=en

## Configuring the Blackbox firmware
The Blackbox currently provides two settings (`blackbox_rate_num` and `blackbox_rate_denom`) that allow you to control 
the rate at which data is logged. These two together form a fraction (`blackbox_rate_num / blackbox_rate_denom`) which
decides what portion of the flight controller's control loop iterations should be logged. The default is 1/1 which logs 
every iteration.

If you are using a short looptime like 2000 or faster, or you're using a slower MicroSD card, you will need to reduce
this rate to reduce the number of corrupted logged frames. A rate of 1/2 is likely to work for most craft.

You can change these settings by entering the CLI tab in the Baseflight Configurator and using the `set` command, like so:

```
set blackbox_rate_num = 1
set blackbox_rate_denom = 2
```

## Usage
The Blackbox starts recording data as soon as you arm your craft, and stops when you disarm. Each time the OpenLog is
power-cycled, it begins a fresh new log file. If you arm and disarm several times without cycling the power (recording
several flights), those logs will be combined together into one file. The command line tools will ask you to pick which
one of these flights you want to display/decode.

If your craft has a buzzer attached, a short beep will be played when you arm. You can later use this beep to
synchronize your recorded flight video with the rendered flight data log (the beep is shown as a blue line in the flight
data log, which you can sync against the beep in your recorded audio track).

The OpenLog requires a couple of seconds of delay after connecting the battery before it's ready to record, so don't
arm your craft immediately after connecting the battery (you'll probably be waiting for the flight controller to become
ready during that time anyway!)

You should also wait a few seconds after disarming the quad to allow the OpenLog to finish saving its data.

Don't insert or remove the SD card while the OpenLog is powered up.

## Converting logs to CSV or PNG
After your flights, you'll have a series of files labeled "LOG00001.TXT" etc. on the microSD card. You'll need to
decode these with the `blackbox_decode` tool to create a CSV (comma-separated values) file for analysis, or render them
into a series of PNG frames with `blackbox_render` tool, which you could then convert into a video using another 
software package.

You'll find those tools along with instructions for using them in this repository:

https://github.com/cleanflight/blackbox-tools

## License

This project is licensed under GPLv3. Both binary and source builds are derived from Baseflight 
https://github.com/multiwii/baseflight (GPLv3) 
