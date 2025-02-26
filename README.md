# This Branch (seeed-mmwave-60ghz-homeassistant):

Is where I'm testing full Home Assistant device/sensor/binary_sensor functionality similar to another project where I used the Seeed Arduino example sketch and extended it to work as an MQTT Home Assistant integration.

## This Fork:

Focuses on using an ESP32 MCU which requires some changes to the Serial configuration and also I've added, or finished adding the code necessary to interpret not only movement but also presence and added both to the debug (serial monitor) output log.

I've tested this and it seems to work nicely using a Seeed Fall Detection module which is very similar other than some microcode to the work done here which is more specific to the Breathe / Heartrate module from Seeed.

```
Presence 0) Movement 0) HR (unknown) 0 RR (unknown) 0 at 0 angled 0 <-no one / no move
Presence 1) Movement 2) HR (unknown) 0 RR (unknown) 0 at 0 angled 0 <-someone / moving
Presence 1) Movement 1) HR (unknown) 0 RR (unknown) 0 at 0 angled 0 <-someone / still

```

### Plans for more:

Originally I was just looking for cleaner code which is what I've found thanks to AronRubin :) but now I'm likely going to use this instead of the example code from Seeed I originally used-to make a new Home Assistant integration via MQTT.

# Original - SeeedHeartBreathRadar - ReadMe from AronRubin:
## An Arduino compatible library to communicate with Seeed Studio's 60GHz mmWave Human Vitals radar

<div align=center><img width = 800 src="https://files.seeedstudio.com/wiki/60GHzradar/MR60BHA1.jpeg"/></div>

[Seeed 60GHz mmWave Radar Sensor - Breathing and Heartbeat Module](https://www.seeedstudio.com/60GHz-mmWave-Radar-Sensor-Breathing-and-Heartbeat-Module-p-5305.html?queryID=7812981435ed0d64d3021f0e03eaa2d8&objectID=5305&indexName=bazaar_retailer_products)

The MR60BHA1 60GHz radar module applies FMCW detected theory to implement simultaneous personal breathing rate and heart rate detention in high accuracy, providing a fully total private and secure environment, independently from other noisy influences. It is a standard biotic radar system in consumer electronics, healthcare as well as industrial applications.

The unit supports secondary developments and the improvable factors like small size, digital output, inside algorithm, allow it can be applied in a variety of scenarios applications by universal UART communication interface through a development board like Wio Terminal or the XIAO series.

For more information please visit [wiki](https://wiki.seeedstudio.com/Radar_MR60BHA1/).

This demo is licensed [under The MIT License](http://opensource.org/licenses/mit-license.php). Check LINCESE for more information.

Contributing to this software is warmly welcomed. You can do this basically by
[forking](https://docs.github.com/en/get-started/quickstart/fork-a-repo), committing modifications and then [pulling requests](https://help.github.com/articles/using-pull-requests) (follow the links above
for operating guide). Adding change log and your contact into file header is encouraged.
Thanks for your contribution.

Seeed Studio is an open hardware facilitation company based in Shenzhen, China.
Benefiting from local manufacture power and convenient global logistic system,
we integrate resources to serve new era of innovation. Seeed also works with
global distributors and partners to push open hardware movement.
