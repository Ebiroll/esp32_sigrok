# Sigrok esp 32 

A SUMP compatible Logical Analyser for the esp32, for use with i.e sigrok
https://en.wikipedia.org/wiki/Sigrok

Pins defined for logical input.

```
            12,
            13,
            14,
            15,
```

The key to using the ESP32 with sigrok was using the WROVER-kit. :-P
It was not the code in this repository... Accidental success. 
I knew there was something strange when the code was working on the first try. It is thanks to the JTAG, chips on the WROVER-KIT board that data acquisition was working so well.

In this code the sump protocol is implemented over USB and tcp/ip. 

When starting pulseview with logging,  -l 5 it seems that the FTDI-LA drivers were used.

https://sigrok.org/gitweb/?p=libsigrok.git;a=tree;f=src/hardware/ftdi-la
For me it solves the problem and allows sampling without a second ESP32 up to 10Mhz.

I found that terminal program cutecom allowed sending single HEX bytes, this is useful for debugging the implementation of the SUMP protocol.


In sigrok the following WROVER-KIT pins are mapped like this.
```
PIN13 = ADBUS0
PIN12 = ADBUS1
PIN15 = ADBUS2
PIN14 = ADBUS3
```


THE Jtag interface uses the following pins.
```
1 	CHIP_PU 	TRST_N
2 	MTDO / GPIO15 	TDO
3 	MTDI / GPIO12 	TDI
4 	MTCK / GPIO13 	TCK
5 	MTMS / GPIO14 	TMS
```


Also the following red/green blue led is available on the WROVER kit.
```
  RED    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT);
  GREEN  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
  BLUE   gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_NUM_0, 0);
```


After making sure you use the correct Component config → 
ESP32-specific -> Main XTAL frequency
A good idea, might be to set these values.
```
  Bootloader log verbosity (No output)  --->    
  Compiler options ->  Optimization Level (Release (-Os)) 
```


To get some test data, on GPIO_NUM_21 a 30/15 ms pulse is generated with the remote device.
```
app_main() {
    ...
    send_remote_pulses();
    rmt_write_items(config.channel, items, 1, 0);
}
```
When starting the sigrok gui, pulseview, the device was identified and this is how the timing analysis looks like.

![sigrok](sigrok.png)
[https://raw.githubusercontent.com/Ebiroll/esp32_sigrok/master/sigrok.png ]

# Analysis of UART 9600 Baud
```
static void uartWRITETask(void *inpar) {
  uart_port_t uart_num = UART_NUM_1;    
  echoLine[0]='s';
  echoLine[1]='u';
  echoLine[2]='m';
  echoLine[3]='p';

  while(true) {
    int size = uart_write_bytes(uart_num, (const char *)echoLine, 4);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
```
![uart](uart.png)

# Project status
Currently not working well.

It has howerer , SCPI over the network ,
sigrok-cli -d rigol-ds:conn=tcp-raw/127.0.0.1/5555  -l 5 --scan
And some sump over network or USB, however libsigrok only supports SUMP over USB.
Instead sump over TCP/IP was used for debugging.
nc 192.168.1.130 5566

For data aquisition High resolution timer is used.
http://esp-idf.readthedocs.io/en/latest/api-reference/system/esp_timer.html

Trying to  add rigol emulation and emulate sending of analouge and digital waveforms

In directory linux, you can build a test client, it listens to port 5555
./test

To try connect 
 sigrok-cli -d rigol-ds:conn=tcp-raw/127.0.0.1/5555  -l 5 --scan
This will send *IDN? to the instrument

You can also try this 
./olas-cli -d rigol-ds:conn=tcp-raw/192.168.1.130/5555  -l 5  --show

Unfortunatley this software currently does not work well enough.


https://assets.tequipment.net/assets/1/26/Documents/Rigol/vs5000_programming.pdf

```
To build a debuggable version of sigrok-cli use the CMakeLists.txt file
 mkdir sigrok;cd sigrok;
 git clone git://sigrok.org/libsigrokcxx.git
 // Not this?? git clone git://sigrok.org/libsigrok.git
 git clone git://sigrok.org/sigrok-cli
 cp CMakeLists.txt .
 mkdir build;cd build; cmake ..
 ./olas-cli -d rigol-ds:conn=tcp-raw/192.168.1.127/5555  -l 5  --show
```

To test reading data with sump.
./olas-cli -d ols:conn=/dev/ttyUSB1 -l 5  -c samplerate=1Mhz --samples 100

```
When using SUMP protocol currently pulseview exits with Caught exception: not applicable
sr: [00:31.316006] ols: Sending cmd 0x00.
... Initial 5 zeroes sent

sr: [00:31.316006] ols: Sending cmd 0x00.
sr: [00:31.316397] serial: Wrote 1/1 bytes.
sr: [00:31.316435] serial: Draining serial port /dev/ttyUSB1.
sr: [00:31.316755] ols: Sending cmd 0x02.
sr: [00:31.317130] serial: Wrote 1/1 bytes.
sr: [00:31.317175] serial: Draining serial port /dev/ttyUSB1.
sr: [00:31.328107] serial: Read 4/4 bytes.
sr: [00:31.328149] ols: Sending cmd 0x04.
sr: [00:31.328596] serial: Wrote 1/1 bytes.
sr: [00:31.328661] serial: Draining serial port /dev/ttyUSB1.
sr: [00:31.339184] ols: Device does not support metadata.
sr: [00:31.339332] ols: Disabling demux mode.
sr: [00:31.339358] serial: Closing serial port /dev/ttyUSB1.
sr: [00:31.340761] hwdriver: Scan of 'ols' found 1 devices.
sr: [00:33.353208] serial: Opening serial port '/dev/ttyUSB1' (flags 1).
sr: [00:33.358160] serial: Parsing parameters from "115200/8n1".
sr: [00:33.358301] serial: Setting serial parameters on port /dev/ttyUSB1.
sr: [00:33.361693] hwdriver: sr_config_list(): key 30002 (pattern) sdi 0x564e0bfc70f0 cg NULL -> ['None', 'External', 'Internal']
sr: [00:33.361750] hwdriver: sr_config_get(): key 30003 (rle) sdi 0x564e0bfc70f0 cg NULL -> false
sr: [00:33.361792] hwdriver: sr_config_get(): key 30002 (pattern) sdi 0x564e0bfc70f0 cg NULL -> 'None'
sr: [00:33.361869] hwdriver: sr_config_get(): key 30001 (captureratio) sdi 0x564e0bfc70f0 cg NULL -> uint64 0
Caught exception: not applicable
sr: [00:33.376098] serial: Closing serial port /dev/ttyUSB1.
srd: Exiting libsigrokdecode.
```


# Some other SUMP implementations,

https://github.com/hydrabus/hydrafw/wiki/HydraFW-SUMP-guide

https://github.com/jpbarraca/LogicAlNucleo/blob/master/src/main.cpp

https://github.com/tuxyme/metal-pi/blob/master/analyzer/main.c


https://github.com/gillham/logic_analyzer/blob/master/logic_analyzer.ino

#Rigol and SCPI

Other than the SUMP protocol we could try to emulate this RIGOL osciloscope,
https://www.batronix.com/shop/oscilloscopes/Rigol-VS5202D.html


https://assets.tequipment.net/assets/1/26/Documents/Rigol/vs5000_programming.pdf


Data format,
https://rigol.desk.com/customer/en/portal/articles/2269119-how-do-i-format-the-data-returned-from-a-ds1000e-d-series-scope-

https://en.wikipedia.org/wiki/Standard_Commands_for_Programmable_Instruments

https://github.com/cjameshuff/rigolutils

http://literature.cdn.keysight.com/litweb/pdf/81180-91020.pdf

https://grenville.wordpress.com/2015/01/04/a-clone-of-the-saleae-logic-8-channel-analyser/

