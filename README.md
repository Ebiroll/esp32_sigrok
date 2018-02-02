# Sigrok esp 32 

A SUMP compatible logic analyser for the esp32, for use with i.e sigrok
https://en.wikipedia.org/wiki/Sigrok

For digital input, an Open bench logic sniffer is emulated. (ols)
A Rigol DS scope (rigol-ds) is emulated in order to allow sampling of analouge input.

# Analogue
```
(ADC1_CHANNEL_6)      //GPIO 34
```

![uart](analog.png)

Still lots of tuning to do here.

# Pins
Pins that can be used for logical input on the WROVER board.

```
            12,
            13,
            14,
            15,
```

Pins defined  for logical input with the code in this repository over SUMP.

```
            21,
            22,
            23,
            24,
```

Testdata in this example. To get testdata connect i.e pin 17 & 22 and pin 23 & 18
```
            17, remote device, square pulse
            18, 2400 Baud UART with repeated text, "sump"
```



If using the WROVER-kit, you can use the JTAG chip to do data aquisition.
To get better debuginfo, start pulseview with logging,  -l 5 
On the WROVER dev kit, it seems that the FTDI-LA drivers are used.

https://sigrok.org/gitweb/?p=libsigrok.git;a=tree;f=src/hardware/ftdi-la
This allows sampling without a second ESP32 up to 10Mhz.

In the code the sump protocol is implemented over serial and tcp/ip. 

I found that the terminal program cutecom allowed sending single HEX bytes, this is useful for debugging the implementation of the SUMP protocol.

# Pin mapping

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

# Wrover debug pins
The following red/green/blue led is available on the WROVER kit.
```
  RED    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT);
  GREEN  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
  BLUE   gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_NUM_0, 0);
```

After making sure you use the correct Component config → 
ESP32-specific -> Main XTAL frequency
Another good idea, might be to set these values.
```
  Bootloader log verbosity (No output)  --->    
  Compiler options ->  Optimization Level (Release (-Os)) 
```
Otherwise they will interfer with the SUMP protocol.

To get some test data, connect  a 30/15 ms pulse is generated with the remote device on pin 17
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

# Analysis of UART 2400 Baud
To get some more testdata the uart sends the text "sump" on pin 18
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
Now it is working with sigrok and the latest version of esp-idf.

Perhaps this bugfix was helpful, but initially the SUMP protocol was not working
https://github.com/espressif/esp-idf/commit/f482e9e54ce83e249e46f5ee082f6ffb61431339

# Sigrok
In pulseview
Chose driver, Open bench Logic sniffer (ols)
Choose the interface, i.e./dev/ttyUSB0
Press Scan for devices,
If successful select ESP32 with 8 channels.
Change samplerate 10kHz. Its currently hardcoded to sample at this rate.

Also you can add more input pins by adding them in,
uint16_t getSample() &  portc_init(void)

For analog data try the rigol-ds over wifi on port 5555.

# Command line run
When developing, this is useful and repeatable


sigrok-cli -d ols:conn=/dev/ttyUSB0 -l 5  -c samplerate=10Khz --samples 100
sigrok-cli -d ols:conn=/dev/ttyUSB0 -l 5 --scan

SCPI over the network works now.

sigrok-cli -d rigol-ds:conn=tcp-raw/127.0.0.1/5555  -l 5 --scan

It is possible tp run sump over network or USB, however ols driver only supports SUMP over serial.
To use SUMP over network you must use PIPISTRELLO p-ols driver.


To debug this use printf for debugging
```
void sump_debug(char *str,unsigned int value) {
    printf("0x%X\n",value);   
}
```
screen /dev/ttyUSB1 115200
sigrok-cli -d p-ols:conn=tcp-raw/192.168.1.130/5566  -l 5 --scan



To debug SUMP over serial, then use  TCP/IP for debugging.
```
To use debug output, remove #if 0 in:
sump_debug()

```

nc 192.168.1.130 5566
This port is also used for the SUMP protocol over TCP/IP
If you use the Pipistrello OLS (p-ols) then you can use this
sigrok-cli -d p-ols:conn=tcp-raw/192.168.1.130/5566  -l 5 --scan
I never got SUMP over TCP/IP to work.


For data aquisition High resolution timer is used.
http://esp-idf.readthedocs.io/en/latest/api-reference/system/esp_timer.html


Trying to  add rigol emulation and emulate sending of analouge and digital waveforms

In directory linux, you can build a test client, it also listens to port 5555
./test

To try connect 
 sigrok-cli -d rigol-ds:conn=tcp-raw/127.0.0.1/5555  -l 5 --scan
This will send *IDN? to the instrument

You can also try this 
./sigrock-cli -d rigol-ds:conn=tcp-raw/192.168.1.130/5555  -l 5  --show


https://assets.tequipment.net/assets/1/26/Documents/Rigol/vs5000_programming.pdf

# Building sigrok

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
./sigrok-cli -d ols:conn=/dev/ttyUSB0 -l 5  -c samplerate=10Khz --samples 100



# Some other SUMP implementations,

https://github.com/hydrabus/hydrafw/wiki/HydraFW-SUMP-guide

https://github.com/jpbarraca/LogicAlNucleo/blob/master/src/main.cpp

https://github.com/tuxyme/metal-pi/blob/master/analyzer/main.c


https://github.com/gillham/logic_analyzer/blob/master/logic_analyzer.ino

# Rigol and SCPI

TODO, contact this person
https://github.com/hackrid/pyrigolla/


Other than the SUMP protocol we could try to emulate this RIGOL osciloscope,
https://www.batronix.com/shop/oscilloscopes/Rigol-VS5202D.html


https://assets.tequipment.net/assets/1/26/Documents/Rigol/vs5000_programming.pdf


Data format,

https://rigol.desk.com/customer/en/portal/articles/2269119-how-do-i-format-the-data-returned-from-a-ds1000e-d-series-scope-

https://en.wikipedia.org/wiki/Standard_Commands_for_Programmable_Instruments

https://github.com/cjameshuff/rigolutils

http://literature.cdn.keysight.com/litweb/pdf/81180-91020.pdf

https://grenville.wordpress.com/2015/01/04/a-clone-of-the-saleae-logic-8-channel-analyser/

