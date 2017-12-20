# Sigrok esp 32 

A SUMP compatible Logical Analyser for the esp32, for use with i.e sigrok
https://en.wikipedia.org/wiki/Sigrok

Pins defined for logical input.

```
            12,
            13,
            14,
            15,
            16,
```

After making sure you use the correct Component config → ESP32-specific -> Main XTAL frequency
A good idea, might be to set these.
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


# Some other SUMP implementations,

https://github.com/hydrabus/hydrafw/wiki/HydraFW-SUMP-guide

https://github.com/jpbarraca/LogicAlNucleo/blob/master/src/main.cpp

https://github.com/tuxyme/metal-pi/blob/master/analyzer/main.c


https://github.com/gillham/logic_analyzer/blob/master/logic_analyzer.ino
