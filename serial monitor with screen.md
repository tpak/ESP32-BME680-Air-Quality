How to use screen on Mac to watch serial port traffic - useful for debugging Arduino and similar boards .

- find the serial port you want to watch:
```
    ❯ ls /dev/tty.*
    /dev/tty.Bluetooth-Incoming-Port 
    /dev/tty.usbserial-0160E5AB
```
- then open it with screen, setting the baud rate:
```
    ❯ screen /dev/cu.usbserial-0160E5AB 115200
```

## [How to get out of screen (link)](https://stackoverflow.com/questions/4847691/how-do-i-get-out-of-screen-without-typing-exit)

tl;dr on Mac with prompt: 

    ^a ^\
