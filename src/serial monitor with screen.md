Use screen on Mac to watcher serial port traffic.

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

## [How to get out of screen](https://stackoverflow.com/questions/4847691/how-do-i-get-out-of-screen-without-typing-exit)

on Mac: 

    ^a ^\
