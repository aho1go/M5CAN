# M5CAN


Use SocketCAN(can-utils) - Linux
```
$ sudo socat pty,link=/dev/vmodem0,raw,echo=0,cr tcp:192.168.4.1:23&
$ sudo slcand -o -c -s6 -S2000000 /dev/vmodem0 can0
$ sudo ifconfig can0 up
$ cansend can0 111#1122
$ candump can0
```
