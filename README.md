# M5CAN

This software is CAN communication software using M5Stack and COMMU module.

  
The following libraries are used:
 
https://github.com/coryjfowler/MCP_CAN_lib

If DEBUG_MODE of mcp_can_dfs.h is 1, please set it to 0.

```
mcp_can_dfs.h:
    #define DEBUG_MODE 1 -> 0
```

## Use SocketCAN(can-utils) - Linux
```
$ sudo socat pty,link=/dev/vmodem0,raw,echo=0,cr tcp:192.168.4.1:23&
$ sudo slcand -o -c -s6 -S2000000 /dev/vmodem0 can0
$ sudo ifconfig can0 up
$ cansend can0 111#1122
$ candump can0
```
