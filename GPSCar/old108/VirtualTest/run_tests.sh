#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
CC=${CC:-gcc}
CFLAGS="-std=c99 -Wall -Wextra -Wno-unused-parameter -Wno-pointer-to-int-cast -IVirtualTest/include -IMid/Inc -IProgram/Include -IDrive/Include"
$CC $CFLAGS VirtualTest/test_map.c -o VirtualTest/test_map.exe -lm
$CC $CFLAGS VirtualTest/test_lidar.c -o VirtualTest/test_lidar.exe
$CC $CFLAGS VirtualTest/test_uart.c -o VirtualTest/test_uart.exe -lm
$CC -std=c99 -Wall -Wextra -Wno-unused-parameter -IVirtualTest/ui_include VirtualTest/test_menu.c -o VirtualTest/test_menu.exe
VirtualTest/test_map.exe
VirtualTest/test_lidar.exe
VirtualTest/test_uart.exe
VirtualTest/test_menu.exe
rm -f VirtualTest/test_map.exe VirtualTest/test_lidar.exe VirtualTest/test_uart.exe VirtualTest/test_menu.exe
