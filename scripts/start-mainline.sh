#!/bin/sh
/etc/init.d/S55klipper_service stop
/etc/init.d/S57klipper_mcu stop
/usr/data/S57klipper_mcu_mainline start
/usr/data/S55klipper_mainline start
/etc/init.d/S56moonraker_service restart
