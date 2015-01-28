#!/bin/sh

echo 170 > /sys/class/gpio/export
sleep 1
echo out > /sys/class/gpio/gpio170/direction
sleep 1
echo 1 > /sys/class/gpio/gpio170/value
sleep 1
insmod pwm.ko servo=1 frequency=400 servo_start=20000 irq_mode=1 timeout=10
sleep 2
echo 10000 > /dev/pwm8
echo 10000 > /dev/pwm9
echo 10000 > /dev/pwm10
echo 10000 > /dev/pwm11
