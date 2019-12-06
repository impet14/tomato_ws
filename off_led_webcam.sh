#!/bin/bash 

sudo usermod -aG dialout $USER
sudo echo 0 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
sudo uvcdynctrl -d video0 -s 'LED1 Mode' 0
sudo uvcdynctrl -d video2 -s 'LED1 Mode' 0
sudo uvcdynctrl -d video4 -s 'LED1 Mode' 0
sudo uvcdynctrl -d video6 -s 'LED1 Mode' 0
sudo uvcdynctrl -d video8 -s 'LED1 Mode' 0
sudo uvcdynctrl -d video10 -s 'LED1 Mode' 0
sudo uvcdynctrl -d video12 -s 'LED1 Mode' 0
sudo uvcdynctrl -d video14 -s 'LED1 Mode' 0
sudo uvcdynctrl -d video16 -s 'LED1 Mode' 0
sudo uvcdynctrl -d video18 -s 'LED1 Mode' 0
sudo uvcdynctrl -d video20 -s 'LED1 Mode' 0
sudo uvcdynctrl -d video22 -s 'LED1 Mode' 0
