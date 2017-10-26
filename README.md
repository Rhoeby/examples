# examples

This repository contains simple example code to help users get started with Rhoeby Dynamics products.

## r2d_radar_plot.c

This program does the following:
  
  - connects to the R2D Scanner
  - starts the scanner
  - recieves and plots the scan data

## How to build and run

Build the example program in Linux (eg. Ubuntu on a PC):

    sudo gcc r2d_radar_plot.c -o r2d_radar_plot -lm

Connect scanner via USB

    - plug in scanner
    - type 'lsusb', you should see something like: "Bus 002 Device 074: ID 0483:5740 STMicroelectronics"
    - confirm sensor blue LED is flashing
    - you could verify *binary* data flow from the scanner, type: 'cat /dev/ttyACM0' (use Ctrl-C to exit)

To run (after plugging in the Rhoeby R2D Scanner to the USB port on your Linux PC):

    sudo chmod 777 /dev/ttyACM0
    
    ./r2d_radar_plot

Screenshot of program output:

![R2D radar screenshot](https://github.com/jordanjohnp/images/blob/master/r2d_radar_plot_screenshot.png)

The program updates the console each time the scan completes.
