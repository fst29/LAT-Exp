# Lossless-Adjustable-Transformer-Experiment
This file covers the setup of how to run the experiments for lossless-adjustable transformer.
An in-depth desciption of the software is included [here](Software.pdf).


## Running the software locally
0. Make sure that the motors and the Raspberry Pi are plugged in

1. Start the frontend application using the desktop icon

2. Start the backend application using the desktop icon (must be done after frontend is running)

3. To close the backend, press CTRL+C in the terminal

## Running the backend remotely
0. Make sure that the motors and the Raspberry Pi are plugged in

1. Start the frontend application using the desktop icon

2. Connect to the Raspberry Pi over ssh, the ip address can be checked by hovering over the Wifi icon on the taskbar

3. Run /home/pi/fst29/gui/build/guiDriver in the remote terminal

4. To close the backend, press CTRL+C in the terminal


## File locations on the Pi
Location of log and data files: /home/pi/fst29/data
Location of the code: /home/pi/fst29/gui/


## Connecting Loadcell
1. Download DSCUSB Toolkit from here on your laptop https://www.mantracourt.com/software/dscusb/dsc-usb-toolkit

2. Connect the USB from the loadcell to your laptop

3. Run the DSCUSB Toolkit program, the loadcell should then appear 

4. To start recording data go to 'Logging' type

5. Select the folder where you want to record the data 

6. Name the file being recorded as required

7. In 'Information' tab select 'Net' so that there is zero torque recorded at rest
