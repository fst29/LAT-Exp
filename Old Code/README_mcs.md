# Lossless-Adjustable-Transformer-Experiment
This file covers the setup of how to run the experiments for lossless-adjustable transformer.
## git setup
1. Download gitHub desktop from https://desktop.github.com/ 
2. Clone this repository to your local computer

## ssh to Raspberry Pi
1. Turn on Raspberry Pi

2. Bring up Top Left -> Accessories -> Keyboard
![20230113_145622](https://user-images.githubusercontent.com/92736605/213215342-f9b5b8ea-c21c-4e0d-aea6-f8f397b807f3.jpg)

3. Open Terminal clicking the terminal icon at the top

4. Using the pop up keyboard type "ifconfig" in terminal
![20230113_145706](https://user-images.githubusercontent.com/92736605/213215824-9354da44-68a9-4dfe-bc53-f71fbb626766.jpg)

5. The following message will be displayed. Find the line that says wlan0. Write down the numbers next to "inet" (starts with 10.248...)
![20230113_145714](https://user-images.githubusercontent.com/92736605/213216100-292bbd5b-fbab-42b6-8004-1873497d422e.jpg)

6. On a Mac open terminal (Command + Space, then type "terminal") 

7. In terminal type: ssh pi@IP-Address where IP-Address is replaced by the number 10.248....

8. You will then be prompted for a password. Type "pass" and press Enter. You should now be connected to Raspberry Pi.

## Navigating the code on Raspberry Pi
To navigate to the correct folder type
1. cd git
2. cd LAT-Exp

## Connecting Loadcell
1. Download DSCUSB Toolkit from here on your laptop https://www.mantracourt.com/software/dscusb/dsc-usb-toolkit

2. Connect the USB from the loadcell to your laptop

3. Run the DSCUSB Toolkit program, the loadcell should then appear 

4. To start recording data go to 'Recording' type

5. Select the folder where you want to record the data (I would recommend that the folder is in the same git repository so that it is synced to git)

6. Name the file being recorded as 'data_loadcell.csv'

## Compiling the script
1) Locate the script that you want to run (from the "Scripts" folder) and save it as a main.cpp script in the main directory on your laptop
<img width="195" alt="image" src="https://user-images.githubusercontent.com/92736605/213391352-14f21a5e-3094-4179-80f4-4a00051ec1eb.png">
2) Commit the changes to git using git desktop

[mcs extra comment: go to github app on laptop (find under "Applications" and double-click);  if it asks "pull origin" do that in order to synchronise between github (cloud) and the laptop;

3) Pull the changes in Raspberry Pi by typing "git pull"

4) Go the directory where main.cpp is located on your Raspberry Pi by using command "cd 'folder name'". You can check what is in the current folder on raspberry pi by typing "ls".

5) type "chmod +x build.sh"

6) type "chmod +x canableStart.sh"

7) type ./build.sh

8) The compiler should build the file

9) If running the code for the first time after turning the Raspberry Pi on, type "canableStart.sh"

## Running the experiment/script in Raspberry Pi
1) Open DSCUSB toolkit program and choose 'Net' for the torque calculation, so that there is no torque recorded at rest

2) Start recording the loadcell data on DSCUSB toolkit

3) Now switch to terminal to operate Raspberry pi

4) In the folder where the main.cpp script is located, type ./bin/main

5) After running the script the file data.csv needs to be uploaded to git. To do this type

6) "git add 'data.csv'"

7) git commit -m "New Data"

8) git push

## Stopping Script
To stop the script at any time press 'Command + C"

Now when on your laptop if you "pull" on GitHub desktop the updated data.csv file should be available


## Code Explained
The code contained in this repository for the following experiments: 1) Code to test stiction with spring disconnected 2) Code to test dynamic resistance with spring disconnected 3) Code to test stiction with spring connected 4) Device Law with sinusoidal carriage motion. These scripts will be covered separately.

### Stiction with spring disconnected
1) Performs the stiction measurement trajectory

2) Parameters that can be modified are: 
<img width="602" alt="image" src="https://user-images.githubusercontent.com/92736605/213392686-e450fc39-a5eb-4416-9026-436e4659c8ea.png">

## Running MATLAB postprocessing code
1. The code is located in folder MATLAB Processing Scripts
2. Copy this folder to some local directory for convenience
3. Copy and paste 'data.csv' and 'data_loadcell.csv' files to some local directory
4. When running processing code you will need to change the path in this line to the path of where the files are located
<img width="544" alt="image" src="https://user-images.githubusercontent.com/92736605/213399148-69f86ead-81f1-4af4-b51c-5500e50ea4f2.png">
5. This is how mine file structure looks
<img width="838" alt="image" src="https://user-images.githubusercontent.com/92736605/213399462-72443f87-48a4-433c-a63a-ed9c3cf68dc8.png">

## Useful links
CTRE - documentation: https://v5.docs.ctr-electronics.com/en/stable/
CTRE - C++ documentation: https://api.ctr-electronics.com/phoenix/release/cpp/
Github troubleshoot - https://docs.github.com/en



