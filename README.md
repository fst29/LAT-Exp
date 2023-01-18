# Lossless-Adjustable-Transformer-Experiment
This file covers the setup of how to run the experiments for lossless-adjustable transformer.
## git setup
1. Download gitHub desktop from https://desktop.github.com/ 
2. Clone this repository to your local computer

## ssh to raspberry pi
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

## 

