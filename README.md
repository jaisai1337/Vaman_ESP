# ESP-IDF for your ESP32
## To compile using ESP-IDF you will need to get the following packages. The command to run depends on which distribution of Linux you are using 
```
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-setuptools cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```
## Get ESP-IDF
```
mkdir -p ~/esp
cd ~/esp
git clone -b v4.3 --recursive https://github.com/espressif/esp-idf.git
```
## Set up the tools
```
cd ~/esp/esp-idf
./install.sh esp32
```
## Set up the environment variables
```
. $HOME/esp/esp-idf/export.sh
```
## Start a Project 
```
cd ~/esp
cp -r $IDF_PATH/examples/get-started/hello_world .
```
## Configure your Project
```
cd ~/esp/hello_world
idf.py set-target esp32
idf.py menuconfig
```
### Just Save Using Q
## Build the Project
```
idf.py build
```
## Flash onto the Device
```
idf.py -p PORT -b 115200 flash
```
# Download The VAMAN_SPI Code
```
cd ~/esp/
svn co https://github.com/jaisai1337/Vaman_ESP/trunk/Vaman_SPI
cd Vaman_SPI
```
## Configure your Project
```
idf.py set-target esp32
idf.py menuconfig
```
### In menuconfig go to Serial flasher config and change Flash size to 4MB
### And then go back and go to Component config and Enable Bluetooth and Save using (S) and Quit (Q) 
## Build the Project
```
idf.py build
```
## Flash onto the Device
```
idf.py -p PORT -b 115200 flash
```

