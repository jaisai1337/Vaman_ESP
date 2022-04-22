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
idf.py -p /dev/ttyUSB0 -b 115200 flash
```
