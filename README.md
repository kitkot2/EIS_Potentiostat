Will update readme later but uh... Just follow the guide in the HunStat2 paper https://doi.org/10.1039/D5AY01179E but use Arduino libraries and hardware files in this repository.
After the installation setup virtual environment and get all the libraries from requirements.txt

Run app.py to access the GUI for the potentiostat. 

This code uses pyserial for communication, you have to provide port name in the GUI to establish connection with the potentiostat.
Tested on MacOS. On Linux permissions have to be properly set. Windows should work fine too.

The device actually supports more commands that aren't available in the GUI. Look into test and ad5941 files for more information.