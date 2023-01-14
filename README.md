# rosflight-configrator
Normally rosflight is software designed to be able to use with autonomous driving of your device. But if you want to control it with rc control, this configurator will be useful for you.

## Supported Mavlink

Mavlink supports communication protocol, please edit the codes in the rosflight.js file according to you
Follow these steps to automatically create the custom xml file developed for your device for the javascript library, enter your js file as mavlink.js and copy it to the msg folder.

```bash 
  sudo apt install python3-pip
  pip3 install future
  sudo apt-get install python3-numpy python3-pytest
  sudo python -m pip install --upgrade future lxml
  git clone https://github.com/ArduPilot/pymavlink.git --recursive 
  
  set PYTHONPATH=C:\path_to_root_of_cloned_mavlink_repository
  
  python3 -m mavgenerate
  
```
