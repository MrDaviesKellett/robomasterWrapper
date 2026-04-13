# robowrap
__a wrapper for the DJI Robomaster library to make use of the library easier for secondary students__

_Please note that this package requires an older version of python3, this is due to the [Robomaster SDK](https://github.com/dji-sdk/RoboMaster-SDK) not supporting Python 3.9 onwards yet._

Please use **Python 3.7** or **Python 3.8**

## Installation

 1. Install [Python 3.8.2](https://www.python.org/downloads/release/python-382/)
 2. update pip
 3. install cv2
 4. Run the following _pip_ command to install robowrap:

2:
``` sh
python3.8 -m pip install --upgrade pip
```
3:
``` sh
python3.8 -m pip install opencv-python
```
4:
``` sh
python3.8 -m pip install robowrap --user
```

look at the [test.py](https://github.com/MrDaviesKellett/robomasterWrapper/blob/main/test.py) file to see how to use this wrapper.

## still to do:
- [ ] documentation
- [x] camera
- [x] line following
