# BlackBox-V2-Arduino
total rewrite of my original BlackBox software. Uses new libraries to limit memory usage and improve functionality. Code was written in 2016, uploading now for the sake of my portfolio. I spoke with the developer of the new libraries I used here to learn how to implement them, so this version of my code was a collaborative effort

Look at BlackBoxFirmwareV2.ino for the interesting code, the other files are mostly configs

Software for a data logger I built using arduino. Logs accelerometer and GPS data together to track movement data over a period of time, potentially to be used as an overlay for go-pro videos or just to view maximum accelerations and speeds for action sports

required non-standard libraries
-NeoSWSerial
-RTClib
-Adafruit_MMA8451
-NMEAGPS
