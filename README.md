# SimpleBLDC

This is simple Arduino based 650W BLDC motor(with Hall sensors) controller. Designed with [KiCAD](https://www.kicad.org/).

![SimpleBLDC PCB without components](https://github.com/techn0man1ac/SimpleBLDC/blob/main/PCB/IMGs/PCB_face_without_components.png)

Simple BLDC 3-Phase BLDC motor driver specs:
- Battery: 8S 
- VCC: 24.0V to 33.6V
- I(max): 20A 
- Сurrent measurements per phase

# Schematic

![BLDC motor driver schematic](https://raw.githubusercontent.com/techn0man1ac/SimpleBLDC/refs/heads/main/PCB/IMGs/Schematic.jpg)

The device has 3 drivers for mosfet transistors, as well as 3 current sensors based on the Hall effect, which measure both in the positive and negative directions. The use of 3 sensors is the reason for the development of the device for regenerative braking.

# PCB

The board consists of two layers: top and bottom. All the main parts are located on the top layer.

![Face PCB side](https://raw.githubusercontent.com/techn0man1ac/SimpleBLDC/refs/heads/main/PCB/IMGs/PCB_face.png)

The bottom layer contains mainly earth and power tracks.

![The PCB bottom laye](https://raw.githubusercontent.com/techn0man1ac/SimpleBLDC/refs/heads/main/PCB/IMGs/PCB_back.png)

Board sizes is 72 x 99 mm:

![BLDC motor driver schematic](https://raw.githubusercontent.com/techn0man1ac/SimpleBLDC/refs/heads/main/PCB/IMGs/PCB_sizes.png)


