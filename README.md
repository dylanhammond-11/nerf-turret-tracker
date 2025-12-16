## nerf-turret-tracker ##

A 3D-printed Nerf turret that tracks objects using a camera and fires automatically.  
See my portfolio for pictures and more project details:
https://dylanhammond-11.github.io/projects/auto-nerf-turret/index/

## Python Scripts: ##

Handle object detection via OpenCV paired with an external USB camera. Object coordinates are sent via serial to microcontroller:

*turret_vision_face.py:* Face Detection

*turret_vision_red.py:* Red object detection

*turret_vision_rgb.py:* RGB color multi-object detection


## Arduino Scripts: ##
Control turret system via Arduino microcontroller. Object coordiniates are recieved from python scripts for system control.

*turret_face:* Turret code to track face objects OR single red objects (Use with *turret_vision_face.py:* or *turret_vision_red.py:*)

*turret_color:* Turret code to track multiple red, green, or blue objects (Use with *turret_vision_rgb.py:*)



## Mechanical/Electrical System  Components ##
- Arduino Uno
- 3 MG90s servo motors
- 2 type 30 DC motors
- l298n Motor Driver
- Adjustable Buck Converter
- RC car battery



