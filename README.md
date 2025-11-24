# nerf-turret-tracker

A 3D-printed Nerf turret that tracks objects using a camera and fires automatically.  
See https://dylanhammond-11.github.io/projects/auto-nerf-turret/index/

Python scripts handle object detection via OpenCV paired with an external USB camera.
Object coordinates are sent via serial to microcontroller:
-Face Detection
-Red object detection
-RGB color multi-object detection

Arduino Scripts control turret system via Arduino microcontroller.
Object coordiniates are recieved from python scripts for system control.
Turret includes:
-Three MG90s servo motors
-Two type 30 DC motors.



