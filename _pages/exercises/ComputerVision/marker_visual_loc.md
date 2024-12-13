---
permalink: /exercises/ComputerVision/marker_visual_loc
title: "Marker Based Visual Loc"

sidebar:
  nav: "docs"

toc: true
toc_label: "TOC Marker Based Visual Loc"
toc_icon: "cog"

layout: archive
classes: wide

gallery:
  - url: /assets/images/exercises/marker_based_visual_loc/marker_teaser.png
    image_path: /assets/images/exercises/marker_based_visual_loc/marker_teaser.png
    alt: "Marker Based Visual Loc"
    title: "Marker Based Visual Loc"

matrix:
  - url: /assets/images/exercises/marker_based_visual_loc/camera-intrinsic.png
    image_path: /assets/images/exercises/marker_based_visual_loc/camera-intrinsic.png
    alt: "Camera Matrix"
    title: "Camera Matrix"

exampletag:
  - url: /assets/images/exercises/marker_based_visual_loc/example_tag.png
    image_path: /assets/images/exercises/marker_based_visual_loc/example_tag.png
    alt: "April Tag"
    title: "April Tag"
---


## Goal

The goal of this exercise is to estimate the position and orientation (pose) of a robot in a 2D space by detecting and analyzing visual markers, specifically AprilTags. This process involves using computer vision to identify the tags in the robot's environment and mathematical methods to derive its relative pose with respect to the detected tags.

{% include gallery caption="Gallery" %}


## Instructions
This is the preferred way for running the exercise.

### Installing and Launching
1. Download [Docker](https://docs.docker.com/get-docker/). Windows users should choose WSL 2 backend Docker installation if possible, as it has better performance than Hyper-V.

2. Pull the current distribution of RoboticsBackend:

	```bash
  docker pull jderobot/robotics-backend:latest
  ```

- In order to obtain optimal performance, Docker should be using multiple CPU cores. In case of Docker for Mac or Docker for Windows, the VM should be assigned a greater number of cores.

- It is recommended to use the latest image. However, older distributions of RoboticsBackend can be found [here](https://hub.docker.com/r/jderobot/robotics-backend/tags).

### How to perform the exercises?
- Start a new docker container of the image and keep it running in the background:

	```bash
  docker run --rm -it -p 7164:7164 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 -p 7163:7163 jderobot/robotics-backend
  ```

- On the local machine navigate to 127.0.0.1:7164/ in the browser and choose the desired exercise.

- Wait for the Connect button to turn green and display "Connected". Click on the "Launch" button and wait for some time until an alert appears with the message `Connection Established` and button displays "Ready". 

- The exercise can be used after the alert.

### Enable GPU Acceleration
- Follow the advanced launching instructions from [here](https://jderobot.github.io/RoboticsAcademy/user_guide/#enable-gpu-acceleration).

**Where to insert the code?**

In the launched webpage, type your code in the text editor,

```python
from GUI import GUI
from HAL import HAL
# Enter sequential code!


while True:
    # Enter iterative code!
```

### Using the Interface

* **Control Buttons**: The control buttons enable the control of the interface. Play button sends the code written by User to the Image. Stop button stops the code that is currently running on the Image. Save button saves the code on the local machine. Load button loads the code from the local machine. Reset button resets the simulation (primarily, the image of the camera).

* **Brain and GUI Frequency**: This input shows the running frequency of the iterative part of the code (under the `while True:`). A smaller value implies the code runs less number of times. A higher value implies the code runs a large number of times. The numerator is the one set as the Measured Frequency who is the one measured by the computer (a frequency of execution the computer is able to maintain despite the commanded one) and the input (denominator) is the Target Frequency which is the desired frequency by the student. The student should adjust the Target Frequency according to the Measured Frequency.

* **RTF (Real Time Factor)**: The RTF defines how much real time passes with each step of simulation time. A RTF of 1 implies that simulation time is passing at the same speed as real time. The lower the value the slower the simulation will run, which will vary depending on the computer. 

* **Pseudo Console**: This shows the error messages related to the student's code that is sent. In order to print certain debugging information on this console. The student is provided with `console.print()` similar to `print()` command in the Python Interpreter. 

## Robot API

* `from HAL import HAL` - to import the HAL library class. This class contains the functions that receives information from the webcam.
* `from GUI import GUI` - to import the GUI (Graphical User Interface) library class. This class contains the functions used to view the debugging information, like image widgets.
* `HAL.getImage()` - to get the image
* `GUI.showImage()` - allows you to view a debug image or with relevant information
* `GUI.showRobotPose()` - allows you to view your estimate pose in the map
* `HAL.setV()` - to set the linear speed
* `HAL.setW()` - to set the angular velocity   


## Theory

### Visual Markers: AprilTags

AprilTags are fiducial markers similar to QR codes but designed for robust detection and pose estimation. They consist of a binary grid with a unique pattern that encodes an identifier. They are ideal for robotics applications due to:

- Their high detection reliability.
- Robustness to noise, partial occlusion, and perspective distortions.
- Easy decoding to obtain a unique ID and retrieve the tag's known position in the environment.

Example code for apriltags detection:
```python
import GUI
import HAL
import apriltag
import cv2

detector = apriltag.Detector()

while True:
    img = HAL.getImage()

    img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    result = detector.detect(img_grey)
```

result is in the form of:

```
[DetectionBase(tag_family='tag36h11', tag_id=2, hamming=0, goodness=0.0, decision_margin=98.58241271972656, homography=array([[ -1.41302664e-01,   1.08428082e+00,   1.67512900e+01],
   [ -8.75899366e-01,   1.50245469e-01,   1.70532040e+00],
   [ -4.89183533e-04,   2.12210247e-03,   6.02052342e-02]]), center=array([ 278.23643912,   28.32511859]), corners=array([[ 269.8939209 ,   41.50381088],
   [ 269.57183838,   11.79248142],
   [ 286.1383667 ,   15.84242821],
   [ 286.18066406,   43.48323059]])),
DetectionBase(tag_family='tag36h11', ... etc
```

### Camera Model and Calibration

Before detecting AprilTags, the camera used to observe the environment must be calibrated. Camera calibration provides intrinsic parameters like focal length and optical center, which are critical for accurate pose estimation. A camera model is typically represented using the pinhole camera model with distortion corrections.

The intrinsic parameters are used in:

    Projection Matrix (P): Converts 3D world coordinates to 2D image coordinates.

{% include gallery id="matrix" caption="Matrix" %}

### AprilTag Detection

The detection process includes:

- Preprocessing: Filtering the image and identifying regions likely to contain tags.
- Decoding: Analyzing the binary pattern to extract the tag's unique ID.
- Pose Estimation: Computing the 6-DoF pose of the tag with respect to the camera.

{% include gallery id="exampletag" caption="Exampletag" %}

### PnP (Perspective-n-Point)

Given the 3D coordinates of the tag and their corresponding 2D image points (without z), the robot's pose can be estimated using algorithms like Direct Linear Transformation (DLT) or iterative methods (e.g., Levenberg-Marquardt).



## Contributors

- Contributors: [Jose María Cañas](https://github.com/jmplaza), [David Duro](https://github.com/dduro2020), [Javier Izquierdo](https://github.com/javizqh).

## References

[https://pyimagesearch.com/2020/11/02/apriltag-with-python/](https://pyimagesearch.com/2020/11/02/apriltag-with-python/)
[https://en.wikipedia.org/wiki/Perspective-n-Point](https://en.wikipedia.org/wiki/Perspective-n-Point)
[https://april.eecs.umich.edu/software/apriltag](https://april.eecs.umich.edu/software/apriltag)
[https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_pose/apriltag-pose.html](https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_pose/apriltag-pose.html)