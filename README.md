# ROS Packages for VIA Bot

**Requirements:**

- Python 3.6
- ROS 1 with Python 3 support
- Python dependencies: `requirements.txt`

## Run

- Download deep learning models from `https://drive.google.com/drive/folders/1gMDUgbnbK0tV9IFUKlFBS8ic2g4qEbX1` and put into `src/via_perception/models`.

- **1. ROS Master**

```
roscore
```

- **2. HTTP Camera Driver (to capture images from VIA Bot Mini)**

```
rosrun via_cam_driver http_cam_de.py
```

- **3. Lane detection node**

```
rosrun via_perception lane_detection_node.py
```

- **4. Traffic sign detection node**

```
rosrun via_perceptionraffic_sign_detection_node.py
```

- **5. Drive node**

```
rosrun via_bot_mini via_bot_mini_chassis.py
```