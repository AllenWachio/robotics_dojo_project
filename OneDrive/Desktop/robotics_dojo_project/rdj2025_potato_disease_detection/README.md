# rdj2025_potato_disease_detection

A ROS 2 Humble package for **potato disease detection** using a machine learning model.  
The package subscribes to an image topic (`/image`), runs inference, and publishes results to `/inference_result`.

## 📦 Package Overview

This package contains:

- **`potato_disease_detection_node`**  
  - Subscribes to `/image` (`sensor_msgs/Image`)  
  - Converts the image into OpenCV format (via `cv_bridge`)  
  - Runs inference using a provided ML model (you can replace with your own)  
  - Publishes results on `/inference_result` (`std_msgs/String`)

- **`test_image_publisher`**  
  - Publishes a static test image (`potato.jpg` or another local image)  
  - Sends it to `/image` for testing the detection node

## Dependencies

⚠️ Note: `cv_bridge` currently only supports **NumPy < 2.0**.  
If you see `_ARRAY_API not found` errors, downgrade NumPy:

```bash
pip install "numpy<2" --force-reinstall
```

Also, install PyTorch (Used for Inference)

```bash
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

## Build Instructions

Clone into your ROS 2 workspace:

```bash
# Change Directory to go to /src
cd ~/ros2_ws/src

# Clone the repo (HTTPS)
git clone https://github.com/roboticsdojo/rdj2025_potato_disease_detection.git

# Or clone the repo (SSH)
git clone git@github.com:roboticsdojo/rdj2025_potato_disease_detection.git

# Go back to root (parent) workspace directory
cd ~/ros2_ws

colcon build
source install/setup.bash
```

## Running the Nodes

### 1. Run the Inference Node

```bash
ros2 run rdj2025_potato_disease_detection potato_disease_detection_node
```

This will start the potato disease detection node. It will wait for images on `/image`.

### 2. Publish a Test Image

In a new terminal (with workspace sourced):

```bash
ros2 run rdj2025_potato_disease_detection publish_test_image
```

This will publish `potato.jpg` (make sure the file exists in the working directory).  
You should see logs like:

```
[INFO] [test_image_publisher]: Published test image.
[INFO] [potato_disease_detector]: Published result: Healthy Potato
```

### 3. Inspect the Inference Results

Use another terminal to view the published inference results:

```bash
ros2 topic echo /inference_result
```


##  Visualizing Images

You can view the published images with:

```bash
ros2 run rqt_image_view rqt_image_view
```


## Next Steps

    
-   Deploy on a robot with a camera publishing to `/image`.
    

