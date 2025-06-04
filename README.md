# Conveyor Belt Waste Classification

[![GitHub License](https://img.shields.io/github/license/sjncojnn/Conveyor-Waste-Classification)](LICENSE.md)
[![GitHub Stars](https://img.shields.io/github/stars/sjncojnn/Conveyor-Waste-Classification)](https://github.com/sjncojnn/Conveyor-Waste-Classification/stargazers)
[![GitHub Forks](https://img.shields.io/github/forks/sjncojnn/Conveyor-Waste-Classification)](https://github.com/sjncojnn/Conveyor-Waste-Classification/network)

## Project Overview

This repository contains the implementation of an AI-based waste sorting system integrated with a simulated conveyor belt environment using Gazebo and ROS 2. Developed as a third-year student project at Vietnam National University, Ho Chi Minh City University of Technology (Course CO3107, Semester 242), the system automates household waste sorting into five categories: Plastic, Glass, Paper/Cardboard, Organic, and Metal/Other, using a single RGB camera. The project, advised by Nguyen An Khuong, leverages YOLOv8 for detection and classification, with MobileNetV2 and EfficientNet-B0 for comparison, achieving a classification accuracy of 99.19% with YOLOv8-cls. The Gazebo simulation includes a conveyor belt, a static RGB camera, and a lever mechanism with four actuators sorting waste into five bins. ROS 2 action interfaces ensure synchronized control, making the system a scalable proof-of-concept for educational and small-scale recycling applications.

### Objectives
- Train and evaluate AI models (YOLOv8, MobileNetV2, EfficientNet-B0) for waste detection and classification.
- Design a Gazebo simulation with a conveyor belt, waste objects, and a robot with an RGB camera.
- Implement ROS 2 nodes to integrate AI with the simulation for near-real-time sorting.
- Compare the performance of a combined detection and classification approach against a classification-only approach, focusing on accuracy and multi-object handling.

### Technologies Used
- **AI Models**: YOLOv8 (detection and classification), MobileNetV2, EfficientNet-B0
- **Simulation**: Gazebo
- **Robotics Framework**: ROS 2
- **Programming Languages**: Python, C++ (for ROS 2)
- **Tools**: Google Colab, OpenCV, TensorFlow, Ultralytics

## Installation

To set up the project environment, follow these steps:

1. **Install ROS 2**: Follow the official [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html) for your operating system (recommended: ROS 2 Humble).
2. **Install Gazebo**: Install Gazebo using your distribution's package manager. For Ubuntu:
   ```bash
   sudo apt update
   sudo apt install gazebo
   ```
3. **Install Python Libraries**: Install dependencies for AI models:
   ```bash
   pip install ultralytics tensorflow opencv-python
   ```
4. **Build ROS 2 Workspace**: Navigate to the `ros/wc_ws/` directory and build:
   ```bash
   cd ros/wc_ws/
   colcon build
   source install/setup.bash
   ```

## Usage

### Running the Simulation
1. Launch Gazebo with the custom world file (located in `ros/wc_ws/src/` or use the default world).
2. Run the ROS 2 launch file to start the simulation and AI pipeline:
   ```bash
   ros2 launch system_launch main.launch.py
   ```
3. Monitor the simulation in Gazebo and visualize results using RViz (optional):
   ```bash
   ros2 run rviz2 rviz2
   ```

### Training AI Models
The AI models are trained using Jupyter notebooks in the `ai/notebooks/` directory:
- **`Data_Preprocessing.ipynb`**: Preprocesses the raw dataset, including label generalization, data augmentation (rotation, shearing, flipping), and resizing (224x224 for classification, 416x416 for detection).
- **`Classification.ipynb`**: Trains classification models (YOLOv8-cls, MobileNetV2, EfficientNet-B0) on the processed dataset.
- **`Detection.ipynb`**: Trains the YOLOv8-detect model for object detection.

To train the models:
1. Open the notebooks in [Google Colab](https://colab.research.google.com/) or a local Jupyter environment.
2. Follow the instructions in each notebook to load datasets, train models, and save weights to `ai/weights/`.

### Testing Models
Use the trained weights in `ai/weights/` for inference:
- **Classification**: Test YOLOv8-cls, MobileNetV2, or EfficientNet-B0 on new images using `Classification.ipynb`.
- **Detection**: Test YOLOv8-detect using `Detection.ipynb`.
- Combine detection and classification for end-to-end testing, as described in the project report.

## Dataset

The datasets are not included in the repository due to size constraints but can be accessed from the following sources:
- **Raw Dataset**: [Recyclable and Household Waste Classification](https://www.kaggle.com/datasets/alistairking/recyclable-and-household-waste-classification) (~15,000 images, 30 waste types reduced to 5 categories).
- **Processed Dataset**: [Google Drive](https://drive.google.com/uc?id=1cwQ3frGJiUEJ1mdCt9GXbIxMDb04hDA2) (preprocessed images with generalized labels and augmentation).
- **Detection Dataset**: Available from Roboflow project "conveyor-waste-belt" version 6. Contact the project team for access details, as an API key is required.

**Note**: The detection dataset uses simplified bounding boxes (images placed on black backgrounds) to enhance accuracy for the YOLOv8-detect model.

## Project Structure

The repository is organized as follows:
- **`ai/`**:
  - **`notebooks/`**: Contains Jupyter notebooks:
    - `Data_Preprocessing.ipynb`: Dataset preprocessing and augmentation.
    - `Classification.ipynb`: Training and testing classification models.
    - `Detection.ipynb`: Training and testing the detection model.
  - **`weights/`**: Stores trained model weights:
    - YOLOv8-cls model
    - MobileNetV2 model
    - EfficientNet-B0 model
    - YOLOv8-detect model
- **`ros/wc_ws/src/`**:
  - **`arm/`**: ROS 2 package for the lever mechanism (4 actuators for sorting).
  - **`camera/`**: ROS 2 package for camera simulation and image processing.
  - **`conveyor/`**: ROS 2 package for conveyor belt simulation.
  - **`system_launch/`**: Launch files for running the entire system.
  - **`trash_bin/`**: ROS 2 package for simulating five waste bins.
- **`docs/`**: Project documentation, including the project report and presentations.

| Directory | Description |
|-----------|-------------|
| `ai/notebooks/` | Jupyter notebooks for preprocessing, training, and testing AI models |
| `ai/weights/` | Trained model weights for classification and detection |
| `ros/wc_ws/src/arm/` | ROS 2 package for lever mechanism control |
| `ros/wc_ws/src/camera/` | ROS 2 package for RGB camera simulation |
| `ros/wc_ws/src/conveyor/` | ROS 2 package for conveyor belt simulation |
| `ros/wc_ws/src/system_launch/` | Launch files for system execution |
| `ros/wc_ws/src/trash_bin/` | ROS 2 package for waste bin simulation |
| `docs/` | Project documentation and reports |

## License

This project is licensed under the MIT License. See the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgments

- **Advisor**: Nguyen An Khuong
- **Contributors**: Nguyen Duc Hanh Nhi, Le Van Duc Anh, Huynh Nga, Phan Quang Minh, Nguyen Anh Kiet
- **Tools and Libraries**: [Ultralytics YOLOv8](https://docs.ultralytics.com/), [TensorFlow](https://www.tensorflow.org/), [OpenCV](https://opencv.org/), [Gazebo](http://gazebosim.org/), [ROS 2](https://docs.ros.org/en/humble/)
- **Dataset Sources**: Kaggle, Google Drive, Roboflow
