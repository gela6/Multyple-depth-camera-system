# Development of a multiple depth-camera system (in C++) for generating datasets to train neural networks
This is the project for my master's thesis on Faculty of mechanical engineering and naval architecture in Zagreb. 
I have used libraries: OpenCV, PCL, Open3D, Eigen, liberalsense(for RealSense cameras) etc. in C++. The whole C++ code is visible in main.cpp
And also required Python libraries for preparing data and machine learning: NumPy, pandas etc.

## Project workflow

### 1. Design and make a laboratory setup with three Intel RealSense D435 cameras.
![Screenshot__1](https://user-images.githubusercontent.com/96240235/167845597-a045333a-baa1-4c78-bd2a-e993177bfe81.png)
![Screenshot_3](https://user-images.githubusercontent.com/96240235/167845633-7793631e-af89-4836-9aa9-8ab96b884bf6.png)
![WhatsApp Image 2022-05-11 at 14 08 29](https://user-images.githubusercontent.com/96240235/167847945-50d6187c-d999-4552-99e9-e49f93237d7a.jpeg)



### Step 2. Single camera calibration             
Each camera has to be calibrated. Here is an example of (one of) a picture taken for the process.
![Output1_screenshot_18 02 2022](https://user-images.githubusercontent.com/96240235/167884116-37fb5a6c-d193-4bfb-9711-a0eff424f1a1.png)



### Step 3. Stereo calibration 

### Step 4. Taking frames and converting them in 2D pictures and 3D point clouds

### Step 5. 2D pictures filtering

### Step 6. 3D point clouds filtering

### Step 7. 3D point clouds stitching

### Step 8. 2D pictures labeling for a neural network       
In progress...

### Step 9. Learn neural network with YOLOv5  
In progress...
