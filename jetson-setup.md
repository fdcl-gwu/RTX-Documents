# Setup for Jetson Orin Nano using NVMe SSD:

1. Follow the instructions linked here to boot the Orin from the installed SSD: https://www.youtube.com/watch?v=q4fGac-nrTI
    - NOTE: in order to flash, you will need a linux x86 host computer running Ubuntu 20.04.
    - If you flash it successfully, but after connecting to peripherals (monitor, keyboard, and mouse) you are prompted with a UEFI shell, just reflash the jetson.
    - For a generic SD card follow the instructions here: https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit#write
    - __Once finished, set the WIFI of the Jetson to Auto Connect on startup__.

2. Install JTOP to easily monitor the Jetson, follow the instructions here: https://jetsonhacks.com/2023/02/07/jtop-the-ultimate-tool-for-monitoring-nvidia-jetson-devices/.

3. Next, install ROS-Noetic (steps found here: http://wiki.ros.org/noetic/Installation/Ubuntu)
4. Install dependencies:
    <ol type="a">
      <li>catkin tools: <code>$ sudo apt-get install python3-catkin-tools python3-vcstool python3-osrf-pycommon </code> </li>
      <li>Ceres Solver (system dependencies): <code>$ sudo apt-get install libglew-dev libyaml-cpp-dev</code> </li>
      <li>Ceres Solver (dependencies): <code>$ sudo apt-get install libblas-dev liblapack-dev libsuitesparse-dev </code> </li>
      <li>OpenVins: <code>$ sudo apt-get install libeigen3-dev libboost-all-dev libceres-dev </code> </li>
      <li>ROS Gazebo: <code>$ sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control </code> </li>
    </ol>
5. Other useful packages:

    <ol type="a">
      <li><code>$ sudo apt-get install ros-noetic-pcl-ros </code> </li>
      <li><code>$ sudo apt-get install ros-noetic-cv-bridge</code> </li>
      <li><code>$ sudo apt-get install libi2c-dev</code> </li>
    </ol>
6. Make sure to associate the Jetson Orin to a github account with these steps (https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) to ensure minimal errors in the future.
    - If you're new to GitHub, follow this guide instead: https://kbroman.org/github_tutorial/pages/first_time.html.

7. For convenience, install VSCode:
    - Download the ```.deb``` for ```Arm64``` architecture file here: https://code.visualstudio.com/download.
    - Once installed, run ```$ sudo dpkg -i installer_file_name.deb```

## Further Notes about OpenCV (libopencv-dev):
The above installation installs multiple conflicting versions of OpenCV. We will purge all versions of OpenCV on the device by executing the following:

```$ sudo apt-get purge libopencv*```

Now we will install the correct OpenCV version. First call:

 ```$ sudo apt list --all-versions libopencv-dev```


Now look for any version that has 4.2.0 in it: there should be one that is called 4.2.0+dfsg-5.
If you do not see the above version or any others that have 4.2.0 you will need to install from source. Follow this guide: https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html (DON’T FORGET to make install at the end).


Now (if you didn’t choose to install from source) install the version 4.2.0 version you saw by executing the following: 

```sudo apt install libopencv-dev=<version>```


Our version = 4.2.0+dfsg-5

Finally, confirm the correct OpenCV is installed by checking the INFO tab in jtop.