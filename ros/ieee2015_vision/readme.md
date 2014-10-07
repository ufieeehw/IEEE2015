Updating OpenCV
===============

The OpenCV version that comes packaged with ROS is insufficient for our needs. For most robot related work, you do not need to upgrade.

# Areas Affected
* Vision-Based Localization Node
* Object Identification Node(s)/Service(s)
* Everything else is unaffected, so don't worry about updating your OpenCV installation unless you need to test vision related stuff

# How to Install

[1] http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html

You should be able to copy these commands and run them without issue. The make/install will take about an hour of time.

```
cd ~/
git clone https://github.com/Itseez/opencv.git
cd ~/opencv

mkdir release
cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ../

make -j 3
sudo make install
```

## Caveats
* If you run into an issue with number of cores or something, change "make -j 3" to "make -j (number of cores that you have minus one)"

* This should install it. If you have any issues, find Jacob.


If you find any issues and end up solving them, add them to this readme!