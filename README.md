# visual-hand-kin-inspector

A tool for superimposing the kinematics of the iCub hand on RGB images

### Requirements
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [superimpose-mesh-lib](https://github.com/robotology/superimpose-mesh-lib)
- [OpenCV (with support for Aruco)](https://opencv.org/)
- [robotology-superbuild](https://github.com/robotology/robotology-superbuild)

In alternative to the `robotology-superbuild` you are required to install at least the following:
 - [YARP](https://github.com/robotology/yarp)
 - [icub-main](https://github.com/robotology/icub-main)
 - [iDynTree](https://github.com/robotology/idyntree)
 - [icub-models](icub)

### How to build

```console
git clone https://MatteoPerotto/visual-hand-kin-inspector
cd visual-hand-kin-inspector
mkdir build
cd build
cmake ../
make
```

### How to run
