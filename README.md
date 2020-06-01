# Master Thesis. Valthor B Gudmundsson

## Requirements

### PX4 Firmware, with Gazebo and ROS

links to the guide
notes
bla bla

#### Changes
Models
The PX4 Firmware folder has been changed a bit to make this stuff work, especially the iris model. 
Since the velocity controller I made didn't really work well with the Typhoon hexacopter model, and since that's the only model in the whole base that actually has a "normal" camera on a stabilized gimbal, I took the liberty of just "copying" the camera onto the Iris.sdf model and configuring some other things as well to make it work.

A copy of the modified Iris model is included in this repos inside the "Changes" folder. 
This .sdf file must be replaced with the iris.sdf model inside Firmware/Tools/sitl_gazebo/models/iris folder.
Do make sure that you have a copy of the real iris.sdf model lying around somewhere. If you want to reset your changes in the .sdf's, you can just `DONT_MAKE=1 make px4_sitl_default` gazebo again, even without cleaning with make clean.

Furthermore, a script must be created in the Firmware/ROMFS/px4fmu_common/init.d-posix/ called 10016_iris.post. This is just a copy of the Typhoon .post script and it just makes sure that the mount stabilizer driver is actually recieving data from the IMU in the .sdf model. If this is not done, the gimbal will just dangle and do nothing.

Another thing is the parameter update. Inside the Typhoon init script in the ROMFS path described above, there are a few parameters that are not updated by default in the Iris that are required for the gimbal to work, and actually turning on the vmount driver. This driver has to be running and receiving data if the Gimbal is to be stabilized. The parameters can be set manually also, but can also be copied into the existing 10016_iris script (not post). The parameters are:

```
param set MNT_MODE_IN 0
param set MNT_DO_STAB 1
```
and can be set when the firmware is running. The parameters do however not take effect before the whole thing is restarted.

The raadhus.world has to be added into the worlds folder under Firmware/Tools/sitl_gazebo/worlds in order to simulated the flight that is included in the video. The City Hall building was used in development as a model of the building was readily available for download on the Sketchup 3D Warehouse. 

There might be some other changes that I'm forgetting. Please contact me at vaguu15@student.sdu.dk if you have problems simulating it.
### House R-CNN
Tensorflow and Keras is necessary to run this node. Both of these packages use GPU acceleration and require a valid version of CUDA and NVidia Drivers to run, as well as a graphics card with a compute capability of 2.0 or greater, depending on the version of Tensorflow you are using. See more at Tensorflow https://www.tensorflow.org/install/gpu. To find your compute capability, visit NVidia's website at https://developer.nvidia.com/cuda-gpus.
For the specific guide for the network in question, visit Songwongs GitHub at https://github.com/songwongtp/Mask_RCNN_ViewAnalysis, which is a fork of https://github.com/matterport/Mask_RCNN.

### CSAIL Segmentation with Pytorch
This node/network also requires CUDA as it makes use of GPU acceleration. If you are using an old graphics card, it does not suffice to merely install it from pip as a binary. You have to build it from source and the recommended approach is using Anaconda. The guide for it is here: https://github.com/pytorch/pytorch#from-source.


## asdf
