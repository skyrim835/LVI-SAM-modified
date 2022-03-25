# LVI-SAM-MODIFIED

This repository is a modified version of [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM).

---

## Modification

- Add function to get extrinsic parameters.The original code assumes there are no translation between the sensors and their parameters are written in the code. But now both our data sets and LVI-SAM official data sets are working well.
- Add "lidar to imu extrinsics" in params_camera.yaml.
- Add "Extrinsics (lidar -> cam)"in params_lidar.yaml.
- Using [MahonyAHRS](https://github.com/PaulStoffregen/MahonyAHRS) to caculate quaternion.So you don't need to prepare a 9-axis IMU.
- Add lidar ring calculation method,whether your lidar provides "ring" information or not,it works.

---

## Notes

- Most of the changes are marked by "# 修改的地方 modified".
- If you are using VSCode and "#include xxx" prompt error,please ctrl+shit+p and enter C/C++ Edit Configurations(UI), add the following paths in Include path.
  /opt/ros/melodic/include/**
  /usr/include/**
- Please make sure you have the same version of dependencies as [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM).If you have problems installing or importing multiple version of dependencies,you can refer to this [blog](https://blog.csdn.net/DumpDoctorWang/article/details/84587331).
- You need to download and compile [yaml-cpp](https://github.com/jbeder/yaml-cpp).

---

## Results
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-03-23%2017-25-35.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-03-23%2021-44-15.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-03-24%2015-55-44.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-03-24%2015-59-28.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-03-24%2016-02-21.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-03-24%2016-08-34.png)

# Tips
  - The original coordinates of LVI-SAM:
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/LVI-SAM-original-coordinates.png)
## Acknowledgement

  - The original version is from [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM).
  - Inspired by this work[LVI_SAM_fixed](https://github.com/epicjung/LVI_SAM_fixed).
