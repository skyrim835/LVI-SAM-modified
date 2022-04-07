# LVI-SAM-MODIFIED

This repository is a modified version of [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM).

---

## Modification

- Add function to get extrinsic parameters.The original code assumes there are no translation between the sensors and their parameters are written in the code. But now both our datasets and LVI-SAM official datasets are working well.
- Add "lidar to imu extrinsics" in params_camera.yaml.
- Using [MahonyAHRS](https://github.com/PaulStoffregen/MahonyAHRS) to caculate quaternion.So you don't need to prepare a 9-axis IMU.
- Add lidar ring calculation method,whether your lidar provides "ring" information or not,it works.
- Make some changes to the code for sensor alignment.
- Fix depth association with camera and lidar (Default lidar orientation is x--->front,y--->left,z--->right).
---

## Notes

- Most of the changes are marked by "# modified".
- If you are using VSCode and "#include xxx" prompt error,please ctrl+shit+p and enter C/C++ Edit Configurations(UI), add the following paths in Include path.
  /opt/ros/melodic/include/**
  /usr/include/**
- Please make sure you have the same version of dependencies as [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM).If you have problems installing or importing multiple version of dependencies,you can refer to this [blog](https://blog.csdn.net/DumpDoctorWang/article/details/84587331).
- You need to download and compile [yaml-cpp](https://github.com/jbeder/yaml-cpp).
- You can use [kalibr](https://github.com/ethz-asl/kalibr) to get cam_to_imu,and [calibration_camera_lidar](https://github.com/XidianLemon/calibration_camera_lidar) to  get cam_to_lidar,lidar_to_imu = cam_to_lidar.transpose * cam_to_imu
- If you are looking up for Chinese annotation, please click this link [LVI-SAM_detailed_comments](https://github.com/electech6/LVI-SAM_detailed_comments).
- You can see the difference between Fix depth association or not in the following pictures.Depth are not easily go through wall, more make sense.
- Here is the link of our datasets: https://pan.baidu.com/s/1hw_P7DGBDmmdQBnhsHh2zA  Code: vnkh
- Only tested in slow-moving robot indoor.Outdoor test is coming soon.

---

## Tips
- Here is an example to calculate extrinsic:(My device)
- From "calibration_camera_lidar" I  get camera->lidar extrinsic
- From "kalibr" I get camera->imu extrinsic
- Eigen::Matrix3d  Rot_cam_to_lidar;
- Rot_cam_to_lidar<<
  -9.9998651710637931e-01, 4.7941080890968359e-04,5.1706644378986897e-03,
  -5.1637058294141756e-03, 1.3529208794012426e-02,-9.9989514282824454e-01,
  -5.4931553803225881e-04, -9.9990836113850179e-01,-1.3526550844741081e-02;
- Eigen::Vector3d t_cam_to_lidar(4.0521203694483053e-02,-1.0921058057045349e-01,-1.1308134312590766e-01);
- Eigen::Matrix3d Rot_cam_to_imu; 
- Rot_cam_to_imu<<
  0.99995824,0.003922,-0.00825459,
  -0.00388915,0.99998447,0.00399195,
  0.00827012,-0.00395968,0.99995796;
- Eigen::Vector3d t_cam_to_imu(0.00304159, 0.00742751,0.01522853); 
- Eigen::Matrix3d Rot_lidar_to_cam= Rot_cam_to_lidar.transpose();
- Eigen::Vector3d t_lidar_to_cam = - Rot_cam_to_lidar.transpose() * t_cam_to_lidar;
- Eigen::MatrixX3d Rot_imu_to_cam = Rot_cam_to_imu.transpose();
- Eigen::Vector3d t_imu_to_cam = -Rot_cam_to_imu.transpose() * t_cam_to_imu;
- Eigen::Matrix3d Rot_imu_to_lidar = Rot_cam_to_lidar * Rot_imu_to_cam;
- Eigen::Vector3d t_imu_to_lidar = Rot_cam_to_lidar * t_imu_to_cam + t_cam_to_lidar;
- Eigen::Vector3d eulerAngle=Rot_lidar_to_cam.eulerAngles(2,1,0);
- double lidar_to_cam_rx = eulerAngle.z();
- double lidar_to_cam_ry = eulerAngle.y();
- double lidar_to_cam_rz = eulerAngle.x();
- You can draw a axis transformation picture to verify it(rotate by z-y-x order, and multiply them by right).

---

## Results
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-03-23%2017-25-35.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-03-23%2021-44-15.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-03-24%2015-55-44.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-03-24%2015-59-28.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-03-24%2016-02-21.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-03-24%2016-08-34.png)
## Results 2.0.0 fix depth association
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-04-07%2015-52-59.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-04-07%2015-53-22.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-04-07%2015-53-33.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-04-07%2015-57-16.png)
![image](https://github.com/skyrim835/LVI-SAM-modified/blob/master/images/Screenshot%20from%202022-04-07%2015-57-36.png)
## Acknowledgement

- The original version is from [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM).
- Inspired by this work[LVI_SAM_fixed](https://github.com/epicjung/LVI_SAM_fixed).