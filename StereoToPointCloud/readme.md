
## Run
```shell
seq=0003
left_path=/home/chen/datasets/kitti/tracking/data_tracking_image_2/training/image_02/${seq}/
stereo_path=/home/chen/datasets/kitti/tracking/stereo/training/${seq}/
calib_path=/home/chen/datasets/kitti/tracking/data_tracking_calib/training/calib/${seq}.txt
./bin/StereoToPointCloud ${left_path} ${stereo_path} ${calib_path}
```



