# CV_Tools

## MaskUtils





### convert_seg_to_mask

用于将一个每个像素代表某个类别的语义分割图像，转换为只有前景、背景类别的mask分割图像。



语义分割图像的类别：

```txt
0-背景 1-天空 2-标志牌 3-机动车、非机动车、行人 4-实线 5-虚线 6-斑马线 7-停止线 8-箭头 9-电线杆 10-Freespace
```

在转换的时候，只将结果掩码的`3-机动车、非机动车、行人`区域设置为0，其它区域设置为255。

* 用法：

```shell
./convert_seg_to_mask ${img_path} ${save_path} ${mask_class_ids_str}
```

`${mask_class_ids_str}`的形式为 `"a b c"`，其中`a`、`b`、`c`是不同的类别ID。

* 示例

```shell
img_path=/home/cjq/dataset/YT_gaojia/bag_mask 
save_path=/home/cjq/dataset/YT_gaojia/bag_mask_binary


./convert_seg_to_mask ${img_path} ${save_path} "1 3" #掩玛掉天空和车辆
```

或

```shell
img_path=/home/cjq/dataset/YT_gaojia/bag_mask 
save_path=/home/cjq/dataset/YT_gaojia/bag_mask_car

./convert_seg_to_mask ${img_path} ${save_path} "3" #掩玛掉车辆
```





### visualze_mask_class

用于可视化Mask中某个类别的掩码



* 用法：

```shell
./visualze_mask_class ${img_path} ${class_id}
```

* 示例：

```shell
img_path=/home/cjq/dataset/YT_gaojia/bag_mask/1666501218.577000.png.png
class_id=0
./visualize_mask_class ${img_path} ${class_id}
```





### add_manual_mask

将手工设置的Mask，添加到已有的Mask上

* 用法

```shell
./add_manual_mask ${mask_dir_path} ${manual_mask_path} ${save_path}
```



* 示例

```shell
mask_dir_path=/home/cjq/dataset/YT_gaojia/bag_mask_binary
manual_mask_path=/home/cjq/dataset/YT_gaojia/20230419-130852.jpg
save_path=/home/cjq/dataset/YT_gaojia/bag_mask_binary_manual

./add_manual_mask ${mask_dir_path} ${manual_mask_path} ${save_path}
```





### erode_mask

腐蚀Mask，使得Mask白色区域（前景）区域减少。

* 用法

```shell
./erode_mask ${mask_dir_path} ${save_path}
```



* 示例

```shell
mask_dir_path=/home/cjq/dataset/YT_gaojia/bag_mask_binary_manual
save_path=/home/cjq/dataset/YT_gaojia/bag_mask_binary_manual_erode

./erode_mask ${mask_dir_path} ${save_path}
```

或

```shell
#扩大车辆掩玛的区域
mask_dir_path=/home/cjq/dataset/YT_gaojia/bag_mask_car
save_path=/home/cjq/dataset/YT_gaojia/bag_mask_car

./erode_mask ${mask_dir_path} ${save_path}
```



### mask_operation

将两个文件夹下的对应的Mask做逻辑运算

* 用法

```shell
./mask_operation ${mask0_dir} ${mask1_dir} ${mask_operation} ${save_path}
```

其中`mask_operation`为`and`或`or`



* 示例

```shell
mask0_dir=/home/cjq/dataset/YT_gaojia/bag_mask_binary_manual
mask1_dir=/home/cjq/dataset/YT_gaojia/bag_mask_car
mask_operation=and
save_path=/home/cjq/dataset/YT_gaojia/bag_mask_binary_manual_erode

./mask_operation ${mask0_dir} ${mask1_dir} ${mask_operation} ${save_path}
```





### mask_rgb_vis

将mask叠加到rgb图像上，用于可视化mask。

* 用法

```shell
./mask_rgb_vis ${mask_dir} ${rgb_dir} ${save_path}
```

或

```shell
./mask_rgb_vis ${mask_dir} ${rgb_dir}
```



示例：

```shell
mask_dir=/media/cjq/新加卷/datasets/220Dataset/22_GND_vslam/20230809_A12/20230805151650.846/dense/road_mask
rgb_dir=/media/cjq/新加卷/datasets/220Dataset/22_GND_vslam/20230809_A12/20230805151650.846/dense/images

./mask_rgb_vis ${mask_dir} ${rgb_dir}
```

或

```shell
/home/cjq/CLionProjects/CV_Tools/opencv/MaskUtils/bin/mask_rgb_vis  ./mask ./images
```









