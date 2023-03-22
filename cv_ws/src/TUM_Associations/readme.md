

# TUM_Associations



## TUM_AssociationGenerate

用于生成ORB-SLAM3所需的TUM数据集的索引文件。

在使用时，先启用 `TUM_AssociationGenerate` 程序，然后播放TUM的数据包.bag文件，生成索引文件.txt。

生成的文件格式如下：

```txt
1341847980.722988 rgb/1341847980.722988.png 1341847980.723020 depth/1341847980.723020.png
1341847980.754743 rgb/1341847980.754743.png 1341847980.754755 depth/1341847980.754755.png
1341847980.786856 rgb/1341847980.786856.png 1341847980.786879 depth/1341847980.786879.png
1341847980.822978 rgb/1341847980.822978.png 1341847980.822989 depth/1341847980.822989.png
...
```



* 用法

```shell
rosrun TUM_Associations TUM_AssociationGenerate ${save_path}
```











