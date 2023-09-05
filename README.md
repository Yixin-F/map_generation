## map_generation
### This repo belongs to my online internship in State Gride

### 1) How to use
将变化单体替换进原始地图

param1: 平移
param2: 旋转
param3： 残缺
param4：缩放

./map pcd地图 pcd单体 param1 param2 param3 param4  + .....  （1执行，0不执行）

#*****************     sh run.sh    *****************

unit中存放了pcd单体和pcd地图

### 2) Basic Show
#### Gazebo Simulation
<centre>
<img src="https://github.com/Yixin-F/map_generation/blob/main/img/gazebo.png" width="60%">
</centre>

#### SLAM and Object Extraction
<centre>
<img src="https://github.com/Yixin-F/map_generation/blob/main/img/object.png" width="60%">
</centre>

#### Map Generation in Random
<centre>
<img src="https://github.com/Yixin-F/map_generation/blob/main/img/mg.png" width="60%">
</centre>

#### Difference Detection and Color
We detect the model difference between CAD provided and Point Cloud scaned, then color the different parts.

<centre>
<img src="https://github.com/Yixin-F/map_generation/blob/main/img/diff.png" width="60%">
</centre>
