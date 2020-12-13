# 单步运动主代码main()入口：
## Input
`wall_foot_step[N][3]` -> 爬壁的每一步的落脚点位置[N : x\*y\*z, unit: m]  <br>
`step_option[N]`  ->  爬壁每一步的情况(1表示壁面内部，2表示跨壁面)[N : 1/2]<br>

## Output
`base_coordinates[N][3]` -> 基坐标 (N: x\*y\*z)<br>
`base_coordinate_system[N][3][3]` -> 基坐标系 (N: 3\*3, Rx-Ry-Rz) <br>
`joint_angle[N][5]` -> 关节旋转角度(绝对角度, unit: degree) (N: 1*5) <br>


# 预处理阶段

## 1、壁面收缩处理(对壁面进行吸附末端半径收缩)

### Input
`surfaces[N][n][3]` ->  每个壁面的顶点[N: n * 3, N个壁面，每个壁面n个顶点，每个顶点三个坐标分量, unit: m] <br>

`r` -> 壁面收缩半径(吸盘圆化后的半径)[unit: m] <br>

### Output
`shrink_surface[N][3][3]` -> 每个壁面收缩后的顶点[N: n * 3, N个壁面，每个壁面n个顶点，每个顶点三个坐标分量, unit: m] <br>

### process

```cpp
for each surfaces:
   for each point: 
       ang_cos <- dot(vn, v2)   // vn, v1为壁面的向量边
       ang_sin <- sqrt(1 - ang_cos^2)  
       l <- r/ang_sin  // l为每一顶点所需要伸展的距离
       shrink_point <- former_point + vn*l + v2*l  
```

## 2、每个单步运动三个点所在的面(以此判断这步运动是在哪个壁面运动，到达哪个壁面)

### Input
` wall_planning_points[N][3] == wall_foot_step`  -> 爬壁的每一步的落脚点位置[N : x\*y\*z, unit: m] <br>


### Output
`surface_step_index[N-2][3]` -> 每次单步运动的三个点(base, end, start)所在的壁面[N-2: 3] <br>
`surface_step[N-2][n][3]` -> 记录下这个每次单步运动的壁面坐标(base, end, start)[N-2: n*3] <br>


### process

```cpp
for each moving:
   for  each surfaces:
       if  the base_point in the surface  then:
              surface_step_index(1) <- surface_idx 
              surface_step(1) <- surfaces{i}

       if  the end_point in the surface  then:
              surface_step_index(2) <- surface_idx 
              surface_step(2) <- surfaces{i}

       if  the start_point in the surface  then:
              surface_step_index(3) <- surface_idx 
              surface_step(3) <- surfaces{i}

```

### [判断点是否在壁面上](./base_code.md##1、判断点是否在空间多边形内部)

## 3、获得基座坐标系和起点坐标系

### Input
`base_surface[N][3]` -> 基座坐标壁面，也就是前面的`surface_step{1}` [n*3] <br>
`step_option[N]`  ->  爬壁每一步的情况(0表示壁面内部，1表示跨壁面)[N : 1/0]<br>
`first_joint` -> 机器人的起始关节[0  20  -40  20 0] <br>


### Output

`base_normal_vector[N][3]` <- N个基座壁面的法向量 [N: 3]<br>
`base_coordinate_system[N][3][3]` <- 基坐标系 [N: 3\*3, Rx-Ry-Rz]<br>
`start_coordinate_system_to_base[N][3][3]` <- 起始坐标系在基座标系下的表示 [N: 3\*3, Rx-Ry-Rz] <br>


### process

```cpp

for each moving:
       base_normal_vector <- norm( cross(base_surface_vector1, base_surface_vector2) )
       base_x <- base_surface_vector1
       base_z <- base_normal_vector
       base_y <- cross(base_x, base_z)
       base_coordinate_system <- [base_x', base_y', base_z']

       //先求出起始壁面的姿态，转换到基座坐标系下，然后将y、z轴方向取反'
       if stepOption == 1 then :  
              // 壁面内，则起始坐标系就是基座标系的y,z轴取反
              start_coordinate_system_to_base <- eye(3)
              start_coordinate_system_to_base(1:3,2) <- base_coordinate_system(1:3,2)*-1
              start_coordinate_system_to_base(1:3,3) <- base_coordinate_system(1:3,3)*-1

       if stepOption == 2 then :
              // 壁面间，计算起始壁面的坐标系，再转换到基座标系下
              start_z <- norm( cross(start_surface_vector1 , start_surface_vector2) )
              start_x <- start_surface_vector1
              start_y <- cross(start_x, start_z)
              start_coordinate_system <- [start_x', start_y', start_z']

              start_coordinate_system_to_base <- (base_coordinate_system') * start_coordinate_system
              start_coordinate_system_to_base(1:3, 2) <- start_coordinate_system_to_base(1:3, 2)*(-1)
              start_coordinate_system_to_base(1:3, 3) <- start_coordinate_system_to_base(1:3, 3)*(-1)

```

## 4、障碍物数据处理

### Input
`obstacle_border[N][n][4][2]` <- n个障碍物在N个壁面上的4个坐标点[N: (n\*4\*2] <br>
`obstacle_height[N][n]` <- n个障碍物在N个壁面上的高度[N: n\*1] <br>
`base_normal_vector[N][3]` <- N个基座壁面的法向量 [N: 3]<br>


### Output
`ranging_point` <- 机器人与障碍物之间的测距点
``

