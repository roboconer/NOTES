## 1、判断点是否在空间多边形内部

insidepoly3(point,poly)：<br>

### Input: 
`point` -> 要判断的点(1: 3\*3) <br>
`poly`  -> 空间多边形的顶点坐标(n: 3\*3) <br>

### Output:
`state` -> 点是否在空间多边形内部[1/0 : 在/不在 空间多边形上]<br>

### process:
<img src="https://cdn.mathpix.com/snip/images/Ikycdwx2oIh5eK3FPZS5fnNoNofo4BjWHPFqOXmJ3nQ.original.fullsize.png">

- 点在多边形所在平面(无限)的判断

```cpp
Normal vector <- cross(v1, v2)  // 法向量Normal vector， v1,v2为多边形的两个相邻向量边

test vector <- norm(point - poly{1}) // 测试向量

if abs(dot(test vector ,Normal vector))>0 then  // 点乘，两向量不垂直，点不在空间面内
    state = 0
    return 
```
- 点在多边形内部的判断

```cpp
for each point_in_poly:

    test_vector <- point - poly{i}
    front_vector <- poly{i} - poly{i+1}
    mean_vector <- mean_p - poly{i}
    // 当按同一旋转方向遍历多边形所有边时，过渡吸附点应在所有边的同一侧
    if (dot (cross(test_vector, front_vector), cross(mean_vector, front_vector)) < 0) then   
        state <- 0   // 小于0说明不符合,state = 0
        break
    
if everything is ok ,then 
    state = 1
```



