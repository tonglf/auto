# Eigen中四元数、欧拉角、旋转矩阵、旋转向量之间的转换

## 旋转向量

1，初始化旋转向量：旋转角为alpha，旋转轴为(x,y,z)

```cpp
Eigen::AngleAxisd rotation_vector(alpha,Vector3d(x,y,z))
```

2，旋转向量转旋转矩阵 

```cpp
Eigen::Matrix3d rotation_matrix;

// 1
rotation_matrix = rotation_vector.matrix();

// 2
rotation_matrix = rotation_vector.toRotationMatrix();
```

3，旋转向量转欧拉角(X-Y-Z，即RPY)

```cpp
Eigen::Vector3d eulerAngle = rotation_vector.matrix().eulerAngles(2, 1, 0);
```

4，旋转向量转四元数

```cpp
Eigen::Quaterniond quaternion(rotation_vector);
```

## 旋转矩阵

1， 初始化旋转矩阵

```cpp
Eigen::Matrix3d rotation_matrix;
rotation_matrix << x_00, x_01, x_02, x_10, x_11, x_12, x_20, x_21, x_22;
```

2， 旋转矩阵转旋转向量

```cpp
// 1
Eigen::AngleAxisd rotation_vector(rotation_matrix);

// 2
Eigen::AngleAxisd rotation_vector;
rotation_vector = rotation_matrix;

// 3
Eigen::AngleAxisd rotation_vector;
rotation_vector.fromRotationMatrix(rotation_matrix);,
```

3， 旋转矩阵转欧拉角(Z-Y-X，即RPY)

```cpp
Eigen::Vector3d eulerAngle = rotation_matrix.eulerAngles(2, 1, 0);
```

4，旋转矩阵转四元数,

```cpp
// 1
Eigen::Quaterniond quaternion(rotation_matrix);

// 2
Eigen::Quaterniond quaternion;
quaternion = rotation_matrix;
```

## 欧拉角

1， 初始化欧拉角(Z-Y-X，即RPY)

```cpp
Eigen::Vector3d eulerAngle(yaw, pitch, roll);
```

2， 欧拉角转旋转向量

```cpp
Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2), Vector3d::UnitX()));
Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1), Vector3d::UnitY()));
Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0), Vector3d::UnitZ()));

Eigen::AngleAxisd rotation_vector;
rotation_vector = yawAngle * pitchAngle * rollAngle;
```

3， 欧拉角转旋转矩阵

```cpp
Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitX()));
Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitZ()));

Eigen::Matrix3d rotation_matrix;
rotation_matrix = yawAngle * pitchAngle * rollAngle;
```

4， 欧拉角转四元数 

```cpp
Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2),Vector3d::UnitX()));
Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1),Vector3d::UnitY()));
Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0),Vector3d::UnitZ()));

Eigen::Quaterniond quaternion;
quaternion=yawAngle * pitchAngle * rollAngle;
```

## 四元数

1，初始化四元数

```cpp
Eigen::Quaterniond quaternion(w, x, y, z);
```

2， 四元数转旋转向量

```cpp
Eigen::AngleAxisd rotation_vector(quaternion);

Eigen::AngleAxisd rotation_vector;
rotation_vector = quaternion;
```

3， 四元数转旋转矩阵

```cpp
// 1
Eigen::Matrix3d rotation_matrix;
rotation_matrix = quaternion.matrix();

// 2
Eigen::Matrix3d rotation_matrix;
rotation_matrix = quaternion.toRotationMatrix();
```

4， 四元数转欧拉角(Z-Y-X，即RPY)

```cpp
Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2, 1, 0);
```