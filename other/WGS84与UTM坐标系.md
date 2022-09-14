# WGS84 与 UTM 坐标系

## 坐标系简介



### WGS84



### UTM



## 坐标系转换

参考链接：[WGS84 与 UTM 互转（Python代码版）](https://blog.csdn.net/qq_41204464/article/details/118905641)

借助第三方库进行坐标系之间的转换。

**安装 pyproj**

```bash
pip install pyproj
```

官方文档：[pyproj官网文档学习](https://www.osgeo.cn/pyproj/examples.html)

**WGS84 转 UTM**

```python
'''
WGS84的经纬度 转 UTM的x,y
'''
from pyproj import Transformer
 
 
# 参数1：WGS84地理坐标系统 对应 4326 
# 参数2：坐标系WKID 广州市 WGS_1984_UTM_Zone_49N 对应 32649
transformer = Transformer.from_crs("epsg:4326", "epsg:32649") 
 
lat = 22.744435950
lon = 113.595417400
x, y = transformer.transform(lat, lon)
print("x:", x, "y:", y)
```

**UTM 转 WGS84**

```python
'''
UTM的x,y 转 WGS84的经纬度
'''
from pyproj import Transformer
 
# 参数1：坐标系WKID 广州市 WGS_1984_UTM_Zone_49N 对应 32649
# 参数2：WGS84地理坐标系统 对应 4326
transformer = Transformer.from_crs("epsg:32649", "epsg:4326")
 
x = 766544.7801555358
y = 2517564.4969607797
lat, lon = transformer.transform(x, y)
print("x:", x, "y:", y)
print("lat:", lat, "lon:", lon)
```

**转换到 mercator 原点**

```python
'''
WGS84的经纬度 转 UTM的x,y
'''
from pyproj import Transformer
 
 
# 参数1：WGS84地理坐标系统 对应 4326 
# 参数2：坐标系WKID 苏州市 WGS_1984_UTM_Zone_51N 对应 32651
transformer = Transformer.from_crs("epsg:4326", "epsg:32651") 
 
lat = 31.42889 
lon = 120.634974
x, y = transformer.transform(lat, lon)
x -= 275000.02		# 苏州所处的 mercator 原点
y -= 3479281.50
print("x:", x, "y:", y)
```



