# WGS84 与 UTM 坐标系

## 坐标系简介

### WGS84

WGS84:World Geodetic System 1984，是为[GPS全球定位系统](https://baike.baidu.com/item/GPS全球定位系统/5864639?fromModule=lemma_inlink)使用而建立的[坐标系统](https://baike.baidu.com/item/坐标系统/4725756?fromModule=lemma_inlink)。通过遍布世界的[卫星观测站](https://baike.baidu.com/item/卫星观测站/22068811?fromModule=lemma_inlink)观测到的坐标建立，其初次WGS84的精度为1-2m。

[WGS-84坐标系](https://baike.baidu.com/item/WGS-84坐标系?fromModule=lemma_inlink) [1] 的几何意义是：坐标系的原点位于地球质心，z轴指向（国际时间局）BIH1984.0定义的协议地球极(CTP)方向，x轴指向BIH1984.0的零度子午面和CTP赤道的交点，y轴通过右手规则确定。

来源：百度百科

### UTM

UTM(Universal Transverse Mercator Grid System, 通用横墨卡托网格系统)，坐标是一种平面直角坐标，这种坐标格网系统及其所依据的投影已经广泛用于地形图，作为卫星影像和自然资源数据库的参考格网以及要求精确定位的其他应用。

参考：

1. [什么是UTM坐标系](https://zhuanlan.zhihu.com/p/99772254)
2. [UTM坐标系简述](https://zhuanlan.zhihu.com/p/255036625)
3. [卫星遥感影像中常见的WGS84 UTM坐标系](http://www.zj-view.com/newsitem/278062789/)

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



