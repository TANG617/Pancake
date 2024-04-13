# PANCAKE: 分布式运输小车集群

## 概述

### 传统的智能运输方案

传统AGV（= Automated Guided Vehicle自动导引车）

| 优势                                                         | 劣势                                      |
| ------------------------------------------------------------ | ----------------------------------------- |
| 节省人力成本 <br />智能化管理 <br />高搬运负载能力 <br />高可靠性 | 初期投资高* <br />性能过剩 <br />场景有限 |

Reference: AGV Market Investigation by Loup Ventures

<img src="https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413120051ZVa9wRimage-20240413120051217.png" alt="image-20240413120051217" style="zoom:25%;" />

### 我们的目标

- 低成本部署
- 广泛的使用场景
- 智能的使用体验

### 我们的方案

![image-20240413120508363](https://cdn.jsdelivr.net/gh/TANG617/images@master/202404131205089StiaWimage-20240413120508363.png)

## 硬件设计

![image-20240413120641864](https://cdn.jsdelivr.net/gh/TANG617/images@master/202404131206416Jginnimage-20240413120641864.png)

### 结构设计



<img src="https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413120653Uj7fWNimage-20240413120653272.png" alt="image-20240413120653272" style="zoom:25%;" />

![image-20240413121033322](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121033j1ayZvimage-20240413121033322.png)

### 核心运动控制器

![image-20240413120851098](https://cdn.jsdelivr.net/gh/TANG617/images@master/202404131208510woyiiimage-20240413120851098.png)

### 核心电源树

![image-20240413120926105](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413120926gH2obximage-20240413120926105.png)

### 核心组合

![image-20240413120956736](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413120956phHW3Simage-20240413120956736.png)

### 总装

![image-20240413121057627](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121057ciGzXfimage-20240413121057627.png)

![image-20240413121103577](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121103zm009Yimage-20240413121103577.png)

## 软件设计

![image-20240413121147697](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121147XatzeGimage-20240413121147697.png)

### STM32 部分

![image-20240413121210083](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121210D5dBLVimage-20240413121210083.png)

#### LVGL移植

![image-20240413121231208](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121231JrVwXGimage-20240413121231208.png)

#### CAN协议调试

![image-20240413121258862](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121258EnmGuwimage-20240413121258862.png)

#### BLE遥控测试以及电机测试

![image-20240413121321794](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121321r7Z4Hqimage-20240413121321794.png)

#### 负载测试

![image-20240413121351291](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121351BcVi05image-20240413121351291.png)

### Jetson 部分

![image-20240413121424316](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121424mermifimage-20240413121424316.png)

#### Kinect V2 + ROS 驱动测试

![image-20240413121456425](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121456ifLoPjimage-20240413121456425.png)

#### ROS远程控制测试

![image-20240413121526428](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121526I0tAXEimage-20240413121526428.png)

#### 视觉SLAM



![image-20240413121554096](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121554PZ8Q7Dimage-20240413121554096.png)

#### 自动化运行测试

![image-20240413121628512](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121628ZWwQYnimage-20240413121628512.png)

![image-20240413121653965](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121654lwXSDOimage-20240413121653965.png)

## 性能指标

- 最大负载：100kg
- 最小速度：0.1m/s
- 最大速度：10m/s

## 场景案例

### 中等距离搬运

#### 搬运的货物

![image-20240413121953642](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121953nz7uXyimage-20240413121953642.png)

#### 现有的方案

![image-20240413122004924](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413122005dSQ0SCimage-20240413122004924.png)

#### 我们的方案

![image-20240413122102773](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413122102tH3ajYimage-20240413122102773.png)

### 短距离特殊搬运

#### 搬运的货物

![image-20240413122125955](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413122126MJicaEimage-20240413122125955.png)

#### 我们的方案

![image-20240413122141373](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413122141ZAuX68image-20240413122141373.png)

##### 集群技术

![image-20240413122303406](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413122303lsHl4Iimage-20240413122303406.png)