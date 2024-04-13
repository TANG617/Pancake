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

![image-20240413123419857](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413123419SmuVoYimage-20240413123419857.png)

![image-20240413123432497](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413123432kZOh2nimage-20240413123432497.png)

## 软件设计

![image-20240413121147697](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121147XatzeGimage-20240413121147697.png)

### STM32 部分

![image-20240413121210083](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121210D5dBLVimage-20240413121210083.png)

#### LVGL移植

![image-20240413123352448](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413123352w7oEDBimage-20240413123352448.png)

#### CAN协议调试

![image-20240413123512891](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413123512pwVBx5image-20240413123512891.png)

#### BLE遥控测试以及电机测试

![image-20240413123504031](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413123504RWSLCjimage-20240413123504031.png)

#### 负载测试

![image-20240413123456112](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413123456HcrTZOimage-20240413123456112.png)

### Jetson 部分

![image-20240413121424316](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413121424mermifimage-20240413121424316.png)

#### Kinect V2 + ROS 驱动测试

![image-20240413123208426](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413123208RaSufgimage-20240413123208426.png)

#### ROS远程控制测试

![image-20240413123158105](https://cdn.jsdelivr.net/gh/TANG617/images@master/202404131231586vP5Ciimage-20240413123158105.png)

#### 视觉SLAM



![image-20240413123245372](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413123245yxe7zSimage-20240413123245372.png)

#### 自动化运行测试

![image-20240413123301087](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413123301wxkuHlimage-20240413123301087.png)

![image-20240413123309381](https://cdn.jsdelivr.net/gh/TANG617/images@master/20240413123309qeNGebimage-20240413123309381.png)

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