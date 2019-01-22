---
title: 机器人仿真填坑计划
date: 2019-01-22 17:17:28 +0800
layout: post
permalink: /blog/2019/01/22/Simulation-Overview.html
categories:
  - 动力学仿真
tags:
  - 仿真
---

# 动力学仿真

### 寒假填坑计划

在本研期间，接触过的仿真平台包括：Gazebo, V-Rep, Choreonoid 和 Simscape Multibody。

机器人动力学仿真一直是个大坑，之前也没有系统性的总结，在关键的接触模型的参数上也只是照葫芦画瓢，并没有专门的花时间研究。但是随着项目的推进，觉得有必要在仿真上下一番功夫，做调研的同时，自己也亲自体验一下这些仿真平台的性能，并且顺手写个日志记录一下。

### 对于仿真的理解

为什么要有仿真？（因为硬件贵呀~）

自古以来，仿真都是在上实物做实验之前的必备，以我们自动化系的课程为例，从电路原理的仿真(Multisim)，到后面数电的仿真模拟，再到我后来开始接触刚体动力学以后从手算动力学公式，再到建模仿真。以上的种种，无非是将大量的数学计算交给计算机来做，便于我们检查算法的正确性以及不同情况下是否有考虑不周的问题。

但是，仿真毕竟是仿真，和实物还是差得很远，我一直认为根据仿真只能定性的去分析算法，实际参数如果能够在量级上和仿真一致，那真的是谢天谢地了。在足式机器人领域，这样的问题会更加明显，尤其是在处理碰撞和与地面摩擦等问题上，我们使用的动力学引擎在解决碰撞和摩擦问题的时候为了计算速度通常会牺牲仿真的精度（动力学仿真的金主通常是游戏行业），而足式机器人行走的稳定性和能耗的关键往往就在于和地面接触的情况。

### 机器人仿真的架构

以人形机器人为例，参考DNN[1]中的图片，我们对仿真的架构做一个梳理：

![Simulation Structure](https://github.com/whtqh/image_files/raw/master/_8_2_Structure_Robot_Sim.png)

如图所示，要在仿真中描述一个机器人和环境（地面或者接触的物体）之间的交互，首先是要用描述语言将机器人的结构配置告诉仿真系统，其次仿真要能够计算动力学（关节空间到工作空间之间的换算关系，比如力矩和加速度）；同时还需要计算是否和环境有碰撞以及计算接触点的力（需要精确的Contact Model来建模）；还需要设计各个关节的控制器（对外的接口，需要关节角或者关节力矩作为输入），和机器人本体上的各种传感器（IMU测量某一个点的姿态和加速度等信息，关节角信息以及在关键的地方设置力和力矩传感器测量力信号）。最后就是人机交互所需要的界面和3D可视化的模块，这个对用户的体验也相当重要！

总结一下，大概就是：(1) 模型载入功能，(2) 碰撞检测和接触模型，(3) 控制器，(4) 动力学仿真(ID&FD)，(5) 可视化

以上是一个基本合格的动力学仿真系统所需要的，更高级的仿真系统无非是在计算规模上、精度还有实时画面的渲染效果上有更一步的表现。

然后总结一下比较流行的仿真软件（不断更新中）：

#### 动力学引擎小知识

"Maximal coordinate solvers like **ODE** and **Bullet** perform well when simulating cluttered environments, while **Featherstone-based solvers** like **DART** and **Simbody** are potentially more accurate in simulating articulated systems such as **humanoid robots**."



Wiki 中有关于robot simulator的表格，但是部分介绍有点过时？

https://en.wikipedia.org/wiki/Robotics_simulator

我们仅选取和足式机器人关系较大的仿真软件进行介绍（好多类似MORSE这样的更适合轮式机器人）



#### 1. Gazebo

官网：http://gazebosim.org/

平台：Mac/Linux/Win，但是Gazebo一般会配合ROS，因此通常在Linux下用。

动力学引擎：默认用ODE，新版本支持源码编译使用 Bullet(used in Gaming), DART(Computer Graphics and Robot Control), Simbody(used in Biomechanics)。

#### 2. V-Rep

官网：

平台：Mac/Linux/Win, 商业软件、教育版免费，各个平台下都差不多。

#### 3. Webots

平台：

#### 4. Choreonoid

平台：Linux/Win

动力学引擎：默认AIST Engine，从源码编译能够实现ODE/Bullet/...

HRP系列，

#### 5. Simscape Multibody

平台：Linux/Mac/Win

MATLAB商业软件包之一

#### 6. Mujoco

平台：

#### 7. Bullet Based(PyBullet)

平台：

动力学引擎：Bullet

#### 8. OpenAI Gym

平台：

动力学引擎：Mujoco/DART

#### 9. ADAMS







### Reference

[1] Chapter 8 - Simulation. Humanoid Robots. T. Tsujita, D. N. Nenchev and A. Konno, Butterworth-Heinemann**:** 421-471

[2] Gazebo, http://gazebosim.org/.

[3] Simscape Multibody, https://jp.mathworks.com/products/simmechanics.html.

[4] V-Rep, http://www.coppeliarobotics.com/index.html.

[5] Webots, https://www.cyberbotics.com/#webots.

[5] S. Nakaoka, Choreonoid: extensible virtual robot environment built on an integrated GUI framework, in:
IEEE/SICE International Symposium on System Integration, 2012, pp. 79–85.