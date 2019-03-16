---
title: Modern robotics Chapter 4
date: 2019-03-16 23:18:23 +0800
layout: post
permalink: /blog/2019/03/16/ModernRoboticsCh4.html
categories:
  - 机器人学
tags:
  - Modern Robotics
---

# Ch4-Forward Kinematics

## Summary:

1. 运动学是后面计算雅克比和动力学的基础。
2. 附录里简单比较了一下DH和PoE的区别，我也是因为PoE的优点才考虑学一学这套符号体系。

#### Production of Exponentials Formula(PoE)

和DH参数需要在每一个link都定义一个frame不同，PoE形式只需要定义我们关心的末端坐标系{b}和基座标系{s}，或者我们也可以定义每一个link的frame：基坐标系{s}为{0}，之后每个关节后的link都定义为{i}，如果n个关节的话，末端还可以定义为{n+1}（或者直接认为{n}就是末端坐标系）

##### A. screw axes 在基坐标系下

PoE形式的核心想法是每一个joint都对它以下的所有link有一个screw运动，只要设置初始关节角为0，每个关节的正方向都确定之后，假设SE(3)下的M是一开始末端的坐标系相对于fixed base的构型。那么可以从最后一个关节反推到所有关节都有运动的末端变换矩阵：

> 因为都是相对于fixed base，所以都是对M或者子节点的变换矩阵左乘


$$
T=e^{[S_1]\theta_1}...e^{[S_{n-1}]\theta_{n-1}}e^{[S_n]\theta_n}M
$$


> 注意：S一定是要在fixed frame下计算，M是初始的末端变换阵

##### 例子

简单的例子就不举了，诀窍就在于每一个旋转轴都是按照基坐标系取，并且对应的线速度只考虑角速度和到基坐标系的外积（qxw = -[w]q）

![Fig1](https://raw.githubusercontent.com/whtqh/image_files/master/ModernRobo_CH4_Fig1.jpg)

 如图是非常经典的UR5臂的结构，用PoE的方法建模：



|  i   |  \omega_i  |            v_i             |
| :--: | :--------: | :------------------------: |
|  1   | (0, 0, 1)  |         (0, 0, 0)          |
|  2   | (0, 1, 0)  |        (-H_1, 0, 0)        |
|  3   | (0, 1, 0)  |       (-H_1, 0, L_1)       |
|  4   | (0, 1, 0)  |    (-H_1, 0, L_1 + L_2)    |
|  5   | (0, 0, -1) |    (-W_1, L_1 +L_2, 0)     |
|  6   | (0, 1, 0)  | (H_2  - H_1, 0, L_1 + L_2) |



简单的说就是用右手定则，旋转角速度叉乘screw轴到base的线速度在base下的大小即为v_i的分量。


$$
T(\theta)=e^{[S_1]\theta_1}...e^{[S_{5}]\theta_{5}}e^{[S_6]\theta_6}M
$$


#### B. Screw Axes 在末端坐标系下

这里涉及到变换基坐标系，很容易想到上一章讲到的Adjoint 变换：


$$
S_a=[Ad_{T_{ab}}]S_b\\
S_b^{i}= [Ad_{T_{bs}}]S_s^{i} = [Ad_{M^{-1}}]S_s^{i}\\
\to [S_b^{i}] = M^{-1}[S_s^{i}]{M^{-1}}^{-1}
$$


这里通过坐标系的变换，我们可以将T(\theta)表示为M在左侧的形式：


$$
e^{M^{-1}PM} = M^{-1} e^{P} M\\ 
\to T(\theta)=e^{[S_1]\theta_1}...e^{[S_n]\theta_n}M\\
=e^{[S_1]\theta_1}...Me^{M^{-1}[S_n]M\theta_n}M^{-1}M \\
=Me^{M^{-1}[S_1]M\theta_1}M^{-1}...Me^{M^{-1}[S_{n-1}]M\theta_{n-1}}e^{M^{-1}[S_n]M\theta_n} \\
=Me^{M^{-1}[S_1]M\theta_1}...e^{M^{-1}[S_n]M\theta_n} \\
=Me^{{\mathcal{B}_1}\theta_1}...e^{{\mathcal{B}_n}\theta_n}
$$


最终相当于在body坐标系下从第一个关节开始不断地对M矩阵做右乘变换矩阵的操作，也就是相对初始的末端body坐标系下按照初始关节顺序和相对末端的旋转轴不断的变换。

> 比较space-form的公式和body-form公式：发现二者顺序的不同，并且S_i不受更远端的关节影响（只要从joint0到该关节构型固定，则S_i固定），类似的B_i不受更近端关节的影响。



正运动学由于有了之前的基础，变得异常的简单...em...是的



#### URDF(Universal Robot Description Format)

这算是个彩蛋？URDF在仿真中描述机器人的构型非常的有用，几乎包含了运动学和动力学所有需要的参数。

> 唯一的问题是URDF只能描述树状结构的机器人构型，对于闭链结构貌似无能为力，需要手动加dummy点。

![Fig2](https://raw.githubusercontent.com/whtqh/image_files/master/ModernRobo_CH4_Fig2.jpg)


