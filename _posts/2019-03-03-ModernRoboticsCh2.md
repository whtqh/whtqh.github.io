---
title: Modern robotics Chapter 2
date: 2019-03-03 20:15:23 +0800
layout: post
permalink: /blog/2019/03/03/ModernRoboticsCh2.html
categories:
  - 机器人学
tags:
  - Modern Robotics
---

# Modern Robotics: Ch2-Configuration Space(C-Space)

## Summary:

1. 构位空间(Configuration Space): 顾名思义，构位即机器人系统的明确描述（每个点都能精确定义，涉及到自由度的概念也可以解释为每个自由度的位置都确定）；而构位空间指的是所有构位组成的控件。
2. 自由度(Degrees of Freedom)：在介绍构位空间之前引入的概念，以及计算自由度的公式Grübler's Formula
3. 工作空间(Workspace)：机器人所在的空间（欧式），也可以指的是末端可达的空间，和具体执行的任务无关。
4. 任务空间(Task Space)：涉及到具体的任务所表述的空间，例子是在纸上写字，task space 约束在二维平面而不是三维。
5. C-Space 描述：显式参数，关节空间的描述，坐标系数量和自由度数量一致。隐式约束，m个坐标系，m-n个约束，n个自由度，此时C-Space可以看做是m维空间中嵌入的n维曲面。

### Grübler's Formula

计算自由度的方法：

$$
dof=m(N-1-J)+\sum_{i=1}^{J}f_i=m(N-1)-\sum_{J}^{i=1}c_i
$$

该方法计算出来的自由度是实际自由度的一个下界[特例：parallelogram linkage]

![Fig1](https://raw.githubusercontent.com/whtqh/image_files/master/ModernRobo_CH2_Fig1.jpg)

![Fig2](https://raw.githubusercontent.com/whtqh/image_files/master/ModernRobo_CH2_Fig2.jpg)

对于平面机器人来说m = 3，对于空间中的机构m=6。

计算例子在作业中可以练习。

### C-Space的表达

C-Space可以用向量来表达，但是和参考系和坐标轴的选取有关系，但是他的拓扑结构是确定不变的。

这里涉及到显式坐标系表达（关节空间）以及隐式约束表达。显式表达往往会存在奇异点（Singularities）比如位置的突变，而隐式则没有这个问题。Lynch 在这本书中贯穿的思想就是用隐式表达，带有6个约束的3x3的姿态矩阵来表达姿态，这样可以利用线性代数进行计算。[四元数或者其他一些表达也具有非奇异性质，但是也有各自的问题，一个姿态对应两个四元数，改进的Rodrigues参数貌似解决了一些问题？]

#### 构位和速度的约束

如果机器人带有闭链，此时用隐式约束能更方便的表示构位。

对有n个关节的带闭链的机器人，定义闭链约束方程：

$$
g(\theta)=\begin{bmatrix} g_1({\theta_1,...,\theta_n}) \\ ...\\g_2({\theta_1,...,\theta_n}) \\ \end{bmatrix}\quad = 0 
$$

$$
\to \frac{d}{dt}g(\theta(t)) = 0
$$

$$
\frac{\partial g}{\partial \theta}(\theta)\dot{\theta}=0=A(\theta)\dot{\theta}, A(\theta)\in \Bbb{R}^{k\times n}
$$


可以看到闭链结构的关节速度存在约束，这也叫做**Pfaffian Constraints**，此时速度的约束到位置的约束关系是不可积分的。这种不可积分的约束也被叫做 **Nonholonomic Constraint** 。

由于这种约束的存在，会导致系统当前状态下速度的可行解维数降低，但这并不意味着可达C-Space的降维。

(简单例子：硬币只能往前滚动或者原地旋转，并不能侧移，但是经过类似侧方停车的操作，在空间位置上最终能间接实现侧方移动)

## Example & HomeWork

#### Delta Robot(3-DOF)

![Fig3](https://raw.githubusercontent.com/whtqh/image_files/master/ModernRobo_CH2_Fig3.jpg)

如图所示的Delta Robot，如果用Grubler公式进行计算：
$$
dof=6(17-1-21)+9\times1+12\times3=15
$$
然而实际末端的自由度只有3个（笛卡尔空间xyz方向的运动），另外12个自由度由平行机构限制。

#### C-Space of N-Dimentional Space Rigid

n=2时，C-Space为平面上点的坐标加上一个方向（可以理解为绕垂直平面的一个旋转）

$$
\Bbb{R}^2\times S^1
$$

n=3时，C-Space是空间中的点坐标加上三维姿态的朝向，关于姿态可以首先在该点为球心的球面上找一个方向，绕该方向将三个坐标轴之一对齐，然后剩下的二维平面坐标系再从垂直于平面找一个轴旋转对齐即可。所以关键是旋转轴的选取需要在n-1的Sphere上进行。

因此当n>3时，也可以理解C-Space是

$$
\Bbb{R}^{n}\times S^{n-1}\times S^{n-2}\times ...\times S^{1}
$$

#### 人手臂建模(7-DOF)

![Fig4](https://raw.githubusercontent.com/whtqh/image_files/master/ModernRobo_CH2_Fig4.jpg)

例子之一，建模方式多种多样，一般保证自由度为7即可。

#### 挖掘机自由度

![Fig5](https://raw.githubusercontent.com/whtqh/image_files/master/ModernRobo_CH2_Fig5.jpg)

该例子是一个比较好的例子，要注意多个R关节重叠的时候记得都要算上去。

$$
m=3,N=14,J=18,\sum{f_i}=18\to dof = 3\times(14-1-18)+18
$$

#### 并联腿的优缺点

![Fig6](https://raw.githubusercontent.com/whtqh/image_files/master/ModernRobo_CH2_Fig6.jpg)

画出该并联机构的工作空间如下

![Fig7](https://raw.githubusercontent.com/whtqh/image_files/master/ModernRobo_CH2_Fig7.jpg)

可以看出来，工作空间相对于串联机构还是少了一些的。

但是优点也比较明显：[1] 刚度相对较高。[2] 质量分布更靠近基座，减小了惯量给电机带来的压力。
