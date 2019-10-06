---
title: S-R-S Arm IK Redundancy
date: 2019-10-06 16:01:12 +0800
layout: post
permalink: /blog/2019/10/06/SRS_Arm_Kinematics_Redundancy.html
categories:
  - 机器人学
tags:
  - Robotics Redundancy
---

# KUKA-iiwa LBR14 Kinematics

## Summary:
1. 解IK，考虑冗余臂角，7-DOF S-R-S 构型位置IK解析解



文献[1]对于7自由度冗余仿人臂的逆运动学位置解做了很好的分析，主要考虑关节有位置限制时逆运动学解问题：

相关工作暂时不表，只记录求解过程和避免奇异的细节。

![Fig1](https://raw.githubusercontent.com/whtqh/image_files/master/Paper_shimizu2008_7DOF_Fig1.jpg)

如图所示为常见的7-DOF机械臂，具有和人类相近的肩球关节和腕球关节以及肘部。

优点：和一般的6自由度机械臂相比，由于多了一个自由度，可以在保证末端位姿不变的情况下变换构型，达到避障、避免关节限幅（位置、速度、加速度），避免奇异，以及减小关节力矩消耗等作用。

缺点：自由度增加，串联系统的整体刚度下降，同时系统变得更加复杂（可能奇异的情况更多等等）。

一般的7自由度多为协作机械臂，也是利用上述特点达到工作中能够避免机械臂和接触到的人产生危险的碰撞等。

![Fig2](https://raw.githubusercontent.com/whtqh/image_files/master/Paper_shimizu2008_LBR_IIWA_Fig2.jpg)

常见的比如KUKA的iiwa力控协作臂，还有安川的SDA10、ABB的yumi协作双臂等等，这些都满足S-R-S构型。



常见的操作都是人为加一个约束，通过调整约束参数同样能够得到解析解（Pieper's Theorem: Closed-form solution always exists for any 6 dof robot with a spherical wrist）。



直接引出这篇文章的方法：

因为当末端tip位姿固定时，是能够求出肘关节的角度的（wrist, elbow, shoulder三点相对位置固定），那么通过引入三点所在的平面（Arm Plane），显然能够加入一个参数（臂角）

![Fig3](https://raw.githubusercontent.com/whtqh/image_files/master/Paper_shimizu2008_ArmPlane_Fig3.jpg)

臂角参数一方面解决了冗余自由度的问题，另外一方面可以直接优化该参数来满足一些限制。



#### DH建模


$$
\begin{array}{|c|c|c|c|c|}\hline i & {\theta_{i}} & {\alpha_{i}(\text { rad })} & {d_{i}} & {a_{i}} \\ \hline 1 & {\theta_{1}} & {-\pi / 2} & {d_{b s}} & {0} \\ {2} & {\theta_{2}} & {\pi / 2} & {d_{b s}} & {0} \\ {3} & {\theta_{2}} & {-\pi / 2} & {d_{s e}} & {0} \\ {4} & {\theta_{4}} & {\pi / 2} & {0} & {0} \\ {5} & {\theta_5} & {-\pi / 2} & {d_{e w}} & {0} \\ {6} & {\theta_{6}} & {\pi / 2} & {0} & {0} \\ {7} & {\theta_{7}} & {0} & {d_{w t}} & {0} \\ \hline\end{array}
$$


本文用的是标准的DH参数（一开始真没注意到，其实看表头a和d的下标就能看出来，改进型用的是i和i-1。

![STD DH 参数](https://raw.githubusercontent.com/whtqh/image_files/master/Paper_shimizu2008_kuka_std_dh.png)

至于DH参数下细节可以参见[另一篇文章](https://whtqh.github.io/blog/2019/10/03/DenavitHartenbergPara.html)

至于在计算Forward Kinematics时使用DH还是POE，其实都无所谓，DH优点是参数少（3n个描述结构，n个描述关节角），而PoE需要（6n个参数描述screw轴，n个描述关节角），但是PoE本质上并不需要Link的坐标系（和Joint重合了）。

总而言之，DH能干的事，PoE都能干，DH设计的目的是为了节约参数，仅此而已（还导致相邻关节平行会有奇异等等问题，导致了魔改版本加 \beta角）。


$$
T_{i-1,i} = ^{i-1}T_{i}=\left(\operatorname{Rot}_{x_{i-1}}\left(\alpha_{i-1}\right) \cdot \operatorname{Trans}_{x_{i-1}}\left(a_{i-1}\right)\right) \cdot \left(\operatorname{Rot}_{z_{i}}\left(\theta_{i}\right) \cdot \operatorname{Trans}_{z_{i}}\left(d_{i}\right)\right) \\
M_i =\left(\operatorname{Rot}_{x_{i-1}}\left(\alpha_{i-1}\right) \cdot \operatorname{Trans}_{x_{i-1}}\left(a_{i-1}\right)\right) \cdot  \operatorname{Trans}_{z_{i}}\left(d_{i}\right) \\
\operatorname{Rot}\left(\hat{\mathrm{z}}, \theta_{i}\right)=e^{\left[\mathcal{A}_{i}\right] \theta_{i}} \\
T_{0, n}=M_{1} e^{\left[\mathcal{A}_{1}\right] \theta_{1}} M_{2} e^{\left[\mathcal{A}_{2}\right] \theta_{2}} \cdots M_{n} e^{\left[\mathcal{A}_{n}\right] \theta_{n}} \\
= e^{\left[\mathcal{S}_{1}\right] \theta_{1}} \ldots e^{\left[\mathcal{S}_{n}\right] \theta_{n}} M
$$


因为每个joint的转动/平动都可以表示为[Ai]，在joint坐标系内的screw又可以通过坐标变换（Ad）转换为{0}坐标系下的screw：


$$
\left[\mathcal{S}_{b}\right]=T_{b a}\left[\mathcal{S}_{a}\right] T_{b a}^{-1} \\
\mathcal{S}_{b}=\mathrm{Ad}_{T_{b a}}\left(\mathcal{S}_{a}\right)
$$


因此只考虑FK计算末端的位姿的话，PoE貌似更方便？（感觉PoE的计算复杂度也并不是很高吗，本质上也还是齐次变换矩阵连乘，然后PoE的通用性会更加好一点）


$$
e^{[\mathcal{S}] \theta}=\left[\begin{array}{cc}{e^{[\omega] \theta}} & {\left(I \theta+(1-\cos \theta)[\omega]+(\theta-\sin \theta)[\omega]^{2}\right) v} \\ {0} & {1}\end{array}\right] \\
(不考虑~~\omega = 0, ||v|| = 1) \\
e^{[\hat{\omega}]\theta} = I+sin{\theta}[\hat{\omega}] + (1-cos{\theta})[\hat{\omega}]^2 \\
= \left[\begin{array}{ccc}{\mathrm{c}_{\theta}+\hat{\omega}_{1}^{2}\left(1-\mathrm{c}_{\theta}\right)} & {\hat{\omega}_{1} \hat{\omega}_{2}\left(1-\mathrm{c}_{\theta}\right)-\hat{\omega}_{3} \mathrm{s}_{\theta}} & {\hat{\omega}_{1} \hat{\omega}_{3}\left(1-\mathrm{c}_{\theta}\right)+\hat{\omega}_{2} \mathrm{s}_{\theta}} \\ {\hat{\omega}_{1} \hat{\omega}_{2}\left(1-\mathrm{c}_{\theta}\right)+\hat{\omega}_{3} \mathrm{s}_{\theta}} & {\mathrm{c}_{\theta}+\hat{\omega}_{2}^{2}\left(1-\mathrm{c}_{\theta}\right)} & {\hat{\omega}_{2} \hat{\omega}_{3}\left(1-\mathrm{c}_{\theta}\right)-\hat{\omega}_{1} \mathrm{s}_{\theta}} \\ {\hat{\omega}_{1} \hat{\omega}_{3}\left(1-\mathrm{c}_{\theta}\right)-\hat{\omega}_{2} \mathrm{s}_{\theta}} & {\hat{\omega}_{2} \hat{\omega}_{3}\left(1-\mathrm{c}_{\theta}\right)+\hat{\omega}_{1} \mathrm{s}_{\theta}} & {\mathrm{c}_{\theta}+\hat{\omega}_{3}^{2}\left(1-\mathrm{c}_{\theta}\right)}\end{array}\right]
$$


个人认为FK中，为了通用性和易于理解，PoE>DH，至于别的地方，哪个好用用哪个吧~~

如果只是建立相对坐标的话，PoE的方法也更容易理解啊，而且也不难算 = =



#### Inverse Kinematics

在解决7-Dof机械臂SRS构型的逆解时，一般都用臂角（arm angle）来描述这个多出的自由度：

通过将\theta_3 初始化为0，然后根据建立臂角和\theta_3和其余关节的关系。

【注意】当\theta_3不动时，该机械臂其实已经退化为符合Pieper条件之一的6自由度臂（末端3轴交于1点）

这样无论是通过几何法还是分离末端位置和姿态矩阵，都能够得到逆解的方法：


$$
T_{0,6}=T_{0,3}T_{3,6}
$$


当然使用旋量计算时同样能够利用该性质，这里先用几何法解释。


$$
\begin{array}{l}{^{0} \boldsymbol{x}_{7}=^{0} l_{b s}+^{0} \boldsymbol{R}_{3}\left\{^{3} l_{s e}+^{3} \boldsymbol{R}_{4}\left(^{4} l_{e w}+^{4} \boldsymbol{R}_{7}^{7} l_{w t}\right)\right\}} \\ {^{0} \boldsymbol{R}_{7}=^{0} \boldsymbol{R}_{3}^{3} \boldsymbol{R}_{4}^{4} \boldsymbol{R}_{7}}\end{array}
$$


利用三角关系，以及肩到腕的距离x_sw，可列等式：


$$
\begin{aligned}^{0} \boldsymbol{x}_{s w} &=^{0} \boldsymbol{x}_{7}-^{0} \boldsymbol{l}_{b s}-^{0} \boldsymbol{R}_{7} ~^{7} \boldsymbol{l}_{w t} \\ &=^{0} \boldsymbol{R}_{3}\left(^{3} \boldsymbol{l}_{s e}+^{3} \boldsymbol{R}_{4}^{4} \boldsymbol{l}_{e w}\right) \end{aligned}
$$


由于臂角变化时，肘关节并不变（三角形也不变），可以用臂角\phi和旋转轴来描述这个旋转矩阵：


$$
^0 \boldsymbol{R}_{\psi}=\boldsymbol{I}_{3}+\sin \psi\left[^{0} \boldsymbol{u}_{s w} \times\right]+(1-\cos \psi)\left[^{0} \boldsymbol{u}_{s w} \times\right]^{2}
$$


因为是世界坐标系下的旋转（左乘），最终三角形上的link的{0}参考系下旋转矩阵可以描述为j3=0时的矩阵左乘臂角矩阵：


$$
^0 \boldsymbol{R}_{4}=^{0} \boldsymbol{R}_{\psi}~^{0} \boldsymbol{R}_{4}^{ini} \\
^0 \boldsymbol{R}_{3}=^{0} \boldsymbol{R}_{\psi}~^{0} \boldsymbol{R}_{3}^{ini} \\
并且 ^{3}\boldsymbol  {R}_{4} = ^{3}\boldsymbol{R}_{4}^{ini}
$$


##### 肘角 Joint 4


$$
\cos \theta_{4}=\frac{\left\|^{0} \boldsymbol{x}_{s w}\right\|^{2}-d_{s e}^{2}-d_{e w}^{2}}{2 d_{s e} d_{e w}} = \frac{\left\| ^{0} \boldsymbol{x}_{7}-^{0} \boldsymbol{l}_{b s}-^{0} \boldsymbol{R}_{7} ~^{7} \boldsymbol{l}_{w t} \right\|^{2}-d_{s e}^{2}-d_{e w}^{2}}{2 d_{s e} d_{e w}}
$$


肘角通常有两个解（正负）

##### Joint 1,2（决定末端位置） + 3 （决定臂角）

由于末端位置和姿态已知，可以轻松计算出x_sw在{0}系下的坐标


$$
\begin{aligned}^{0} \boldsymbol{x}_{s w} &= ^{0}\boldsymbol{R}_{3}\left(^{3} \boldsymbol{l}_{s e}+^{3} \boldsymbol{R}_{4}^{4} \boldsymbol{l}_{e w}\right) \\
&= \left.^{0} \boldsymbol{R}_{1}^{ini} ~ ^{1}\boldsymbol{R}_{2}^{ini}~^{2} \boldsymbol{R}_{3}\right|_{\theta_{3}=0}\left(^{3} l_{s e}+^{3} \boldsymbol{R}_{4}~^{4} l_{e w}\right)
\end{aligned}
$$


等式右侧未知量为两个，也即初始的\theta_1,\theta_2，该矩阵等式，可以解得初始的\theta_1,\theta_2以及初始的矩阵R_3^{ini}


$$
~R_{0,3}^{ini}=\left[\begin{array}{ccc}
{c_1c_2} & {- s_1} & {c_1s_2} \\
{s_1c_2} & {c_1} & {s_1s_2} \\ 
{-s_2} & {0} & {c_2}\end{array}\right]
$$


然后我们再规定臂角为已知，则可以求得对应臂角下的\theta_1,2,3：


$$
^0 \boldsymbol{R}_{3}=^{0} \boldsymbol{R}_{\psi}~^{0} \boldsymbol{R}_{3}^{ini} \\
=\left(\boldsymbol{I}_{3} +\left[^{0} \boldsymbol{u}_{s w} \times\right]^{2} +\sin \psi\left[^{0} \boldsymbol{u}_{s w} \times\right]-\cos \psi\left[^{0} \boldsymbol{u}_{s w} \times\right]^{2} \right)~^{0} \boldsymbol{R}_{3}^{ini} \\
=\left(\left[^{0} \boldsymbol{u}_{s w}~^{0} \boldsymbol{u}_{s w}^T \right] +\sin \psi\left[^{0} \boldsymbol{u}_{s w} \times\right]-\cos \psi\left[^{0} \boldsymbol{u}_{s w} \times\right]^{2} \right)~^{0} \boldsymbol{R}_{3}^{ini} \\
=A_s sin{\psi} + B_s cos{\psi} + C_s
$$


右侧\phi假设已知，左侧未知量为\theta_1,\theta_2，\theta_3，则很容易求\theta_1,2,3，Paper上的Trans：


$$
~^{0}R_{3}=\left[\begin{array}{ccc}{*} & {-\cos \theta_{1} \sin \theta_{2}} & {*} \\ {*} & {-\sin \theta_{1} \sin \theta_{2}} & {*} \\ {-\sin \theta_{2} \cos \theta_{3}} & {-\cos \theta_{2}} & {\sin \theta_{2} \sin \theta_{3}}\end{array}\right]
$$


【此处使用PoE的坐标】，计算FK验证通过，使用Matlab syms计算T03，发现也存在类似的能够计算的特点


$$
R_{0,3}=\left[\begin{array}{ccc}
{c_1c_2c_3 - s_1s_3} & {-c_1c_2s_3 - s_1c_3} & {c_1s_2} \\
{s_1c_2c_3 + c_1s_3} & {-s_1c_2s_3 + c_1c_3} & {s_1s_2} \\ 
{-c_3s_2} & {s_2s_3} & {c_2}\end{array}\right]
$$


##### Wrist（姿态计算Joint 5 6 7）


$$
{^{0} \boldsymbol{R}_{7}=\boldsymbol{^{0}R}_{3}\boldsymbol{^{3}R}_{4}~ \boldsymbol{^{4}R}_{7}}\to \\\boldsymbol{^{4}R}_{7} = \boldsymbol{^{3}R}_{4}^T~\boldsymbol{^{0}R}_{3}^T ~\boldsymbol{^{0}R}_{7} \\=A_w sin{\psi} + B_w cos{\psi}+C_w \\A_w = \boldsymbol{^{3}R}_{4}^T~A_s^T ~\boldsymbol{^{0}R}_{7} \\B_w = \boldsymbol{^{3}R}_{4}^T~B_s^T ~\boldsymbol{^{0}R}_{7} \\C_w = \boldsymbol{^{3}R}_{4}^T~C_s^T ~\boldsymbol{^{0}R}_{7} \\
$$


由符号运算可以得到R_{4,7}关于\theta{5,6,7}的矩阵:


$$
R_{4,7}=\left[\begin{array}{ccc}
{c_5c_6c_7 - s_5s_7} & {-c_5c_6s_7 - s_5c_7} & {c_5s_6} \\
{-s_6c_7} & {s_6s_7} & {c_6} \\ 
{-s_5c_6c_7-c_5s_7} & {-c_5c_7+s_5c_6s_7} & {-s_5s_6}\end{array}\right]
$$


得到\theta{5,6,7}的计算公式（围绕R_47和Aw,Bw,Cw的矩阵等式进行计算）

最终，\theta1,2,3和\theta5,6,7均需要验算，修改正负号或者+pi满足矩阵完全相等。



![](https://github.com/whtqh/image_files/blob/master/Paper_shimizu2008_7-DOF_animation.gif?raw=true)

图：Theta = [0.16, pi/2, 0.5, pi/3, 0.6,pi/6,0.3]的一组末端姿态对应的冗余解

IKforIIWA14(CartesianIIWA,-1,1,1,psi);

其中psi为输入的臂角。



![](https://raw.githubusercontent.com/whtqh/image_files/master/Paper_shimizu2008_7-DOF-Redundancy-psi.png)

![](https://raw.githubusercontent.com/whtqh/image_files/master/Paper_shimizu2008_7-DOF-InverseKinematicsError.png)



Ik solver的精度得到验证的同时，也能看到在基本无关节奇异的情况下，各个关节只受到限幅的影响（+pi~-pi之间的跳变），而在某些情况下则有可能出现限幅范围内的跳变。

因此需要对某个状态下的KUKA进行冗余度分析和冗余范围的确定：



#### 关节限幅下的可行IK计算

留空...



#### code(FK&IK)

```matlab
function CaetesianEEFtoBase = FKforIIWA14(theta,Mlist)
T01 = Mlist(:,:,1) * ExpScrewZ(theta(1));
T12 = Mlist(:,:,2) * ExpScrewZ(theta(2));
T23 = Mlist(:,:,3) * ExpScrewZ(theta(3));
T34 = Mlist(:,:,4) * ExpScrewZ(theta(4));
T45 = Mlist(:,:,5) * ExpScrewZ(theta(5));
T56 = Mlist(:,:,6) * ExpScrewZ(theta(6));
T67 = Mlist(:,:,7) * ExpScrewZ(theta(7));

CaetesianEEFtoBase = T01 * T12 * T23 * T34 * T45 * T56 * T67 * Mlist(:,:,8);

end
function TransZ = ExpScrewZ(theta)

    TransZ = eye(4);
    TransZ(1:3,1:3) = [cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1];
    
end
```

```matlab
function [thetaList] = IKforIIWA14(CartesianIIWA,ElbowFlag,ShoulderFlag, WristFlag, ArmAnglePsi)
%IKforIIWA14 Inverse Kinematics for IIWA14
%   Example:iiwa_ref =  [ 0.35, 0.3, 0.0,0.5, 0.3, 0.2, 0.7];
%   CartesianIIWA = FKforIIWA14(iiwa_ref,Mlist);
%   IKforIIWA14(CartesianIIWA,1,1,-1,0);
%   Three Flag means 2^3 space with psi

% Given Cartesian Matrix or xyz,rpy
CartesianIK = CartesianIIWA;
% Calculate theta4
R07 = CartesianIK(1:3,1:3);

L_link_1 = 0.1575;
L_link_2 = 0.2025;
L_link_3 = 0.2045;
L_link_4 = 0.2155;
L_link_5 = 0.1845;
L_link_6 = 0.2155;
L_link_7 = 0.081;
L_link_e = 0.045;

X_sw_0 = CartesianIK(1:3,4) - [0;0;L_link_1 + L_link_2] - R07 * [0;0;L_link_7+ L_link_e];
X_se_3 = [0,0, L_link_3 + L_link_4]';
X_ew_4 = [0, L_link_5 + L_link_6,0]';

if(ElbowFlag > 0)
    theta4 = acos( (norm(X_sw_0)^2 - norm(X_se_3)^2 - norm(X_ew_4)^2) /2/ norm(X_se_3)/norm(X_ew_4));
else
    theta4 = -acos( (norm(X_sw_0)^2 - norm(X_se_3)^2 - norm(X_ew_4)^2) /2/ norm(X_se_3)/norm(X_ew_4));
end
% Calculate x_sw in {0}
R34 = [cos(theta4), -sin(theta4),0;0,0,-1;sin(theta4), cos(theta4),0];
X_sw_3 = X_se_3 + R34 * X_ew_4;
% solve for theta_1_ini & theta_2_ini

% R_3_ini = [c1c2, -s1, c1s2; s1c2, c1, s1s2; -s2, 0, c2]

% -X_sw_3(1) * s2 + X_sw_3(3) * c2 = X_sw_0(3) ... A sinx + B cosx = C
alpha = atan2(X_sw_3(1)/ norm([X_sw_3(1),X_sw_3(3)]) , X_sw_3(3)/ norm([X_sw_3(1),X_sw_3(3)]));
% alpha = atan(-X_sw_3(1)/X_sw_3(3));

theta2_ini = acos(X_sw_0(3) / norm([X_sw_3(1),X_sw_3(3)])) - alpha;
%sum_s2c2 = cos(theta2_ini) * X_sw_3(1) + sin(theta2_ini) * X_sw_3(3); 
theta1_ini = atan(X_sw_0(2)/X_sw_0(1));
if(abs(sin(theta1_ini)*cos(theta2_ini) * X_sw_3(1) + sin(theta1_ini)*sin(theta2_ini) * X_sw_3(3) - X_sw_0(2)) > 1e-3)
    theta2_ini = -(theta2_ini + alpha) - alpha;
elseif(abs(cos(theta1_ini)*cos(theta2_ini) * X_sw_3(1) + cos(theta1_ini)*sin(theta2_ini) * X_sw_3(3) - X_sw_0(1)) > 1e-3)
    theta2_ini = -(theta2_ini + alpha) - alpha;
end

% Get R_03_ini to cal R_03 and theta1,theta2,theta3
u_sw_0 =  X_sw_0 / norm(X_sw_0);
R_03_ini = [cos(theta1_ini) * cos(theta2_ini), -sin(theta1_ini), cos(theta1_ini)*sin(theta2_ini); ...
    sin(theta1_ini) * cos(theta2_ini), cos(theta1_ini), sin(theta1_ini) * sin(theta2_ini); ...
    -sin(theta2_ini), 0, cos(theta2_ini)];
As = VecToso3(u_sw_0) * R_03_ini;
Bs = -VecToso3(u_sw_0)^2 * R_03_ini;
Cs = u_sw_0 * u_sw_0' * R_03_ini;
% As * sin(\phi) + Bs * cos(\phi) + Cs = R_03

phi = ArmAnglePsi;
theta1 = atan( (As(2,3) * sin(phi) + Bs(2,3)*cos(phi) + Cs(2,3)) / ...
    (As(1,3) * sin(phi) + Bs(1,3)*cos(phi) + Cs(1,3)));
%theta2 = + - should be decided by if == theta2_ini when phi = 0.
if( abs(acos(As(3,3) * sin(0) + Bs(3,3)*cos(0) + Cs(3,3)) - theta2_ini) < 1e-3)
    theta2 = acos(As(3,3) * sin(phi) + Bs(3,3)*cos(phi) + Cs(3,3));
else
    theta2 = - acos(As(3,3) * sin(phi) + Bs(3,3)*cos(phi) + Cs(3,3));
end
theta3 = atan(-(As(3,2) * sin(phi) + Bs(3,2)*cos(phi) + Cs(3,2)) / ...
    (As(3,1) * sin(phi) + Bs(3,1)*cos(phi) + Cs(3,1)));

%double check to change theta2
if(abs(sin(theta2) * sin(theta3) - (As(3,2) * sin(phi) + Bs(3,2)*cos(phi) + Cs(3,2))) > 1e-3)
    theta3 = theta3 + pi;
elseif(abs(-cos(theta3) * sin(theta2) - (As(3,1) * sin(phi) + Bs(3,1)*cos(phi) + Cs(3,1))) > 1e-3)
    theta3 = theta3 + pi;
end

if(abs(sin(theta1) * sin(theta2) - (As(2,3) * sin(phi) + Bs(2,3)*cos(phi) + Cs(2,3))) > 1e-3)
    theta1 = theta1 + pi;
elseif(abs(cos(theta1) * sin(theta2) - (As(1,3) * sin(phi) + Bs(1,3)*cos(phi) + Cs(1,3))) > 1e-3)
    theta1 = theta1 + pi;
end

% ErrorR03 = [cos(theta1)*cos(theta2)*cos(theta3) - sin(theta1)*sin(theta3), ...
%     -cos(theta1)*cos(theta2)*sin(theta3) - sin(theta1)*cos(theta3), ...
%     cos(theta1) * sin(theta2); ...
%     sin(theta1)*cos(theta2)*cos(theta3) + cos(theta1)*sin(theta3), ...
%     -sin(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3), ...
%     sin(theta1) * sin(theta2); ...
%     -cos(theta3) * sin(theta2), sin(theta2) * sin(theta3), cos(theta2)] - ...
%     (As .* sin(phi) + Bs .* cos(phi) + Cs)

Aw = R34'*As'* R07;
Bw = R34'*Bs'* R07;
Cw = R34'*Cs'* R07;

theta5 = atan(-(Aw(3,3) * sin(phi) + Bw(3,3)*cos(phi) + Cw(3,3)) / ...
    (Aw(1,3) * sin(phi) + Bw(1,3)*cos(phi) + Cw(1,3)));
% ? + - should be decided by double check
theta6 =  acos(Aw(2,3) * sin(phi) + Bw(2,3)*cos(phi) + Cw(2,3));

theta7 = atan(-(Aw(2,2) * sin(phi) + Bw(2,2)*cos(phi) + Cw(2,2)) / ...
    (Aw(2,1) * sin(phi) + Bw(2,1)*cos(phi) + Cw(2,1)));
if( abs(Aw(1,3) * sin(phi) + Bw(1,3)*cos(phi) + Cw(1,3) - cos(theta5)*sin(theta6)) > 1e-3)
    theta6 = - theta6;
end

if( abs(Aw(3,3) * sin(phi) + Bw(3,3)*cos(phi) + Cw(3,3) - -sin(theta5)*sin(theta6)) > 1e-3)
    theta5 = theta5 + pi;
elseif( abs(Aw(1,3) * sin(phi) + Bw(1,3)*cos(phi) + Cw(1,3) - cos(theta5)*sin(theta6)) > 1e-3)
    theta5 = theta5 + pi;
end

if( abs(Aw(2,2) * sin(phi) + Bw(2,2)*cos(phi) + Cw(2,2) - sin(theta6)*sin(theta7)) > 1e-3)
    theta7 = theta7 + pi;
elseif(abs(Aw(2,1) * sin(phi) + Bw(2,1)*cos(phi) + Cw(2,1) - -sin(theta6)*cos(theta7)) > 1e-3)
    theta7 = theta7 + pi;
end


if(ShoulderFlag > 0)
else
    theta1 = theta1 + pi;
    theta2 = -theta2;
    theta3 = theta3 + pi;
end
if(WristFlag > 0)
else
    theta5 = theta5 + pi;
    theta6 = -theta6;
    theta7 = theta7 + pi;
end

thetaList = [theta1;theta2;theta3;theta4;theta5;theta6;theta7];

for i = 1:1:7
    if(thetaList(i) > pi)
        thetaList(i) = thetaList(i) - 2 * pi;
    elseif(thetaList(i) < -pi)
        thetaList(i) = thetaList(i) + 2 * pi;
    end
end
end


```





#### REFERENCE

[1] Shimizu, M., Kakuya, H., Yoon, W. K., Kitagaki, K., & Kosuge, K. (2008). Analytical inverse kinematic computation for 7-DOF redundant manipulators with joint limits and its application to redundancy resolution. *IEEE Transactions on Robotics*, *24*(5), 1131-1142.

[2] https://whtqh.github.io/blog/2019/10/03/DenavitHartenbergPara.html

