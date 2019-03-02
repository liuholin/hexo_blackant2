---
title: SVO详解
date: 2018-05-07 08:25:58
tags:
mathjax: true
---

<script type="text/javascript" async src="http://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-MML-AM_CHTML"></script>

首先科普下计算机视觉和机器视觉

## 计算机视觉
主要是质的分析，比如分类识别，这是一个杯子那是一条狗。或者做身份确认，比如人脸识别，车牌识别。或者做行为分析，比如人员入侵等。

## 机器视觉
主要侧重对量的分析，比如用视觉去测量一个零件的直径。
<!--more-->

## 接下来开始说论文的重点
__我们对于矩阵求逆的时候，要主要必须得保证这个矩阵是一个方阵才行哦__

### 相机类型
刚才介绍了普通的相机，现在介绍下RGB-D相机

1. 红外结构光来测量像素距离，例如kinect1
2. 飞行时间法（ToF）测量像素距离，例如kinect2

### 相机映射模型

由相似三角形比例得：这里负号是指反向的意思

$$\frac{Z}{f}=-\frac{X}{X'}=-\frac{Y}{Y'}$$
由于相机内置旋转算法，所以我们拍出来的照片都是正面的
$$\frac{Z}{f}=\frac{X}{X'}=\frac{Y}{Y'}$$
$$ X'=\frac{Xf}{Z}$$
$$ Y'=\frac{Yf}{Z}$$


照片上一点像素 $$\vec p(u,v)$$ ，

$$u=\alpha X'+c_{x}$$
$$v=\beta Y'+c_{y}$$

$$u=\alpha f\frac{X}{Z}+c_{x}$$
$$v=\beta f\frac{Y}{Z}+c_{y}$$

$$u=f_{x}\frac{X}{Z}+c_{x}$$
$$v=f_{y}\frac{Y}{Z}+c_{y}$$
化成齐次矩阵
$$\begin{bmatrix}
u\\
v\\
1\\
\end{bmatrix}=\frac{1}{Z} \begin{bmatrix}
f_{x}&0&c_{x}\\
0&f_{y}&c_{y}\\
0&0&1\\
\end{bmatrix} \begin{bmatrix}
X\\
Y\\
Z\\
\end{bmatrix}$$
$$=\frac{1}{Z}KP$$
$$Z\begin{bmatrix}
u\\
v\\
1\\
\end{bmatrix}=KP$$
其中 $$K$$ 是内参，至此就是 $$\mathbb{R}^{3} \mapsto \mathbb{R}^{2}$$

我们现在将推到 $$\mathbb{R}^{2} \mapsto \mathbb{R}^{3}$$，不过在此之前我们必须要知道距离$$d$$

$$ Z=d$$
$$ X=\frac{u-c_{x}}{f_{x}}Z$$
$$ Y=\frac{u-c_{x}}{f_{x}}Z$$

上式就是*像素到相机坐标*的公式

内参属于相机出产自带的属性，我们可以通过棋盘标定法来实现相机的标定

#### 棋盘标定法
#### 推导外参
接下来推倒外参，外参表示相机运动的参数
$$Zp_{u,v}=Z\begin{bmatrix}
u\\
v\\
1\\
\end{bmatrix} $$

### 向量到矩阵的变换

#### 外积计算
$$\vec a\times \vec b=\begin{bmatrix}
i&j&k\\
a_{1}&a_{2}&a_{3}\\
b_{1}&b_{2}&b_{3}\\
\end{bmatrix}=\begin{bmatrix}
a_{2}b_{3}-a_{3}b_{2}\\
a_{3}b_{1}-a_{1}b_{3}\\
a_{1}b_{2}-a_{2}b_{1}\\
\end{bmatrix}=\begin{bmatrix}
0&-a_{3}&a_{2}\\
a_{3}&0&-a_{1}\\
-a_{2}&a_{1}&0\\
\end{bmatrix}\vec b$$
$$=a^{\wedge}\vec b=\begin{bmatrix}
0&b_{3}&-b_{2}\\
-b_{3}&0&b_{1}\\
b_{2}&-b_{1}&0\\
\end{bmatrix}\vec a=-b^{\wedge}\vec a$$
故$$a^{\wedge}\vec b=-b^{\wedge}\vec a$$，这个公式转换很最重要在后文中有提及

### 旋转矩阵

#### 刚体变换
相机运动是一个刚体运动，他保证了同一个向量在各个坐标系下的长度和夹角都不会发生变化，这种变换属于欧式变换。欧式变换包括了旋转和平移。首先我们考虑旋转。我们设某个正交单位基 $$(e_{1},e_{2},e_{3})$$ 经过一次旋转后变成了 $$(e'_{1},e'_{2},e'_{3})$$，那么同一向量 $$\vec a$$，在两个坐标系下的坐标分别是 $$[a_{1},a_{2},a_{3}]^T$$和 $$[a'_{1},a'_{2},a'_{3}]^T$$。根据定义，有：
$$(e_{1},e_{2},e_{3})\begin{bmatrix}
a_{1}\\
a_{2}\\
a_{3}\\
\end{bmatrix}=(e'_{1},e'_{2},e'_{3})\begin{bmatrix}
a'_{1}\\
a'_{2}\\
a'_{3}\\
\end{bmatrix}$$
两边同时左乘一个 \begin{bmatrix}
e^T_{1}\\
e^T_{2}\\
e^T_{3}\\
\end{bmatrix}。

$$\begin{bmatrix}
a_{1}\\
a_{2}\\
a_{3}\\
\end{bmatrix}=\begin{bmatrix}
e^T_{1}\\
e^T_{2}\\
e^T_{3}\\
\end{bmatrix}(e'_{1},e'_{2},e'_{3})\begin{bmatrix}
a'_{1}\\
a'_{2}\\
a'_{3}\\
\end{bmatrix}=\begin{bmatrix}
e^T_{1}e'_{1}&e^T_{2}e'_{1}&e^T_{3}e'_{1}\\
e^T_{2}e'_{1}&e^T_{2}e'_{2}&e^T_{2 }e'_{3}\\
e^T_{3}e'_{1}&e^T_{3}e'_{2}&e^T_{3}e'_{3}\\
\end{bmatrix}\begin{bmatrix}
a'_{1}\\
a'_{2}\\
a'_{3}\\
\end{bmatrix}=R\vec a'$$

其中 $$R$$ 就是旋转矩阵 *此处解释了旋转矩阵为啥是乘法*，旋转矩阵满足一下性质

1. 秩为1
2. 该矩阵与其转置相乘等于单位矩阵
我们把旋转矩阵的集合定义如下：
$$SO(3)=\{R\ \in \mathbb{R}^{n*n}|RR^T=I,det(R)=1\}$$

$$SO(3)是特殊正交群的意思。
由于旋转的特殊性质，即可逆性。

$$\vec a'=R^{-1}\vec a=R^T\vec a$$

在欧式变换里，除了考虑旋转，还有平移。考虑到世界坐标系下的一个向量 $$\vec a$$,经过旋转和平移之后得到 $$\vec a'$$。
$$\vec {a'}=R\vec a+t$$


### 变换矩阵
假设我们做了两次变换: $$R_{1},t_{1}和 R_{2},t_{2}$$，满足
$$\vec b=R_{1}\vec a+t_{1}, \vec c=R_{2}\vec b+t_{2}$$
这样一来向量 $$\vec c$$计算比较复杂
$$\vec c=R_{2}(R_{1}\vec a+t_{1})+t_{2}$$

引入齐次坐标和变换矩阵

$$\begin{bmatrix}
\vec {a'}\\
1\\
\end{bmatrix}=\begin{bmatrix}
R&t\\
0^T&1\\
\end{bmatrix}\begin{bmatrix}
\vec a\\
1\\
\end{bmatrix}=T\begin{bmatrix}
\vec a\\
1\\
\end{bmatrix}$$
其中， $$T$$是变换矩阵(Transform Matrix)，本文我们定义 $$\tilde{a}$$ 表示 $$\vec a$$的齐次坐标, $$\tilde{b}$$ 表示 $$\vec b$$的齐次坐标, $$\tilde{c}$$ 表示 $$\vec c$$的齐次坐标,
$$\tilde{b}=T_{1} \tilde{a},\ \ \tilde{c}=T_{2} \tilde{b} => \tilde{c}=T_{2}T_{1} \tilde{a}$$
区分齐次和非齐次坐标符号很麻烦，我们直接就用 $$\vec b=T \vec a$$，代表齐次坐标。

关于变换矩阵 $$T$$，它的结构比较特殊，左上角为旋转矩阵，右上角为平移向量，左下角为0向量，右下角为1。这种特殊矩阵又称为特殊欧式群（Special Euclidean Group）：
$$SE(3)=\begin{Bmatrix}T=\begin{bmatrix}R&t\\0^T&1\\\end{bmatrix} \in \mathbb{R}^{4*4}|R \in SO(3),t \in \mathbb{R}^3 \end{Bmatrix}$$

同理该矩阵的逆

$$T^{-1}=\begin{bmatrix}R^T&-R^Tt\\0^T&1\end{bmatrix}$$

### 旋转向量
假设有一个旋转轴为 $$\vec n$$，角度为 $$\theta$$，显然他对于的旋转向量为 $$\vec n\theta$$。从旋转向量都旋转矩阵的转换过程由罗德里格斯公式：
$$R=cos\theta I+(1-cos\theta)\vec n\vec n^T+sin\theta \vec n^{\wedge}$$
这里 $$^\wedge$$是向量到反对称矩阵的转换符，见前面公式。

*补充一个知识点，单位矩阵 I=\begin{bmatrix}1&0&0\\0&1&0\\0&0&1\end{bmatrix}*

对于转角 $$\theta$$，有：
$$tr(R)=cos\theta tr(I)+(1-cos\theta)tr(\vec n \vec n^T)+sin\theta tr(\vec n^{\wedge})$$
$$=3cos\theta +(1-cos\theta)$$
$$=1+2cos\theta$$

因此
$$\theta=arccos(\frac{tr(R)-1}{2}) $$

*注： $$tr(N)$$，表示求矩阵 $N$的主对角元素之和。$tr(I)=3$，因为由于前面旋转矩阵推导可知，$R$是3\*3矩阵。 $\vec n^{\wedge}$ 是对角线为0的反对称矩阵，所以 $tr(\vec n^{\wedge})=0$。*

关于转轴 $\vec n$，由于旋转前后旋转轴不发生变化，所以
$$R\vec n=\vec n$$

### 李群

李群的主要核心是表述连续变化的特性。变换矩阵包含的实际意义就是旋转和平移，所以肯定是连续的。故旋转矩阵和变换矩阵都是李群的一种，分别是特殊正交群和特殊欧式群。

### 李代数
#### $$so(3)$$
我们知道旋转矩阵的一个性质
$$RR^T=I$$
它是有时间意义的，随着时间不断的变化，所以可以写成对时间的函数：$$R(t)$$
$$R(t)R(t)^T=I$$
对时间$$t$$进行求导
$$\dot{R}(t)R(t)^T+\dot{R}(t)^TR(t)=0$$
$$\dot{R}(t)R(t)^T=-\dot{R}(t)^TR(t)$$
$$\dot{R}(t)R(t)^T=-(\dot{R}(t)R(t)^T)^T$$
所以 $$\dot{R}(t)R(t)^T$$ 是反对称矩阵。由前面介绍查积所知，向量到矩阵就是一个反对称矩阵。
$$\vec a^{\wedge}=A=\begin{bmatrix}
0&-a_{3}&a_{2}\\
a_{3}&0&-a_{1}\\
-a_{2}&a_{1}&0\\
\end{bmatrix}, \ \ A^{\wedge}=\vec a$$
所以 $$\dot{R}(t)R(t)^T$$ 必然对应一个向量 $$\phi(t)^\wedge$$。

于是
$$\dot R(t)R(t)^T=\phi(t)^\wedge$$

右乘一个$$R(t)$$。
$$\dot R(t)=\phi(t)^\wedge R(t)$$
当 $$t_0=0$$时，则还未开始旋转，所以 $$R(t_0)=I$$。 由一阶泰勒展开得：
$$R(t)\approx R(t_0)+\dot{R}(t_0)(t-t_0)$$
将上式带入
$$=I+\phi(t_0)^\wedge R(t_0)t$$
$$=I+\phi(t_0)^\wedge t$$

*我们看到 $$\phi$$ 反映了 $$R$$ 的导数性质，故称它在SO(3)原点附近的正切空间上。引用书上原话，我不是很明白*

由齐次微分方程性质（*$$y'+p(x)y=0，通解为y=Ce^{-\int p(x)dx}$$*）得
$$\dot R(t)=\phi(t)^\wedge R(t)$$
通解$$R(t)=Ce^{\phi(t)^\wedge}$$

由于初始值$$R(0)=I$$,我们考虑的是$$t_0=0$$时，此时 $$\phi(t_0)^\wedge$$ 为常数，故简化 $$\phi(t_0)^\wedge$$为 $$\phi_0$$所以其特解为
$$\phi(0)^\wedge=0,\ C=I$$
$$R(t)=e^{\phi_0^\wedge t}$$

对其换种写法也即

$$R(t)=exp(\phi_0^\wedge t)$$

$$\Phi=\phi^\wedge=\begin{bmatrix}0&-\phi_3&\phi_2\\\phi_3&0&-\phi_1\\-\phi_2&\phi_1&0\end{bmatrix}\in\mathbb{R}^{3\times 3} $$
一般我们说 $$so(3)$$的元素是3维向量或者3维反对称矩阵。
$$so(3)=\begin{Bmatrix} \phi \in \mathbb{R}^3, \Phi=\phi^\wedge \in \mathbb{R}^{3\times 3} \end{Bmatrix}$$

与$$SO(3)$$的关系对应为
$$R=exp(\phi^\wedge)$$

#### $$se(3)$$

同理我们可以推导
$$se(3)=\begin{Bmatrix}\xi=\begin{bmatrix}\rho \\ \phi \end{bmatrix}\in \mathbb{R}^6,\ \rho \in \mathbb{R}^3,\ \phi \in so(3),\ \xi^\wedge=\begin{bmatrix}\phi^\wedge & \rho\\0^T&1 \end{bmatrix} \in \mathbb{R}^{4\times 4}\end{Bmatrix}$$
每个$$se(3)$$ 的元素 $$\xi$$是六维向量，前三维是平移向量，记做 $$\rho$$。 后三维是旋转向量，记做 $$\phi$$。同时我们这里的 $$\wedge$$不再单纯表示向量到反对称矩阵，而是向量到矩阵的转换， $$\vee$$表示矩阵到向量的转换。

### $$SO(3)=R=\phi^\wedge$$ 指数映射

由于 $$\phi$$是三维向量，我们可以换种方式来定义它，通过模长 $$\vec a$$ 和方向 $$\theta$$，这里 $$\vec a$$是长度为1的方向向量，对于 $$a^\wedge$$(后面我们不说明的情况下$$a$$就是 $$\vec a$$)，有以下两个性质:

*PS:这是人家前人发现的，我们直接用就行了*
$$a^\wedge a^\wedge=aa^T-I$$
$$a^\wedge a^\wedge a^\wedge=-a^\wedge$$

*本文通过假设 $$a^\wedge=\begin{bmatrix}0&-\frac{1}{\sqrt{3}}&\frac{1}{\sqrt{3}} \\ \frac{1}{\sqrt{3}}&0&-\frac{1}{\sqrt{3}} \\ -\frac{1}{\sqrt{3}}&\frac{1}{\sqrt{3}}&0\end{bmatrix}$$ ，强行往里带入可以验证上述性质成立*

由麦克劳林公式得

$$exp(x)=\sum_{k=0}^{\infty }\frac {1}{k!}x^n$$
$$sinx=\sum_{k=1}^{\infty }(-1)^{k-1}\frac {1}{(2k-1)!}x^{2k-1}=x-\frac{1}{3!}x^3+\frac{1}{5!}x^5...$$
 $$cosx=\sum_{k=0}^{\infty }(-1)^k\frac {1}{2k!}x^{2k}=1-\frac{1}{2!}x^2+\frac{1}{4!}x^4+...$$

$$exp(\phi^{\wedge})=\sum_{n=0}^{\infty }\frac {1}{n!}(\phi^{\wedge})^n$$

由上式得

$$exp(\theta a^{\wedge})=\sum_{n=0}^{\infty }\frac {1}{n!}(\theta a^{\wedge})^n$$
$$=I+\theta a+\frac {1}{2!}\theta^2 a^{\wedge}a^{\wedge}+\frac {1}{3!}\theta^3a^{\wedge}a^{\wedge}a^{\wedge}+\frac {1}{4!}\theta^4a^{\wedge}a^{\wedge}a^{\wedge}a^{\wedge}+...$$
将上式关于 $$a^{\wedge}$$ 的次方推导公式，带入
$$=I+\theta a+\frac {1}{2!}\theta^2 a^{\wedge}a^{\wedge}-\frac {1}{3!}\theta^3a^{\wedge}-\frac {1}{4!}\theta^4a^{\wedge}a^{\wedge}+\frac {1}{5!}\theta^5a^{\wedge}+\frac {1}{6!}\theta^6a^{\wedge}a^{\wedge}...$$
$$=aa^T+(\theta-\frac{1}{3!}\theta^3+\frac{1}{5!}\theta^5-\frac{1}{7!}\theta^7+...)a^{\wedge}-(1-\frac{1}{2!}\theta^2+\frac{1}{4!}\theta^4-\frac{1}{6!}\theta^6+...)a^{\wedge}a^{\wedge}$$
$$=a^{\wedge}a^{\wedge}+I+sin{\theta}a^{\wedge}-cos{\theta}a^{\wedge}a^{\wedge}$$
$$=(1-cos{\theta})a^{\wedge}a^{\wedge}+I+sin\theta a^{\wedge}$$
$$=cos\theta I+(1-cos\theta)aa^T+sin\theta a^{\wedge}$$

### $$SE(3)=T=exp(\xi^\wedge)$$ 指数映射

$$exp(\xi^\wedge)=\begin{bmatrix}exp(\phi^{\wedge})&exp(\rho)\\ 0^T&0    \end{bmatrix}=\begin{bmatrix}\sum_{n=0}^{\infty }\frac {1}{n!}(\phi^{\wedge})^n&\sum_{n=0}^{\infty }\frac {1}{n!}(\rho)^n\\ 0^T&1\end{bmatrix}$$
$$=\begin{bmatrix}\sum_{n=0}^{\infty }\frac {1}{n!}(\phi^{\wedge})^n&\sum_{n=0}^{\infty }\frac {1}{(n+1)!}(\phi^{\wedge})^n\rho\\ 0^T&1\end{bmatrix}$$
$$=\begin{bmatrix}R&J\rho\\ 0^T&1\end{bmatrix}=T$$
即
$$J=\sum_{n=0}^{\infty }\frac {1}{(n+1)!}(\phi^{\wedge})^n$$
按照之前推导$exp(\phi^{\wedge})$ 的思路，将$a^{\wedge}$ 平方和立方的计算规律带入得：
$$J=\frac{sin\theta}{\theta}I+(1-\frac{sin\theta}{\theta})aa^T+\frac{(1-cos\theta)}{\theta}a^{\wedge}$$

图像像素灰度在k的时刻表示为I_k:Ω⊂R^2⊢R， 其中Ω是图像的整个域（二维的）。任意3维点p  在可见的场景表面S⊂R^3，通过相机投影模型 映射到图像的坐标

其中，前注k表示在k时刻相机框架点坐标的表示。投影函数π由相机内参（由校准之后得到）决定。3维点可以由图像坐标u恢复，不过需要给定反向投影函数π-1和深度

#### 总结下李群、李代数之间的关系

李群是一个矩阵(R，T)，李代数是一个向量 $\phi$，$\xi$，每个李代数都可以对应一个反对称矩阵 $\phi^{\wedge}$，$\xi^{\wedge}$，同时我们对这个反对称矩阵求指数就是李群了。也即：
$$exp(\phi^{\wedge})=R$$
$$exp(\xi^{\wedge})=T$$


### 为什么要提出李群，李代数
李群为了更好的描述相机的运动，李代数是为了进一步更好的计算李群（单纯的李群是不容易直接进行数学运算的）

假设我们知道一个世界坐标系上的点p（路标点），运动了T之后，它的观测量为z，则理论上：
$$z=Tp$$
然而我们知道真实情况系必然存在误差，所以应该是
$$z=Tp+w$$
我们通常比较想知道计算理想和真实情况下的误差，使其最小
$$e=z-Tp$$
假设我们有N个观测点和路标点，则
$$minJ(T)=\sum_{i=1}^{N}\left \| z_i-Tp_i \right \|_{2}^{2}$$
我们的目标就是求$T$，使得整体误差$J$最小。

#### BCH近似

$$exp(\phi_1^{\wedge})exp(\phi_2^{\wedge})=exp(J_l^{-1}\phi_1+\phi_2), \ \phi_1微小$$
$$exp(\phi_1^{\wedge})exp(\phi_2^{\wedge})=exp(J_r^{-1}\phi_2+\phi_1), \ \phi_2微小$$
其中$J_l=J=\frac{sin\theta}{\theta}I+(1-\frac{sin\theta}{\theta})aa^T+\frac{1-cos\theta}{\theta}a^{\wedge}$，它的逆
$$J_l^{-1}=J^{-1}=\frac{\theta}{2}cot\frac{\theta}{2}I+(1-\frac{\theta}{2}cot\frac{\theta}{2})aa^T+\frac{\theta}{2}a^{\wedge}$$
右乘雅克比仅需要对自变量取符号即可：
$$J_r(\phi)=J_l(-\phi)$$

将 $\phi_1^{\wedge}$ 与$J_l^{-1}\phi_1$对应，所以 $\phi_1$与 $(J_l\phi_1)^{\wedge}$ 对应。即：
$$exp((\phi_1+\phi_2)^{\wedge})=exp((J_l\phi_1)^{\wedge})exp(\phi_2^{\wedge}), \ \phi_1微小$$
$$exp((\phi_1+\phi_2)^{\wedge})=exp((J_r\phi_2)^{\wedge})exp(\phi_1^{\wedge}), \ \phi_2微小$$

#### 李代数求导
$$\frac{\partial{Rp}}{\partial{R}}=\frac{\partial{(exp(\phi^{\wedge})p)}}{\partial{R}} $$

由导数定义可知

$$\frac{\partial{(exp(\phi^{\wedge})p)}}{\partial{\phi}}=\lim_{\xi \phi\rightarrow 0}\frac{exp((\phi+\xi\phi)^{\wedge})p-exp(\phi^{\wedge})p}{\xi \phi}$$
$$=\lim_{\xi \phi\rightarrow 0}\frac{exp((J_l\xi\phi)^{\wedge})exp(\phi^{\wedge})p-exp(\phi^{\wedge})p}{\xi \phi}$$
$$=\lim_{\xi \phi\rightarrow 0}\frac{(I+(J_l\xi\phi)^{\wedge})exp(\phi^{\wedge})p-exp(\phi^{\wedge})}{\xi \phi}$$
$$=\lim_{\xi \phi\rightarrow 0}\frac{(J_l\xi\phi)^{\wedge}exp(\phi^{\wedge})p}{\xi \phi}$$
从叉积公式得 $\vec a\times \vec b=a^{\wedge}b=-ab^{\wedge}$
$$=\lim_{\xi \phi\rightarrow 0}\frac{-(exp(\phi^{\wedge})p)^{\wedge}(J_l\xi\phi)}{\xi \phi}$$
$$=-(Rp)^{\wedge}J_l$$
自此，我们推导出旋转后的点相对于李代数的导数：
$$\frac{\partial{Rp}}{\partial{\phi}}=-(Rp)^{\wedge}J_l$$

#### 扰动模型（左乘）
$$\frac{\partial{Rp}}{\partial{R}}=\lim_{
\varphi \rightarrow 0}\frac{exp(\varphi ^{\wedge})exp(\phi^{\wedge})p-exp(\phi^{\wedge})p}{\varphi}$$
$$\approx \lim_{
\varphi \rightarrow 0}\frac{(1+\varphi^{\wedge})exp(\phi^{\wedge})p-exp(\phi^{\wedge})p}{\varphi}$$
$$=\lim_{
\varphi \rightarrow 0}\frac{\varphi^{\wedge}exp(\phi^{\wedge})p}{\varphi}$$
$$=\lim_{
\varphi \rightarrow 0}\frac{-(exp(\phi^{\wedge})p)^{\wedge}\varphi}{\varphi}$$
$$=-(Rp)^{\wedge}$$

### SE(3)

其中， 是已知深度域
相机的位置和方向在k时刻用刚体变换 表示 SE(3)
我们可以映射一个3D点从世界坐标框架到参考相机框架
Tk,w就是上面说的外参，
两个连续帧直接相关变化可以用 计算
在优化过程中，我们需要最小的表示变化，因此，使用李代数se(3) 对应恒等于正切空间SE(3)。我们用代数元素ξ=〖(ω,υ)〗^T∈R^6表示旋转坐标，其中w是角速度和v线性速度。旋转坐标ξ用指数映射到SE(3)

## 对极约束

$$s_{1}p_{1}=KP \ \ \ s_{2}p_{2}=K(RP+t)$$

$s_{1}$ 是一个数值，表示距离

对其进行归于化，得

$$p_{1}=KP \ \ \ p_{2}=K(RP+t)$$

记

$$x_{1}=K^{-1}p_{1} \ \ \ \ x_{2}=K^{-1}p_{2}$$
$$x_{2}=Rx_{1}+t$$
两边同时乘以$t^{\wedge}$，由于$t^{\wedge}t=0$
$$t^{\wedge}x_{2}=t^{\wedge}Rx_{1}$$
两边同时乘以$x_{2}^{T}$
$$x_{2}^{T}t^{\wedge}x_{2}=x_{2}^{T}t^{\wedge}Rx_{1}$$
由于向量也是一维矩阵，所以左边满足矩阵乘法结合律
$$(AB)C=A(BC)$$
由于$t^{\wedge}x_{2}$ 是垂直于$t$ 和 $x_{2}$ 的向量（一维矩阵），所以左边等0，即
$$x_{2}^{T}t^{\wedge}Rx_{1}=0$$
将上面的$x_{1},x_{2}$ 代入
$$p_{2}^{T}K^{-T}t^{\wedge}RK^{-1}p_{1}=0$$
令$E=t^{\wedge}R$(本质矩阵)，$F=K^{-T}EK^{-1}$（基础矩阵）

故

$$x_{2}^{T}Ex_{1}=0, \ p_{2}^{T}Fp_{1}=0$$
根据特征点匹配求出$E$，从中分解出$R 和 t$ 出来即可
由E的性质可知，E为3*3的矩阵

对于任意的一对匹配点$x_{1}=[u_{1}, v_{1}, 1]^{T},x_{2}=[u_{2}, v_{2}, 1]^{T}$
$$x_{2}^{T}Ex_{1}=0$$
$$\begin{bmatrix}
u_{2} & v_{2} & 1
\end{bmatrix}\begin{bmatrix}
e_{1} & e_{2} & e_{3}\\
e_{4} & e_{5} & e_{6}\\
e_{7} & e_{8} & e_{9}
\end{bmatrix}\begin{bmatrix}
u_{1}\\
v_{1}\\
1
\end{bmatrix}=0$$
由矩阵乘法结合律

$$\begin{bmatrix}
u_{2} & v_{2} & 1
\end{bmatrix}\begin{bmatrix}
u_{1}\\
v_{1}\\
1
\end{bmatrix}\begin{bmatrix}
e_{1} & e_{2} & e_{3}\\
e_{4} & e_{5} & e_{6}\\
e_{7} & e_{8} & e_{9}
\end{bmatrix}=0$$
将$E$写成向量的形式
$$e=[e_{1}, e_{2}, e_{3}, e_{4}, e_{5}, e_{6}, e_{7}, e_{8}, e_{9}]^{T}$$

上式可以写成
$$[u_{1}u_{2}, u_{2}v_{1}, u_{2}, v_{2}u_{1}, v_{1}v_{2}, v_{2}, u_{1}, v_{1}, 1]e=0$$

对于其他特征点也有同样的性质
$$\begin{bmatrix}
 u_{1}^{1}u_{2}^{1}& u_{2}^{1}v_{1}^{1}& u_{2}^{1}& v_{2}^{1}u_{1}^{1}& v_{1}^{1}v_{2}^{1}& v_{2}^{1}& u_{1}^{1}& v_{1}^{1}& 1\\
 u_{1}^{2}u_{2}^{2}& u_{2}^{2}v_{1}^{2}& u_{2}^{2}& v_{2}^{2}u_{1}^{2}& v_{1}^{2}v_{2}^{2}& v_{2}^{2}& u_{1}^{2}& v_{1}^{2}& 1\\
 \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & 1\\
 u_{1}^{8}u_{2}^{8}& u_{2}^{8}v_{1}^{8}& u_{2}^{8}& v_{2}^{8}u_{1}^{8}& v_{1}^{8}v_{2}^{8}& v_{2}^{8}& u_{1}^{8}& v_{1}^{8}& 1\\
\end{bmatrix}\begin{bmatrix}
 e_{1}\\
 e_{2}\\
 e_{3}\\
 e_{4}\\
 e_{5}\\
 e_{6}\\
 e_{7}\\
 e_{8}\\
 e_{9}
\end{bmatrix}=0$$

## 光束平差法(Bundle Ajustment)
我们通过特征匹配的方式，已经可以到N组特征点
$$z_{1}=\begin{Bmatrix}
z_{1}^{1},z_{1}^{2}....,z_{1}^{N}
\end{Bmatrix},z_{2}=\begin{Bmatrix}
z_{2}^{1},z_{2}^{2}....,z_{2}^{N}
\end{Bmatrix}$$
其中$z_{i}^{j}$ 下标是指第几帧图，上标表示第几个特征点，其在图像上的坐标为 $z_{i}^{j}=\left [ u,v \right ]_{i}^{j}$

![](/img/camera.png)
上图模型表示为相机在1处，2处分别观测空间中X点位置。
$$
\lambda_{1}\begin{bmatrix}
z_{1}^{j}\\
1
\end{bmatrix}=CX^{j},\lambda_{2}\begin{bmatrix}
z_{2}^{j}\\
1
\end{bmatrix}=C\left ( RX^{j}+t \right )
$$
其中 $\lambda_{1},\lambda_{2}$ 表示像素深度值，也就是相机的z坐标。正常的思路就行，两个公式约掉 $X^{j}$ ，使用对极约束和SVD分解，理论上我们需要8个点就可以解算出来$R, t$，这是常规思路，我们这里主要介绍图优化。
我们尝试构造一个优化问题
由于存在着精度误差，所以 $\left \| \frac{1}{\lambda_{1}}CX^{j}-\left [ z_{1}^{j},1 \right ]^{T}  \right \|^{2}，\left \| \frac{1}{\lambda_{2}}C(RX^{j}+t)-\left [ z_{2}^{j},1 \right ]^{T}  \right \|^{2}$ 不可能都为0，所以我们只能通过调整 $R,t$ 来使他们的误差尽量下。

所以可以构造一个最优化问题，调整$R,t$使得对于所有的特征点$z^{j}$，误差二范数累计最小，也即
$$
min_{X,R,t}\sum_{j=1}^{N}\left \| \frac{1}{\lambda_{1}}CX^{j}-\left [ z_{1}^{j},1 \right ]^{T}  \right \|^{2} + \left \| \frac{1}{\lambda_{2}}C(RX^{j}+t)-\left [ z_{2}^{j},1 \right ]^{T}  \right \|^{2}
$$
这个就是最小化重投影误差问题（minimizaiton of reprojection error），实际操作中，我们在调整每个$X^{j}$，使得更符合每一次观测$z^{j}$，也就是每个误差项都尽量小。由于这个原因，所以他也叫捆绑调整（Bundle Adjustment）。

BA很容易描述成图优化形式。图的结点为

* 相机的位置位姿：一个SE(3)的元素

* 空间中的特征点：是个XYZ坐标

边主要是有空间点到像素坐标的投影关系。也就是
$$\lambda_{2}\begin{bmatrix}
z_{2}^{j}\\
1
\end{bmatrix}=C\left ( RX^{j}+t \right )$$
*注：向量转置与其本身实质和意义都是一样的，与矩阵不同*

## g2o
其主要步骤如下：

1. 选择一个线性方程求解器，从 PCG, CSparse, Choldmod中选，实际则来自 g2o/solvers 文件夹中定义的文件夹。

![](/img/solvers.png)

```// 使用Cholmod中的线性方程求解器
g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
```
2. 选择一个 BlockSolver。
```// 6*3 的参数
g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
```
3. 选择一个迭代策略，从GN, LM, Doglog中选。
```// L-M 下降
g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( block_solver );
 ```
 4. 构造结点和边

结点，空间中的点和位姿，边是有空间中的点跟像素坐标的投影关系

#### 测试BA和对级约束求解(详见ubuntu虚拟机的代码)

 *对极约束*
$$ R=\begin{bmatrix}
0.9989066940689537 & -0.01520945268654653 & 0.04420507994800801\\
0.01389019427348682 & 0.9994533963165625 & 0.02999951823502051\\
-0.04463719354169673 & -0.02935270243545288 & 0.9985719502431745
 \end{bmatrix}, t=\begin{bmatrix}
 -0.287181547543988\\
  -0.1877533687454072\\
  -0.9392951779259762
 \end{bmatrix}$$
 *BA*
 $$
 R=\begin{bmatrix}
 0.999959 & -0.00778231 & -0.00471191\\
  0.0079586 & 0.999222 & 0.0386278\\
 0.00440763 & -0.0386637 & 0.999243
  \end{bmatrix}, t=\begin{bmatrix}
0.046212\\
-0.00945899\\
-0.00141173
  \end{bmatrix}
 $$

### 最小光度误差

 直接法主要根据灰度不变性假设进行实验，所以我们要实现最小光度误差
 等价于一个空间中的点，在两个不同位置上的相机投影产生的光度理论上是应该一致，所以整个最小化光度误差公式应由灰度方程组成。即

$$
e=I_{1}(p_{1})-I_{2}(p_{2}),
\min_{\xi}J(\xi)=e^{T}e
$$

$\xi$为相机变换李代数，不断调整相机位姿使得$e$的二范数值最小，这就是整个最小光度误差的原理。

由前面的投影方程可知
$$
p_{1}=\begin{bmatrix}
u\\
v
\end{bmatrix}_{1}=D\frac{1}{Z_{1}}KP,p_{2}=\begin{bmatrix}
u\\
v
\end{bmatrix}_{2}=D\frac{1}{Z_{2}}K(RP+t)=D\frac{1}{Z_{2}}Kexp(\xi^\wedge)P
$$

其中D是齐次到非齐次的转换

$$
D=\begin{bmatrix}
1&0&0\\
0&1&0
\end{bmatrix}
$$

因为空间中可能有许多类似于上述的三维点，所以构成了一个累和公式

$$
\min_{\xi}J(\xi)=\sum_{i=1}^{N}e_{i}^{T}e_{i},e_{i}=I_{1}(p_{1,i})-I_{2}(p_{2,i})
$$

既然要求最小值，所以一般我们先考虑其导数的情况，对于 $\min_{\xi}J(\xi)$ 的最小值，等价于求 $e_{i}$ 最小值，一般来说，我们想要求最小值先分析其导数比较合适，但是$e_{i}$不便对其进行求导，所以我们采用最基本的导数公式
$$
f^{'}(x_{0})=\frac{f(x_{0}+\Delta x)-f(x_{0})}{\Delta x}
$$

所以我们对 $e_{i}$ 加个扰动量，即

$$
e(\xi \oplus \delta \xi )\\
=I_{1}(\frac{1}{Z_{1}}DKP)-I_{2}(\frac{1}{Z_{2}}DKexp(\delta\xi^\wedge)exp(\xi^\wedge)P)\\
\approx I_{1}(\frac{1}{Z_{1}}DKP)-I_{2}(\frac{1}{Z_{2}}DK(1+\delta\xi^\wedge)exp(\xi^\wedge)P)\\
=I_{1}(\frac{1}{Z_{1}}DKP)-I_{2}(\frac{1}{Z_{2}}DKexp(\xi^\wedge)P)-I_{2}(\frac{1}{Z_{2}}DK\delta\xi^\wedge exp(\xi^\wedge)P)\\
=e(\xi)-I_{2}(\frac{1}{Z_{2}}DK\delta\xi^\wedge exp(\xi^\wedge)P)\\
$$

记， $q=\delta\xi^\wedge exp(\xi^\wedge)P$，$u=\frac{1}{Z_{2}}DKq$ 所以，上式一阶泰勒展开等于

$$
e(\xi \oplus \delta \xi )\\
=e(\xi)-I_{2}(u)\\
\approx e(\xi)-\frac{\partial I_{2}}{\partial u}\frac{\partial u}{\partial q}\frac{\partial q}{\partial \delta\xi^\wedge}\delta\xi^\wedge
$$
第一个式子 $\frac{\delta I_{2}}{\mathbf{u}}$ 是在 $\mathbf{u}$ 处的像素梯度
第二个式子 $\frac{\delta \mathbf{u}}{\delta q}$ 是投影方程关于相机坐标系下的三维点的导数。由前面的投影方程可知
$$
u=\frac{f_{x}X+c_{x}}{Z},v=\frac{f_{y}Y+c_{y}}{Z}
$$
于是，导数为

$$
\frac{\partial \mathbf{u}}{\partial q}=\begin{bmatrix}
\frac{\partial u}{\partial X} & \frac{\partial u}{\partial Y} & \frac{\partial u}{\partial Z}\\
\frac{\partial v}{\partial X} & \frac{\partial v}{\partial Y} & \frac{\partial v}{\partial Z}
\end{bmatrix}\\
=\begin{bmatrix}
\end{bmatrix}
$$

 误差相对于李代数的雅克比矩阵

 $$
J=\frac{\partial I_{2}}{\partial u}\frac{\partial u}{\partial \delta \xi }
 $$


## 2018.2.25
### Sophus
Li algebra
Eigen is a matrix operation library, including create Vector and Matrix

### Opencv
Mat can from a image file, and it's has many invariables, like rows, cols, channel_number

also, we learn PointCloud, its has a function __push_back（）__, which can storage class PointT, IsometryId, ImageDepth, and ImageColor

In addition, formation from pixel frame to 3D frame. However, real World Point must use 3D frame multiple SE(3). like this:

``Eigen::Vector3d pointWorld = T ** point;``

__this function can save pointcloud map__

``pcl::io::savePCDFileBinary("map.pcd", **pointCloud);``


## 2018.2.26
### point
when we define a type point, like char* p, it means p is the type char of start address, as we all known each type has different bytes, float is 4 bytes, char is 1 bytes, each address is a bytes.

    char var = 's';
    int var = 12;
    char* p = &var;  // right

``*p = var is value, and p = &var is address``

### OptimizationAlgorithm
#### Guass-Newton
$$
f(x+\Delta x)\approx f(x)+J(x)\Delta x. \\
\left \| f(x+\Delta x) \right \|_{2}^{2} \approx _{\Delta x}^{argmin}\frac{1}{2}\left \| f(x)+J(x)\Delta x \right \|_{2}^{2}.\\
=\frac{1}{2}(f(x)+J(x)\Delta x)^{T}(f(x)+J(x)\Delta x) \\
=\frac{1}{2}(\left \| f(x) \right \|_{2}^{2}+2f(x)J(x)^{T}\Delta x+\Delta x^{T}J(x)^{T}J(x)\Delta x)). \\
$$
对 $\Delta x$进行求导，并令其为零。
so,
$$
J(x)^{T}J(x)\Delta x=-f(x)J(x)^{T}
$$
将左侧定义为H，右侧定义为g
$$
H\Delta x=  g
$$

#### Levenberg-Marquardt

$$
_{\Delta x}^{min}\frac{1}{2}\left \| f(x_{k})+J(x_{k})\Delta x_{k}\right \|^{2}+\frac{\lambda}{2}\left \|D\Delta x\right \|^{2}.\\
(H+\lambda D^{T}D)\Delta x = g.\\
$$

we assume $D=I$ so,
$$
(H+\lambda D^{T}D)\Delta x = g.\\
$$

#### Eigen
functions:

``1. Identity()``

solve identity matrix（单位矩阵）

$$
\begin{bmatrix}
1 & 0 & 0\\
0 & 1 & 0\\
0 & 0 & 1
\end{bmatrix}
$$


``2. transpose()``

transposition of a matrix（矩阵的转置）

``3. norm()``

求向量的模长

#### g2o
### 2018.3.1
#### function: ``img.at()``
for a single channel grey scale image, *img.at()* is get the value of intensity

``Scalar intensity = img.at<uchar>(y, x)``

when consider a 3 channel image with RGB color

    Vec3b intensity = img.at<Vec3b>(y, x);
    uchar blue = intensity.val[0];
    uchar green = intensity.val[1];
    uchar red = intensity.val[2];

#### struct DMatch

Class for matching keypoint descriptors: query descriptor index, train descriptor index, train image index, and distance between descriptors.

#### class KeyPoint
Data structure for salient point detectors.

*Point2f pt*

coordinates of the keypoint

so, ``keypoints[m.queryIdx].pt.y`` is y direction coordinate.

#### pixel2cam
$$ Z=d$$
$$ X=\frac{u-c_{x}}{f_{x}}Z$$
$$ Y=\frac{u-c_{x}}{f_{x}}Z$$

this forum is __pixel2cam__

### 2018.3.9
#### LK OpticalFlow
it's main function is tracking and match features. it saves more time with respect to tranditional feature methods. So, when it finish match features, then, we should use SVD, PnP, ICP.

__calcOpticalFlowPyrLK(prevImg, nextImg, prevPts, nextPts, status, erros)__

#### Parameter:

status – Output status vector. Each element of the vector is set to 1 if the flow for the corresponding features has been found. Otherwise, it is set to 0.

__简单说光流法是不是只做到了特征点跟踪的而已，等于替换了特征点法的特征点匹配那个步骤
如果要基于光流法得到相机的运动，是不是就需要继续补充PnP或者ICP，而且光流法如果想进一步使用BA原理进行计算运动的话，那是不是就是直接法的第一种，稀疏直接法__

### 2018.3.14
#### OpticalFlow essence
1. extract features with fast
2. match features using intensity consistent assume
3. if we want to farther get the pose of camera, in addition to, it needs to PnP or ICP after matching
4. if step three is to solver pose using BA based on the assume of consistent intensity, it is called direct method.

### 2018.3.18
#### Eigen
这里是双精度，如果是单精度，只需要修改d为f即可

旋转矩阵（3*3）： Eigen::Matrix3d

旋转向量（3*1）：Eigen::AngleAxisd

欧拉角（3*1）：Eigen::Vector3d

四元数（4*1）：Eigen::Quaterniond

欧式变换矩阵（4*4）：Eigen::Isometry3d

仿射变换（4*4）：Eigen::Affine3d

射影变换（4*4）：Eigen::Projective3d



#### schur消元

对于线性方程
$$
H \Delta x = g
$$
求H矩阵的方法，其中 $\Delta x$ 是 $\Delta x_{c}=\begin{bmatrix}
\xi_{1}, & \xi_{2}, & ..., & \xi_{m}
\end{bmatrix}^{T} \in \mathbb{R}^{6m}, \Delta x_{p}=\begin{bmatrix}
p_{1}, & p_{2}, & ..., & p_{n}
\end{bmatrix}^{T} \in \mathbb{R}^{3n}$ 的统称，分别表示相机位姿和空间点的变量。

针对H矩阵的性质，我们将H矩阵分解成4块，$B, E, E^T, C$，该方法的精髓在于将一次性求解 $\Delta x, \Delta p$ 变成了先求解 $\Delta x$，再求解 $\Delta p$
$$
\begin{bmatrix}
B & E\\
E^T & C
\end{bmatrix}
\begin{bmatrix}
\Delta x_{c} \\
\Delta x_{p}
\end{bmatrix}=\begin{bmatrix}
\nu  \\
\omega
\end{bmatrix}
$$
我们将两边同时乘以 $\begin{bmatrix}
I & -EC^{-1}  \\
0 & I
\end{bmatrix}$

$$
\begin{bmatrix}
I & -EC^{-1}  \\
0 & I
\end{bmatrix}
\begin{bmatrix}
B & E\\
E^T & C
\end{bmatrix}
\begin{bmatrix}
\Delta x_{c} \\
\Delta x_{p}
\end{bmatrix}=\begin{bmatrix}
I & -EC^{-1}  \\
0 & I
\end{bmatrix}
\begin{bmatrix}
\nu  \\
\omega
\end{bmatrix}
$$

$$
\begin{bmatrix}
B-EC^{-1}E^T & 0  \\
E^T & C
\end{bmatrix}
\begin{bmatrix}
\Delta x_{c} \\
\Delta x_{p}
\end{bmatrix}=\begin{bmatrix}
\nu-EC^{-1}\omega  \\
\omega
\end{bmatrix}
$$
所以，就变成了先求 $(B-EC^{-1}E^T)\Delta x_{c} = \nu - EC^{-1}\omega$

之后将 $\Delta x_{c}$ 带入到 $\Delta x_{p}=C^{-1}(\omega - E^T\Delta x_{c})$ 即可

## 2018.3.19
### 光流法的核心
$$
A\Delta x = -b \\
\Delta x = -(A^TA)^{-1}A^Tb
$$
为啥要进行同时左乘一个 $A^T$ ，因为左边不是一个方阵，所以没法直接移过来求逆，所以我们需要通过变换，将其变成方阵

### 卡尔曼滤波的推导
__复合高斯分布的概念__

假设随机变量 $x \sim N(u_{x},\Sigma_{xx})$，另一个变量y满足：
$$
y=Ax+b+w
$$
其中 $A, b$ 为线性变量的系数矩阵和偏移量，$w$为噪声项，$w$ 为零均值的高斯分布：$w \sim N(0, R)$

y的高斯分布即为：
$$
p(y)=N(Au_{x}+b, R+A\Sigma_{xx}A^{T})
$$

__状态估计__

我们知道对于一个物体的状态估计应该分为预测和测量，其中预测主要是基于运动方程动模型得到的数据，测量主要是基于传感器等感知的数据
$$
x_{k}=f(x_{k-1},u_{k}) + w  \\
z_{k}=h(x_{k}) + v
$$
具体来说应该符合如下线性方程来描述：
$$
x_{k}=A_{k}x_{k-1}+u_{k}+w_{k}  \\
z_{k}=C_{k}x_{k}+v_{k}
$$
$C_{k}$ 也是状态变量到测量的转换矩阵。利用马尔可夫性，我们知道了k-1时刻的后验（在k-1时刻看来）状态估计 $\hat x_{k-1}$ 以及协方差 $\hat P_{k-1}$ ，现在要根据k时刻的输入（ $\hat x_{k-1}, \hat P_{k-1}$ ）和观测数据（ $z_{k}$ ），确定k时刻的后验分布 $\hat{x}_{k}$ 。为区分先验和后验，我们分别以 $\bar{x}_{k}$ ，$\hat{x}_{k}$ 表示。


其中假设 $x_{k-1}, z_{k}, w_{k}, v_{k}$ 是满足高斯分布，分别为 $x_{k-1} \sim N(\hat{x}_{k-1}, \hat{P}_{k-1}), w_{k} \sim N(0, R), v_{k} \sim N(0, Q)$

通过运动方程，确定 $x_{k}$ 的先验分布，根据复合高斯分布的性质，显然有
$$
x_{k}\sim N(\bar{x}_{k},\bar P_{k}) \\
\bar{P}_{k}=A_{k}\hat{P}_{k-1}A^T_{k}+Q \\
\bar{x}_{k}=A_{k}\hat{x}_{k-1}+u_{k}\\
\\
z_{k} \sim N(C_{k}\bar{x}_{k}, C_{k}\bar{P}_{k-1}C^T_{k}+Q)$$
所以 $x_{k}$ ， $z_{k}$ 组成的联合高斯分布的形式为：
$$
P(x_{k},z_{k})=N(\begin{bmatrix}
u_{x} \\
u_{y}
\end{bmatrix},\begin{bmatrix}
\Sigma_{xx} & \Sigma_{xy} \\
\Sigma_{yx} & \Sigma_{yy}
\end{bmatrix})\\
=N(\begin{bmatrix}
\bar x_{k} \\
C_{k}\bar x_{k}
\end{bmatrix},\begin{bmatrix}
\bar P_{k} & \bar P_{k}C^T_{k} \\
C_{k}\bar{P}_{k} & C_{k}\bar{P}_{k-1}C^T_{k}+Q
\end{bmatrix})
$$
由于 $x_{k-1} \sim N(\hat{x}_{k-1}, \hat{P}_{k-1})$，即 $x_{k} \sim N(\hat{x}_{k}, \hat{P}_{k})$ 。由贝叶斯公式 $p(x,y)=p(x|y)p(y)$，下文引自公式
$$
x_{k} \sim N(\hat{x}_{k}, \hat{P}_{k})=p(x|y)=p(x_{k}|z_{k})=N(u_{x}+\Sigma_{xx}\Sigma^{-1}_{xy}(y_{k}-u_{y}),\Sigma_{xx}-\Sigma_{xy}\Sigma^{-1}_{yy}\Sigma_{yx})
$$

$$
\hat x_{k}=\bar x_{k} + K_{k}(z_{k}-C_{k}\bar x_{k})\\
K_{k}=\bar P_{k}C^T_{k}(C_{k}\bar P_{k}C^T_{k}+R_{k})^{-1}\\
\hat P_{k}=(1-K_{k}C_{k})\bar P_{k}
$$



贝叶斯滤波的基本假设：

1. 马儿可夫性假设：t时刻的状态由t-1时刻的状态和t时刻的动作决定。t时刻的观测仅同t时刻的状态相关，即：
$$
p(x_t|x_{1:t-1}, z_{1:t}, u_{1:t})=p(x_t|x_{t-1},u_t)\\
$$
2. 观测噪声、模型噪声等是相互独立的
3. 静态环境，即对象周边的环境假设是不变的


$$
Bel(x_t)=P(x_t|u_1, z_1, ..., u_t, z_t)\\
=\eta P(z_t|x_t) \int P(x_t|u_t, x_{t-1})P(x_{t-1}|u_1, z_1,...,z_{t-1})dx_{t-1}
$$

## 2018.3.20
### 名词解释
先验概率 P(X)：仅仅依赖主观上的经验，事先根据已有的知识的推断

后验概率 P(X|Z)：是在相关证据或者背景给定并纳入考虑以后的条件概率

似然P(Z|X)：已知结果去推测固有性质的可能性

贝叶斯公式：
$$
P(A|B)=\frac{P(A)\times P(B|A)}{P(B)}
$$
后验分布正比于先验分布乘以似然

## 2018.3.25
### 最大似然估计（Maximize Likelihood Estimation, MLE)
$$
x_{MLE}^{*}=argmaxP(z|x)
$$
__在怎样的状态下，最可能产生现在观测到的数据__
