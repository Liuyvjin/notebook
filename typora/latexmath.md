# $L^AT_EX$公式语法

[TOC]

## 1. 自定义命令

```latex
$$
\def\bold#1{\boldsymbol{#1}}
\bold{this\ is\ now\ bold}
$$
```

效果如下：

$$
\def\bold#1{\boldsymbol{#1}}
\bold{this\ is\ now\ bold}
$$

## 2. 常用元素

* 分数：`1\over 2`   $1\over 2$  用于分子分母比较长的情况；`frac{1}{2}`用于分子分母短的情况
* 求和： `\sum_1^n`  $\sum_1^n$
* 积分： `\int_1^n` $\int_1^n$ ;   环路积分`\oint_1^n`  $\oint_1^n$
* 极限：`\lim_{x \to \infty}` $\lim_{x \to \infty}$
* 括号：大小括号直接使用，大括号由于用来分组，需要转义`\{`；小括号在有分数的时候建议用`\left( \right)`$\left( \frac{1}{\frac{1}{2}}\right)$,对比直接使用$(\frac{1}{\frac{1}{2}})$
* 对数： `log_{10} `  $\log_{10}​$
* 三角函数： `\sin \cos`  $\sin{\theta}​$
* 点乘与叉乘： `\cdot` $a\cdot b$；`\times` $a\times b$
* 向量符号与加粗：`\vec{a}` $\vec A$ ;  `\mathbf`  $\mathbf{A}$
* 矩阵中省略号： `\cdots` $\cdots$ ；`\ddots`  $\ddots$    ；`\vdots`     $\vdots$ 
* 矢量微分算子：`\nabla`  $\nabla$

## 3. 公式环境

1. 对齐环境`align`

   `$` 用于标志对齐位置，`\\` 用于换行
   $$
   \begin{align}
   ac+ad+bc+bd &= a(c+d)+b(c+d)\\
   			&= (a+b)(c+d)\\
   \end{align}
   $$

2. 无括号矩阵 `matrix` ； 方括号`bmatrix`；圆括号`bmatrix`；圆括号`Bmatrix`；行列式`vmatrix`；取模`Vmatrix`

   `$` 用于标志对齐位置，`\\` 用于换行
   $$
   \begin{matrix}
        1 & x & x^2 \\
        1 & y & y^2 \\
        1 & z & z^2 \\
    \end{matrix}\quad
    \begin{bmatrix}
        1 & x & x^2 \\
        1 & y & y^2 \\
        1 & z & z^2 \\
    \end{bmatrix}\quad
    \begin{pmatrix}
        1 & x & x^2 \\
        1 & y & y^2 \\
        1 & z & z^2 \\
    \end{pmatrix}\quad
    \begin{vmatrix}
        1 & x & x^2 \\
        1 & y & y^2 \\
        1 & z & z^2 \\
    \end{vmatrix}
   $$

3. 大括号(条件定义)环境 `cases`
   $$
   f(n) =
   \begin{cases}
   	n/2,  & \text{if $n$ is even} \\
   	3n+1, & \text{if $n$ is odd}
   \end{cases}
   $$

4. 数组环境  `array`

   对齐位置`&` ，对齐方式`r/c/l`，分割线 `|` 
   $$
   \left(                 %左括号
     \begin{array}{cc|r}   %该矩阵一共3列，每一列都居中放置
       a11 & a12 & b\times c\\  %第一行元素
       a21 & a22 & d\\  %第二行元素
     \end{array}
   \right)
   $$

## 3. 一些例子

* 加减和分号

```latex
x = {-b \pm \sqrt{b^2-4ac} \over 2a}
```

$$
x = {-b \pm \sqrt{b^2-4ac} \over 2a}
$$

* 积分号

```latex
f(a) = \oint\frac{f(z)}{z-a}dz
```

$$
f(a) = \frac{1}{2\pi i} \oint_1^2\frac{f(z)}{z-a}dz
$$

* 三角函数

```latex
\cos(θ+φ)=\cos(θ)\cos(φ)−\sin(θ)\sin(φ)
```

$$
\cos(θ+φ)=\cos(θ)\cos(φ)−\sin(θ)\sin(φ)
$$

* 矢量微分算子，偏导数，点乘

```latex
\int_D ({\nabla\cdot} F)dV=\int_{\partial D} F\cdot ndS
```

$$
\int_D ({\nabla\cdot} F)dV=\int_{\partial D} F\cdot ndS
$$

* 向量标志，叉乘，公式粗体，左右括号

```latex
\vec{\nabla} \times \vec{F} = \left( \frac{\partial F_z}{\partial y} - \frac{\partial F_y}{\partial z} \right) \mathbf{i} ...
```

$$
\vec{\nabla} \times \vec{F} = \left( \frac{\partial F_z}{\partial y} - \frac{\partial F_y}{\partial z} \right) \mathbf{i} ...
$$

* 求和符号

```latex
\sigma = \sqrt{ \frac{1}{N} \sum_{i=1}^N (x_i -\mu)^2}
```

$$
\sigma = \sqrt{ \frac{1}{N} \sum_{i=1}^N (x_i -\mu)^2}
$$





