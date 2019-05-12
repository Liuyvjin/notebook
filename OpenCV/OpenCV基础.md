# OpenCV 学习笔记

#### 目录

[TOC]

## 一、开始使用

### 1. 头文件

OpenCV中的C++类和函数都是定义在命名空间cv之内的，有两种方法可以访问：

* 第一种是，在代码开头的适当位置，加上`usingnamespace cv`;这句。

* 另外一种是在使用OpenCV类和函数时，都加入cv::命名空间。不过这种情况难免会不爽，每用一个OpenCV的类或者函数，都要多敲四下键盘写出cv::，很麻烦。[# OpenCV学习笔记]

所以一般开头标配：

```cpp
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
```

### 2. CmakeLists.txt文件的配置

在文件中添加如下两行：

```makefile
find_package(OpenCV REQUIRED)
target_link_libraries(Proj_name ${OpenCV_LIBS})#Proj_name是工程名
```

[回到目录](#目录)

## 二、 图像的读取、显示和写入

### 1. 图像读取`imread`

```cpp
Mat imread(const string& filename, intflags=1 );
```
第二个参数，int类型的flags，为载入标识，它指定一个加载图像的颜色类型。下面是一些例子：

> Mat image1 = imread("dota.jpg",0); //载入灰度图
>
> Mat image2 = imread("dota.jpg",1); //载入3通道的彩色图像
>
> Mat logo = imread("dota_logo.jpg"); //载入3通道的彩色图像

### 2. 打开窗口`namedWindow`

打开一个窗口，并指定窗口名其实可以直接`imshow("windowname",image)` 来创建窗口+show图像

```cpp
void namedWindow(const string& winname,int flags=WINDOW_AUTOSIZE ); 
```

* 第一个参数，const string&型的name，即填被用作窗口的标识符的窗口名称。
* 第二个参数，int 类型的flags ，窗口的标识，可以填如下的值：
  + WINDOW_NORMAL设置了这个值，用户便可以改变窗口的大小（没有限制）
  + WINDOW_AUTOSIZE如果设置了这个值，窗口大小会自动调整以适应所显示的图像，并且不能手动改变窗口大小。
  + WINDOW_OPENGL 如果设置了这个值的话，窗口创建的时候便会支持OpenGL

我们可以调用destroyWindow()或者destroyAllWindows()函数来关闭窗口，并取消之前分配的与窗口相关的所有内存空间。

但其实对于代码量不大的简单小程序来说，完全没有必要手动调用上述的destroyWindow()或者destroyAllWindows()函数，因为在退出时，所有的资源和应用程序的窗口会被操作系统会自动关闭

### 3. 显示图像`imshow`

```cpp
void imshow(const string& winname, InputArray mat);
```

+ 第一个参数，const string&类型的winname，填需要显示的窗口标识名称。如果没有窗口就新建一个
+ 第二个参数，InputArray 类型的mat，填需要显示的图像。
  遇到函数原型中的InputArray类型，我们把它简单地当做Mat类型就行了

函数对像素的处理如下：

- 如果载入的图像是8位无符号类型（8-bit unsigned），就显示图像本来的样子。
- 如果图像是16位无符号类型（16-bit unsigned）或32位整型（32-bit integer），便用像素值除以256。也就是说，值的范围是[0,255 x 256]映射到[0,255]。
- 如果图像是32位浮点型（32-bit floating-point），像素值便要乘以255。也就是说，该值的范围是[0,1]映射到[0,255]。

### 4. 写入图像`imwrite`

将图像保存到指定的文件。图像格式是基于文件扩展名的

```cpp
bool imwrite(const string& filename, InputArray img, const 
             vector<int>& params=vector<int>() ); 
```

- 第一个参数，const string&类型的filename，填需要写入的文件名就行了，带上后缀，比如，“123.jpg”这样。
- 第二个参数，InputArray类型的img，一般填一个Mat类型的图像数据就行了。
- 第三个参数，const vector<int>&类型的params，表示为特定格式保存的参数编码，它有默认值vector<int>()，所以一般情况下不需要填写。

[回到目录](#目录)

## 三、图像的叠加

### 1. 设定兴趣区域 ROI

相当于在矩阵中取一块出来，类似于python里的切片操作，切片出来的imageROI和原来的image1共享同一块空间。有两种常用方法：

（1）cv::rect  指定矩形区域

指定矩形的左上角坐标（构造函数的前两个参数）和矩形的长宽（构造函数的后两个参数）就可以定义一个矩形区域。

```cpp
Mat imageROI; //定义一个Mat
imageROI=image1(Rect(500,250,logo.cols,logo.rows));
```

（2）Range 指定行列的范围

Range是指从起始索引到终止索引（==不包括终止索引==）的一连段连续序列。cv::Range可以用来定义Range对象。如果使用cv::Range来定义ROI，那么前例中定义ROI的代码可以重写为：

```cpp
imageROI=image1(Range(250,250+logoImage.rows),Range(200,200+logoImage.cols));
```

### 2. 将图像拷贝到ROI完成叠加

```cpp
//加载掩模（要和原图logoImage大小相同，通道数要么相同，要么是1通道，而灰度图是单通道的）
Mat mask = imread("logoImage.jpg", 0);
//将logoImage在掩膜过滤下（对应掩膜为0的logoImage的像素被过滤），拷贝到ROI
logoImage.copyTo(imageROI, mask);
```

### 3. 图像线性叠加`addweighted`

```cpp
void addWeighted(InputArray src1, double alpha, 
                  InputArray src2, double beta, 
                  double gamma, OutputArray dst, 
                 int dtype=-1)；
```

* src1，InputArray类型，表示需要加权的第一个数组，常常填一个Mat。
* alpha，表示第一个数组的权重
* src2，表示第二个数组，它需要和第一个数组拥有相同的尺寸和通道数。
* beta，表示第二个数组的权重值。
* dst，输出的数组，它和输入的两个数组拥有相同的尺寸和通道数。
* gamma，一个加到权重总和上的标量值。看下面的式子自然会理解。
* dtype，输出阵列的可选深度，有默认值-1。当两个输入数组具有相同的深度时，这个参数设置为-1（默认值），即等同于src1.depth（）。   

上述函数可以用下面的公式来理解：
$$
dst = src_1\times \alpha +src_2\times \beta +\gamma
$$
[图像叠加—代码](<https://github.com/Liuyvjin/OpenCV_begin/tree/master/EX1>)

[回到目录](#目录)

## 四、 多通道图像的分离 和 混合

### 1. 分离通道 split

```cpp
void split(const Mat& src, Mat*mvbegin);
void split(InputArray m,OutputArrayOfArrays mv); //第二个参数是输出的色彩通道，类型为mat的数组例如 vector<mat> channels(3)，分离成channels.at(1/2/3)
```

### 2. 合并通道 merge

```cpp
void merge(const Mat* mv, size_t count, OutputArray dst);
void merge(InputArrayOfArrays mv, OutputArray dst);
```

* mv，填需要被合并的输入矩阵或vector容器的阵列，这个mv参数中所有的矩阵必须有着一样的尺寸和深度。
* count，当mv为一个空白的C数组时，代表输入矩阵的个数，这个参数显然必须大于1。

[图像的分离和混合—代码](<https://github.com/Liuyvjin/OpenCV_begin/tree/master/EX2>)

[回到目录](#目录)

## 五、创建Trackbar & 图像对比度、亮度值调整

### 1. 创建Trackbar

```cpp
int createTrackbar(conststring& trackbarname, conststring& winname,
                   int* value, int max, TrackbarCallback onChange=0,
                   void* userdata=0);
```

* trackbarname，const string&类型，表示轨迹条的名字，用来代表我们创建的轨迹条。
* winname，const string&类型，窗口的名字，表示这个轨迹条会依附到哪个窗口上，即对应namedWindow（）创建窗口时填的某一个窗口名。
* value，int* 类型，表示滑块的位置。并且在创建时，滑块的初始位置就是该变量当前的值。程序会把value的指针传给回调函数，同时主程序也可以访问value。
* max，int类型，表示滑块可以达到的最大位置的值，0～max。
* onChange，TrackbarCallback类型，有默认值0。这是指向回调函数的指针，每次滑块位置改变时，这个函数都会进行回调。并且这个函数的原型必须为`void XXX(int,void\*)`。其中第一个参数是轨迹条的位置，第二个参数是用户数据（看下面的第六个参数）。如果回调是NULL指针，表示没有回调函数的调用，仅第三个参数value有变化。
* userdata，void*类型，有默认值0。这个参数是用户传给回调函数的数据，用来处理轨迹条事件。作为value的补充。

### 2. 获取当前轨迹条的位置 getTrackbarPos

```cpp
int getTrackbarPos(conststring& trackbarname, conststring& winname);
```

### 3. 亮度和对比度调整

亮度和对比度的调整可以由以下公式概括：
$$
g(x,y)=a\times f(x,y)+b
$$

- 参数f(x,y)表示源图像像素。
- 参数g(x,y) 表示输出图像像素。
- 参数a（需要满足a>0）被称为增益（gain），常常被用来控制图像的对比度。
- 参数b通常被称为偏置（bias），常常被用来控制图像的亮度。

在程序中可写为

```cpp
dstImage = srcImage * ContrastValue + BrightValue;
```

[用轨迹条调整亮度对比度—代码](<https://github.com/Liuyvjin/OpenCV_begin/tree/master/EX3>)

[回到目录](#目录)

