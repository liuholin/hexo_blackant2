---
title: 一步步完善视觉里程计2——FAST特征检测
date: 2017-09-17 23:45:17
tags:
---

**前一篇博客中已经将项目简单搭建起来，目前肯定不完善也可能会出现错误，这些在接下来的时间里会一步步的完善。**<!--more--></br>
前面视觉里程计总结的时候，我们知道视觉里程计的大致流程如下图:

![](http://7xl6tk.com1.z0.glb.clouddn.com/main_components.png)

对于[svo][svo]的作者，通过采用计算最小光度误差，来避免计算耗时较多的特征检测和鲁棒性的特征匹配。这一点在我们前面简单的视觉里程计就已经有所介绍。</br>
对于svo的流程具体如下：

![](http://7xl6tk.com1.z0.glb.clouddn.com/pipeline.png)

这一篇博客目前先实现Feature Extraction，如果是简单的FAST特征检测，我们在之前的博客中也已经说明，并且也基于OpenCV进行了实现，不过现在我们要从整个系统的角度去考虑这件事，也就是处理帧不仅仅是一幅图片。

基础成员数据结构
---

Frame
---
对于帧按照面向对象的思想，其应该包含自己固有的属性，比如帧的编号，帧对应的相机模型，对应的特征等等。具体定义如下：

{% codeblock %}
namespace mvo{

struct Feature;

typedef std::list<Feature*> Features;//特征list
typedef std::vector<cv::Mat> ImgPyr;//图像金字塔

/**	定义帧，保证帧的唯一性
*/
class Frame : public Noncopyable
{
public:
/**	帧的实例化，通过传入相机参数，获得的当前帧，及时间戳来确定
*/
Frame(AbstractCamera* cam, const cv::Mat& img, double timestamp);
~Frame();
/// 初始化新的图像帧，创建图像金字塔
void initFrame(const cv::Mat& img);

private:
/// 通过半采用的方式创建图像金字塔
void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr);

public:
static int                    frame_counter_;         //!< 创建帧的计数器，用于设置帧的唯一id
int                           id_;                    //!< 帧的唯一id
double                        timestamp_;             //!< 帧被记录的时间戳
AbstractCamera                *cam_;                  //!< 相机模型
ImgPyr                        img_pyr_;               //!< 图像金字塔
Features                      fts_;                   //!< 图像中的特征List
};
typedef std::shared_ptr<Frame> FramePtr;
}
{% endcodeblock %}

这边继承类Noncopyable，主要保证帧的唯一性，具体Noncopyable的思想就是把构造函数和析构函数设置为protected权限，这样子类可以调用，其它类不可以调用，然后其把复制构造函数和赋值函数设置为private，这也就意味着除非子类定义自己的copy构造和赋值函数，否则在子类没有定义的情况下，外面的调用者是不能够通过赋值和copy构造等手段来产生一个新的子类对象的，这样确保类的对象的唯一性，具体如下：

{% codeblock %}
class  Noncopyable
{
protected:
Noncopyable() {}
~Noncopyable() {}
private:
Noncopyable(const Noncopyable &);  // 这不需要再其它地方实现
Noncopyable& operator =(const Noncopyable &);
}; // End of class def.
{% endcodeblock %}

目前定义的帧成员变量主要有，帧的计数器用于设置帧的唯一id，帧创建的时间戳（后期用于同步，多传感器融合），相机模型（用于对图像进行畸变矫正），图像金字塔（主要用于提取不同尺度下的特征，具体尺度的讲解可以参考：[sift][sift]）以及帧对应的特征。

Camera
----

对于相机模型，定义了相机抽象类，提供了相机分辨率，定义摄像机坐标与图像像素坐标转换的相关抽象方法，具体如下：
{% codeblock %}
class  AbstractCamera
{
public:
AbstractCamera() {}; // 此构造函数供全景相机模型使用
AbstractCamera(int width, int height) : width_(width), height_(height) {};

virtual ~AbstractCamera() {};

/// 图像像素坐标转摄像机坐标系下的点
virtual Vector3d cam2world(const double& x, const double& y) const = 0;

/// 图像像素坐标转摄像机坐标系下的点
virtual Vector3d cam2world(const Vector2d& px) const = 0;

/// 摄像机坐标系下的点转图像像素坐标
virtual Vector2d world2cam(const Vector3d& xyz_c) const = 0;

/// 图像平面像素的世界坐标转像素坐标
virtual Vector2d world2cam(const Vector2d& uv) const = 0;

/// 返回x方向的焦距值
virtual double getFocalLength() const = 0;

/// 返回相机分辨率的宽度
inline int width() const { return width_; }
/// 返回相机分辨率的高度
inline int height() const { return height_; }

protected:
int width_; //!< 相机分辨率的宽度
int height_; //!< 相机分辨率的高度
};
{% endcodeblock %}

定义了抽象的相机类，目前实现了小孔相机模型，后期进一步考虑实现ATAN相机模型。</br>
小孔相机模型主要考虑了5个畸变参数，具体定义如下：
{% codeblock %}
class PinholeCamera : public AbstractCamera {

public:
// 考虑畸变参数k1,k2,p1,p2,k3
PinholeCamera(double width, double height,
double fx, double fy, double cx, double cy,
double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);

~PinholeCamera();

/// 图像像素坐标转摄像机坐标系下的点
virtual Vector3d cam2world(const double& x, const double& y) const;

/// 图像像素坐标转摄像机坐标系下的点
virtual Vector3d cam2world(const Vector2d& px) const;

/// 摄像机坐标系下的点转图像像素坐标
virtual Vector2d world2cam(const Vector3d& xyz_c) const;

/// 图像平面像素的世界坐标转像素坐标
virtual Vector2d world2cam(const Vector2d& uv) const;

/// 返回x方向的焦距值
virtual double getFocalLength() const
{
return fabs(fx_);
}

/// 获得矫正之后的图像，主要用于显示
void undistortImage(const cv::Mat& raw, cv::Mat& rectified);

/// 分别得到相机矩阵的4个参数
inline double fx() const { return fx_; };
inline double fy() const { return fy_; };
inline double cx() const { return cx_; };
inline double cy() const { return cy_; };

private:
double fx_, fy_;  //!< 相机两个方向的焦距值
double cx_, cy_;  //!< 相机的中心点
bool distortion_; //!< 是单纯的小孔相机模型，还是带有畸变？
double d_[5];     //!< 畸变参数，参考 http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
cv::Mat cvK_, cvD_;//!< 通过OpenCV表示的相机的相机矩阵和相机畸变参数
cv::Mat undist_map1_, undist_map2_;//!<相机畸变在两个方向的map，提供给remap函数使用
};
{% endcodeblock %}

具体实现参考：

{% codeblock %}
// 考虑畸变参数k1,k2,p1,p2,k3
PinholeCamera::PinholeCamera(double width, double height,
double fx, double fy,
double cx, double cy,
double k1, double k2, double p1, double p2, double k3) :
AbstractCamera(width, height),
fx_(fx), fy_(fy), cx_(cx), cy_(cy),
distortion_(fabs(k1) > 0.0000001),
undist_map1_(height_, width_, CV_16SC2),
undist_map2_(height_, width_, CV_16SC2)
{
// 径向畸变参数
d_[0] = k1; d_[1] = k2; d_[2] = p1; d_[3] = p2; d_[4] = k3;
cvK_ = (cv::Mat_<float>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
cvD_ = (cv::Mat_<float>(1, 5) << d_[0], d_[1], d_[2], d_[3], d_[4]);
// 根据相机矩阵和畸变参数构建map
cv::initUndistortRectifyMap(cvK_, cvD_, cv::Mat_<double>::eye(3, 3), cvK_,
cv::Size(width_, height_), CV_16SC2, undist_map1_, undist_map2_);
}

PinholeCamera::~PinholeCamera(){}

Vector3d PinholeCamera::cam2world(const double& u, const double& v) const
{
Vector3d xyz;
if (!distortion_)
{
xyz[0] = (u - cx_) / fx_;
xyz[1] = (v - cy_) / fy_;
xyz[2] = 1.0;
}
else
{
cv::Point2f uv(u, v), px;
const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
cv::undistortPoints(src_pt, dst_pt, cvK_, cvD_);
xyz[0] = px.x;
xyz[1] = px.y;
xyz[2] = 1.0;
}
return xyz.normalized();
}

Vector3d PinholeCamera::cam2world(const Vector2d& uv) const
{
return cam2world(uv[0], uv[1]);
}

/// 摄像机坐标系下的点转图像像素坐标
Vector2d PinholeCamera::world2cam(const Vector3d& xyz) const
{
Vector2d  uv = xyz.head<2>() / xyz[2];
return world2cam(uv);
}

/// 图像平面像素的世界坐标转像素坐标
Vector2d PinholeCamera::world2cam(const Vector2d& uv) const
{
Vector2d px;
if (!distortion_)
{
px[0] = fx_*uv[0] + cx_;
px[1] = fy_*uv[1] + cy_;
}
else
{
double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
x = uv[0];
y = uv[1];
r2 = x*x + y*y;
r4 = r2*r2;
r6 = r4*r2;
a1 = 2 * x*y;
a2 = r2 + 2 * x*x;
a3 = r2 + 2 * y*y;
cdist = 1 + d_[0] * r2 + d_[1] * r4 + d_[4] * r6;//1+k1r2+k2r4+k3r6
xd = x*cdist + d_[2] * a1 + d_[3] * a2;
yd = y*cdist + d_[2] * a3 + d_[3] * a1;
px[0] = xd*fx_ + cx_;
px[1] = yd*fy_ + cy_;
}
return px;
}

void PinholeCamera::undistortImage(const cv::Mat& raw, cv::Mat& rectified)
{
if (distortion_)
cv::remap(raw, rectified, undist_map1_, undist_map2_, CV_INTER_LINEAR);
else
rectified = raw.clone();
}
{% endcodeblock %}

对于小孔相机模型就不多说了，主要公式可以参考：</br>
[http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html](http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)

image and feature
----

那这样我们基础的相机模型已经设计与实现完毕，那下面我们就考虑帧对应的图像金字塔，对于图像金字塔，直接采用了OpenCV的数据结构 **cv::Mat**，定义为 **typedef std::vector<cv::Mat> ImgPyr**;而对于特征feature我们定义如下：

{% codeblock %}
struct Feature
{
**EIGEN_MAKE_ALIGNED_OPERATOR_NEW**

///特征类型,目前先只考虑角点，后续考虑其它特征的时候，再进行添加
enum FeatureType {
CORNER//角点
};
FeatureType type;     //!< 特征类型，角点
Frame* frame;         //!< 指针指向特征被检测到所对应的帧
Vector2d px;          //!< 特征在金字塔等级为0时的像素坐标
int level;            //!< 特征被提取时，图像金字塔的等级

Feature(Frame* _frame, const Vector2d& _px, int _level) :
type(CORNER),
frame(_frame),
px(_px),
level(_level)
{}

~Feature(){}
};
{% endcodeblock %}

由于包含了Eigen库中的成员变量，通过采用EIGEN_MAKE_ALIGNED_OPERATOR_NEW，会对结构体进行重载new操作，生成16字节对齐指针，具体可以参考[http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html](http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html)</br>
对于特征类型，目前先考虑为角点，后期考虑添加其它特征时再进行添加，具体还包含了指针指向特征被检测所对应的帧，特征在金字塔等级为0时的像素坐标，以及考虑特征所在的金字塔等级。



帧的初始化
---

而具体帧初始化时，也就是对帧对应的图像进行构造图像金字塔。具体如下：
{% codeblock %}
void Frame::initFrame(const cv::Mat& img)
{
// 检测图像，保证图像大小与相机模型大小一致，以及图像为灰度图像
if (img.empty() || img.type() != CV_8UC1 || img.cols != cam_->width() || img.rows != cam_->height())
throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");

// 构建图像金字塔，默认金字塔的等级为5
createImgPyramid(img, 5, img_pyr_);
}
{% endcodeblock %}

更详细实现参考[github][git]


特征检测
---

上面基本的类型已经定义好了，下面进行特征提取，对于特征提取，为了特征跟踪的时候，不容易出现突变，通过将图像进行划分为单元格，使得检测到的特征尽可能的分散，尽量保证每个单元格有一个特征。</br>
对特征检测也进行抽象，为了后期可以方便的切换到其它特征检测算法，具体定义如下：
{% codeblock %}
/// 特征检测的抽象类
class AbstractDetector
{
public:
AbstractDetector(
const int img_width,
const int img_height,
const int cell_size,
const int n_pyr_levels);

virtual ~AbstractDetector() {};

virtual void detect(
Frame* frame,
const ImgPyr& img_pyr,
const double detection_threshold,
Features& fts) = 0;

protected:
/// 将所有格子重新设置，设置为没有占用
void resetGrid();

protected:

const int cell_size_;             //!< 设置寻找角点单元格的大小
const int n_pyr_levels_;          //!< 图像金字塔的等级
const int grid_n_cols_;           //!< 将图像划分为格子后的列数
const int grid_n_rows_;           //!< 将图像划分为格子后的行数
std::vector<bool> grid_occupancy_;//!< 设定划分的所有格子数是否被占用
};
{% endcodeblock %}

定义抽象特征提取的时候，考虑了特征的分散，将图像分成 **grid_n_cols_** \* **grid_n_rows_**个格子，每个格子尽量对应一个特征。
具体抽象类的实现如下：
{% codeblock %}
AbstractDetector::AbstractDetector(
const int img_width,
const int img_height,
const int cell_size,
const int n_pyr_levels) :
cell_size_(cell_size),
n_pyr_levels_(n_pyr_levels),
grid_n_cols_(ceil(static_cast<double>(img_width) / cell_size_)),
grid_n_rows_(ceil(static_cast<double>(img_height) / cell_size_)),
grid_occupancy_(grid_n_cols_*grid_n_rows_, false)
{}

void AbstractDetector::resetGrid()
{
std::fill(grid_occupancy_.begin(), grid_occupancy_.end(), false);
}
{% endcodeblock %}

定义好特征检测的抽象类，下面我们主要使用的特征检测算法为FAST特征检测，我们定义FAST特征检测类，实现抽象类中的抽象方法。</br>
具体定义如下：
{% codeblock %}
class FastDetector : public AbstractDetector
{
public:
FastDetector(
const int img_width,
const int img_height,
const int cell_size,
const int n_pyr_levels);

virtual ~FastDetector() {}

virtual void detect(
Frame* frame,
const ImgPyr& img_pyr,
const double detection_threshold,
Features& fts);

private:
float shiTomasiScore(const cv::Mat& img, int u, int v);
};
{% endcodeblock %}

我们看到，定义了**shiTomasiScore**方法，我们对通过FAST检测出的角点进行了进一步的筛选，添加Shi-Tomasi角点提取的方法，确保提取出的角点的稳定性。</br>
而具体的FAST特征检测，我们采用了第三方库[fast][fast]，主要作者通过SSE2指令进行了实现，效率高。</br>
具体实现如下：
{% codeblock %}
void FastDetector::detect(
Frame* frame,
const ImgPyr& img_pyr,
const double detection_threshold,
Features& fts)
{
Corners corners(grid_n_cols_*grid_n_rows_, Corner(0, 0, detection_threshold, 0, 0.0f));
// 对每层金字塔都进行fast特征检测
for (int L = 0; L < n_pyr_levels_; ++L)
{
const int scale = (1 << L);
std::vector<fast::fast_xy> fast_corners;
#if __SSE2__
fast::fast_corner_detect_10_sse2(
(fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#else
fast::fast_corner_detect_10(
(fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#endif
std::vector<int> scores, nm_corners;
fast::fast_corner_score_10((fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols, fast_corners, 20, scores);
fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

for (auto it = nm_corners.begin(), ite = nm_corners.end(); it != ite; ++it)
{
fast::fast_xy& xy = fast_corners.at(*it);
const int k = static_cast<int>((xy.y*scale) / cell_size_)*grid_n_cols_
+ static_cast<int>((xy.x*scale) / cell_size_);// 获得一维列的索引
if (grid_occupancy_[k])// 如果这个格子里面已经有特征，则该特征可以不必再进行计算了
continue;
const float score = shiTomasiScore(img_pyr[L], xy.x, xy.y);//计算shi-Tomasi角点检测，根据阈值选择更好的角点
if (score > corners.at(k).score)
corners.at(k) = Corner(xy.x*scale, xy.y*scale, score, L, 0.0f);
}
}

// 返回所有的特征
std::for_each(corners.begin(), corners.end(), [&](Corner& c) {
if (c.score > detection_threshold)
fts.push_back(new Feature(frame, Vector2d(c.x, c.y), c.level));
});

resetGrid();
}
{% endcodeblock %}

我们对金字塔的每层图像都进行fast特征检测，对检测出的角点再通过shi-Tomasi算法进一步进行角点确定，具体实现如下：

{% codeblock %}
float FastDetector::shiTomasiScore(const cv::Mat& img, int u, int v)
{
assert(img.type() == CV_8UC1);

float dXX = 0.0;
float dYY = 0.0;
float dXY = 0.0;
const int halfbox_size = 4;
const int box_size = 2 * halfbox_size;
const int box_area = box_size*box_size;
const int x_min = u - halfbox_size;
const int x_max = u + halfbox_size;
const int y_min = v - halfbox_size;
const int y_max = v + halfbox_size;

if (x_min < 1 || x_max >= img.cols - 1 || y_min < 1 || y_max >= img.rows - 1)
return 0.0; // 面片太靠近边界，返回0

const int stride = img.step.p[0];//一行元素的个数
for (int y = y_min; y < y_max; ++y)
{
const uint8_t* ptr_left = img.data + stride*y + x_min - 1;
const uint8_t* ptr_right = img.data + stride*y + x_min + 1;
const uint8_t* ptr_top = img.data + stride*(y - 1) + x_min;
const uint8_t* ptr_bottom = img.data + stride*(y + 1) + x_min;
for (int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom)
{
float dx = *ptr_right - *ptr_left;
float dy = *ptr_bottom - *ptr_top;
dXX += dx*dx;
dYY += dy*dy;
dXY += dx*dy;
}
}

// 返回小的特征值
dXX = dXX / (2.0 * box_area);
dYY = dYY / (2.0 * box_area);
dXY = dXY / (2.0 * box_area);
return 0.5 * (dXX + dYY - sqrt((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
}
{% endcodeblock %}

对于return的结果不是很明白，如果有人看博客，了解的话，还请不吝赐教。

特征检测测试
----

上述过程，整个特征检测lib就已经差不多完善了，下面我们就写一个测试程序。</br>
具体如下：
{% codeblock %}
#include <string>
#include <opencv2/opencv.hpp>
#include <openmvo/utils/cmd_line.h>
#include <openmvo/utils/timer.h>
#include <openmvo/mvo/pinhole_camera.h>
#include <openmvo/mvo/fast_detector.h>
#include <openmvo/mvo/frame.h>

using namespace mvo;
using namespace std;

int main(int argc, char *argv[])
{
CmdLine cmd;
std::string img_name;
cmd.add(make_option('i', img_name, "imgname"));
try {
if (argc == 1) throw std::string("Invalid command line parameter.");
cmd.process(argc, argv);
}
catch (const std::string& s) {
std::cerr << "Feature detector \nUsage: " << argv[0] << "\n"
<< "[-i|--imgname name]\n"
<< std::endl;

std::cerr << s << std::endl;
return EXIT_FAILURE;
}
cv::Mat img(cv::imread(img_name, 0));
assert(img.type() == CV_8UC1 && !img.empty());

AbstractCamera* cam = new PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
FramePtr frame(new Frame(cam, img, 0.0));
Features fts;
FastDetector fast_detector(img.cols, img.rows, 25, 3);
Timer t;
for (int i = 0; i < 100; ++i)
{
fast_detector.detect(frame.get(), frame->img_pyr_, 20.0, fts);
}
printf("Fast corner detection took %f ms, %d corners detected (ref i7-W520: 7.166360ms, 40000)\n", t.Stop() * 10, fts.size());
printf("Note, in this case, feature detection also contains the cam2world projection of the feature.\n");
cv::Mat img_rgb = cv::Mat(img.size(), CV_8UC3);
cv::cvtColor(img, img_rgb, CV_GRAY2RGB);
std::for_each(fts.begin(), fts.end(), [&](Feature* i){
cv::circle(img_rgb, cv::Point2f(i->px[0], i->px[1]), 4 * (i->level + 1), cv::Scalar(0, 255, 0), 1);
});
cv::imshow("ref_img", img_rgb);
cv::waitKey(0);

std::for_each(fts.begin(), fts.end(), [&](Feature* i){ delete i; });
return 0;
}
{% endcodeblock %}

这边添加了CmdLine处理，通过对相关可调参数通过给main函数传参获得。具体效果如下：

![](http://7xl6tk.com1.z0.glb.clouddn.com/fast_detector.png)

圈的大小代表了尺度的大小，也就是在不同金字塔下检测的特征。

总结
---

到目前为止，一个fast特征检测差不多基本结束，完整的代码可参考[https://github.com/yueying/OpenMVO](https://github.com/yueying/OpenMVO)  </br>
主要讲述了整个流程，而具体FAST特征检测的过程所述不多，第三方库FAST采用了SSE2指令，后续对这一块进一步学习。





转载自冯兵的博客，[原文链接][link]

[link]:http://fengbing.net/2015/08/08/%E4%B8%80%E6%AD%A5%E6%AD%A5%E5%AE%9E%E7%8E%B0%E5%8D%95%E7%9B%AE%E8%A7%86%E8%A7%89%E9%87%8C%E7%A8%8B%E8%AE%A12%E2%80%94%E2%80%94FAST%E7%89%B9%E5%BE%81%E6%A3%80%E6%B5%8B/



[svo]:https://github.com/uzh-rpg/rpg_svo
[sift]:http://www.cs.ubc.ca/~lowe/keypoints/
[git]:https://github.com/yueying/OpenMVO
[fast]:https://github.com/uzh-rpg/fast
