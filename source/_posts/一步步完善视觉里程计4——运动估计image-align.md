---
title: 一步步完善视觉里程计4——运动估计image align
date: 2017-09-18 20:45:47
tags:
---
**上一步中已经确定了初始位置，下面就是对连续帧的处理，也就是开始进行运动估计。</br>
前面已经说了，对后续姿态的估计不是采用特征检测和鲁棒性的特征匹配（因此操作较耗时），而采用最小化光度误差。**<!--more-->

具体思路
---

![](/img/formula.png)

具体示意图如下：

![](http://7xl6tk.com1.z0.glb.clouddn.com/sparse_img_alignment.png)
![](/img/t.png)

这边要注意的，在光流计算中，要考虑warp，目前考虑时间不对pitch进行操作。因此目前算法是建立在帧到帧的运动较小，以及计算的pitch较小，因此希望相机的帧率较高。</br>
具体上述问题是一个非线性最小二乘，通过Levenberg Marquardt 和 Gauss Newton方法进行求解。

最小二乘求解过程
----

具体过程写成通用形式如下,参考：《孟繁雪，非线性最小二乘问题的混合算法》：

![](http://7xl6tk.com1.z0.glb.clouddn.com/nlls1.png)
![](http://7xl6tk.com1.z0.glb.clouddn.com/nlls2.png)
![](http://7xl6tk.com1.z0.glb.clouddn.com/nlls3.png )
![](http://7xl6tk.com1.z0.glb.clouddn.com/nlls4.png )
![](http://7xl6tk.com1.z0.glb.clouddn.com/nlls5.png )
![](http://7xl6tk.com1.z0.glb.clouddn.com/nlls6.png )
![](http://7xl6tk.com1.z0.glb.clouddn.com/nlls7.png )

具体代码
---

首先对当前图像进行Warp处理以对应参考图像
{% codeblock %}
void SparseImgAlign::precomputeReferencePatches()
{
const int border = patch_halfsize_ + 1;
const cv::Mat& ref_img = ref_frame_->img_pyr_.at(level_);//得到当前金字塔等级参考图像
const int stride = ref_img.cols;
const float scale = 1.0f / (1 << level_);
const Vector3d ref_pos = ref_frame_->pos();//当前帧世界坐标系中的坐标
const double focal_length = ref_frame_->cam_->getFocalLength();
size_t feature_counter = 0;
std::vector<bool>::iterator visiblity_it = visible_fts_.begin();
for (auto it = ref_frame_->fts_.begin(), ite = ref_frame_->fts_.end();
it != ite; ++it, ++feature_counter, ++visiblity_it)
{
// 确保面片在图像内
const float u_ref = (*it)->px[0] * scale;
const float v_ref = (*it)->px[1] * scale;
const int u_ref_i = floorf(u_ref);
const int v_ref_i = floorf(v_ref);
if ((*it)->point == NULL || u_ref_i - border < 0 || v_ref_i - border < 0 || u_ref_i + border >= ref_img.cols || v_ref_i + border >= ref_img.rows)
continue;
*visiblity_it = true;

// 这边不能直接使用3d点的坐标，会存在重投影的误差，而是通过单位点乘以深度的方式进行计算
const double depth(((*it)->point->pos_ - ref_pos).norm());
const Vector3d xyz_ref((*it)->f*depth);

// 估计投影的雅克比矩阵
Matrix<double, 2, 6> frame_jac;
Frame::jacobian_xyz2uv(xyz_ref, frame_jac);

// 对参考图像进行双边差值操作
const float subpix_u_ref = u_ref - u_ref_i;
const float subpix_v_ref = v_ref - v_ref_i;
const float w_ref_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
const float w_ref_tr = subpix_u_ref * (1.0 - subpix_v_ref);
const float w_ref_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
const float w_ref_br = subpix_u_ref * subpix_v_ref;
size_t pixel_counter = 0;
float* cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_*feature_counter;
// 对一个面片进行计算
for (int y = 0; y < patch_size_; ++y)
{
uint8_t* ref_img_ptr = (uint8_t*)ref_img.data + (v_ref_i + y - patch_halfsize_)*stride + (u_ref_i - patch_halfsize_);
for (int x = 0; x < patch_size_; ++x, ++ref_img_ptr, ++cache_ptr, ++pixel_counter)
{
// 通过插值计算每个特征面片灰度值，主要进行测试查看
*cache_ptr = w_ref_tl*ref_img_ptr[0] + w_ref_tr*ref_img_ptr[1] + w_ref_bl*ref_img_ptr[stride] + w_ref_br*ref_img_ptr[stride + 1];

// 采用逆向组合算法(inverse compositional): 通过采取梯度总是在相同位置这一性质
// 得到warped的图像 
float dx = 0.5f * ((w_ref_tl*ref_img_ptr[1] + w_ref_tr*ref_img_ptr[2] + w_ref_bl*ref_img_ptr[stride + 1] + w_ref_br*ref_img_ptr[stride + 2])
- (w_ref_tl*ref_img_ptr[-1] + w_ref_tr*ref_img_ptr[0] + w_ref_bl*ref_img_ptr[stride - 1] + w_ref_br*ref_img_ptr[stride]));
float dy = 0.5f * ((w_ref_tl*ref_img_ptr[stride] + w_ref_tr*ref_img_ptr[1 + stride] + w_ref_bl*ref_img_ptr[stride * 2] + w_ref_br*ref_img_ptr[stride * 2 + 1])
- (w_ref_tl*ref_img_ptr[-stride] + w_ref_tr*ref_img_ptr[1 - stride] + w_ref_bl*ref_img_ptr[0] + w_ref_br*ref_img_ptr[1]));

// cache the jacobian
jacobian_cache_.col(feature_counter*patch_area_ + pixel_counter) =
(dx*frame_jac.row(0) + dy*frame_jac.row(1))*(focal_length / (1 << level_));
}
}
}
have_ref_patch_cache_ = true;
}
{% endcodeblock %}

接着就是计算残差：
{% codeblock %}
double SparseImgAlign::computeResiduals(
const SE3& T_cur_from_ref,
bool linearize_system,
bool compute_weight_scale)
{
// 对当前图像进行Warp处理以对应参考图像
const cv::Mat& cur_img = cur_frame_->img_pyr_.at(level_);

if (linearize_system && display_)
resimg_ = cv::Mat(cur_img.size(), CV_32F, cv::Scalar(0));

if (have_ref_patch_cache_ == false)
precomputeReferencePatches();

std::vector<float> errors;
if (compute_weight_scale)
errors.reserve(visible_fts_.size());
const int stride = cur_img.cols;
const int border = patch_halfsize_ + 1;
const float scale = 1.0f / (1 << level_);
const Vector3d ref_pos(ref_frame_->pos());
float chi2 = 0.0;
size_t feature_counter = 0; // 计算cached jacobian的索引，对应每个特征
std::vector<bool>::iterator visiblity_it = visible_fts_.begin();
for (auto it = ref_frame_->fts_.begin(); it != ref_frame_->fts_.end();
++it, ++feature_counter, ++visiblity_it)
{
// 检测特征在图像中是否可见
if (!*visiblity_it)
continue;

// 计算在当前图像中投影的像素位置
const double depth = ((*it)->point->pos_ - ref_pos).norm();
const Vector3d xyz_ref((*it)->f*depth);//避免了重投影的误差
const Vector3d xyz_cur(T_cur_from_ref * xyz_ref);
const Vector2f uv_cur_pyr(cur_frame_->cam_->world2cam(xyz_cur).cast<float>() * scale);// 计算投影到当前帧的像素
const float u_cur = uv_cur_pyr[0];
const float v_cur = uv_cur_pyr[1];
const int u_cur_i = floorf(u_cur);
const int v_cur_i = floorf(v_cur);

// 检测投影值是否在图像中
if (u_cur_i < 0 || v_cur_i < 0 || u_cur_i - border < 0 || v_cur_i - border < 0 || u_cur_i + border >= cur_img.cols || v_cur_i + border >= cur_img.rows)
continue;

// 对当前图像进行双边插值加权
const float subpix_u_cur = u_cur - u_cur_i;
const float subpix_v_cur = v_cur - v_cur_i;
const float w_cur_tl = (1.0 - subpix_u_cur) * (1.0 - subpix_v_cur);
const float w_cur_tr = subpix_u_cur * (1.0 - subpix_v_cur);
const float w_cur_bl = (1.0 - subpix_u_cur) * subpix_v_cur;
const float w_cur_br = subpix_u_cur * subpix_v_cur;
float* ref_patch_cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_*feature_counter;
size_t pixel_counter = 0; // 用于计算cached jacobian的索引，每个特征对应的像素索引
for (int y = 0; y < patch_size_; ++y)
{
uint8_t* cur_img_ptr = (uint8_t*)cur_img.data + (v_cur_i + y - patch_halfsize_)*stride + (u_cur_i - patch_halfsize_);

for (int x = 0; x < patch_size_; ++x, ++pixel_counter, ++cur_img_ptr, ++ref_patch_cache_ptr)
{
// 计算残差
const float intensity_cur = w_cur_tl*cur_img_ptr[0] + w_cur_tr*cur_img_ptr[1] + w_cur_bl*cur_img_ptr[stride] + w_cur_br*cur_img_ptr[stride + 1];
const float res = intensity_cur - (*ref_patch_cache_ptr);

// 用于计算scale用于robust cost
if (compute_weight_scale)
errors.push_back(fabsf(res));

// 给出权重
float weight = 1.0;
if (use_weights_) {
weight = weight_function_->value(res / scale_);
}

chi2 += res*res*weight;
n_meas_++;

if (linearize_system)
{
// 计算Jacobian, 带权重的Hessian 和残差图像
const Vector6d J(jacobian_cache_.col(feature_counter*patch_area_ + pixel_counter));
H_.noalias() += J*J.transpose()*weight;
Jres_.noalias() -= J*res*weight;
if (display_)
resimg_.at<float>((int)v_cur + y - patch_halfsize_, (int)u_cur + x - patch_halfsize_) = res / 255.0;
}
}
}
}

// 在第一次迭代时计算权重
if (compute_weight_scale && iter_ == 0)
scale_ = scale_estimator_->compute(errors);

return chi2 / n_meas_;
}
{% endcodeblock %}

求解过程也就是计算H*x ==J，H是Hessian的近似，只取一次项展开的；J是对应的Jacobi矩阵。
{% codeblock %}
int SparseImgAlign::solve()
{
x_ = H_.ldlt().solve(Jres_);
if ((bool)std::isnan((double)x_[0]))
return 0;
return 1;
}
{% endcodeblock %}

对于更新：
{% codeblock %}
void SparseImgAlign::update(
const ModelType& T_curold_from_ref,
ModelType& T_curnew_from_ref)
{
T_curnew_from_ref = T_curold_from_ref * SE3::exp(-x_);
}
{% endcodeblock %}


    注意求雅克比矩阵的时候，对Rt无法求导数，改成李代数的形式，这边用SE3::exp

总结
---
这一部分还有很多地方不是很理解，基本的思路是通过最小光度误差来计算两帧之间的相对偏转量。最小光度误差方程也转换为非线性最小二乘，具体求解即通过高斯牛顿法。</br>
总的来说也就是根据初始化形成的3d点，投影到下一帧，根据下一帧投影点形成匹配关系，通过计算最小光度误差计算两帧相对偏移量，得到姿态之后一并可以对深度进行估计，不过这边的深度估计有点难，通过多个三角化形成的3d点进行分析，通过逆深度进行分析，具体还不是很清楚，可能是逆深度符合高斯分布，不过svo这边是假设了内点符合高斯分布，外点符合均匀分布，后续再理解，先提一提。</br>
后期会对这一部分进一步深入。</br>
最后使用测试程序给出计算结果：
{% codeblock %}
int main(int argc, char *argv[])
{
CmdLine cmd;
std::string first_frame_name;
std::string second_frame_name;
std::string third_frame_name;

cmd.add(make_option('f', first_frame_name, "firstname"));
cmd.add(make_option('s', second_frame_name, "secondname"));
cmd.add(make_option('t', third_frame_name, "thirdname"));
try {
if (argc == 1) throw std::string("Invalid command line parameter.");
cmd.process(argc, argv);
}
catch (const std::string& s) {
std::cerr << "Feature detector \nUsage: " << argv[0] << "\n"
<< "[-f|--firstname name]\n"
<< "[-s|--secondname name]\n"
<< "[-t|--thirdname name]\n"
<< std::endl;

std::cerr << s << std::endl;
return EXIT_FAILURE;
}
cv::Mat first_img(cv::imread(first_frame_name, 0));
cv::Mat second_img(cv::imread(second_frame_name, 0));
cv::Mat third_img(cv::imread(third_frame_name, 0));
assert(first_img.type() == CV_8UC1 && !first_img.empty());
assert(second_img.type() == CV_8UC1 && !second_img.empty());
assert(third_img.type() == CV_8UC1 && !third_img.empty());

AbstractCamera* cam = new PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);

FramePtr fisrt_frame(new Frame(cam, first_img, 0.0));
FramePtr second_frame(new Frame(cam, second_img, 1.0));
FramePtr third_frame(new Frame(cam, third_img, 1.0));

Initialization init;
init.addFirstFrame(fisrt_frame);
init.addSecondFrame(second_frame);

SparseImgAlign img_align(4, 1,
30, SparseImgAlign::GaussNewton, false, false);
size_t img_align_n_tracked = img_align.run(second_frame, third_frame);
std::cout << "Img Align:\t Tracked = " << img_align_n_tracked << std::endl;
std::cout << "first pose:" << fisrt_frame->T_f_w_ << '\n'
<< "second pose:" << second_frame->T_f_w_ << '\n'
<< "third pose:"<< third_frame->T_f_w_ << std::endl;
getchar();
return 0;
}
{% endcodeblock %}


后期研习：lucas-kanade 20 years on a unifying framework这个系列

具体结果如下：

![](http://7xl6tk.com1.z0.glb.clouddn.com/image_aline.png )





文章转载自冯兵的博客，[原文链接][link]

[link]:http://fengbing.net/2015/08/29/%E4%B8%80%E6%AD%A5%E6%AD%A5%E5%AE%9E%E7%8E%B0%E5%8D%95%E7%9B%AE%E8%A7%86%E8%A7%89%E9%87%8C%E7%A8%8B%E8%AE%A14%E2%80%94%E2%80%94%E8%BF%90%E5%8A%A8%E4%BC%B0%E8%AE%A1image%20align/



