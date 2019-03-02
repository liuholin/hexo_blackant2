---
title: 一步步完善视觉里程计6——运动估计Pose and Structure Refinement
date: 2017-09-18 23:38:20
tags:
---
**前面进行了feature align，也就是不仅考虑了相连的两帧，而且考虑了前面有相同视角的N个关键帧。通过前面的关键帧确定的特征及三维点，投影到当前帧中，来对当前帧中的特征进行优化。但是这一步发现，只是在图像层面进行了优化，而没有考虑级线约束等。</br>
这就通过最小化投影误差来优化运动和结构,具体如下图：**<!--more-->

![](http://7xl6tk.com1.z0.glb.clouddn.com/pose_and_structure.png )

pose optimize
---
首先考虑单帧，对帧的运动即图Tw,kTw,k进行优化。</br>
根据前面我们大概清楚优化主要通过计算残差，雅克比矩阵以及hessian矩阵。</br>
**数学基础有点薄弱，补习中。。。</br>
计算权重，这边计算权重的方式是计算所有误差的绝对中值作为scale，通过Redescending M进行估计，至于为什么这么做就不懂了**</br>
通过重投影计算残差得到残量函数，该函数对TT进行求解雅克比矩阵，具体如下：

{% codeblock %}
inline static void jacobian_xyz2uv(
const Vector3d& xyz_in_f,
Matrix<double, 2, 6>& J)
{
const double x = xyz_in_f[0];
const double y = xyz_in_f[1];
const double z_inv = 1. / xyz_in_f[2];
const double z_inv_2 = z_inv*z_inv;

J(0, 0) = -z_inv;              // -1/z
J(0, 1) = 0.0;                 // 0
J(0, 2) = x*z_inv_2;           // x/z^2
J(0, 3) = y*J(0, 2);            // x*y/z^2
J(0, 4) = -(1.0 + x*J(0, 2));   // -(1.0 + x^2/z^2)
J(0, 5) = y*z_inv;             // y/z

J(1, 0) = 0.0;                 // 0
J(1, 1) = -z_inv;              // -1/z
J(1, 2) = y*z_inv_2;           // y/z^2
J(1, 3) = 1.0 + y*J(1, 2);      // 1.0 + y^2/z^2
J(1, 4) = -J(0, 3);             // -x*y/z^2
J(1, 5) = -x*z_inv;            // x/z
}
{% endcodeblock %}

注意这边计算的是到单位平面，也就是相机焦距fx和fy为1.接着计算雅克比矩阵和Hessian矩阵，然后求解，具体如下：
{% codeblock %}
// 计算残差
for (auto it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
{
if ((*it)->point == NULL)
continue;
Matrix26d J;
Vector3d xyz_f(frame->T_f_w_ * (*it)->point->pos_);
Frame::jacobian_xyz2uv(xyz_f, J);
Vector2d e = project2d((*it)->f) - project2d(xyz_f);
double sqrt_inv_cov = 1.0 / (1 << (*it)->level);
e *= sqrt_inv_cov;
if (iter == 0)
chi2_vec_init.push_back(e.squaredNorm()); // 主要用于调试，看结果
J *= sqrt_inv_cov;
double weight = weight_function.value(e.norm() / scale);
A.noalias() += J.transpose()*J*weight;
b.noalias() -= J.transpose()*e*weight;//这边为负号，是因为雅克比矩阵已经添加负号了
new_chi2 += e.squaredNorm()*weight;
}

//求解线性方程
const Vector6d dT(A.ldlt().solve(b));
{% endcodeblock %}

求解完成之后就进行更新，不过更新之前先确定误差有没有增加，具体如下:
{% codeblock %}
// 检测误差是否增加
if ((iter > 0 && new_chi2 > chi2) || (bool)std::isnan((double)dT[0]))
{
frame->T_f_w_ = T_old; // 如果误差增加则回滚
break;
}

// 更新模型
SE3 T_new = SE3::exp(dT)*frame->T_f_w_;
{% endcodeblock %}

**这边模型的更新方式再理解**</br>
最后移除投影误差较大的点以减轻结构优化的负担。
{% codeblock %}
// 移除残差较大的投影的测量
double reproj_thresh_scaled = reproj_thresh / frame->cam_->getFocalLength();
size_t n_deleted_refs = 0;
for (Features::iterator it = frame->fts_.begin(); it != frame->fts_.end(); ++it)
{
if ((*it)->point == NULL)
continue;
Vector2d e = project2d((*it)->f) - project2d(frame->T_f_w_ * (*it)->point->pos_);
double sqrt_inv_cov = 1.0 / (1 << (*it)->level);
e *= sqrt_inv_cov;
chi2_vec_final.push_back(e.squaredNorm());
if (e.norm() > reproj_thresh_scaled)
{
// 我们不需要删除这个指针，因为还没有创建
(*it)->point = NULL;
++n_deleted_refs;
}
}
{% endcodeblock %}

这边要注意的是一个特征对应的3D点，目前由于还没有考虑depth filter，3D点的初始化时再depth filter中实现。

structure optimize
---

上面对运行进行了优化，这对结构进行优化，也就是对3D点进行优化。</br>
这里也就是残量函数对3D点求解雅克比矩阵，具体如下：
{% codeblock %}
void Point3D::optimize(const size_t n_iter)
{
Vector3d old_point = pos_;
double chi2 = 0.0;
Matrix3d A;
Vector3d b;

for (size_t i = 0; i < n_iter; i++)
{
A.setZero();
b.setZero();
double new_chi2 = 0.0;

// 计算残差
for (auto it = obs_.begin(); it != obs_.end(); ++it)
{
Matrix23d J;
const Vector3d p_in_f((*it)->frame->T_f_w_ * pos_);
Point3D::jacobian_xyz2uv(p_in_f, (*it)->frame->T_f_w_.rotation_matrix(), J);
const Vector2d e(project2d((*it)->f) - project2d(p_in_f));
new_chi2 += e.squaredNorm();
A.noalias() += J.transpose() * J;
b.noalias() -= J.transpose() * e;
}

// 求解线性系统
const Vector3d dp(A.ldlt().solve(b));

// 检测误差有没有增长
if ((i > 0 && new_chi2 > chi2) || (bool)std::isnan((double)dp[0]))
{
pos_ = old_point; // 回滚
break;
}

// 更新模型
Vector3d new_point = pos_ + dp;
old_point = pos_;
pos_ = new_point;
chi2 = new_chi2;

// 收敛则停止
if (norm_max(dp) <= EPS)
break;
}

}
{% endcodeblock %}

下面就对整个运动估计做一个汇总，写出如下测试程序：
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

third_frame->T_f_w_ = second_frame->T_f_w_;//将上一帧的初值赋给这一帧，便于优化

SparseImgAlign img_align(4, 1,
30, SparseImgAlign::GaussNewton, false, false);
size_t img_align_n_tracked = img_align.run(second_frame, third_frame);
std::cout << "Img Align:\t Tracked = " << img_align_n_tracked << std::endl;
mvo::Map map;
fisrt_frame->setKeyframe();
second_frame->setKeyframe();
map.addKeyframe(fisrt_frame);
map.addKeyframe(second_frame);
Reprojector reprojector(cam, map);
std::vector< std::pair<FramePtr, size_t> > overlap_kfs;
reprojector.reprojectMap(third_frame, overlap_kfs);
const size_t repr_n_new_references = reprojector.n_matches_;
const size_t repr_n_mps = reprojector.n_trials_;
std::cout << "Reprojection:\t Points = " << repr_n_mps << "\t \t Matches = " << repr_n_new_references << std::endl;

size_t sfba_n_edges_final;
double sfba_thresh, sfba_error_init, sfba_error_final;
std::cout << "pose:" << third_frame->T_f_w_;
poseOptimize(2.0, 10, false,third_frame, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
std::cout << "PoseOptimizer:pose" << third_frame->T_f_w_;
std::cout << "point:" << third_frame->fts_.front()->point->pos_<<std::endl;
structureOptimize(third_frame, 20, 5);
std::cout << "structureOptimize:point:" << third_frame->fts_.front()->point->pos_;
getchar();
return 0;
}
{% endcodeblock %}

具体结果如下：

![](http://7xl6tk.com1.z0.glb.clouddn.com/pose_structure.png )

总结
---

到目前整个运动估计的部分就差不多结束了，整个过程看起来有点复杂，主要的目的是为了不是直接通过光流跟踪，计算基础矩阵或者单应矩阵分解的方式来估计姿态，主要是考虑了时间成本。目前具体的时间还没有测试，整个参数设置也没有实验，后续补充。

转载自冯兵的博客，[原文链接][link]

[link]:http://fengbing.net/2015/09/12/%E4%B8%80%E6%AD%A5%E6%AD%A5%E5%AE%9E%E7%8E%B0%E5%8D%95%E7%9B%AE%E8%A7%86%E8%A7%89%E9%87%8C%E7%A8%8B%E8%AE%A16%E2%80%94%E2%80%94%E8%BF%90%E5%8A%A8%E4%BC%B0%E8%AE%A1pose%20and%20structure%20refinement/
