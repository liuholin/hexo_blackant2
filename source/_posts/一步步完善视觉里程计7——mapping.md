---
title: 一步步完善视觉里程计7——mapping
date: 2017-09-18 23:44:38
tags:
---
前面运动估计部分就已经完成了，虽然步骤很多但是总的目的是为了局部姿态估计的准确性。接下来我们要做的操作就是mapping，给mapping单独开出一个线程，主要的目的是根据给出的图像和其姿态I<sub>k</sub> , T<sub>k,w</sub>,用于估计那些不知道对应3D点的二维特征的深度信息。<!--more-->

seed
---
首先构建种子点结构用于存储没有获得深度信息的特征，对其进行深度估计，具体设计如下：
{% codeblock %}
struct Seed
{
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

static int batch_counter;    //!< 用于设置种子点对应帧的数目
static int seed_counter;     //!< 用于设置种子点的唯一id
int batch_id;                //!< batch_id是种子点被创建所对应的关键帧的id
int id;                      //!< 种子ID,仅用来可视化显示
Feature* ftr;                //!< 在关键帧上的特征，这些特征的深度需要被计算
float a;                     //!< Beta分布的参数a: a越高，则内点的概率就越大
float b;                     //!< Beta分布的参数b: b越高，则外点的概率就越大
float mu;                    //!< 正态分布的均值
float z_range;               //!< 可能深度的最大范围
float sigma2;                //!< 正态分布的方差
Seed(Feature* ftr, float depth_mean, float depth_min);
};
{% endcodeblock %}

种子点的结构定义了，接下来就是初始化种子点，对于种子点只是在添加关键帧的时候才会初始化，具体初始化如下：
{% codeblock %}
void DepthFilter::initializeSeeds(FramePtr frame)
{
Features new_features;
// 将帧划分为格子，确定通过光流跟踪确定的特征是否在格子内，在，标识格子被占用
// 以减小下一步特征检测的计算量
feature_detector_->setExistingFeatures(frame->fts_);
feature_detector_->detect(frame.get(), frame->img_pyr_,
Config::triangMinCornerScore(), new_features);

//对每个特征初始化一个新种子点
seeds_updating_halt_ = true;
lock_t lock(seeds_mut_); // 确保独立执行，updateSeeds函数暂停
++Seed::batch_counter;
std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
seeds_.push_back(Seed(ftr, new_keyframe_mean_depth_, new_keyframe_min_depth_));
});

seeds_updating_halt_ = false;// 表明种子已经添加完成
}
{% endcodeblock %}

对种子点进行更新，更新的过程有不少地方还没有看懂。先把代码贴过来，对其中部分做一个讲述。
{% codeblock %}
void DepthFilter::updateSeeds(FramePtr frame)
{
// 更新有限数目的种子点，因为我们没有时间对每帧中的所有种子点进行处理
size_t n_updates = 0, n_failed_matches = 0, n_seeds = seeds_.size();
lock_t lock(seeds_mut_);
std::list<Seed>::iterator it = seeds_.begin();

const double focal_length = frame->cam_->getFocalLength();// 得到焦距
double px_noise = 1.0;//设置像素误差为1个像素
double px_error_angle = atan(px_noise / (2.0*focal_length))*2.0; // 计算误差带来的角度变化误差

while (it != seeds_.end())
{
// 表明种子点在添加
if (seeds_updating_halt_)
return;

// 确保当前种子点对应的帧数不太大，以保证种子点较新
if ((Seed::batch_counter - it->batch_id) > options_.max_n_kfs) {
it = seeds_.erase(it);
continue;
}

// 检测点是否在当前图像中可见
SE3 T_ref_cur = it->ftr->frame->T_f_w_ * frame->T_f_w_.inverse();// 参考帧向当前帧的变换（姿态变换右乘）
const Vector3d xyz_f(T_ref_cur.inverse()*(1.0 / it->mu * it->ftr->f));//1.0 / it->mu 也就是深度的均值
if (xyz_f.z() < 0.0)  {
++it; // 在相机后面
continue;
}
if (!frame->cam_->isInFrame(frame->f2c(xyz_f).cast<int>())) {
++it; // 点没有投影到相机中
continue;
}

// 使用逆的深度坐标，这边为什么用逆呢
float z_inv_min = it->mu + sqrt(it->sigma2);
float z_inv_max = std::max(it->mu - sqrt(it->sigma2), 0.00000001f);
double z;
if (!matcher_.findEpipolarMatchDirect(
*it->ftr->frame, *frame, *it->ftr, 1.0 / it->mu, 1.0 / z_inv_min, 1.0 / z_inv_max, z))
{
it->b++; // 如果没有发现匹配，则增加外点的概率
++it;
++n_failed_matches;
continue;
}

// 计算测量的不确定性
double tau = computeTau(T_ref_cur, it->ftr->f, z, px_error_angle);
double tau_inverse = 0.5 * (1.0 / std::max(0.0000001, z - tau) - 1.0 / (z + tau));

// 更新种子点
updateSeed(1. / z, tau_inverse*tau_inverse, &*it);
++n_updates;

if (frame->isKeyframe())
{
feature_detector_->setGridOccupancy(matcher_.px_cur_);
}
///？？？
if (sqrt(it->sigma2) < it->z_range / options_.seed_convergence_sigma2_thresh)
{
assert(it->ftr->point == NULL); // TODO this should not happen anymore
Vector3d xyz_world(it->ftr->frame->T_f_w_.inverse() * (it->ftr->f * (1.0 / it->mu)));
Point3D* point = new Point3D(xyz_world);
point->obs_.push_front(it->ftr);
it->ftr->point = point;			
seed_converged_cb_(point, it->sigma2); // 添加到候选list			
it = seeds_.erase(it);
}
else if (isnan(z_inv_min))
{
it = seeds_.erase(it);
}
else
++it;
}
}
{% endcodeblock %}

目前对于计算不确定性，可以参考如下图：

![](http://7xl6tk.com1.z0.glb.clouddn.com/compute_tau.png )

更详细的过程如下，可参考：

![](http://7xl6tk.com1.z0.glb.clouddn.com/tau_process1.png )
![](http://7xl6tk.com1.z0.glb.clouddn.com/tau_process2.png )

代码如下：
{% codeblock %}
/// 计算测量的不确定性
double DepthFilter::computeTau(
const SE3& T_ref_cur,
const Vector3d& f,
const double z,
const double px_error_angle)
{
Vector3d t(T_ref_cur.translation());
Vector3d a = f*z - t;
double t_norm = t.norm();
double a_norm = a.norm();
double alpha = acos(f.dot(t) / t_norm); // 点乘计算角度
double beta = acos(a.dot(-t) / (t_norm*a_norm)); // 点乘计算角度
double beta_plus = beta + px_error_angle;
double gamma_plus = M_PI - alpha - beta_plus; // 三角之和180
double z_plus = t_norm*sin(beta_plus) / sin(gamma_plus); // 正玄定理
return (z_plus - z); // tau
}
{% endcodeblock %}

对于其它地方后续继续研究，根据作者给出测试程序，参考如下：
{% codeblock %}
void DepthFilterTest::testReconstruction(
std::string dataset_dir,
std::string experiment_name)
{
//读取图像名和姿态
std::string file_name = dataset_dir + "/trajectory.txt";
mvo::FileReader<mvo::blender_utils::file_format::ImageNameAndPose> sequence_file_reader(file_name);
std::vector<mvo::blender_utils::file_format::ImageNameAndPose> sequence;
sequence.reserve(10000);
sequence_file_reader.skipComments();
if (!sequence_file_reader.next())
std::runtime_error("Failed to open sequence file");
sequence_file_reader.readAllEntries(sequence);
std::cout << "RUN EXPERIMENT: read " << sequence.size() << " dataset entries." << std::endl;

// 构建depth filter
DetectorPtr feature_detector(new mvo::FastDetector(
cam_->width(), cam_->height(), mvo::Config::gridSize(), mvo::Config::nPyrLevels()));
DepthFilter::callback_t depth_filter_cb = std::bind(&DepthFilterTest::depthFilterCB, this, std::placeholders::_1, std::placeholders::_2);
depth_filter_ = new mvo::DepthFilter(feature_detector, depth_filter_cb);
depth_filter_->options_.verbose = true;

std::vector<mvo::blender_utils::file_format::ImageNameAndPose>::iterator it = sequence.begin();

std::list<size_t> n_converged_per_iteration;
// 遍历获取图像
for (int i = 0; it != sequence.end() && i < 20; ++it, ++i)
{
std::string img_name(dataset_dir + "/img/" + (*it).image_name_ + "_0.png");
printf("reading image: '%s'\n", img_name.c_str());
cv::Mat img(cv::imread(img_name, 0));
assert(!img.empty());

Sophus::SE3 T_w_f(it->q_, it->t_);
if (i == 0)
{
// 创建参考帧导入真实地图信息
frame_ref_ = std::shared_ptr<mvo::Frame>(new mvo::Frame(cam_, img, 0.0));
frame_ref_->T_f_w_ = T_w_f.inverse();//姿态变Rt
depth_filter_->addKeyframe(frame_ref_, 2, 0.5);
mvo::blender_utils::loadBlenderDepthmap(dataset_dir + "/depth/" + (*it).image_name_ + "_0.depth", *cam_, depth_ref_);
continue;
}

n_converged_seeds_ = 0;
frame_cur_ = std::shared_ptr<mvo::Frame>(new mvo::Frame(cam_, img, 0.0));
frame_cur_->T_f_w_ = T_w_f.inverse();
depth_filter_->addFrame(frame_cur_);
n_converged_per_iteration.push_back(n_converged_seeds_);
}
//printf("Experiment '%s' took %f ms\n", experiment_name.c_str(), t.stop() * 1000);

// compute mean, median and variance of error in converged area
{
printf("# converged:  \t %zu (ref: 287)\n", errors_.size());
double sum_error = 0;
std::for_each(errors_.begin(), errors_.end(), [&](double& e){sum_error += e; });
printf("mean error:   \t %f cm (ref: 0.080357)\n", sum_error * 100 / errors_.size());
std::vector<double>::iterator it = errors_.begin() + 0.5*errors_.size();
std::nth_element(errors_.begin(), it, errors_.end());
printf("50-percentile: \t %f cm (ref: 0.062042)\n", *it * 100);
it = errors_.begin() + 0.8*errors_.size();
std::nth_element(errors_.begin(), it, errors_.end());
printf("80-percentile: \t %f cm (ref: 0.124526)\n", *it * 100);
it = errors_.begin() + 0.95*errors_.size();
std::nth_element(errors_.begin(), it, errors_.end());
printf("95-percentile: \t %f cm (ref: 0.200417)\n", *it * 100);
}

// trace error
std::string trace_name("./depth_filter_" + experiment_name + ".txt");
std::ofstream ofs(trace_name.c_str());
for (std::list<ConvergedSeed>::iterator i = results_.begin(); i != results_.end(); ++i)
ofs << i->x_ << ", " << i->y_ << ", " << fabs(i->error_) << std::endl;
ofs.close();

// trace convergence rate
trace_name = "./depth_filter_" + experiment_name + "_convergence.txt";
ofs.open(trace_name.c_str());
for (std::list<size_t>::iterator it = n_converged_per_iteration.begin();
it != n_converged_per_iteration.end(); ++it)
ofs << *it << std::endl;
ofs.close();

// write ply file for pointcloud visualization in Meshlab
trace_name = "./depth_filter_" + experiment_name + ".ply";
ofs.open(trace_name.c_str());
ofs << "ply" << std::endl
<< "format ascii 1.0" << std::endl
<< "element vertex " << results_.size() << std::endl
<< "property float x" << std::endl
<< "property float y" << std::endl
<< "property float z" << std::endl
<< "property uchar blue" << std::endl
<< "property uchar green" << std::endl
<< "property uchar red" << std::endl
<< "end_header" << std::endl;

for (std::list<ConvergedSeed>::iterator i = results_.begin(); i != results_.end(); ++i)
{
//cv::Vec3b c = frame_ref_->img_pyr_[0].at<cv::Vec3b>(i->y_, i->x_);
Eigen::Vector3d p = cam_->cam2world(i->x_, i->y_)*i->depth_;
ofs << p[0] << " " << p[1] << " " << p[2] << " "
//<< (int)c[0] << " " << (int)c[1] << " " << (int)c[2] << std::endl;
<< "255 255 255" << std::endl;
}
}
{% endcodeblock %}

最终可得相关结果

![](http://7xl6tk.com1.z0.glb.clouddn.com/depth_filter_result.png )

总结
---
整个程序到这里，让人看起来是越来越复杂了，目前只能写部分能看懂的，部分剩下的没有明白的后续继续研究。



转载自冯兵的博客，[原文链接][link]

[link]:http://fengbing.net/2015/09/26/%E4%B8%80%E6%AD%A5%E6%AD%A5%E5%AE%9E%E7%8E%B0%E5%8D%95%E7%9B%AE%E8%A7%86%E8%A7%89%E9%87%8C%E7%A8%8B%E8%AE%A17%E2%80%94%E2%80%94mapping/


