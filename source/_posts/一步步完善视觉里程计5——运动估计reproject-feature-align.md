---
title: 一步步完善视觉里程计5——运动估计reproject,feature align
date: 2017-09-18 22:29:14
tags:
---
**通过前一篇博客我们目前可以计算出3d点和相机pose，但是我们根据之前阐述知道通过两帧的计算结果是不精确，会出现漂移的现象。这一篇文章考虑有共同视角的多个关键帧，具体示意图如下：**<!--more-->

![](http://7xl6tk.com1.z0.glb.clouddn.com/feature_align.png )

也就是不仅仅考虑相连的两帧的，而是考虑前面所有有相同视角的关键帧，通过多帧来进行约束。

reproject
---

我们很容易想到是通过前面计算的3D点的投影到当前帧，看是否可以投影来判断，这个时候我们就要考虑3D点的存储。对于SLAM问题我们考虑构建map，在map中存储3D点和关键帧，提供根据当前帧寻找有共同视野的关键帧，具体定义如下：
{% codeblock %}
class Map : public Noncopyable
{
public:
Map();
~Map();

/// 得到跟目前帧有重叠视野的所有关键帧
void getCloseKeyframes(const FramePtr& frame, std::list< std::pair<FramePtr, double> >& close_kfs) const;

public:
std::list< FramePtr > keyframes_;        //!< 地图中存储的所有关键帧
std::list< Point3D* >   points_;         //!< 存放3D点
};
{% endcodeblock %}

获得有重叠视野的关键帧，就根据当前帧的特征对应的3D点，是否能投影到其它关键帧中，可以则这个关键帧与当前帧有相关视野，但是我们并不希望计算所有的特征，我们在关键帧上选取5个特征作为帧上的关键特征。</br>
在Frame类中添加关键特征，如下：
{% codeblock %}
std::vector<Feature*> key_pts_; //!<使用5个特征，用于检测两帧之间是否有重叠的视野
{% endcodeblock %}

这些点用于快速检测是否两个帧有重叠的视野，选取5个特征，一个在图像中点另外4个靠近图像的4个边角，并且这5个特征都要有对应的3D点,具体如下：
{% codeblock %}
void Frame::setKeyPoints()
{
// 如果特征指向的3d点为空，则设置该特征为NULL
for (size_t i = 0; i < 5; ++i)
if (key_pts_[i] != NULL)
if (key_pts_[i]->point == NULL)
key_pts_[i] = NULL;
// 找到5个特征
std::for_each(fts_.begin(), fts_.end(), [&](Feature* ftr){ if (ftr->point != NULL) checkKeyPoints(ftr); });
}
{% endcodeblock %}

具体5个关键特征点如何选择，如下：
{% codeblock %}
void Frame::checkKeyPoints(Feature* ftr)
{
// 首先得到相机中心点
const int cu = cam_->width() / 2;
const int cv = cam_->height() / 2;

// 如果第一个特征为空，则进入的第一个特征转为关键特征，如果不为空，则判断新进入的特征是否比之前特征
// 更接近中心，是，则替换该特征
if (key_pts_[0] == NULL)
key_pts_[0] = ftr;
else if (std::max(std::fabs(ftr->px[0] - cu), std::fabs(ftr->px[1] - cv))
< std::max(std::fabs(key_pts_[0]->px[0] - cu), std::fabs(key_pts_[0]->px[1] - cv)))
key_pts_[0] = ftr;
// 找到中间的特征之后，将图片分成4块，在每块中找出1个特征，离中心越远的特征
if (ftr->px[0] >= cu && ftr->px[1] >= cv)
{
if (key_pts_[1] == NULL)
key_pts_[1] = ftr;
else if ((ftr->px[0] - cu) * (ftr->px[1] - cv)
> (key_pts_[1]->px[0] - cu) * (key_pts_[1]->px[1] - cv))
key_pts_[1] = ftr;
}
if (ftr->px[0] >= cu && ftr->px[1] < cv)
{
if (key_pts_[2] == NULL)
key_pts_[2] = ftr;
else if ((ftr->px[0] - cu) * (ftr->px[1] - cv)
> (key_pts_[2]->px[0] - cu) * (key_pts_[2]->px[1] - cv))
key_pts_[2] = ftr;
}
if (ftr->px[0] < cu && ftr->px[1] < cv)
{
if (key_pts_[3] == NULL)
key_pts_[3] = ftr;
else if ((ftr->px[0] - cu) * (ftr->px[1] - cv)
> (key_pts_[3]->px[0] - cu) * (key_pts_[3]->px[1] - cv))
key_pts_[3] = ftr;
}
if (ftr->px[0] < cu && ftr->px[1] >= cv)
{
if (key_pts_[4] == NULL)
key_pts_[4] = ftr;
else if ((ftr->px[0] - cu) * (ftr->px[1] - cv)
> (key_pts_[4]->px[0] - cu) * (key_pts_[4]->px[1] - cv))
key_pts_[4] = ftr;
}
}
{% endcodeblock %}

最后这边要注意的是如果特征点删除了，其可能对于关键特征点，这个关键特征点也必须要移除，因此我们添加如下：
{% codeblock %}
void Frame::removeKeyPoint(Feature* ftr)
{
bool found = false;
std::for_each(key_pts_.begin(), key_pts_.end(), [&](Feature*& i){
if (i == ftr) {
i = NULL;
found = true;
}
});
if (found)
setKeyPoints();
}
{% endcodeblock %}

下面我们在Map类中添加根据当前帧获得具体相关视野的关键帧，具体如下：
{% codeblock %}
/// 获得有重叠视野的关键帧，就根据当前帧的特征对应的3D点，是否能投影到其它关键帧中，可以则
/// 这个关键帧与当前帧有相关视野
void Map::getCloseKeyframes(
const FramePtr& frame,
std::list< std::pair<FramePtr, double> >& close_kfs) const
{
for (auto kf : keyframes_)
{
// 检测当前帧与关键帧之间是否有重叠的视野，通过关键点(特征)来进行计算
for (auto keypoint : kf->key_pts_)
{
if (keypoint == nullptr)
continue;

if (frame->isVisible(keypoint->point->pos_))// 判断目前帧的特征所对应的3d点是否在关键帧中可见
{
close_kfs.push_back(std::make_pair(
kf, (frame->T_f_w_.translation() - kf->T_f_w_.translation()).norm()));
break; // 这个关键帧跟目前帧有重叠的视野，则加入close_kfs
}
}
}
}
{% endcodeblock %}

找到了有共同视野的关键帧之后，我们要对其它有共同视野的关键帧上的特征点对应的3D点投影到当前帧上，但是为了很好的效率，我们不对所有的点进行投影匹配，只在一个单元格子中匹配一个点，这样，我们可以对所有的匹配特征进行均匀分布，而且不用投影所有的点这样可以更好的节约时间。</br>
具体定义一个Grid如下：
{% codeblock %}
class Reprojector
{
/// candidate是一个点投影到图片平面，我们在图像中找到了与该点匹配的特征
struct Candidate {
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
Point* pt;       //!< 3D点
Vector2d px;     //!< 投影的2D像素点
Candidate(Point* pt, Vector2d& px) : pt(pt), px(px) {}
};
typedef std::list<Candidate, aligned_allocator<Candidate> > Cell;
typedef std::vector<Cell*> CandidateGrid;

/// grid用于存储一系列候选匹配.对于每一个grid单元格努力寻找一个匹配
struct Grid
{
CandidateGrid cells;//!< 用于存放3D点和对应的投影2D像素坐标列表
std::vector<int> cell_order;//!< 单元格的顺序编号
int cell_size;//!< 单应格的大小
int grid_n_cols;//!< 图像划分单元格的列数
int grid_n_rows;//!< 图像划分单元格的行数
};

Grid grid_;//!< 图像划分为网格

///  初始化Grid
void initializeGrid(AbstractCamera* cam);
}
{% endcodeblock %}

首先对Grid进行初始化，具体如下：
{% codeblock %}
void Reprojector::initializeGrid(AbstractCamera* cam)
{
grid_.cell_size = Config::gridSize();
grid_.grid_n_cols = ceil(static_cast<double>(cam->width()) / grid_.cell_size);
grid_.grid_n_rows = ceil(static_cast<double>(cam->height()) / grid_.cell_size);
grid_.cells.resize(grid_.grid_n_cols*grid_.grid_n_rows);
// 初始化单元格，在析构中删除
std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell*& c){ c = new Cell; });
grid_.cell_order.resize(grid_.cells.size());
for (size_t i = 0; i < grid_.cells.size(); ++i)
grid_.cell_order[i] = i;
std::random_shuffle(grid_.cell_order.begin(), grid_.cell_order.end()); // 随机排列，一种策略
}
{% endcodeblock %}

这样Grid划分好了之后，那下面就开始重投影，具体是找到与当前帧有相关视野最靠近的N个关键帧 ，这边设置N的最大值为10，靠近的N个关键帧的个数不超过10个。遍历这N个关键帧，对每个关键帧观察到的点投影到当前帧中，记录这每个关键帧与当前帧共同的观察点的个数。具体如下：
{% codeblock %}
void Reprojector::reprojectMap(
FramePtr frame,
std::vector< std::pair<FramePtr, std::size_t> >& overlap_kfs)
{
resetGrid();

// 选出与目前帧有重叠视野的关键帧
std::list< std::pair<FramePtr, double> > close_kfs;
map_.getCloseKeyframes(frame, close_kfs);

// 对靠近的关键帧根据靠近程度进行排序
close_kfs.sort(compareDistance);

// 对有有重叠部分的N个关键帧对应的mappoints进行重投影，我们只存储格子中特征点减少的
size_t n = 0;
overlap_kfs.reserve(options_.max_n_kfs);
// 对最近的N个关键帧进行迭代，找到有重叠视野
for (auto it_frame = close_kfs.begin(), ite_frame = close_kfs.end();
it_frame != ite_frame && n < options_.max_n_kfs; ++it_frame, ++n)
{
FramePtr ref_frame = it_frame->first;
overlap_kfs.push_back(std::pair<FramePtr, size_t>(ref_frame, 0));

// 对这个参考帧观察到的点投影到当前帧中
for (auto it_ftr = ref_frame->fts_.begin(), ite_ftr = ref_frame->fts_.end();
it_ftr != ite_ftr; ++it_ftr)
{
// 检测这个特征是否有分配的mappoint
if ((*it_ftr)->point == NULL)
continue;

// 确保我们只投影一次，不同帧上的特征会对应同一个3D点
if ((*it_ftr)->point->last_projected_kf_id_ == frame->id_)
continue;
(*it_ftr)->point->last_projected_kf_id_ = frame->id_;
if (reprojectPoint(frame, (*it_ftr)->point))
overlap_kfs.back().second++;//相同观察点的数目
}
}
}
{% endcodeblock %}

feature align
---

根据下图，我们知道特征的投影会存在偏差，这样我们就需要进行feature align

![](http://7xl6tk.com1.z0.glb.clouddn.com/feature_align.png)

在上一个过程中，将图像划分为了格子，则在这个一个格子的同一个特征可能会存在偏差，即同一个格子中会出现多个近似真实帧的特征及其对应3D点。feature align的工作即对同一个格子中的相似特征进行优化。</br>
具体做法：通过投影的特征与当前特征进行对比。</br>
首先根据当前帧的位置得到最近帧对应的特征，具体如下：
{% codeblock %}
bool Point3D::getCloseViewObs(const Vector3d& framepos, Feature*& ftr) const
{
// TODO: 后期要确保点是相同的视图和相同的金字塔层
// 得到观察的方向向量
Vector3d obs_dir(framepos - pos_);
obs_dir.normalize();
auto min_it = obs_.begin();
double min_cos_angle = 0;
for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
{
Vector3d dir((*it)->frame->pos() - pos_);
dir.normalize();
double cos_angle = obs_dir.dot(dir);// 单位向量点乘得到cos角度
if (cos_angle > min_cos_angle)//保证特征是距离较近的两个帧
{
min_cos_angle = cos_angle;
min_it = it;
}
}
ftr = *min_it;
if (min_cos_angle < 0.5) // 假设观察夹角大于60度没有用
return false;
return true;
}
{% endcodeblock %}

然后再计算两帧之间的仿射变换。
{% codeblock %}
void Matcher::getWarpMatrixAffine(
const AbstractCamera& cam_ref,
const AbstractCamera& cam_cur,
const Vector2d& px_ref,
const Vector3d& f_ref,
const double depth_ref,
const SE3& T_cur_ref,
const int level_ref,
Matrix2d& A_cur_ref)
{
// 计算仿射变换矩阵A_ref_cur
const int halfpatch_size = 5;
const Vector3d xyz_ref(f_ref*depth_ref);
Vector3d xyz_du_ref(cam_ref.cam2world(px_ref + Vector2d(halfpatch_size, 0)*(1 << level_ref)));
Vector3d xyz_dv_ref(cam_ref.cam2world(px_ref + Vector2d(0, halfpatch_size)*(1 << level_ref)));
xyz_du_ref *= xyz_ref[2] / xyz_du_ref[2];
xyz_dv_ref *= xyz_ref[2] / xyz_dv_ref[2];
const Vector2d px_cur(cam_cur.world2cam(T_cur_ref*(xyz_ref)));
const Vector2d px_du(cam_cur.world2cam(T_cur_ref*(xyz_du_ref)));
const Vector2d px_dv(cam_cur.world2cam(T_cur_ref*(xyz_dv_ref)));
A_cur_ref.col(0) = (px_du - px_cur) / halfpatch_size;
A_cur_ref.col(1) = (px_dv - px_cur) / halfpatch_size;
}
{% endcodeblock %}

**具体上述仿射变换的求解应该只求解了旋转变换，具体还不是很理解为什么这么计算.**</br>
两帧之间的仿射变换计算好了之后，计算当前帧面片在另外一帧所处的金字塔等级，具体如下：
{% codeblock %}
int Matcher::getBestSearchLevel(const Matrix2d& A_cur_ref, const int max_level)
{
// 计算在其它图像中面片所处金字塔等级
int search_level = 0;
double D = A_cur_ref.determinant();
while (D > 3.0 && search_level < max_level)
{
search_level += 1;
D *= 0.25;
}
return search_level;
}
{% endcodeblock %}

**这边为什么通过仿射变换计算行列式可以寻找合适的尺度，也不是很理解**</br>
下一步就是对面片应用仿射变换
{% codeblock %}
void Matcher::warpAffine(
const Matrix2d& A_cur_ref,
const cv::Mat& img_ref,
const Vector2d& px_ref,
const int level_ref,
const int search_level,
const int halfpatch_size,
uint8_t* patch)
{
const int patch_size = halfpatch_size * 2;
const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
if (isnan(A_ref_cur(0, 0)))
{
printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
return;
}

// 对面片执行warp操作
uint8_t* patch_ptr = patch;
const Vector2f px_ref_pyr = px_ref.cast<float>() / (1 << level_ref);
for (int y = 0; y < patch_size; ++y)
{
for (int x = 0; x < patch_size; ++x, ++patch_ptr)
{
Vector2f px_patch(x - halfpatch_size, y - halfpatch_size);
px_patch *= (1 << search_level);
const Vector2f px(A_ref_cur*px_patch + px_ref_pyr);// 进行仿射变换
if (px[0] < 0 || px[1] < 0 || px[0] >= img_ref.cols - 1 || px[1] >= img_ref.rows - 1)
*patch_ptr = 0;
else
*patch_ptr = (uint8_t)interpolateMat_8u(img_ref, px[0], px[1]);
}
}
}
{% endcodeblock %}

**这边对上一帧的特征进行仿射变换计算新的对应特征，这边两帧之间的仿射变换的逆变换乘以标准面片加上上一帧的特征，这也不是很理解**</br>
注意上面的面片计算，都是采用了带有边框的面片，即面片大小是88，而实际计算是1010，这样可以保证面片的每个像素都被计算，这样，这一步就是通过带边框的面片计算面片，具体如下：
{% codeblock %}
void Matcher::createPatchFromPatchWithBorder()
{
uint8_t* ref_patch_ptr = patch_;
for (int y = 1; y < patch_size_ + 1; ++y, ref_patch_ptr += patch_size_)
{
uint8_t* ref_patch_border_ptr = patch_with_border_ + y*(patch_size_ + 2) + 1;
for (int x = 0; x < patch_size_; ++x)
ref_patch_ptr[x] = ref_patch_border_ptr[x];
}
}
{% endcodeblock %}

最后就是feature align 特征优化，残差就是当前图像可能特征与投影特征的差值。</br>
具体代码如下：
{% codeblock %}
bool align2D(
const cv::Mat& cur_img,
uint8_t* ref_patch_with_border,
uint8_t* ref_patch,
const int n_iter,
Vector2d& cur_px_estimate,
bool no_simd)
{
const int halfpatch_size_ = 4;
const int patch_size_ = 8;
const int patch_area_ = 64;
bool converged = false;

#ifdef _MSC_VER
__declspec(align(16)) float ref_patch_dx[patch_area_];
__declspec(align(16)) float ref_patch_dy[patch_area_];
#else
float __attribute__((__aligned__(16))) ref_patch_dx[patch_area_];
float __attribute__((__aligned__(16))) ref_patch_dy[patch_area_];
#endif
Matrix3f H; H.setZero();

// 计算梯度和hessian
const int ref_step = patch_size_ + 2;
float *it_dx = ref_patch_dx;
float *it_dy = ref_patch_dy;
for (int y = 0; y < patch_size_; ++y)
{
uint8_t* it = ref_patch_with_border + (y + 1)*ref_step + 1;
for (int x = 0; x < patch_size_; ++x, ++it, ++it_dx, ++it_dy)
{
Vector3f J;
J[0] = 0.5 * (it[1] - it[-1]);//x方向梯度值
J[1] = 0.5 * (it[ref_step] - it[-ref_step]);//y方向梯度值
J[2] = 1;
*it_dx = J[0];
*it_dy = J[1];
H += J*J.transpose();
}
}
Matrix3f Hinv = H.inverse();
float mean_diff = 0;

// 计算在新图像中的像素位置
float u = cur_px_estimate.x();
float v = cur_px_estimate.y();

// 终止条件
const float min_update_squared = 0.03*0.03;
const int cur_step = cur_img.step.p[0];
Vector3f update;
update.setZero();
for (int iter = 0; iter < n_iter; ++iter)
{
int u_r = floor(u);
int v_r = floor(v);
if (u_r < halfpatch_size_ || v_r < halfpatch_size_ || u_r >= cur_img.cols - halfpatch_size_ || v_r >= cur_img.rows - halfpatch_size_)
break;
// TODO very rarely this can happen, maybe H is singular? should not be at corner.. check
if (isnan(u) || isnan(v))
return false;

// 计算插值权重
float subpix_x = u - u_r;
float subpix_y = v - v_r;
float wTL = (1.0 - subpix_x)*(1.0 - subpix_y);
float wTR = subpix_x * (1.0 - subpix_y);
float wBL = (1.0 - subpix_x)*subpix_y;
float wBR = subpix_x * subpix_y;

// 循环遍历 插值
uint8_t* it_ref = ref_patch;
float* it_ref_dx = ref_patch_dx;
float* it_ref_dy = ref_patch_dy;

Vector3f Jres; Jres.setZero();
for (int y = 0; y < patch_size_; ++y)
{
uint8_t* it = (uint8_t*)cur_img.data + (v_r + y - halfpatch_size_)*cur_step + u_r - halfpatch_size_;
for (int x = 0; x < patch_size_; ++x, ++it, ++it_ref, ++it_ref_dx, ++it_ref_dy)
{
float search_pixel = wTL*it[0] + wTR*it[1] + wBL*it[cur_step] + wBR*it[cur_step + 1];
float res = search_pixel - *it_ref + mean_diff;
Jres[0] -= res*(*it_ref_dx);
Jres[1] -= res*(*it_ref_dy);
Jres[2] -= res;
}
}

update = Hinv * Jres;//负号前面已经添加
u += update[0];
v += update[1];
mean_diff += update[2];

if (update[0] * update[0] + update[1] * update[1] < min_update_squared)
{
converged = true;
break;
}
}

cur_px_estimate << u, v;
return converged;
}
{% endcodeblock %}

**具体高斯牛顿求解还是有欠缺**</br>
到这里整个重投影及feature align就结束了，最后记录好匹配上的跟踪特征数，对这个数设好上下限。</br>
写了简单的测试程序如下：
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
mvo::Map map;
fisrt_frame->setKeyframe();
second_frame->setKeyframe();
map.addKeyframe(fisrt_frame);
map.addKeyframe(second_frame);
Reprojector reprojector(cam,map);
std::vector< std::pair<FramePtr, size_t> > overlap_kfs;
reprojector.reprojectMap(third_frame, overlap_kfs);
const size_t repr_n_new_references = reprojector.n_matches_;
const size_t repr_n_mps = reprojector.n_trials_;
std::cout << "Reprojection:\t nPoints = " << repr_n_mps << "\t \t nMatches = " << repr_n_new_references<<std::endl;
getchar();
return 0;
}
{% endcodeblock %}

最后结果如下：

![](http://7xl6tk.com1.z0.glb.clouddn.com/feature_align_project.png )

总结
---

运动估计总共分成了三块，image align ，feature align以及接下来的pose and structure refinement.主要的都是优化，解决的问题基本都是非线性最小二乘问题，确定好合适的残差，求解的过程。对于非线性最小二乘的实现这个基础要好好掌握。




转载自冯兵的博客，[原文链接][link]



[link]:http://fengbing.net/2015/09/06/%E4%B8%80%E6%AD%A5%E6%AD%A5%E5%AE%9E%E7%8E%B0%E5%8D%95%E7%9B%AE%E8%A7%86%E8%A7%89%E9%87%8C%E7%A8%8B%E8%AE%A15%E2%80%94%E2%80%94%E8%BF%90%E5%8A%A8%E4%BC%B0%E8%AE%A1reprojector,feature%20align/


