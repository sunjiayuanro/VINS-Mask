#include "feature_tracker.h"
#include "parameters.h"
#include <algorithm>
#include <sys/types.h>

#ifndef MASK
#define MASK 1
#endif


// FeatureTracker中的static成员变量n_id初始化为0
int FeatureTracker::n_id = 0;

// 判断跟踪的特征点是否在图像边界内
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x); // cvRound 四舍五入取整
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

struct greaterThanPtr
{
    bool operator () (const float * a, const float * b) const
    // Ensure a fully deterministic result of the sort
    { return (*a > *b) ? true : (*a < *b) ? false : (a > b); }
};

// 去除无法跟踪的特征点
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
  : first_frame(true)
  , uri("192.168.1.10:50055")
  , min_dist(MIN_DIST)
  , initial_status(false)
{

  vio_api.reset(new mvins_api::mvins::RpcApi(grpc::CreateChannel(uri.c_str(), grpc::InsecureChannelCredentials())));

  auto check_r = mvins_api::Common::ProtoString();
  check_r.set_data("tracker"); // ros::Time::now().toSec()
  auto response = mvins_api::Common::CommandFeedback();
  if(vio_api->CheckCommunication(check_r,response).status() != mvins_api::Common::StateEnum::SUCCEEDED){
    ROS_ERROR("MVINS API is not found!");
    exit(1);
  }
}

void FeatureTracker::setMinDist(int dist)
{
    // if(this->min_dist > dist)
        this->min_dist = dist;
}
    
void FeatureTracker::setInitialStatus(bool status)
{
    this->initial_status = status;
    if(this->initial_status)
    {
        setMinDist(MIN_DIST_INITIALIZED);
    } else {
        setMinDist(MIN_DIST);
    }
}

// 对跟踪点进行排序并去除密集点
// 对跟踪到的特征点按照被跟踪到的次数排序并依次选点
// 使用mask进行类似非极大值抑制，半径为30，去掉密集点，使特征分布均匀
void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else {
#if MASK
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));
#else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
#endif
    }
        
    // mask_edges = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));
    
    // prefer to keep features that are tracked for long time
    // 构造<track_cnt，<forw_pts，id> >序列
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    // 对光流跟踪到的特征点forw_pts，按照被跟踪到的次数cnt从大到小排序
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    // 清空
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

#if MASK

    cv::Mat dstImage;

    cv::blur(forw_img, dstImage, cv::Size(7,7));
    cv::Canny(dstImage, mask, 200, 50, 3, false);

    int kernel_size = std::min(min_dist*2,MIN_DIST);
    cv::Mat kernel(kernel_size,kernel_size,CV_8UC1,cv::Scalar(1));
    cv::filter2D(mask, mask, CV_8UC1, kernel);

    std::vector<uchar> data_encode;
    cv::imencode(".png", forw_img, data_encode);
    std::string encoded_im = base64Encode(reinterpret_cast<const unsigned char*>(data_encode.data()), data_encode.size());

    auto res = mvins_api::Common::PointCloudXY();
    auto req = mvins_api::Common::ProtoString();
    req.set_data(encoded_im);
    if(vio_api->GetSuperPoint(req,res).status() != mvins_api::Common::StateEnum::SUCCEEDED)
    {
        ROS_WARN_STREAM("GetSuperPoint");
    } else {
        for(int i = 0; i < res.points_size();i++)
        {
            cv::circle(mask, cv::Point(res.points(i).x(),res.points(i).y()), min_dist, 255, -1);
        }
    }

#endif

    for (auto &it : cnt_pts_id)
    {
        // 当前特征点位置对应的mask值为255，则保留当前特征点
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            // 在mask中将当前特征点周围半径为min_dist的区域设置为0，后面不在选取该区域的点，使跟踪点不集中在一个区域上
            cv::circle(mask, it.second.first, min_dist, 0, -1);
        }
    }
    // cv::imwrite("/tmp/mask.png",mask);
}

// 添加新检测到的特征点n_pts
void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1); // id初始化为-1
        track_cnt.push_back(1); // track_cnt初始化为1
    }
}
void FeatureTracker::addEdgePoints()
{
    for (auto &p : n_pts_edges)
    {
        forw_pts.push_back(p);
        ids.push_back(-1); // id初始化为-1
        track_cnt.push_back(1); // track_cnt初始化为1
    }
}

// 对图像使用光流法进行特征点跟踪
void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    // EQUALIZE == 1，表示太亮或太暗，进行直方图均衡化处理
    if (EQUALIZE)
    {
        // 自适应直方图均衡
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    // 当前帧的图像数据为空 （第一次读入图像）
    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else // 已有图像读入，更新当前帧 forw_img
    {
        forw_img = img;
    }

    // 清除上一帧特征点数据 forw_pts
    forw_pts.clear();

    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        // 对前一帧的特征点 cur_pts 进行LK金字塔光流跟踪，得到 forw_pts
        // status: 标记了从前一帧 cur_img 到 forw_img 特征点的跟踪状态，无法被追踪到的点标记为0
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        // 位于图像边界的点标记为0
        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;

        // 根据 status 把跟踪失败的点剔除
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    // 光流跟踪成功，特征点被成功追踪的次数就加1，数值代表被追踪的次数
    for (auto &n : track_cnt)
        n++;

    // 是否发布特征点
    if (PUB_THIS_FRAME)
    {
        // 通过基本矩阵剔除 outliers
        rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask(); // 保证相邻的特征点之间要相隔30个像素，设置mask
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        // 计算是否需要提取新的特征点
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0) //
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;

            // 在mask中不为0的区域检测新的特征点
            if(0) // SuperPoint detector
            {
                std::vector<uchar> data_encode;
                cv::imencode(".png", forw_img, data_encode);
                std::string encoded_im = base64Encode(reinterpret_cast<const unsigned char*>(data_encode.data()), data_encode.size());

                auto res = mvins_api::Common::PointCloudXY();
                auto req = mvins_api::Common::ProtoString();
                req.set_data(encoded_im);
                if(vio_api->GetSuperPoint(req,res).status() != mvins_api::Common::StateEnum::SUCCEEDED)
                {
                  ROS_WARN_STREAM("GetSuperPoint");
                } else {
                  n_pts.clear();
                  for(int i = 0; i < res.points_size();i++)
                  {
                      if(mask.at<uchar>(cv::Point2f(res.points(i).x(),res.points(i).y())) == 255)
                      {
                          n_pts.push_back(cv::Point2f(res.points(i).x(),res.points(i).y()));
                      }
                  }
                }
            } else {
                cv::goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.01, min_dist, mask, 3, false);
            }
        }
        else {
            n_pts.clear();
            n_pts_edges.clear();
        }
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        
        // 将新检测到的特征点n_pts添加到forw_pts中
        addPoints();
        addEdgePoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    // 更新上一帧数据
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    // 把当前帧 forw_* 赋值给 上一帧 cur_*
    cur_img = forw_img;
    cur_pts = forw_pts;
    // 根据不同的相机模型去畸变矫正和转换到归一化坐标系上，计算速度
    undistortedPoints();
    prev_time = cur_time;
}

// 通过基础矩阵去除outliers
void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            // 根据相机模型将二维坐标转换到三维坐标
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            // 转换为归一化像素坐标
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

// 更新特征点id
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

// 读取相机内参
void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

// 显示去畸变矫正后的特征点
// name: 图像帧名称
void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            
            m_camera->liftProjective(a, b);
            
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }

    
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        // 根据相机模型将二维坐标转换到三维坐标
        m_camera->liftProjective(a, b);
        // 在延伸到深度归一化平面上
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    // 计算每个特征点的速度到pts_velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}
