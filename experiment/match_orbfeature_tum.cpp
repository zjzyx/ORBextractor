#include "common_headers.h"
#include "parameter_reader.h"
#include "rgbdframe.h"
#include "orb.h"

#include <opencv2/features2d/features2d.hpp>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Thirdparty/orbslam_modified/include/ORBextractor.h"

using namespace std;
using namespace rgbd_tutor;
using namespace cv;
/**
 * @brief 这里的代码完成的功能是orb特征的提取与匹配，提取采用的是orb-slam2的方法，匹配是采用opencv自带的knnMatch方法
 *        有时间的话还需要把这个匹配也改成orb-slam2里面的匹配方式
 * @return
 */
int main()
{
    cout<<"running orbfeature_tum"<<endl;
    ParameterReader para;
    FrameReader frameReader( para );
    OrbFeature  orb( para );
//    RGBDFrame::Ptr last_frame = frameReader.next();
//    while ( RGBDFrame::Ptr frame = frameReader.next() )
//    {
//        boost::timer    timer;
//        orb.detectFeatures( frame );
//        vector<cv::DMatch>  matches = orb.match( last_frame, frame );
//        cout<<"matches = "<<matches.size()<<endl;
//        cout<<"timer used for detecting and matching features"<<timer.elapsed()<<endl;
//        last_frame = frame;
//    }

    //-- 读取图像
    RGBDFrame::Ptr img_1 = frameReader.next();
    //-- 检测 Oriented FAST 角点位置且根据角点位置计算 BRIEF 描述子
    orb.detectFeatures( img_1 );
    RGBDFrame::Ptr img_2 = frameReader.next();
    orb.detectFeatures( img_2 );
    //-- 对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<cv::DMatch>  matches = orb.match( img_1, img_2 );
//    cout << " bbb " << matches.size() << endl;
    vector<cv::KeyPoint> keypoints_1, keypoints_2;
    for (int i = 0, iend = img_1->features.size(); i < iend; i++) {
        keypoints_1.push_back((img_1->features[i]).keypoint);
    }
//    cout << " ccc " << keypoints_1.size() << endl;
    for (int i = 0, iend = img_2->features.size(); i < iend; i++) {
        keypoints_2.push_back((img_2->features[i]).keypoint);
    }
//    cout << " ddd " << keypoints_2.size() << endl;
    cv::Mat matchesshow;
    cv::drawMatches ( img_1->rgb, keypoints_1, img_2->rgb, keypoints_2, matches, matchesshow );
    cv::imshow ( "orb-matches", matchesshow );
    cv::waitKey(0);

    return 0;
}
