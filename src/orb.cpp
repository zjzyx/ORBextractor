/*************************************************************************
	> File Name: orb.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年02月29日 星期一 12时14分06秒
 ************************************************************************/

#include <iostream>
#include "common_headers.h"
#include "Thirdparty/orbslam_modified/include/Converter.h"
#include "orb.h"
using namespace std;

using namespace rgbd_tutor;

vector<cv::DMatch> OrbFeature::match( const RGBDFrame::Ptr& frame1, const RGBDFrame::Ptr& frame2 ) const
{
    vector< vector<cv::DMatch> > matches_knn;
    cv::Mat desp1 = frame1->getAllDescriptors();
    cv::Mat desp2 = frame2->getAllDescriptors();
    // 使用KNN-matching算法，令K=2。则每个match得到两个最接近的descriptor，然后计算最接近距离和次接近距离之间的比值，当比值大于既定值时，才作为最终match
    // 参考: http://blog.csdn.net/yangtrees/article/details/19928191
    matcher->knnMatch( desp1, desp2, matches_knn, 2 );
    vector< cv::DMatch > matches;
    for ( size_t i=0; i<matches_knn.size(); i++ )
    {
        if (matches_knn[i][0].distance < knn_match_ratio * matches_knn[i][1].distance )
            matches.push_back( matches_knn[i][0] );
    }
    return matches;
}
