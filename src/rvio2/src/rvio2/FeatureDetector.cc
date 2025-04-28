/**
* This file is part of R-VIO2.
*
* Copyright (C) 2022 Zheng Huai <zhuai@udel.edu> and Guoquan Huang <ghuang@udel.edu>
* For more information see <http://github.com/rpng/R-VIO2>
*
* R-VIO2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* R-VIO2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with R-VIO2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <opencv2/opencv.hpp>
#include <cmath>
#include <cassert>

#include "FeatureDetector.h"
#include <rclcpp/rclcpp.hpp>

namespace RVIO2
{

FeatureDetector::FeatureDetector(const rclcpp::Node::SharedPtr& node)
{
    // Declare parameters with default values
    node->declare_parameter("Camera.width", 752);
    node->declare_parameter("Camera.height", 480);
    node->declare_parameter("Tracker.nMinDist", 15);
    node->declare_parameter("Tracker.nQualLvl", 0.01);
    node->declare_parameter("Tracker.nBlockSizeX", 150);
    node->declare_parameter("Tracker.nBlockSizeY", 120);

    // Retrieve parameters
    mnImageCols   = node->get_parameter("Camera.width").as_int();
    mnImageRows   = node->get_parameter("Camera.height").as_int();
    mnMinDistance = node->get_parameter("Tracker.nMinDist").as_int();
    mnQualityLevel = node->get_parameter("Tracker.nQualLvl").as_double();
    mnBlockSizeX  = node->get_parameter("Tracker.nBlockSizeX").as_int();
    mnBlockSizeY  = node->get_parameter("Tracker.nBlockSizeY").as_int();

    // Compute grid layout and offsets
    mnGridCols = mnImageCols / mnBlockSizeX;
    mnGridRows = mnImageRows / mnBlockSizeY;
    mnOffsetX  = 0.5 * (mnImageCols - mnGridCols * mnBlockSizeX);
    mnOffsetY  = 0.5 * (mnImageRows - mnGridRows * mnBlockSizeY);
    mnBlocks   = mnGridCols * mnGridRows;

    int nMaxFeatsPerImage = node->get_parameter("Tracker.nFeatures").as_int();
    mnMaxFeatsPerBlock = nMaxFeatsPerImage / mnBlocks;

    mvvGrid.resize(mnBlocks);
}


int FeatureDetector::DetectWithSubPix(const cv::Mat& im, 
                                        const int nCorners, 
                                        const int s, 
                                        std::vector<cv::Point2f>& vCorners)
{
    vCorners.clear();
    vCorners.reserve(nCorners);

    cv::goodFeaturesToTrack(im, vCorners, nCorners, mnQualityLevel, s * mnMinDistance);

    if (!vCorners.empty())
    {
        cv::Size subPixWinSize(static_cast<int>(std::floor(0.5 * mnMinDistance)),
                               static_cast<int>(std::floor(0.5 * mnMinDistance)));
        cv::Size subPixZeroZone(-1, -1);
        cv::TermCriteria subPixCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-2);
        cv::cornerSubPix(im, vCorners, subPixWinSize, subPixZeroZone, subPixCriteria);
    }

    return static_cast<int>(vCorners.size());
}


void FeatureDetector::ChessGrid(const std::vector<cv::Point2f>& vCorners)
{
    mvvGrid.clear();
    mvvGrid.resize(mnBlocks);

    for (const cv::Point2f& pt : vCorners)
    {
        if (pt.x <= mnOffsetX || pt.y <= mnOffsetY ||
            pt.x >= (mnImageCols - mnOffsetX) || pt.y >= (mnImageRows - mnOffsetY))
            continue;

        int col = static_cast<int>((pt.x - mnOffsetX) / mnBlockSizeX);
        int row = static_cast<int>((pt.y - mnOffsetY) / mnBlockSizeY);
        assert((col >= 0 && col < mnGridCols) && (row >= 0 && row < mnGridRows));

        int nBlockIdx = row * mnGridCols + col;
        mvvGrid.at(nBlockIdx).emplace_back(pt);
    }
}


int FeatureDetector::FindNewer(const std::vector<cv::Point2f>& vCorners, 
                                 const std::vector<cv::Point2f>& vRefCorners, 
                                 std::vector<cv::Point2f>& vNewCorners)
{
    ChessGrid(vRefCorners);

    for (const cv::Point2f& pt : vCorners)
    {
        if (pt.x <= mnOffsetX || pt.y <= mnOffsetY ||
            pt.x >= (mnImageCols - mnOffsetX) || pt.y >= (mnImageRows - mnOffsetY))
            continue;

        int col = static_cast<int>((pt.x - mnOffsetX) / mnBlockSizeX);
        int row = static_cast<int>((pt.y - mnOffsetY) / mnBlockSizeY);
        assert((col >= 0 && col < mnGridCols) && (row >= 0 && row < mnGridRows));

        float xl = col * mnBlockSizeX + mnOffsetX;
        float xr = xl + mnBlockSizeX;
        float yt = row * mnBlockSizeY + mnOffsetY;
        float yb = yt + mnBlockSizeY;

        if (std::fabs(pt.x - xl) < mnMinDistance || std::fabs(pt.x - xr) < mnMinDistance ||
            std::fabs(pt.y - yt) < mnMinDistance || std::fabs(pt.y - yb) < mnMinDistance)
            continue;

        int nBlockIdx = row * mnGridCols + col;

        if (static_cast<float>(mvvGrid.at(nBlockIdx).size()) < 0.75f * mnMaxFeatsPerBlock)
        {
            if (!mvvGrid.at(nBlockIdx).empty())
            {
                int cnt = 0;
                for (const cv::Point2f& bpt : mvvGrid.at(nBlockIdx))
                {
                    if (cv::norm(pt - bpt) > mnMinDistance)
                        cnt++;
                    else
                        break;
                }
                if (cnt == static_cast<int>(mvvGrid.at(nBlockIdx).size()))
                {
                    vNewCorners.push_back(pt);
                    mvvGrid.at(nBlockIdx).push_back(pt);
                }
            }
            else
            {
                vNewCorners.push_back(pt);
                mvvGrid.at(nBlockIdx).push_back(pt);
            }
        }
    }

    return static_cast<int>(vNewCorners.size());
}

} // namespace RVIO2
