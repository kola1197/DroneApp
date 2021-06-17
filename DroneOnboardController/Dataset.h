//
// Created by nickolay on 17.06.2021.
//

#ifndef DRONEAPP_DATASET_H
#define DRONEAPP_DATASET_H


#include <vector>
#include <opencv2/core/mat.hpp>
#include <librealsense2/h/rs_types.h>

class Dataset {
public:
    void saveDataset(std::string dirToSave);

    std::vector <cv::Mat> forwardCamImages;
    std::vector <cv::Mat> depthImages;
    std::vector <cv::Mat> rightBoardImages;
    //std::vector<double[960][540]> depthValues;
    std::vector<std::vector<std::vector<double>>> depthValues;
    rs2_intrinsics camIntristics;
    rs2_intrinsics depthIntristics;

    void saveDepth(const std::string &path, int num);

    void saveIntristics(std::string path);
};


#endif //DRONEAPP_DATASET_H
