//
// Created by nickolay on 17.06.2021.
//

#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
#include "Dataset.h"

void Dataset::saveDataset(std::string dirToSave) {
    std::string path0 = dirToSave + "/0";
    std::string path1 = dirToSave + "/1";
    std::string path2 = dirToSave + "/2";
    std::string path3 = dirToSave + "/3";
    std::string path4 = dirToSave + "/4";


    path0.erase(std::remove(path0.begin(), path0.end(), ' '), path0.end());
    path1.erase(std::remove(path1.begin(), path1.end(), ' '), path1.end());
    path2.erase(std::remove(path2.begin(), path2.end(), ' '), path2.end());
    path3.erase(std::remove(path3.begin(), path3.end(), ' '), path3.end());
    path4.erase(std::remove(path4.begin(), path4.end(), ' '), path4.end());


    path0.erase(std::remove(path0.begin(), path0.end(), '\n'), path0.end());
    path1.erase(std::remove(path1.begin(), path1.end(), '\n'), path1.end());
    path2.erase(std::remove(path2.begin(), path2.end(), '\n'), path2.end());
    path3.erase(std::remove(path3.begin(), path3.end(), '\n'), path3.end());
    path4.erase(std::remove(path4.begin(), path4.end(), '\n'), path4.end());

    std::string com0 = "mkdir -p " + path0;
    std::string com1 = "mkdir -p " + path1;
    std::string com2 = "mkdir -p " + path2;
    std::string com3 = "mkdir -p " + path3;
    std::string com4 = "mkdir -p " + path4;

    system(com0.data());
    system(com1.data());
    system(com2.data());
    system(com3.data());
    system(com4.data());

    int saveCounter = 0;

    for (int i=0;i<forwardCamImages.size()-1;i++){
        std::string path0 = dirToSave + "/0";
        std::string path1 = dirToSave + "/1";
        std::string path2 = dirToSave + "/2";
        std::string path3 = dirToSave + "/3";
        std::string path4 = dirToSave + "/4";


        path0.erase(std::remove(path0.begin(), path0.end(), ' '), path0.end());
        path1.erase(std::remove(path1.begin(), path1.end(), ' '), path1.end());
        path2.erase(std::remove(path2.begin(), path2.end(), ' '), path2.end());
        path3.erase(std::remove(path3.begin(), path3.end(), ' '), path3.end());
        path4.erase(std::remove(path4.begin(), path4.end(), ' '), path4.end());

        path0.erase(std::remove(path0.begin(), path0.end(), '\n'), path0.end());
        path1.erase(std::remove(path1.begin(), path1.end(), '\n'), path1.end());
        path2.erase(std::remove(path2.begin(), path2.end(), '\n'), path2.end());
        path3.erase(std::remove(path3.begin(), path3.end(), '\n'), path3.end());
        path4.erase(std::remove(path4.begin(), path4.end(), '\n'), path4.end());

        path0 += "/" + std::to_string(saveCounter) + ".jpg";
        path1 += "/" + std::to_string(saveCounter) + ".jpg";
        path2 += "/" + std::to_string(saveCounter) + ".jpg";
        path3 += "/" + std::to_string(saveCounter) + ".dpt";
        path4 += "/" + std::to_string(saveCounter) + ".intr";

        cv::imwrite(path0, forwardCamImages[i]);
        cv::imwrite(path1, depthImages[i]);
#ifdef RIGHT_CAMERA_EXISTS
        cv::imwrite(path2, *rightBoardImage.getImage());
#endif
        saveDepth(path3, saveCounter);
        if (saveCounter==0){
            saveIntristics(path4);
        }
        saveCounter++;
        std::cout << "Saving dataset. "<< saveCounter<<" / "<<forwardCamImages.size()<<" done "<<std::endl;
    }
}


void Dataset::saveDepth(const std::string& path, int num){
    std::ofstream fout;
    fout.open(path,std::ios_base::out);
    double d = 0;
    for (int i=0;i<960;i++){
        for (int j=0;j<540;j++){
            d = depthValues[num][i][j];
            fout<<" "<<d<<" ";
        }
        fout<<"\n";
    }
    fout.close();
}

void Dataset::saveIntristics(std::string path){
    std::ofstream fout;
    fout.open(path,std::ios_base::out);
    rs2_intrinsics r = depthIntristics;
    fout.write(reinterpret_cast<char*>(&r), sizeof(r));
}

void Dataset::clear() {
    forwardCamImages.clear();
    rightBoardImages.clear();
    depthValues.clear();
    depthImages.clear();
}