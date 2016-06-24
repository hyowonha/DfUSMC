//
//  DfUSMC.hpp
//  DfUSMC
//
//  Created by Hyowon Ha on 2016. 6. 11..
//  Copyright © 2016년 Hyowon Ha. All rights reserved.
//

#ifndef DfUSMC_hpp
#define DfUSMC_hpp

#include <stdio.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include "ceres/ceres.h"
#include <time.h>
#include "qx_basic.h"
#include "qx_tree_upsampling.h"

using namespace std;
using namespace cv;

class DfUSMC
{
public:
    DfUSMC() {}
    ~DfUSMC() {
        delete[] poses;
        delete[] inv_depths;
    }
    
    void LoadSmallMotionClip(char *fullpath);
    void FeatureExtractionAndTracking();
    void BundleAdjustment(double f_init, double k1_init, double k2_init); // bundle adjustment with given initial parameters
    void BundleAdjustment();            // no input -> use our initial parameters
    void SavePointCloudPLY(char *fullpath);
    void UndistortImages();
    void DenseMatching(double scale, int num_label, double lambda, double sigma);
    void SaveDepthmapWTA(char *fullpath);
    void SaveDepthmapFiltered(char *fullpath);
    void SaveDepthmapRefined(char *fullpath);
    
private:
    std::vector<Mat> images;
    int num_image;
    int image_width;
    int image_height;
    
    Mat features;
    int num_feature;
    
    Mat ud_map, vd_map;
    Mat confidencemap;
    Mat depthmapWTA;
    Mat depthmapFiltered; // outlier removal
    Mat depthmapRefined;
    
    double cx, cy;
    double f, k1, k2;
    double f_new;
    double *poses;
    double *inv_depths;
        
};

#endif /* DfUSMC_hpp */
