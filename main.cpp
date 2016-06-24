//
//  main.cpp
//  DfUSMC: Depth from Uncalibrated Small Motion Clip
//
//  Created by Hyowon Ha on 2016. 6. 11..
//  Copyright © 2016년 Hyowon Ha. All rights reserved.
//

#include <iostream>
#include "DfUSMC.hpp"

int main(int argc, const char * argv[]) {
    DfUSMC dfusmc;
    
    // input file path
    char *data_name="Bikes";
    char *video_format="avi";
    
    // load small motion clip
    char fullpath[1024];
    sprintf(fullpath,"Dataset/%s.%s",data_name,video_format);
    dfusmc.LoadSmallMotionClip(fullpath);
    
    // feature extraction and tracking
    dfusmc.FeatureExtractionAndTracking();
    
    // bundle adjustment
    dfusmc.BundleAdjustment();
    
    // (optional) save sparse reconstruction result.
    sprintf(fullpath,"Result/%s.ply",data_name);
    dfusmc.SavePointCloudPLY(fullpath);
    
    // image undistortion
    dfusmc.UndistortImages();
    
    // dense matching
    dfusmc.DenseMatching(0.5, 64, 3.0, 0.1);
    
    // (optional) save Winner-Takes-All depthmap result.
    sprintf(fullpath,"Result/%s_WTA.bmp",data_name);
    dfusmc.SaveDepthmapWTA(fullpath);
    
    // (optional) save filtered depthmap result.
    sprintf(fullpath,"Result/%s_Filtered.bmp",data_name);
    dfusmc.SaveDepthmapFiltered(fullpath);

    // (optional) save refined depthmap result.
    sprintf(fullpath,"Result/%s_Filtered.bmp",data_name);
    dfusmc.SaveDepthmapRefined(fullpath);

    
    waitKey(0);
    
    std::cout << "Done.\n";
    return 0;
}
