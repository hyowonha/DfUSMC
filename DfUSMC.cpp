//
//  DfUSMC.cpp
//  DfUSMC
//
//  Created by Hyowon Ha on 2016. 6. 11..
//  Copyright © 2016 Hyowon Ha. All rights reserved.
//

#include "DfUSMC.hpp"

void DfUSMC::LoadSmallMotionClip(char *fullpath)
{
    VideoCapture cap(fullpath);
    if(!cap.isOpened())  // check if we succeeded
        return;
    
    num_image=cap.get(CV_CAP_PROP_FRAME_COUNT)+1;
    cout<<"num frame: "<<num_image<<endl;
    num_image=(num_image>30)?30:num_image;
    cout<<"We use "<<num_image<<" images in the beginning."<<endl;
    
    image_width=cap.get(CV_CAP_PROP_FRAME_WIDTH);
    image_height=cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    cout<< "image size: "<<image_width<<" "<<image_height<<endl;
    
    for(int i=0;i<num_image;i++)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        images.push_back(frame);
    }
}

void DfUSMC::SaveReferenceImage(char *fullpath)
{
    imwrite(fullpath, images[0]);
    imshow("Reference image", images[0]);
    waitKey(1);
}

void DfUSMC::FeatureExtractionAndTracking()
{
    Mat img_ref=images.at(0);
    Mat gray_ref;
    cvtColor(img_ref, gray_ref, CV_RGB2GRAY);
    
    // feature extraction (other features can be used instead of Harris corner)
    vector<Point2f> corners_ref;
    goodFeaturesToTrack(gray_ref, corners_ref, 20000, 1e-10, 5, noArray(), 10);
    
    int num_corner=corners_ref.size();
    cout<<"num extracted corner:"<<num_corner<<endl;
    Mat img=img_ref.clone();
    for(int i = 0; i < num_corner; i++ )
        circle( img, corners_ref[i], 3, Scalar(0,0,255), 2);
    
    bool *inlier_mask=new bool[num_corner];
    features=Mat(2,num_corner,CV_32FC1);
    for(int i=0;i<num_corner;i++) {
        inlier_mask[i]=true;
        features.at<float>(0,i)=corners_ref[i].x;
        features.at<float>(1,i)=corners_ref[i].y;
    }
    
    // feature tracking (reference <-> non-reference)
    for(int i=1;i<num_image;i++) {
        vector<Point2f> corners_fwd;
        vector<Point2f> corners_bwd;
        vector<unsigned char> status_fwd;
        vector<unsigned char> status_bwd;
        vector<float> err_fwd;
        vector<float> err_bwd;
        
        Mat gray;
        cvtColor(images.at(i), gray, CV_RGB2GRAY);
        
        calcOpticalFlowPyrLK(gray_ref, gray, corners_ref, corners_fwd, status_fwd, err_fwd);
        calcOpticalFlowPyrLK(gray, gray_ref, corners_fwd, corners_bwd, status_bwd, err_bwd);
        
        Mat features_i=Mat(2,num_corner,CV_32FC1);
        for(int j=0;j<num_corner;j++){
            features_i.at<float>(0,j)=corners_fwd[j].x;
            features_i.at<float>(1,j)=corners_fwd[j].y;
            
            float bidirectional_error=norm(corners_ref[j]-corners_bwd[j]);
            if(status_fwd[j]==0 || status_bwd[j]==0 || bidirectional_error>0.1)
                inlier_mask[j]=false;
        }
        features.push_back(features_i);
    }
    
    int num_inlier=0;
    for(int i=0;i<num_corner;i++) {
        if(inlier_mask[i]) {
            num_inlier++;
            circle( img, corners_ref[i], 3, Scalar(0,255,0), 2);
        }
    }

    namedWindow("corners");
    imshow("corners",img);
    waitKey(100);
    
    cout<<"num inlier: "<<num_inlier<<"/"<<num_corner<<endl;
    
    Mat features_bak=features.clone();
    features=Mat(2*num_image,num_inlier,CV_32FC1);
    int idx=0;
    for(int i=0;i<num_corner;i++) {
        if(inlier_mask[i]){
            for(int j=0;j<2*num_image;j++) {
                features.at<float>(j,idx)=features_bak.at<float>(j,i);
            }
            idx++;
        }
    }
    num_feature=num_inlier;
}

struct BACostFunction {
    BACostFunction(double focal_length, double cx, double cy, double u0, double v0, double u, double v)
    : focal_length(focal_length), cx(cx), cy(cy), u0(u0), v0(v0), u(u), v(v) {}
    
    template <typename T>
    bool operator()(const T* const camera,
                    const T* const point,
                    const T* const intrinsic,
                    T* residuals) const {
        
        T f=intrinsic[0]*T(focal_length);
        T k1=intrinsic[1]/T(10.0);
        T k2=intrinsic[2]/T(10.0);
        
        T du0_= T(u0)-T(cx);
        T dv0_= T(v0)-T(cy);
        T dx0_= du0_/f;
        T dy0_= dv0_/f;
        T r20_= dx0_*dx0_ + dy0_*dy0_;
        
        T distort0_=T(1.0) + k1*r20_ + k2*r20_*r20_;
        
        T du0=du0_*distort0_;
        T dv0=dv0_*distort0_;
        T w=point[0];
        
        T p[3];
        T rx=camera[0], ry=camera[1], rz=camera[2];
        T tx=camera[3], ty=camera[4], tz=camera[5];
        
        p[0] = du0		- rz*dv0	+ f*ry	+ f*tx*w;
        p[1] = rz*du0	+ dv0		- f*rx	+ f*ty*w;
        p[2] = -ry*du0	+ rx*dv0	+ f		+ f*tz*w;
        
        T up = f * p[0] / p[2];
        T vp = f * p[1] / p[2];
        
        T du_ = T(u)-T(cx);
        T dv_ = T(v)-T(cy);
        T dx_ = du_/f;
        T dy_ = dv_/f;
        T r2_ = dx_*dx_ + dy_*dy_;
        
        T distort_=T(1.0) + k1*r2_ + k2*r2_*r2_;
        
        T du=du_*distort_;
        T dv=dv_*distort_;
        
        residuals[0] = du - up;
        residuals[1] = dv - vp;
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double focal_length, const double cx, const double cy,
                                       const double u0, const double v0,
                                       const double u, const double v) {
        return (new ceres::AutoDiffCostFunction<BACostFunction, 2, 6, 1, 3>(new BACostFunction(focal_length,cx,cy,u0,v0,u,v)));
    }
    
    double focal_length;
    double cx;
    double cy;
    double u0;
    double v0;
    double u;
    double v;
};

void DfUSMC::BundleAdjustment(double f_init, double k1_init, double k2_init) {
    // fix principal point at the center
    cx=image_width/2;
    cy=image_height/2;
    
    poses=new double[6*num_image];
    inv_depths=new double[num_feature];
    
    // initialization
    for(int i=0;i<6*num_image;i++)
        poses[i]=0.0;
    
    srand(time(NULL));
    double w_min=0.01, w_max=1.0;
    for(int i=0;i<num_feature;i++)
        inv_depths[i]=w_min+(w_max-w_min)*double(rand())/RAND_MAX;
    
    double variables[3];
    variables[0]=1.0; // scaling factor w.r.t. f_init
    variables[1]=k1_init*10.0; // regularize scale
    variables[2]=k2_init*10.0; // regularize scale
    
    ceres::Problem problem;
    for (int i=0;i<num_feature;i++) {
        double u0=features.at<float>(0,i);
        double v0=features.at<float>(1,i);
        
        for (int j=1;j<num_image;j++) {
            double u=features.at<float>(j*2+0,i);
            double v=features.at<float>(j*2+1,i);
            
            ceres::CostFunction* cost_function = BACostFunction::Create(f_init, cx, cy, u0, v0, u, v);
            problem.AddResidualBlock(cost_function,
                                     new ceres::HuberLoss(1.0),
                                     poses + (j*6),
                                     inv_depths + (i),
                                     variables);
        }
    }
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.max_num_iterations = 100;
    //    options.num_threads = 8;
    options.minimizer_progress_to_stdout = true;
    
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    
    f=f_init*variables[0];
    k1=variables[1]/10.0;
    k2=variables[2]/10.0;
    
    cout<<"focal length: "<<f<<endl;
    cout<<"radial distortion: "<<k1<<" "<<k2<<endl;
    
    int cnt=0;
    for(int i=0;i<num_feature;i++) {
        if(inv_depths[i]<0) cnt++;
    }
    // when result is flipped
    if(cnt>num_feature-cnt) {
        for(int i=0;i<num_feature;i++) {
            inv_depths[i]*=-1.0;
        }
        for(int i=0;i<num_image;i++) {
            poses[i*6+3]*=-1.0;
            poses[i*6+4]*=-1.0;
            poses[i*6+5]*=-1.0;
        }
    }
}

void DfUSMC::BundleAdjustment() { // when no input is given
    BundleAdjustment((image_width>image_height)?image_width:image_height,
                     0.0, 0.0);
}

void DfUSMC::SavePointCloudPLY(char *fullpath) {
    Mat img_ref=images.at(0);
    unsigned char *imageData = (unsigned char*)img_ref.data;
    
    FILE *out=fopen(fullpath,"wb");
    fprintf(out,"ply\n");
    fprintf(out,"format binary_little_endian 1.0\n");
    fprintf(out,"element vertex %d\n",num_image+num_feature);
    fprintf(out,"property float x\n");
    fprintf(out,"property float y\n");
    fprintf(out,"property float z\n");
    fprintf(out,"property uchar diffuse_red\n");
    fprintf(out,"property uchar diffuse_green\n");
    fprintf(out,"property uchar diffuse_blue\n");
    fprintf(out,"end_header\n");
    
    for(int i=0;i<num_image;i++) {
        double rx=poses[i*6+0],ry=poses[i*6+1],rz=poses[i*6+2];
        double tx=poses[i*6+3],ty=poses[i*6+4],tz=poses[i*6+5];
        float x_cam=-tx-rz*ty+ry*tz;
        float y_cam=rz*tx-ty-rx*tz;
        float z_cam=-ry*tx+rx*ty-tz;
        fwrite(&x_cam, sizeof(float), 1, out);
        fwrite(&y_cam, sizeof(float), 1, out);
        fwrite(&z_cam, sizeof(float), 1, out);
        
        double h=i/double(num_image);
        double B=fmax(fmin(1.5-4*fabs(h-1/4.0),1.0),0.0);
        double G=fmax(fmin(1.5-4*fabs(h-2/4.0),1.0),0.0);
        double R=fmax(fmin(1.5-4*fabs(h-3/4.0),1.0),0.0);
        unsigned char R2=(unsigned char)(R*255);
        unsigned char G2=(unsigned char)(G*255);
        unsigned char B2=(unsigned char)(B*255);
        fwrite(&R2, sizeof(unsigned char), 1, out);
        fwrite(&G2, sizeof(unsigned char), 1, out);
        fwrite(&B2, sizeof(unsigned char), 1, out);
    }
    for(int i=0;i<num_feature;i++) {
        double u0_=features.at<float>(0,i);
        double v0_=features.at<float>(1,i);
        double du0_=u0_-cx;
        double dv0_=v0_-cy;
        double x0_=du0_/f;
        double y0_=dv0_/f;
        double r20_=x0_*x0_+y0_*y0_;
        double distort0_=1+k1*r20_+k2*r20_*r20_;
        double x0=x0_*distort0_;
        double y0=y0_*distort0_;
        
        float X=x0/inv_depths[i];
        float Y=y0/inv_depths[i];
        float Z=1.0/inv_depths[i];
        fwrite(&X, sizeof(float), 1, out);
        fwrite(&Y, sizeof(float), 1, out);
        fwrite(&Z, sizeof(float), 1, out);
        
        // take the RGB intensities of the closest pixel.
        int u0=int(u0_+0.5);
        int v0=int(v0_+0.5);
        unsigned char R=imageData[v0*image_width*3+u0*3+2];
        unsigned char G=imageData[v0*image_width*3+u0*3+1];
        unsigned char B=imageData[v0*image_width*3+u0*3+0];
        fwrite(&R, sizeof(unsigned char), 1, out);
        fwrite(&G, sizeof(unsigned char), 1, out);
        fwrite(&B, sizeof(unsigned char), 1, out);
    }
    fclose(out);
}

void DfUSMC::UndistortImages() {
    // find 'f_new' to keep every pixel
    double min_x=0, max_x=0;
    double min_y=0, max_y=0;
    
    for(int i=0;i<image_height;i++){
        for(int j=0;j<image_width;j++){
            double xij=(j-cx)/f;
            double yij=(i-cy)/f;
            double r2ij=xij*xij+yij*yij;
            double rad=1+k1*r2ij+k2*r2ij*r2ij;
            double xij_=xij*rad;
            double yij_=yij*rad;
            if(xij_<min_x) min_x=xij_;
            if(xij_>max_x) max_x=xij_;
            if(yij_<min_y) min_y=yij_;
            if(yij_>max_y) max_y=yij_;
        }
    }
    double f_min_x=-cx/min_x;
    double f_max_x=(image_width-cx)/max_x;
    double f_min_y=-cy/min_y;
    double f_max_y=(image_height-cy)/max_y;
    double tempx=(f_min_x<f_max_x)?f_min_x:f_max_x;
    double tempy=(f_min_y<f_max_y)?f_min_y:f_max_y;
    f_new=(tempx<tempy)?tempx:tempy;
    
    // meshgrid
    float *x_=new float[image_height*image_width];
    float *y_=new float[image_height*image_width];
    for(int i=0;i<image_height;i++) {
        for(int j=0;j<image_width;j++) {
            x_[i*image_width+j]=(j-cx)/f_new;
            y_[i*image_width+j]=(i-cy)/f_new;
        }
    }
    Mat x=Mat(image_height,image_width,CV_32F,x_);
    Mat y=Mat(image_height,image_width,CV_32F,y_);
    Mat x_bak=x.clone();
    Mat y_bak=y.clone();
    
    // iteratively find the inverse mapping
    for(int i=0;i<10;i++) {
        Mat x1=x.clone();
        Mat y1=y.clone();
        Mat x2=x1.mul(x1);
        Mat y2=y1.mul(y1);
        Mat x3=x2.mul(x1);
        Mat y3=y2.mul(y1);
        Mat r2=x2+y2;
        Mat r4=r2.mul(r2);
        Mat r2x=2*x1;
        Mat r2y=2*y1;
        Mat r4x=(4*x3)+(4*x1.mul(y2));
        Mat r4y=(4*x2.mul(y1))+(4*y3);
        Mat dist=1.0+(k1*r2)+(k2*r4);
        Mat x__=x1.mul(dist);
        Mat y__=y1.mul(dist);
        Mat rx=x_bak-x__;
        Mat ry=y_bak-y__;
        Mat rxx=-1-(k1*r2)-(k2*r4)-(k1*x1.mul(r2x))-(k2*x1.mul(r4x));
        Mat rxy=-(k1*x1.mul(r2y))-(k2*x1.mul(r4y));
        Mat ryx=-(k1*y1.mul(r2x))-(k2*y1.mul(r4x));
        Mat ryy=-1-(k1*r2)-(k2*r4)-(k1*y1.mul(r2y))-(k2*y1.mul(r4y));
        Mat a=rxx.mul(rxx)+ryx.mul(ryx);
        Mat b=rxx.mul(rxy)+ryx.mul(ryy);
        Mat c=b;
        Mat d=rxy.mul(rxy)+ryy.mul(ryy);
        Mat det=(a.mul(d))-(b.mul(c));
        Mat dx=-(1/det).mul( (d.mul(rxx)-b.mul(rxy)).mul(rx) + (d.mul(ryx)-b.mul(ryy)).mul(ry) );
        Mat dy=-(1/det).mul( (-c.mul(rxx)+a.mul(rxy)).mul(rx) + (-c.mul(ryx)+a.mul(ryy)).mul(ry) );
        x=x+dx;
        y=y+dy;
    }
    Mat udx=(f*x+cx);
    ud_mapx=udx.clone();
    Mat udy=(f*y+cy);
    ud_mapy=udy.clone();
}

void DfUSMC::DenseMatching(double scale, int num_label, double lambda, double sigma){
    double _f=f_new*scale;
    double _cx=cx*scale;
    double _cy=cy*scale;
    
    Mat temp;
    resize(images[0],temp,Size(),scale,scale);
    int row=temp.rows;
    int col=temp.cols;
    
    // min. depth
    double max_inv_depth=0;
    for(int i=0;i<num_feature;i++) {
        if(max_inv_depth<inv_depths[i])
            max_inv_depth=inv_depths[i];
    }
    
    // set the close end of the depth range to be closer than the min. depth
    double min_w=0.0; // starting inv.depth; 0 means infinity
    double max_w=max_inv_depth*1.1; // you can adjust this scaling factor for depth margin.
    double dw=(max_w-min_w)/(num_label-1);
    
    // 3x3 box filter
    float window_[9]={1/9.0,1/9.0,1/9.0,1/9.0,1/9.0,1/9.0,1/9.0,1/9.0,1/9.0};
    Mat window=Mat(3,3,CV_32FC1,window_);
    
    Mat *rimg=new Mat[num_image];
    Mat *gimg=new Mat[num_image];
    Mat *bimg=new Mat[num_image];
    Mat *gximg=new Mat[num_image];
    Mat *gyimg=new Mat[num_image];
    
    for(int i=0;i<num_image;i++) {
        rimg[i]=Mat(row,col,CV_32FC1);
        gimg[i]=Mat(row,col,CV_32FC1);
        bimg[i]=Mat(row,col,CV_32FC1);
        
        Mat img_dist;
        images[i].convertTo(img_dist,CV_32FC3,1/255.0);
        Mat img_undist;
        remap(img_dist, img_undist, ud_mapx, ud_mapy, CV_INTER_CUBIC);
        Mat _img_undist;
        resize(img_undist,_img_undist,Size(),scale,scale);
        imshow("undistorted images",_img_undist);
        waitKey(33);
        
        vector<Mat> channels(3);
        split(_img_undist, channels);
        channels[2].copyTo(rimg[i]);
        channels[1].copyTo(gimg[i]);
        channels[0].copyTo(bimg[i]);
        
        Mat gray_undist;
        cvtColor(_img_undist,gray_undist,COLOR_BGR2GRAY);
        Sobel(gray_undist,gximg[i],gray_undist.depth(),1,0,1,1/2.0);
        Sobel(gray_undist,gyimg[i],gray_undist.depth(),0,1,1,1/2.0);
    }
    
    Mat u0=Mat(row,col,CV_32FC1);
    Mat v0=Mat(row,col,CV_32FC1);
    Mat x0=Mat(row,col,CV_32FC1);
    Mat y0=Mat(row,col,CV_32FC1);
    
    // meshgrid
    for(int i=0;i<row;i++){
        for(int j=0;j<col;j++){
            u0.at<float>(i,j)=j;
            v0.at<float>(i,j)=i;
            x0.at<float>(i,j)=j/_f - _cx/_f;
            y0.at<float>(i,j)=i/_f - _cy/_f;
        }
    }
    // compute the d->u mapping for distorting the final depth map.
    Mat xd=Mat(image_height, image_width, CV_32F);
    Mat yd=Mat(image_height, image_width, CV_32F);
    for(int i=0;i<image_height;i++) {
        for(int j=0;j<image_width;j++) {
            xd.at<float>(i,j)=(j-cx)/f;
            yd.at<float>(i,j)=(i-cy)/f;
        }
    }
    Mat r2d=xd.mul(xd)+yd.mul(yd);
    Mat rad=1+k1*r2d+k2*r2d.mul(r2d);
    Mat xd_=xd.mul(rad);
    Mat yd_=yd.mul(rad);
    
    du_mapx=_f*xd_+_cx;
    du_mapy=_f*yd_+_cy;
    
    Mat *rps=new Mat[num_image];
    Mat *gps=new Mat[num_image];
    Mat *bps=new Mat[num_image];
    Mat *gxps=new Mat[num_image];
    Mat *gyps=new Mat[num_image];
    
    float *confidence=new float[row*col];
    float *cost=new float[row*col];
    float *minconfidence=new float[row*col];
    float *mincost=new float[row*col];
    int *mincostlabel=new int[row*col];
    
    for(int i=0;i<row*col;i++) {
        mincost[i]=9999999;
        mincostlabel[i]=0;
    }
    
    for(int l=0;l<num_label;l++) {
        cout << "Plane Sweeping: " << l << " / " << num_label << endl;
        
        float w = min_w + float(l)*dw;
        for(int n=0;n<num_image;n++) {
            rps[n]=Mat(row,col,CV_32FC1);
            gps[n]=Mat(row,col,CV_32FC1);
            bps[n]=Mat(row,col,CV_32FC1);
            gxps[n]=Mat(row,col,CV_32FC1);
            gyps[n]=Mat(row,col,CV_32FC1);
            
            float h00=1,			h01=-poses[n*6+2],	h02=poses[n*6+1]+poses[n*6+3]*w;
            float h10=poses[n*6+2], h11=1,				h12=-poses[n*6+0]+poses[n*6+4]*w;
            float h20=-poses[n*6+1],h21=poses[n*6+0],	h22=1+poses[n*6+5]*w;
            
            Mat x1=h00*x0 + h01*y0 + h02;
            Mat y1=h10*x0 + h11*y0 + h12;
            Mat z1=h20*x0 + h21*y0 + h22;
            x1=x1.mul(1/z1);
            y1=y1.mul(1/z1);
            
            Mat u1 = _f*x1 + _cx;
            Mat v1 = _f*y1 + _cy;
            
            Mat rwarp, gwarp, bwarp, gxwarp, gywarp;
            remap(rimg[n], rwarp, u1, v1, INTER_CUBIC); //INTER_LINEAR
            remap(gimg[n], gwarp, u1, v1, INTER_CUBIC);
            remap(bimg[n], bwarp, u1, v1, INTER_CUBIC);
            remap(gximg[n], gxwarp, u1, v1, INTER_CUBIC);
            remap(gyimg[n], gywarp, u1, v1, INTER_CUBIC);
            
            //rwarp=min(max(rwarp,0.0),1.0);
            //gwarp=min(max(gwarp,0.0),1.0);
            //bwarp=min(max(bwarp,0.0),1.0);
            
            rwarp.copyTo(rps[n]);
            gwarp.copyTo(gps[n]);
            bwarp.copyTo(bps[n]);
            gxwarp.copyTo(gxps[n]);
            gywarp.copyTo(gyps[n]);
        }
        
        Mat rmean=Mat::zeros(row,col,CV_32FC1);
        Mat gmean=Mat::zeros(row,col,CV_32FC1);
        Mat bmean=Mat::zeros(row,col,CV_32FC1);
        Mat gxmean=Mat::zeros(row,col,CV_32FC1);
        Mat gymean=Mat::zeros(row,col,CV_32FC1);
        
        for(int n=0;n<num_image;n++) {
            rmean=rmean+rps[n];
            gmean=gmean+gps[n];
            bmean=bmean+bps[n];
            gxmean=gxmean+gxps[n];
            gymean=gymean+gyps[n];
        }
        rmean=rmean/num_image;
        gmean=gmean/num_image;
        bmean=bmean/num_image;
        gxmean=gxmean/num_image;
        gymean=gymean/num_image;
        
        Mat rvar=Mat::zeros(row,col,CV_32FC1);
        Mat gvar=Mat::zeros(row,col,CV_32FC1);
        Mat bvar=Mat::zeros(row,col,CV_32FC1);
        Mat gxvar=Mat::zeros(row,col,CV_32FC1);
        Mat gyvar=Mat::zeros(row,col,CV_32FC1);
        
        for(int n=0;n<num_image;n++) {
            Mat rdiff=rps[n]-rmean;
            Mat gdiff=gps[n]-gmean;
            Mat bdiff=bps[n]-bmean;
            Mat gxdiff=gxps[n]-gxmean;
            Mat gydiff=gyps[n]-gymean;
            
            rvar=rvar+rdiff.mul(rdiff);
            gvar=gvar+gdiff.mul(gdiff);
            bvar=bvar+bdiff.mul(bdiff);
            gxvar=gxvar+gxdiff.mul(gxdiff);
            gyvar=gyvar+gydiff.mul(gydiff);
        }
        rvar=rvar/num_image;
        gvar=gvar/num_image;
        bvar=bvar/num_image;
        gxvar=gxvar/num_image;
        gyvar=gyvar/num_image;
        
        rmean=rmean+1e-16; gmean=gmean+1e-16; bmean=bmean+1e-16;
        
        Mat rgbcost=rvar + gvar + bvar;
        Mat _confidence = Mat(row,col,CV_32F,confidence);
        _confidence=1 - (rvar.mul(1/rmean) + gvar.mul(1/gmean) + bvar.mul(1/bmean));
        
        Mat gradcost=gxvar + gyvar;
        Mat _cost = Mat(row,col,CV_32F,cost);
        _cost = rgbcost + lambda*gradcost;
        
        filter2D(_cost,_cost,_cost.depth(),window);
        
        for(int i=0;i<row*col;i++) {
            if ( mincost[i]>cost[i] ) {
                mincost[i]=cost[i];
                mincostlabel[i]=l;
                minconfidence[i]=confidence[i];
            }
        }
    }
    
    depthmapWTA=Mat(row,col,CV_8UC1);
    for(int i=0;i<row;i++) {
        for(int j=0;j<col;j++) {
            depthmapWTA.at<unsigned char>(i,j)=(unsigned char)(255*mincostlabel[i*col+j]/float(num_label-1));
        }
    }
    
    Mat white=Mat::ones(image_height,image_width,CV_32F);
    Mat white_undist;
    remap(white, white_undist, ud_mapx, ud_mapy, CV_INTER_CUBIC);
    Mat _white_undist;
    resize(white_undist, _white_undist, Size(), scale, scale);
    bool *mask=new bool[row*col];
    int cnt_valid=0;
    for(int i=0;i<row;i++) {
        for(int j=0;j<col;j++) {
            if(_white_undist.at<float>(i,j)>0.01) {
                mask[i*col+j]=true;
                cnt_valid++;
            }
        }
    }
    
    Mat _minconfidence=Mat(row,col,CV_32F,minconfidence);
    confidencemap=_minconfidence.clone();
    
    // statistical filtering
    _minconfidence=Mat(1,row*col,CV_32F,minconfidence);
    cv::sort(_minconfidence, _minconfidence, SORT_ASCENDING);
    float thresh=minconfidence[row*col-int(cnt_valid*0.90)]; // 10 percent
    
    // or fixed threshold
    //float thresh=0.99; // 5 percent
    
    cout<< "confidence threshold: "<<thresh << endl;
    
    double scalar=255.0/(num_label-1);
    double **disparity=new double*[row];
    //double **disparity_gt=new double*[row];
    depthmapFiltered=depthmapWTA.clone();
    for(int i=0;i<row;i++) {
        disparity[i]=new double[col];
        //disparity_gt[i]=new double[col];
        for(int j=0;j<col;j++) {
            if(mask[i*col+j]==false||confidencemap.at<float>(i,j)<thresh) {
                depthmapFiltered.at<unsigned char>(i,j)=(unsigned char)0;
            }
            //disparity_gt[i][j]=double(depthmapFiltered.at<unsigned char>(i,j));
            disparity[i][j]=(double)(depthmapFiltered.at<unsigned char>(i,j))/scalar;
            if(disparity[i][j]<=2)
            {
                disparity[i][j]=0;
            }
        }
    }
    
    // Refinement: Qingxiong Yang's method, A Non-Local Cost Aggregation Method for Stereo Matching, CVPR 2012

    qx_tree_upsampling m_tree_upsampling;//upsampling class
    m_tree_upsampling.init(row,col,num_label-1,sigma);
    
    Mat img0_undist;
    remap(images[0], img0_undist, ud_mapx, ud_mapy, CV_INTER_CUBIC);
    unsigned char ***guidance_img_=new unsigned char**[1];
    guidance_img_[0]=new unsigned char*[1];
    guidance_img_[0][0]=new unsigned char[row*col*3];
    Mat guidance_img=Mat(row,col,CV_8UC3,guidance_img_[0][0]);
    resize(img0_undist,guidance_img,Size(),scale,scale);
    imshow("guidance_img",guidance_img);
    waitKey(1);
    m_tree_upsampling.build_minimum_spanning_tree(guidance_img_);
    m_tree_upsampling.disparity_upsampling(disparity);
    
    depthmapRefined=depthmapFiltered.clone();
    for(int i=0;i<row;i++) {
        for(int j=0;j<col;j++) {
            depthmapRefined.at<unsigned char>(i,j)=(unsigned char)(disparity[i][j]*scalar+0.5);
        }
    }
}

void DfUSMC::SaveDepthmapWTA(char *fullpath, bool bwdmapping) {
    Mat temp;
    if(bwdmapping) remap(depthmapWTA, temp, du_mapx, du_mapy, CV_INTER_CUBIC);
    else temp=depthmapWTA;
    
    imwrite(fullpath, temp);
    imshow("depthmapWTA",temp);
    waitKey(1);
}

void DfUSMC::SaveDepthmapFiltered(char *fullpath, bool bwdmapping) {
    Mat temp;
    if(bwdmapping) remap(depthmapFiltered, temp, du_mapx, du_mapy, CV_INTER_CUBIC);
    else temp=depthmapFiltered;
    
    imwrite(fullpath, temp);
    imshow("depthmapFiltered",temp);
    waitKey(1);
}

void DfUSMC::SaveDepthmapRefined(char *fullpath, bool bwdmapping) {
    Mat temp;
    if(bwdmapping) remap(depthmapRefined, temp, du_mapx, du_mapy, CV_INTER_CUBIC);
    else temp=depthmapRefined;
    
    imwrite(fullpath, temp);
    imshow("depthmapRefined",temp);
    waitKey(1);
}
