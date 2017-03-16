# DfUSMC
(DfUSMC: Depth from Uncalibrated Small Motion Clip)

Source code and datasets for the paper:

_H. Ha, S. Im, J. Park, H.-G. Jeon and I.S. Kweon_, [**High-quality Depth from Uncalibrated Small Motion Clip**](https://drive.google.com/file/d/0B7-4XlDU1W6rbUtlRW92b01YakU/view?usp=sharing), [**CVPR 2016**](http://cvpr2016.thecvf.com/)

- [**Project page**](http://sites.google.com/site/hyowoncv/ha_cvpr16)

## Dependencies

- [**OpenCV 2.4.12**](http://opencv.org)
- [**Ceres solver**](http://ceres-solver.org)

## How to Run

1. Put your small motion clip in "Dataset" folder
2. Run (with sudo)

```
cmake .
make
./DfUSMC data_name video_extension
(for example, ./DfUSMC Bikes avi)
```

## Important Information

**Frame selection**

The current implementation uses only the first 30 frames of your video clip. If you want to try with a different number of images or different sampling rate, please modify the "LoadSmallMotionClip" function.

**Dense matching step**

We implemented the function "DenseMatching" to receive a scale for image downsampling and the number of labels for your convenience in testing. (Default: 0.5 scale and 64 labels for quick tests, but please remind that 1.0 and 256 were used in the paper)

For the depth refinement, we utilized a tree-based depth upsampling approach [1,2].

## Authors

- [Hyowon Ha](http://sites.google.com/site/hyowoncv)
- [Sunghoon Im](http://sites.google.com/site/shimrcv)
- [Jaesik Park](http://sites.google.com/site/jsparkcv)
- [Hae-Gon Jeon](http://sites.google.com/site/hgjeoncv)
- [In So Kweon](http://rcv.kaist.ac.kr)

&copy; 2016 Hyowon Ha, Korea Advanced Institute of Science and Technology (KAIST)


**IMPORTANT**: If you use this software please cite the following in any resulting publication:
```
@inproceedings{ha2016high,
  title={High-quality Depth from Uncalibrated Small Motion Clip},
  author={Ha, Hyowon and Im, Sunghoon and Park, Jaesik and Jeon, Hae-Gon and Kweon, In So},
  booktitle={Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR)},
  year={2016}
}
```
## References

1. Yang, Qingxiong. "Stereo matching using tree filtering." IEEE transactions on pattern analysis and machine intelligence 37.4 (2015): 834-846.
2. Yang, Qingxiong. "A non-local cost aggregation method for stereo matching." Computer Vision and Pattern Recognition (CVPR), 2012 IEEE Conference on. IEEE, 2012.

