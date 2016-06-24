# DfUSMC
(DfUSMC: Depth from Uncalibrated Small Motion Clip)

Source code and dataset for the paper:

_H. Ha, S. Im, J. Park, H.-G. Jeon and I.S. Kweon_, [**High-quality Depth from Uncalibrated Small Motion Clip**](https://drive.google.com/file/d/0B7-4XlDU1W6rbUtlRW92b01YakU/view?usp=sharing), [**CVPR 2016**](http://cvpr2016.thecvf.com/)

- [**Project page**](http://sites.google.com/site/hyowoncv/ha_cvpr16)

## Dependencies

- [**OpenCV**](http://opencv.org)
- [**Ceres solver**](http://ceres-solver.org)

## How to Run

1. Put your clip in "Dataset" folder
2. Open main.cpp
3. Change "data_name" and "video_format" accordingly.
4. Run

```
cmake .
make
```

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
