/******************************************************************************************
\Author:	Qingxiong Yang
\Function:	Disparity map upsampling
\Reference:	Qingxiong Yang, Stereo Matching Using Tree Filtering, PAMI 2014.
*******************************************************************************************/
#ifndef QX_NONLOCAL_COST_AGGREGATION_H
#define QX_NONLOCAL_COST_AGGREGATION_H
#include "qx_tree_filter.h"

class qx_tree_upsampling
{
public:
    qx_tree_upsampling();
    ~qx_tree_upsampling();
    void clean();
	int init(int h,int w,int nr_plane,
		double sigma_range=0.1
		);
	int build_minimum_spanning_tree(unsigned char***guidance_image);
	int disparity_upsampling(double**disparity);//Don't use unsigned char disparity values
private:
	qx_tree_filter m_tf;
	int	m_h,m_w,m_nr_plane; double m_sigma_range;

	unsigned char**m_disparity,**m_disparity_mf;
	double***m_cost_vol,***m_cost_vol_temp;
};
#endif