/*$Id: qx_tree_upsampling.cpp,v 1.1 2007/02/16 04:11:12 liiton Exp $*/
#include "qx_basic.h"
#include "qx_tree_upsampling.h"
qx_tree_upsampling::qx_tree_upsampling()
{
	m_disparity=NULL;
	m_disparity_mf=NULL;
	m_cost_vol=NULL;
	m_cost_vol_temp=NULL;
}
qx_tree_upsampling::~qx_tree_upsampling()
{
    clean();
}
void qx_tree_upsampling::clean()
{
	qx_freeu(m_disparity); m_disparity=NULL;
	qx_freeu(m_disparity_mf); m_disparity_mf=NULL;
	qx_freed_3(m_cost_vol); m_cost_vol=NULL;
	qx_freed_3(m_cost_vol_temp); m_cost_vol_temp=NULL;
}
int qx_tree_upsampling::init(int h,int w,int nr_plane,double sigma_range)
{
	clean();
	m_h=h; m_w=w; m_nr_plane=nr_plane; m_sigma_range=sigma_range;
	m_disparity=qx_allocu(m_h,m_w);
	m_disparity_mf=qx_allocu(m_h,m_w);
	m_cost_vol=qx_allocd_3(m_h,m_w,m_nr_plane);
	m_cost_vol_temp=qx_allocd_3(m_h,m_w,m_nr_plane);

	m_tf.init(m_h,m_w,3,m_sigma_range,4);
    return(0);
}
int qx_tree_upsampling::build_minimum_spanning_tree(unsigned char***guidance_image)
{
	m_tf.build_tree(guidance_image[0][0]);
    return(0);
}
int qx_tree_upsampling::disparity_upsampling(double**disparity)
{
	image_zero(m_cost_vol,m_h,m_w,m_nr_plane);//compute cost volume
	for(int y=0;y<m_h;y++) for(int x=0;x<m_w;x++)
	{
		if(disparity[y][x]>0)
		{
			for(int d=0;d<m_nr_plane;d++) 
				m_cost_vol[y][x][d]=abs(disparity[y][x]-d);
		}
	}

	m_tf.update_table(m_sigma_range/2);//set parameter
	m_tf.filter(m_cost_vol[0][0],m_cost_vol_temp[0][0],m_nr_plane);//tree filtering on the cost volume
	
	for(int y=0;y<m_h;y++) for(int x=0;x<m_w;x++)//winner-take-all
	{
		int d;
		vec_min_pos(d,m_cost_vol[y][x],m_nr_plane);
		m_disparity[y][x]=d;
	}
	
	int radius=2;//median filter
	ctmf(m_disparity[0],m_disparity_mf[0],m_w,m_h,m_w,m_w,radius,1,m_h*m_w);
	for(int y=0;y<m_h;y++) for(int x=0;x<m_w;x++) disparity[y][x]=m_disparity_mf[y][x];

    return(0);
}
