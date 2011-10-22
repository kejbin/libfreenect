/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include <libfreenect.hpp>
#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <cmath>
#include <vector>
#include "pcl/common/common_headers.h"
#include "pcl/features/normal_3d.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl/visualization/cloud_viewer.h"
#include "boost/lexical_cast.hpp"
#include "pcl/filters/voxel_grid.h"

class Mutex {
public:
	Mutex() {
		pthread_mutex_init( &m_mutex, NULL );
	}
	void lock() {
		pthread_mutex_lock( &m_mutex );
	}
	void unlock() {
		pthread_mutex_unlock( &m_mutex );
	}

	class ScopedLock
	{
		Mutex & _mutex;
	public:
		ScopedLock(Mutex & mutex)
			: _mutex(mutex)
		{
			_mutex.lock();
		}
		~ScopedLock()
		{
			_mutex.unlock();
		}
	};
private:
	pthread_mutex_t m_mutex;
};

/* thanks to Yoda---- from IRC */
class MyFreenectDevice : public Freenect::FreenectDevice {
public:
	MyFreenectDevice(freenect_context *_ctx, int _index)
		: Freenect::FreenectDevice(_ctx, _index), depth(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED).bytes),m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes), m_new_rgb_frame(false), m_new_depth_frame(false)
	{
		
	}
	//~MyFreenectDevice(){}
	// Do not call directly even in child
	void VideoCallback(void* _rgb, uint32_t timestamp) {
		Mutex::ScopedLock lock(m_rgb_mutex);
		uint8_t* rgb = static_cast<uint8_t*>(_rgb);
		std::copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
		m_new_rgb_frame = true;
	};
	// Do not call directly even in child
	void DepthCallback(void* _depth, uint32_t timestamp) {
		Mutex::ScopedLock lock(m_depth_mutex);
		depth.clear();
		uint16_t* call_depth = static_cast<uint16_t*>(_depth);
		for (size_t i = 0; i < 640*480 ; i++) {
			depth.push_back(call_depth[i]);
		}
		m_new_depth_frame = true;
	}
	bool getRGB(std::vector<uint8_t> &buffer) {
		Mutex::ScopedLock lock(m_rgb_mutex);
		if (!m_new_rgb_frame)
			return false;
		buffer.swap(m_buffer_video);
		m_new_rgb_frame = false;
		return true;
	}

	bool getDepth(std::vector<uint16_t> &buffer) {
		Mutex::ScopedLock lock(m_depth_mutex);
		if (!m_new_depth_frame)
			return false;
		buffer.swap(depth);
		m_new_depth_frame = false;
		return true;
	}

private:
	std::vector<uint16_t> depth;
	std::vector<uint8_t> m_buffer_video;
	Mutex m_rgb_mutex;
	Mutex m_depth_mutex;
	bool m_new_rgb_frame;
	bool m_new_depth_frame;
};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr bgcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxcloud (new pcl::PointCloud<pcl::PointXYZRGB>);

bool BackgroundSub = false;
bool hasBackground = false;
bool Voxelize = false;
unsigned int voxelsize = 1; //in mm
unsigned int cloud_id = 0;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::CloudViewer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::CloudViewer> *> (viewer_void);
  if (event.getKeySym () == "c" && event.keyDown ())
  {
    std::cout << "c was pressed => capturing a pointcloud" << std::endl;
    std::string filename = "KinectCap";
    filename.append(boost::lexical_cast<std::string>(cloud_id));
    filename.append(".pcd");
    if (Voxelize)
    	pcl::io::savePCDFileASCII (filename, *voxcloud);
    else
    	pcl::io::savePCDFileASCII (filename, *cloud);
    cloud_id++;
  }

  if (event.getKeySym () == "b" && event.keyDown ())
  {
  	std::cout << "b was pressed" << std::endl;
  	if (BackgroundSub == false) 
  	{
  		//Start background subtraction
  		if (hasBackground == false) 
  		{
  			//Copy over the current cloud as a BG cloud.
  			pcl::copyPointCloud(*cloud, *bgcloud);
  			hasBackground = true;
  		}
  		BackgroundSub = true;
  	}
  	else 
  	{
  		//Stop Background Subtraction
  		BackgroundSub = false;
  	}
  }
  
  if (event.getKeySym () == "v" && event.keyDown ())
  {
  	std::cout << "v was pressed" << std::endl;
  	Voxelize = !Voxelize;
  }

}

Freenect::Freenect freenect;
MyFreenectDevice* device;
freenect_video_format requested_format(FREENECT_VIDEO_RGB);

double freenect_angle(0);
int got_frames(0),window(0);
int g_argc;
char **g_argv;

int user_data = 0;


int run() {
	// Fill in the cloud data
	cloud->width    = 640;
	cloud->height   = 480;
	cloud->is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);
	
	boost::shared_ptr<pcl::visualization::CloudViewer> viewer (new pcl::visualization::CloudViewer ("OK-PCL Corral"));
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    
    //blocks until the cloud is actually rendered
    viewer->showCloud (cloud, "Kinect Cloud");
    
    static std::vector<uint16_t> depth(640*480);
	static std::vector<uint8_t> rgb(640*480*4);
	
	//Voxelizer Setup
	pcl::VoxelGrid<pcl::PointXYZRGB> vox;
	
	
    while (!viewer->wasStopped ())
    {

	// using getTiltDegs() in a closed loop is unstable
	/*if(device->getState().m_code == TILT_STATUS_STOPPED){
		freenect_angle = device->getState().getTiltDegs();
	}*/
	device->updateState();
	device->getDepth(depth);
	device->getRGB(rgb);
	
    size_t i = 0;
    size_t cinput = 0;
    for (size_t v=0 ; v<480 ; v++)
    {
    	for ( size_t u=0 ; u<640 ; u++, i++)
        {
        	//pcl::PointXYZRGB result;
        	int iRealDepth = depth[i];
			//printf("fRealDepth = %f\n",fRealDepth);
			//fflush(stdout);
			double x = NULL;
			double y = NULL;
			freenect_camera_to_world(device->getDevice(), u, v, iRealDepth, &x, &y);
			cloud->points[i].x  = x;//1000.0;
			cloud->points[i].y  = y;//1000.0;
            cloud->points[i].z = iRealDepth;//1000.0;
            cloud->points[i].r = rgb[i*3];
            cloud->points[i].g = rgb[(i*3)+1];
            cloud->points[i].b = rgb[(i*3)+2];
            //cloud->points[i] = result;
            //printf("x,y,z = %f,%f,%f\n",x,y,fRealDepth);
        }
	}
	
	if (Voxelize) {
		vox.setInputCloud (cloud);
  		vox.setLeafSize (10.0f, 10.0f, 10.0f);
  		vox.filter (*voxcloud);
  		viewer->showCloud (voxcloud, "Kinect Cloud");
  	}
  	else
	    viewer->showCloud (cloud, "Kinect Cloud");
	
	printf("\r demanded tilt angle: %+4.2f device tilt angle: %+4.2f total points added: %u", freenect_angle, device->getState().getTiltDegs(), i);
	fflush(stdout);
    
    }
    
	return 0;
}

int main(int argc, char **argv) {
	device = &freenect.createDevice<MyFreenectDevice>(0);
	device->startVideo();
	device->startDepth();
	//device->setDepthFormat(FREENECT_DEPTH_REGISTERED);
	int runResult = run();
	device->stopVideo();
	device->stopDepth();
	return runResult;
}
