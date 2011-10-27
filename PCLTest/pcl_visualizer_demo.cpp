/* \author Geoffrey Biggs */


#include <iostream>
#include <vector>
#include <ctime>

#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include "pcl/common/common_headers.h"
#include "pcl/features/normal_3d.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/console/parse.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include "pcl/octree/octree.h"


// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");  //9 for voxels
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 25, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
                                     cloud->points[cloud->size() - 1], "line");
  viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  viewer->addPlane (coeffs, "plane");
  coeffs.values.clear ();
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (5.0);
  viewer->addCone (coeffs, "cone");

  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->addCoordinateSystem (1.0);

  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

  return (viewer);
}


unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
  viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

  return (viewer);
}

int whichBG;
bool switchBG;
void kEO_BG (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "s" && event.keyDown ())
  {
    std::cout << "s was pressed => flipping between fg, bg and full" << std::endl;
    if (whichBG == 0) {
    	whichBG = 1;
    } else if (whichBG == 1) {
    	whichBG = 2;
    } else if (whichBG == 2) {
    	whichBG = 0;
    }
    switchBG = true;
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> bgTestVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr fullcloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr fgcloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr bgcloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(fullcloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (fullcloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");  //9 for voxels
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->registerKeyboardCallback (kEO_BG, (void*)&viewer);
  whichBG = 0;
  switchBG = false;
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> octVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr fullcloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr fgcloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr bgcloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(fullcloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (fullcloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");  //9 for voxels
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->registerKeyboardCallback (kEO_BG, (void*)&viewer);
  whichBG = 0;
  switchBG = false;
  return (viewer);
}


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  std::string cloud_name = argv[1];
  std::string bg_name = argv[2];
  
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  bool simple(false), rgb(false), custom_c(false), normals(false),
    shapes(false), viewports(false), interaction_customization(false), mlsdemo(false), bgdemo(false), octdemo(false);
  if (pcl::console::find_argument (argc, argv, "-s") >= 0)
  {
    simple = true;
    std::cout << "Simple visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-c") >= 0)
  {
    custom_c = true;
    std::cout << "Custom colour visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-r") >= 0)
  {
    rgb = true;
    std::cout << "RGB colour visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-n") >= 0)
  {
    normals = true;
    std::cout << "Normals visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-a") >= 0)
  {
    shapes = true;
    std::cout << "Shapes visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-v") >= 0)
  {
    viewports = true;
    std::cout << "Viewports example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-i") >= 0)
  {
    interaction_customization = true;
    std::cout << "Interaction Customization example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-m") >= 0)
  {
    mlsdemo = true;
    std::cout << "MLS example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-b") >= 0)
  {
    bgdemo = true;
    std::cout << "BG example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-o") >= 0)
  {
    octdemo = true;
    std::cout << "Oct example\n";
  }
  else
  {
    printUsage (argv[0]);
    return 0;
  }

  // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bg_points (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr fg_points (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr true_bg_points (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  std::cout << "Genarating example point clouds.\n\n";
  // We're going to make an ellipse extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0)
    {
      pcl::PointXYZ basic_point;
      basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
      basic_point.y = sinf (pcl::deg2rad(angle));
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (cloud_name, *point_cloud_ptr) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file.\n");
    return (-1);
  }
  
  if (bgdemo || octdemo) {
  	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (bg_name, *bg_points) == -1) //* load the file
  	{
    	PCL_ERROR ("Couldn't read file.\n");
    	return (-1);
  	}
  }
  
  //Filter point cloud!
  //Voxelization
  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  vox.setInputCloud (point_cloud_ptr);
  vox.setLeafSize (5.0f, 5.0f, 5.0f);
  vox.filter (*cloud_filtered);
  std::cerr << "PointCloud before filtering: " << point_cloud_ptr->width * point_cloud_ptr->height << " data points (" << pcl::getFieldsList (*point_cloud_ptr) << ").";
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  //Voxelization
  pcl::VoxelGrid<pcl::PointXYZRGB> voxdisp;
  voxdisp.setInputCloud (point_cloud_ptr);
  voxdisp.setLeafSize (1.0f, 1.0f, 1.0f);
  //voxdisp.setSaveLeafLayout(true);
  voxdisp.filter (*cloud_filtered2);
  std::cerr << "PointCloud before filtering: " << point_cloud_ptr->width * point_cloud_ptr->height << " data points (" << pcl::getFieldsList (*point_cloud_ptr) << ").";
  std::cerr << "PointCloud after filtering: " << cloud_filtered2->width * cloud_filtered2->height << " data points (" << pcl::getFieldsList (*cloud_filtered2) << ").";  
  
  //Outlier Removal
  /*pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (point_cloud_ptr);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);*/
  
  if(mlsdemo)
  {
  //MLS Smoothing
  // Create a KD-Tree
  pcl::KdTree<pcl::PointXYZRGB>::Ptr treemls (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
  treemls->setInputCloud (cloud_filtered2);

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::Normal> mls;

  // Optionally, a pointer to a cloud can be provided, to be set by MLS
  mls.setOutputNormals (mls_normals);

  // Set parameters
  mls.setInputCloud (cloud_filtered2);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (treemls);
  mls.setSearchRadius (30); //0.03

  // Reconstruct
  mls.reconstruct (*mls_points);
  }
  
  if(bgdemo)
  {
  	//BG Subtraction!
  	//Voxelization
  	/*pcl::VoxelGrid<pcl::PointXYZRGB> voxbg;
  	voxbg.setInputCloud (bg_points);
  	voxbg.setLeafSize (1.0f, 1.0f, 1.0f);
  	//voxbg.setSaveLeafLayout(true);
  	voxbg.filter (*bg_points);*/
  	
  	//Get the gridCoords...
  	//x and y from -3 m to 3 m, z to 0 m to 5 m
  	//Eigen::Vector3i minGrid = voxdisp.getGridCoordinates(-3000, -3000, 0);
  	//Eigen::Vector3i maxGrid = voxdisp.getGridCoordinates(3000, 3000, 5000);
  	int iFullIndex;
  	int iBGIndex;
  	float xdiff;
  	float ydiff;
  	float zdiff;
  	float maxdiff = 100.0;
  	Eigen::Vector3i curCentroid;
  	
  	//Get the key indexes, and compare, pushing non matching to a new cloud.
  	/*size_t cloudSize = cloud_filtered2->size();
  	if (cloudSize > bg_points->size()) {
  		printf("BG smaller than Main Cloud");
  		cloudSize = bg_points->size();
  	}*/
  	int i = 0;
  	for (int u = 0;u<640;u++) {
  		//curCentroid[0] = i;
  		for (int v = 0;v<480;v++, i++) {
	  		zdiff = abs((*bg_points)(u,v).z - (*point_cloud_ptr)(u,v).z);
	  		if (zdiff > maxdiff)
	  		{
	  			fg_points->push_back((*point_cloud_ptr)(u,v));
	  		}
	  		else
	  		{
	  			true_bg_points->push_back((*bg_points)(u,v));
	  		}
	  	}
	} 
	//Outlier Removal
  	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  	sor.setInputCloud (fg_points);
  	sor.setMeanK (25);
  	sor.setStddevMulThresh (2.0);
  	sor.filter (*fg_points);
  	sor.setInputCloud (true_bg_points);
  	sor.filter (*true_bg_points);
}	
	if(octdemo)
	{
		//BG Subtraction using Octrees!
		float resolution = 50.0;
		// Instantiate octree-based point cloud change detection class
		pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree (resolution);
		
		// Add points from background to octree
  		octree.setInputCloud (bg_points);
  		octree.addPointsFromInputCloud ();
  		std::cout << "Leaf count for BG: " << octree.getLeafCount() << std::endl;
  		
  		// Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  		octree.switchBuffers ();
  		
  		// Add points from the mixed data to octree
  		octree.setInputCloud (point_cloud_ptr);
  		octree.addPointsFromInputCloud ();
  		std::cout << "Leaf count for Full Image: " << octree.getLeafCount() << std::endl;
  		
  		std::vector<int> newPointIdxVector;
		
		// Get vector of point indices from octree voxels which did not exist in previous buffer
		octree.getPointIndicesFromNewVoxels (newPointIdxVector);
		
		for (size_t i = 0; i < newPointIdxVector.size(); ++i)
		{
			//std::cout << i << "," << newPointIdxVector[i] << "," << point_cloud_ptr->points[newPointIdxVector[i]].x << "," << point_cloud_ptr->points[newPointIdxVector[i]].y << "," << point_cloud_ptr->points[newPointIdxVector[i]].z << std::endl;
			fg_points->push_back(point_cloud_ptr->points[newPointIdxVector[i]]);
		}
		
		// Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  		octree.switchBuffers ();
		// Add points from background to octree
  		octree.setInputCloud (fg_points);
  		octree.addPointsFromInputCloud ();
		// Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  		octree.switchBuffers ();
		// Add points from background to octree
  		octree.setInputCloud (point_cloud_ptr);
  		octree.addPointsFromInputCloud ();

  		
  		// Get vector of point indices from octree voxels which did not exist in previous buffer
		octree.getPointIndicesFromNewVoxels (newPointIdxVector);
		
		for (size_t i = 0; i < newPointIdxVector.size(); ++i)
		{
			//std::cout << i << "," << newPointIdxVector[i] << "," << point_cloud_ptr->points[newPointIdxVector[i]].x << "," << point_cloud_ptr->points[newPointIdxVector[i]].y << "," << point_cloud_ptr->points[newPointIdxVector[i]].z << std::endl;
			true_bg_points->push_back(bg_points->points[newPointIdxVector[i]]);
		}
		
	}	

  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (cloud_filtered);
  pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  //ne.setRadiusSearch (0.05);
  //ne.compute (*cloud_normals1);

  // ---------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.1-----
  // ---------------------------------------------------------------
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (25.0);
  ne.compute (*cloud_normals2);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (simple)
  {
    viewer = simpleVis(basic_cloud_ptr);
  }
  else if (rgb)
  {
    viewer = rgbVis(point_cloud_ptr);
  }
  else if (custom_c)
  {
    viewer = customColourVis(basic_cloud_ptr);
  }
  else if (normals)
  {
    viewer = normalsVis(cloud_filtered, cloud_normals2);
  }
  else if (mlsdemo)
  {
    viewer = normalsVis(mls_points, mls_normals);
  }
  else if (shapes)
  {
    viewer = shapesVis(point_cloud_ptr);
  }
  else if (viewports)
  {
    viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
  }
  else if (interaction_customization)
  {
    viewer = interactionCustomizationVis();
  }
  else if (bgdemo)
  {
  	viewer = bgTestVis(point_cloud_ptr, fg_points, true_bg_points);
  }
  else if (octdemo)
  {
  	viewer = octVis(point_cloud_ptr, fg_points, true_bg_points);
  }	

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    if((bgdemo || octdemo) && switchBG) {
    	if (whichBG == 0) {
    		whichBG = 1;
    		viewer->updatePointCloud(point_cloud_ptr, "sample cloud");
    		printf("Full\n");
    	} else if (whichBG == 1) {
    		whichBG = 2;
    		viewer->updatePointCloud(true_bg_points, "sample cloud");
    		printf("BG\n");
    	} else if (whichBG == 2) {
    		whichBG = 0;
    		viewer->updatePointCloud(fg_points, "sample cloud");
    		printf("FG\n");
    	}
    	switchBG = false;
    }
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}