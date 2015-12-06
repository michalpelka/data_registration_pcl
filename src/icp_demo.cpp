/*
 * Software License Agreement (BSD License)
 *
 *  Data Registration Framework - Mobile Spatial Assistance System
 *  Copyright (c) 2014-2015, Institute of Mathematical Machines
 *  http://lider.zms.imm.org.pl/
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Institute of Mathematical Machines nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 */

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <vector>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/parse.h>

#include <pcl/registration/icp.h>

#include "dataFramework/data_model.hpp"

#include "dataFramework/viewerLog.hpp"
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <eigen3/Eigen/Dense>
template <typename PointSource, typename PointTarget, typename Scalar = float> class myIcp : public  pcl::IterativeClosestPoint <PointSource, PointTarget, Scalar >
{
public:
	pcl::CorrespondencesPtr getCorrespondeces()
	{
        return pcl::IterativeClosestPoint <PointSource, PointTarget, Scalar >::correspondences_;
	}
};

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
data_model inputXML;
data_model gtruthXML;

data_model outputXML;

std::vector<std::string> indices;
std::vector<std::string> pcNames;

bool useGroundTruth = false;
std::string currentFileName;
std::string outputXMLFn;


pcl::PointCloud<PointT>::Ptr currentlyRegisteredPc;
bool registration_accepted = false;
std::vector<std::string> msg;

bool isUseMetascan = false;
float  cummulativeTime =0;
double icp_CorrespondenceDistance = 0.3;
double icp_RANSACOutlierRejectionThreshold = 0;
int icp_MaximumIterations = 100;
int RANSACIterations=0;
double TransformationEpsilon=0.0;


Eigen::Matrix4f increment;
Eigen::Matrix4f increment_current;
Eigen::Matrix4f increment_groundtruth;




typedef pcl::PointXYZ PointType;




PointCloud::Ptr p1;
PointCloud::Ptr p2;
pcl::IterativeClosestPoint<PointType, PointType> icp;
pcl::visualization::PCLVisualizer v;
Eigen::Matrix4f prev;
int i =0;
void icp_perform()
{
    
    icp.align(*p2);
    Eigen::Matrix4f m = icp.getFinalTransformation();
    v.removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerCustom<PointType> source_cloud_color_p1 (p1, 255,0,0);
    v.addPointCloud(p1, source_cloud_color_p1, "p1");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> source_cloud_color_p2 (p2, 0,255,0);
    v.addPointCloud(p2, source_cloud_color_p2, "p2");
    increment_current = increment_current*icp.getFinalTransformation ();
    float incr = fabs ((increment_current - increment_groundtruth).sum ());
    std::cout << increment_current<<"\n";
    std::cout << "===========\n";
    std::cout << increment_groundtruth<<"\n";
    
    PCL_INFO ("Iteration Nr. %d.\n", i++);
    if (useGroundTruth)PCL_INFO ("increment %f.\n", incr);
    
    prev = icp.getLastIncrementalTransformation ();
        
        
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	if ((event.getKeySym()=="i") && event.keyUp())
	{
           icp_perform(); 
        }
}
int main (int argc, char** argv)
{
	if(argc<3)
	{
		std::cout << "Usage:\n";
		std::cout << argv[0] <<" inputModel.xml outputModel.xml parameters\n";
		std::cout << " -d\tSets the maximum distance threshold between two correspondent points in source <-> target.\
If the distance is larger than this threshold, the points will be ignored in the alignment process.\tDefault: " << icp_CorrespondenceDistance << std::endl;
		std::cout << " -r\tSets the inlier distance threshold for the internal RANSAC outlier rejection loop.\
The method considers a point to be an inlier, if the distance between the target data index and the \
transformed source index is smaller than the given inlier distance threshold.\tDefault: " << icp_RANSACOutlierRejectionThreshold << std::endl;
		std::cout << " -i\tSets the maximum number of iterations the internal optimization should run for.\tDefault: " << icp_MaximumIterations << std::endl;
		std::cout << " -m\tSets the usage of metascan.\tDefault: " << isUseMetascan << std::endl;

		std::cout << " -ri\tSets the number of iterations RANSAC should run for.\tDefault: " << RANSACIterations << std::endl;
		std::cout << " -eps\tSets the usage of metascan.\tDefault: " << TransformationEpsilon << std::endl;

		return -1;
	}
        
        std::string scanFirst;
        std::string scanSecond;
        
	pcl::console::parse_argument (argc, argv, "-d", icp_CorrespondenceDistance);
	pcl::console::parse_argument (argc, argv, "-r", icp_RANSACOutlierRejectionThreshold);

	pcl::console::parse_argument (argc, argv, "-m", isUseMetascan);
		
	pcl::console::parse_argument (argc, argv, "-ri", RANSACIterations);
	pcl::console::parse_argument (argc, argv, "-eps", TransformationEpsilon);
        pcl::console::parse_argument (argc, argv, "-s1", scanFirst);
        pcl::console::parse_argument (argc, argv, "-s2", scanSecond);
        
        
        
     
	
        std::vector<int> xml_indices;
        xml_indices = pcl::console::parse_file_extension_argument (argc, argv, ".xml");
	
	if(xml_indices.size()== 0)
	{
		return -2;
	}

	std::string inputXMLFn = argv[xml_indices[0]];
        std::string gtruthXMLFn = "";
        
        if (xml_indices.size()>=2)
        {
            gtruthXMLFn = argv[xml_indices[0]];
            useGroundTruth = true;
        }

	std::cout << "Input model  :"<< inputXMLFn <<"\n";


	currentlyRegisteredPc = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
        

	inputXML.loadFile(inputXMLFn);
	inputXML.getAllScansId(indices);

	if (useGroundTruth)gtruthXML.loadFile(gtruthXMLFn); 

	std::string dataPath;
	inputXML.getDataSetPath(dataPath);
	

        
        
        std::string p1fn = inputXML.getFullPathOfPointcloud(scanFirst);
        std::string p2fn = inputXML.getFullPathOfPointcloud(scanSecond);
        
        Eigen::Matrix4f p1matrix;
        Eigen::Matrix4f p2matrix;
        
        Eigen::Matrix4f p1gtruthmatrix;
        Eigen::Matrix4f p2gtruthmatrix;
        
        
        
        inputXML.getAffine(scanFirst, p1matrix);
        inputXML.getAffine(scanSecond, p2matrix);
        
        std::cout << "loading file " << p1fn <<"\n";
        
        std::cout << "loading file " << p2fn <<"\n";
        
        p1 = PointCloud::Ptr(new PointCloud);
        p2 = PointCloud::Ptr(new PointCloud);
        
        pcl::io::loadPCDFile(p1fn, *p1);
        pcl::io::loadPCDFile(p2fn, *p2);
        
        
        
        increment =  p1matrix.inverse()*p2matrix;
        increment_current = increment;
        
        
        if (useGroundTruth)
        {
            gtruthXML.getAffine(scanFirst, p1gtruthmatrix);
            gtruthXML.getAffine(scanSecond, p2gtruthmatrix);
            increment_groundtruth = p1gtruthmatrix.inverse()*p2gtruthmatrix;
        }
        pcl::transformPointCloud(*p2,*p2, increment);
        
        
        
        icp.setTransformationEpsilon(TransformationEpsilon);
	icp.setRANSACIterations(RANSACIterations);
	icp.setMaximumIterations(2);
	icp.setMaxCorrespondenceDistance(icp_CorrespondenceDistance);
	icp.setRANSACOutlierRejectionThreshold(icp_RANSACOutlierRejectionThreshold);

	icp.setInputSource(p2);
        icp.setInputTarget (p1);
        

        pcl::visualization::PointCloudColorHandlerCustom<PointType> source_cloud_color_p1 (p1, 255,0,0);
        v.addPointCloud(p1, source_cloud_color_p1, "p1");

        pcl::visualization::PointCloudColorHandlerCustom<PointType> source_cloud_color_p2 (p2, 0,255,0);
        v.addPointCloud(p2, source_cloud_color_p2, "p2");

        v.registerKeyboardCallback (keyboardEventOccurred, (void*)&v);
        v.spin();
        
	
}
/* ]--- */
