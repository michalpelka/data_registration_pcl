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

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <dataFramework/data_model.hpp>
#include <boost/filesystem.hpp>
#include <pcl/common/time.h>
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int		MeanK=50;
float	StddevMulThresh=1.0f;


//bool saveASC(std::string fileName,pcl::PointCloud<PointXYZ>::Ptr p)
//{
//	std::ofstream fs;
//	fs.open(fileName.c_str());
//	if (!fs.is_open()) return false;
//	for (int i =0; i < p->size(); i++)
//	{
//		fs << (*p)[i].x <<" "<< (*p)[i].y << " " <<(*p)[i].z <<"\n";
//	}
//	fs.close();
//	return true; 
//}



int main (int argc, char** argv)
{
	if(argc<3)
	{
		std::cout << "Usage:\n";
		std::cout << "Program creates new metamodel connecting data from two metamodels. First metamodel has priority \n";
		std::cout << argv[0] << " input_model_withMetadata.xml input_model_withTransforms.xml output_model.xml\n";

		return -1;
	}

	std::vector<int> xml_indices;
	xml_indices = pcl::console::parse_file_extension_argument (argc, argv, ".xml");

	if(xml_indices.size()!=3)
	{
		return -2;
	}


	std::string param_inputModel_transform = argv[xml_indices[0]];
	std::string param_inputModel_meta = argv[xml_indices[1]];
	std::string param_outputModel = argv[xml_indices[2]];


	/// models
	data_model inputModel_meta;
	data_model inputModel_transform;

	data_model outputModel;

	if(!inputModel_meta.loadFile(param_inputModel_meta))
	{
		std::cout << "Error loading: " << param_inputModel_meta << std::endl;
		return -3;
	}
	std::cout << "Loaded: " << param_inputModel_meta << " correctly.\n";


	if(!inputModel_transform.loadFile(param_inputModel_transform))
	{
		std::cout << "Error loading: " << param_inputModel_transform << std::endl;
		return -3;
	}
	std::cout << "Loaded: " << param_inputModel_transform << " correctly.\n";



	// loads clouds ids in input model
	std::vector <std::string> cloud_ids;
	inputModel_transform.getAllScansId(cloud_ids);
		
	std::cout <<"pointclouds count: "<< cloud_ids.size() <<"\n";

	double totalTime=0;
	for (int i=0; i < cloud_ids.size(); i++)
	{
	

		//copy filename
		std::string  fn;
		inputModel_transform.getPointcloudName(cloud_ids[i],fn);
		outputModel.setPointcloudName(cloud_ids[i],fn);
		//copy matrix
		Eigen::Matrix4f tr;
		inputModel_transform.getAffine(cloud_ids[i], tr);
		outputModel.setAffine(cloud_ids[i], tr);

		// copy metadata
		
		std::string gps;
		std::string photo;
		boost::posix_time::ptime tt;
		
		if (inputModel_meta.getGPS(cloud_ids[i], gps)) outputModel.setGPS(cloud_ids[i], gps);
		if (inputModel_meta.getTimestamp(cloud_ids[i], tt)) outputModel.setTimestamp(cloud_ids[i], tt);
		if (inputModel_meta.getPhoto(cloud_ids[i], photo))outputModel.setPhoto(cloud_ids[i], photo);

		

		if (inputModel_transform.getGPS(cloud_ids[i], gps)) outputModel.setGPS(cloud_ids[i], gps);
		if (inputModel_transform.getTimestamp(cloud_ids[i], tt)) outputModel.setTimestamp(cloud_ids[i], tt);
		if (inputModel_transform.getPhoto(cloud_ids[i], photo))outputModel.setPhoto(cloud_ids[i], photo);




	}
	std::cout << "saving output \n";
	std::string relativePathToData;
	inputModel_transform.getDataSetPath(relativePathToData);
	outputModel.setDataSetPath(relativePathToData);


	outputModel.saveFile(param_outputModel);

}
