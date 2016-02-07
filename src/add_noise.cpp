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

#include<pcl/visualization/pcl_plotter.h>

#include<iostream>
#include<vector>
#include<utility>
#include<math.h>  
#include"dataFramework/data_model.hpp"
#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <eigen3/Eigen/Dense>
#include <pcl/console/parse.h>
using namespace std;
using namespace pcl::visualization;



void createTrajectory (data_model &tr1, std::vector<double> &x, std::vector<double> &y)
{
	std::vector<std::string> ids;
	tr1.getAllScansId(ids);

	x.resize(ids.size());
	y.resize(ids.size());

	for (int i =0 ; i < ids.size(); i++)
	{
		Eigen::Vector3f f = Eigen::Vector3f(0,0,0);
		Eigen::Affine3f affine;
		
		tr1.getAffine(ids[i],affine.matrix());
		Eigen::Vector3f out = affine * f;
		//std::cout << out.x()<<"\t" << out.y()<<"\n";

		x[i]=static_cast<double>(out.x());
		y[i]=static_cast<double>(out.y());
		
	}
}


void addTrajectory(std::string xmlName, PCLPlotter * p)
{
  data_model tr1;
  tr1.loadFile(xmlName);
  std::vector<double> ax;
  std::vector<double> ay;
  createTrajectory(tr1, ax,ay);

  //addPlotData (std::vector< double > const &array_x, std::vector< double >const &array_y, char const *name="Y Axis", int type=vtkChart::LINE, std::vector< char > const &color=std::vector< char >())
  p->addPlotData(ax,ay,xmlName.c_str());
  
  p->setXRange(-50,50);
  p->setYRange(-50,50);

  
}
Eigen::Matrix4f increment;
Eigen::Matrix4f increment_current;

int main (int argc, char * argv [])
{
    // This is the underlying integer random number generator
  boost::mt19937 igen;
  // The second template parameter is the actual floating point
  // distribution that the user wants
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > gen(igen, boost::normal_distribution<>());

  std::cout <<"usage:\n";
  std::cout << argv[0] <<" ground_truth_model.xml model_with_noise.xml -mu1 0 -sig1 0 -mu2 0 -sig2 0 \n";
  
  std::string fn_gtruth = argv[1];
  std::string fn_noise = argv[2];
  
  float mu1=0;
  float mu2 =0;
  float sig1=0;
  float sig2=0;
  pcl::console::parse_argument (argc, argv, "-mu1", mu1);
  pcl::console::parse_argument (argc, argv, "-sig1", sig1);
  
  pcl::console::parse_argument (argc, argv, "-mu2", mu2);
  pcl::console::parse_argument (argc, argv, "-sig2", sig2);
  
  data_model xml_gruth;
  data_model xml_mnoise;
  
  xml_gruth.loadFile(fn_gtruth);
  
  
  std::vector<std::string> ids;
  xml_gruth.getAllScansId(ids);
  

  
  Eigen::Matrix4f currentMatrix;
  xml_gruth.getAffine(ids[0],currentMatrix);
  std::string fn;
  xml_gruth.getPointcloudName(ids[0], fn);
  
  xml_mnoise.setAffine(ids[0], currentMatrix);
  xml_mnoise.setPointcloudName(ids[0], fn);
  for (int i =1 ; i < ids.size(); i++)
  {
            
    Eigen::Matrix4f p2matrix;
    Eigen::Matrix4f p1matrix;

    xml_gruth.getAffine(ids[i],p2matrix);
    xml_gruth.getAffine(ids[i-1],p1matrix);
    
    //create normal PDF for rotation and translation
    Eigen::Affine3f noise = Eigen::Affine3f::Identity();
    noise.translation()<<mu1+ sig1*gen(),mu1+ sig1*gen(),0;
    noise.rotate(Eigen::AngleAxisf(mu2+sig2*gen(), Eigen::Vector3f::UnitZ()));
    
    std::string fn;
    increment =  p1matrix.inverse()*p2matrix;
    currentMatrix = currentMatrix *noise* increment;
    
    xml_gruth.getPointcloudName(ids[i], fn);
    xml_mnoise.setPointcloudName(ids[i], fn);
    xml_mnoise.setAffine(ids[i], currentMatrix);
  }
  std::string data_path; 
  xml_gruth.getDataSetPath(data_path);
  xml_mnoise.setDataSetPath(data_path);
  xml_mnoise.saveFile(fn_noise);
  return 1;
}
