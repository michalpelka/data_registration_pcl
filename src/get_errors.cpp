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
#include <Eigen/Dense>
#include <pcl/console/parse.h>
#include <pcl-1.7/pcl/common/transforms.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace std;
using namespace pcl::visualization;



struct model
{
    data_model xml;
    Eigen::Matrix4f localTransform;
    pcl::PointCloud<pcl::PointXYZ> point;
    std::vector<float> errors;
    
    void compute_errors (model &groundTruth)
    {
        errors.resize(groundTruth.point.size());
        for (int i=0; i < groundTruth.point.size(); i++)
        {
            pcl::PointXYZ p1 = groundTruth.point[i];
            pcl::PointXYZ p2 = this->point[i];
            
            errors[i] = sqrt((p1.x - p2.x)*(p1.x - p2.x)+(p1.y - p2.y)*(p1.y - p2.y)+(p1.z - p2.z)*(p1.z - p2.z));
        }
    }
    void load (std::string xml_fn)
    {
        
        std::cout << "loading model " << xml_fn << "\n";
        xml.loadFile(xml_fn);
        std::vector<std::string> ids;
        xml.getAllScansId(ids);
        std::cout << "number of pointclouds :" << ids.size() <<"\n";
        for (int j = 0; j < ids.size(); j ++)
        {
            Eigen::Affine3f m;
            xml.getAffine(ids[j], m.matrix());
            Eigen::Vector3f f = Eigen::Vector3f(0,0,0);
            Eigen::Vector3f p;
            p = m * f;
            point.push_back(pcl::PointXYZ(p[0],p[1],p[2]));
        }
    }
  
    
};




int main (int argc, char * argv [])
{
  
  std::cout <<"usage:\n";
  std::cout << argv[0] <<" ground_truth_model.xml model_with_noise1.xml model_with_noise2.xml\n";
  std::string outputFileName;
  pcl::console::parse_argument (argc, argv, "-out", outputFileName); 
  
 
  
 
  // load pointcloud as distance 
  
    std::vector<int> xml_indices;
    xml_indices = pcl::console::parse_file_extension_argument (argc, argv, ".xml");
    model ground_turth;
    ground_turth.load(argv[xml_indices[0]]);
    std::vector<model> models;
    models.resize(xml_indices.size()-1);
    
    for (int i = 0; i < models.size(); i++)
    {
        std::string fn = argv[xml_indices[i+1]];
        
        models[i].load(fn);

        pcl::registration::TransformationEstimationSVD <pcl::PointXYZ,pcl::PointXYZ> est;
        //models[i].point.resize(ground_turth.point.size());
        
        est.estimateRigidTransformation(models[i].point,  ground_turth.point, models[i].localTransform);
        //pcl::transformPointCloud(models[i].point,models[i].point,models[i].localTransform);
    }
    
    std::ofstream dat;
    std::string dat_out = outputFileName +".dat";
    dat.open(dat_out.c_str());
    for (int i=0; i < models.size(); i++)
    {
        models[i].compute_errors(ground_turth);
    }
  
    for (int i=0; i < ground_turth.point.size(); i++)
    {
        dat << i;
        for (int j =0; j < models.size(); j++)
        {
            float f = models[j].errors[i];
            dat <<  "\t" << f ;
            
        }
        dat << "\n";
    }
  
  
  std::ofstream plot;
  std::string g_out = outputFileName +".g";
  plot.open(g_out.c_str());
  plot << "set terminal pngcairo size 1000,800 enhanced font 'Verdana,20'\n";
  plot << "set output '"<<outputFileName<<".png'\n";
  plot << "set xlabel \"Numer skanu\" \n";
  plot << "set ylabel \"Błąd odległości [m] \" \n";
  plot << "plot ";
  
  for (int i=1; i<xml_indices.size(); i++)
  {
      std::string fn = argv[xml_indices[i]];
      plot << "\""<<dat_out<<"\" using 1:"<<i+1<<" with lines title \""<<fn<<"\", ";
  }

  
  plot.close();
          
  dat.close();
  
  system(("gnuplot -persist "+g_out).c_str());
  return 1;
}
