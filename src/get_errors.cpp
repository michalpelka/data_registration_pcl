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

using namespace std;
using namespace pcl::visualization;







int main (int argc, char * argv [])
{
    // This is the underlying integer random number generator
  boost::mt19937 igen;
  // The second template parameter is the actual floating point
  // distribution that the user wants
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > gen(igen, boost::normal_distribution<>());

  std::cout <<"usage:\n";
  std::cout << argv[0] <<" ground_truth_model.xml model_with_noise1.xml model_with_noise2.xml";
  
  std::vector<int> xml_indices;
  xml_indices = pcl::console::parse_file_extension_argument (argc, argv, ".xml");
  


  
  data_model xml_gruth;
  std::vector<data_model> xml_models;
  
  xml_gruth.loadFile(argv[xml_indices[0]]);
  xml_models.resize(xml_indices.size()-1);
  
  std::ofstream rotation;
  std::ofstream translation;
  translation.open("output.dat");
  translation <<"#No\t";
  

  
  for (int i=1; i< xml_indices.size(); i++)
  {
      std::cout << "loading "<< argv[xml_indices[i]] <<"\n";
      xml_models[i-1].loadFile(argv[xml_indices[i]]);
      translation<<"\t"<<argv[xml_indices[i]];
  }
  translation<<"\n";
  
  std::vector<std::string> ids;
  xml_gruth.getAllScansId(ids);
  
  
  
  for (int i =1 ; i < ids.size(); i++)
  {
    Eigen::Affine3f truth;
    xml_gruth.getAffine(ids[i],truth.matrix());

    
    translation << i;
    for (int j =0; j < xml_models.size();j++)
    {
        Eigen::Affine3f error;

        xml_models[j].getAffine(ids[i],error.matrix());

        Eigen::Vector3f tr = truth.translation();
        Eigen::Vector3f er = error.translation();

        translation<<"\t"<< (tr[0]-er[0])*(tr[0]-er[0])+(tr[1]-er[1])*(tr[1]-er[1])*(tr[2]-er[2])*(tr[2]-er[2]);
    }
    translation<<"\n";
  }
  
  std::ofstream plot;
  plot.open("output.g");
  plot << "plot ";
  for (int i=1; i<xml_indices.size(); i++)
  {
      std::string fn = argv[xml_indices[i]];
      plot << "\"output.dat\" using 1:"<<i+1<<" with lines title \""<<fn<<"\", ";
  }
  plot.close();
          
  translation.close();
  system("gnuplot -persist output.g");
  return 1;
}
