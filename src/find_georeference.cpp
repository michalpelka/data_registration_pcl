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

#include <iostream>
#include "dataFramework/data_model.hpp"
#include <boost/lexical_cast.hpp>
#include <pcl/registration/transformation_estimation.h>
#include <pcl-1.7/pcl/registration/transformation_estimation_svd.h>
#include "UTM.h"
#include <cv.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <opencv2/highgui/highgui.hpp>
#include <iomanip>

typedef pcl::PointXYZRGBA PointT;


/*
   Return a RGB colour value given a scalar v in the range [vmin,vmax]
   In this case each colour component ranges from 0 (no contribution) to
   1 (fully saturated), modifications for other ranges is trivial.
   The colour is clipped at the end of the scales if v is outside
   the range [vmin,vmax]
*/

typedef struct {
    double r,g,b;
} COLOUR;

COLOUR GetColour(double v,double vmin,double vmax)
{
   COLOUR c = {1.0,1.0,1.0}; // white
   double dv;

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c.r = 0;
      c.g = 4 * (v - vmin) / dv;
   } else if (v < (vmin + 0.5 * dv)) {
      c.r = 0;
      c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
   } else if (v < (vmin + 0.75 * dv)) {
      c.r = 4 * (v - vmin - 0.5 * dv) / dv;
      c.b = 0;
   } else {
      c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      c.b = 0;
   }

   return(c);
}

void writeKML (std::vector<pcl::PointXY> quadCoordinates, std::string fileName )
{
    std::stringstream ss;
    ss<<std::setprecision(10);
    for (int i=0; i < quadCoordinates.size(); i++)
    {
        ss<<quadCoordinates[i].y <<","<< quadCoordinates[i].x <<",0 ";
    }
    std::string kml(
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">\n"
    "<Folder>\n"
    "	<name>Robot Scan</name>\n"
    "	<open>1</open>\n"
    "   <GroundOverlay>\n"
    "       <name>Height Ramp</name>\n"
    "       <description>Height ramp</description>\n"
    "       <gx:balloonVisibility>1</gx:balloonVisibility>\n"
    "       <Icon>\n"
    "           <href>heightRamp.png</href>\n"
    "           <viewBoundScale>0.75</viewBoundScale>\n"
    "       </Icon>\n"
    "       <gx:LatLonQuad>\n"
    "           <coordinates>\n");
                kml  = kml+	ss.str() + std::string(
    "\n         </coordinates>\n"
    "       </gx:LatLonQuad>\n"
    "   </GroundOverlay>\n"
    "   <GroundOverlay>\n"
    "       <name>Height Ramp</name>\n"
    "       <description>Color scan</description>\n"
    "       <gx:balloonVisibility>1</gx:balloonVisibility>\n"
    "       <Icon>\n"
    "           <href>colorRamp.png</href>\n"
    "           <viewBoundScale>0.75</viewBoundScale>\n"
    "       </Icon>\n"
    "       <gx:LatLonQuad>\n"
    "           <coordinates>\n");
                kml  = kml+	ss.str() + std::string(
    "\n         </coordinates>\n"
    "       </gx:LatLonQuad>\n"
    "       </GroundOverlay>\n"
    "   </Folder>\n"
    "</kml>");
    std::ofstream file;
    file.open(fileName.c_str());
    file << kml;
    file.close();
}

int main (int argc, char** argv)
{
	if (argc != 3)
	{
            std::cerr<< "Finds georeference \n";
            std::cerr<< "USAGE :" << argv[0]<<" xmlModel.xml output.kml \n";
            return -1;
	}

	data_model tr;
	std::string fn_xml = argv[1];
        std::string fn_kml = argv[2];
        
	std::cout << "opening file "<< fn_xml <<"\n";
	tr.loadFile(fn_xml);

	std::vector<std::string> ids;
        
        
        char UTM_zone[3];
	tr.getAllScansId(ids);
        pcl::PointCloud<PointT> coord_local;
        pcl::PointCloud<PointT> coord_utm;
        coord_local.resize(ids.size());
        coord_utm.resize(ids.size());
        
	for (int i=0; i< ids.size(); i++)
        {
            // get local coordinate system
            PointT originPoint;
            Eigen::Vector3f f = Eigen::Vector3f(0,0,0);
            Eigen::Affine3f affine;
            tr.getAffine(ids[i],affine.matrix());
            Eigen::Vector3f out = affine * f;
            coord_local[i].x=static_cast<double>(out.x());
            coord_local[i].y=static_cast<double>(out.y());
            
        }
        for (int i=0; i< ids.size(); i++)
        {
            // get utm coordinate system
            double lon;
            double lat;
            std::string gps;
            tr.getGPS(ids[i], gps);
            std::stringstream(gps)>>lat>>lon;
            double northing,easting;
            UTM::LLtoUTM(lat,lon,northing,easting, UTM_zone);
            coord_utm[i].x=northing;
            coord_utm[i].y=easting;
        }

        for (int i=0; i< ids.size(); i++)
        {
            std::cout << i << ":"<<ids[i]<<"\n";
            std::cout << "\tlocal:"<<coord_local[i].x << "\t" <<coord_local[i].y <<"\n";
            std::cout << "\tutm:"<<coord_utm[i].x << "\t" <<coord_utm[i].y <<"\n";
        }

        pcl::registration::TransformationEstimationSVD <PointT,PointT> est;
        Eigen::Matrix4f mat;
        est.estimateRigidTransformation(coord_local, coord_utm, mat);
        std::cout << "local to utm transformation matrix :\n";
        std::cout << mat<<"\n";
        //read data
        pcl::PointCloud<PointT> metamodel;
        std::vector<pcl::PointCloud<PointT>::Ptr> clouds;
        
        
        for (int i=0; i < ids.size(); i++)
	{
		pcl::PointCloud<PointT>::Ptr pc (new pcl::PointCloud<PointT>);
		std::string fn;
		Eigen::Matrix4f transform;
		fn = tr.getFullPathOfPointcloud(ids[i]);
		bool isOkTr = tr.getAffine(ids[i], transform);
		if (isOkTr)
		{
			std::cout <<"============\n";
			std::cout <<"adding pc     :"<< ids[i]<<"\n";
			std::cout <<"filen name    :"<<fn <<"\n";
			std::cout <<"with transform: \n"<< transform<<"\n";
			pcl::io::loadPCDFile<PointT>(fn, *pc);
			
			pcl::transformPointCloud(*pc,*pc, transform);
                        clouds.push_back(pc);
			//metamodel = metamodel+ (*pc);
                        
		}		
	}

        
        // find AABB
        PointT minPoint, maxPoint;
        minPoint.x = FLT_MIN;
        minPoint.y = FLT_MIN;
        minPoint.z = FLT_MIN; 
        
        maxPoint.x = FLT_MAX;
        maxPoint.y = FLT_MAX;
        maxPoint.z = FLT_MAX; 
               
       
        pcl::getMinMax3D(metamodel, minPoint, maxPoint);
        for (int i =0; i < clouds.size(); i++)
        {
            for (int j =0; j < clouds[i]->size(); j++)
            {
                if (minPoint.x > (*clouds[i])[j].x) minPoint.x = (*clouds[i])[j].x;
                if (minPoint.y > (*clouds[i])[j].y) minPoint.y = (*clouds[i])[j].y;
                if (minPoint.z > (*clouds[i])[j].z) minPoint.z = (*clouds[i])[j].z;
                
                if (maxPoint.x < (*clouds[i])[j].x) maxPoint.x = (*clouds[i])[j].x;
                if (maxPoint.y < (*clouds[i])[j].y) maxPoint.y = (*clouds[i])[j].y;
                if (maxPoint.z < (*clouds[i])[j].z) maxPoint.z = (*clouds[i])[j].z;
                
                
            }
        }
        
        
        // project on image
        float rangeInX = maxPoint.x - minPoint.x;
        float rangeInY = maxPoint.y - minPoint.y;
        float rangeInZ = maxPoint.z - minPoint.z;
        
        int resX = rangeInX*15;
        int resY = rangeInY*15;
        
        cv::Mat img_heigh = cv::Mat (resY,resX,  CV_8UC4,cv::Scalar(0,0,0,0));
        cv::Mat img_color = cv::Mat (resY,resX,  CV_8UC4,cv::Scalar(0,0,0,0));
        
        std::cout << "max " << maxPoint <<"\n";
        std::cout << "min " << minPoint <<"\n";
        
        std::cout << "rangeX "  << rangeInX << "\n";
        std::cout << "rangeY "  << rangeInY << "\n";
        
        for (int i =0; i < clouds.size(); i++)
        {
            for (int j =0; j < clouds[i]->size(); j++)
            {
                int x = resX*((*clouds[i])[j].x -minPoint.x )/rangeInX;
                int y = resY*((*clouds[i])[j].y -minPoint.y )/rangeInY;
                float z = 100000.0f*((*clouds[i])[j].z -minPoint.z )/rangeInZ;

                int r = (0xff0000 & (*clouds[i])[j].rgba) >> 16;
                int g = (0x00ff00 & (*clouds[i])[j].rgba) >> 8;
                int b =  0x0000ff & (*clouds[i])[j].rgba; 
                
                COLOUR c = GetColour(z, 0,100000.0f);
                        
                cv::circle(img_heigh,cv::Point(x,resY-y), 1,cv::Scalar(255*c.r, 255*c.g, 255*c.b, 255));
                cv::circle(img_color,cv::Point(x,resY-y), 1,cv::Scalar(r,g,b,255));
                
            }
        }
        // get image  corners coordinates in pointcloud local coordinate system
        Eigen::Vector4f c1 (minPoint.x,minPoint.y,0,1);
        Eigen::Vector4f c4 (minPoint.x,maxPoint.y,0,1);
        Eigen::Vector4f c3 (maxPoint.x,maxPoint.y,0,1);
        Eigen::Vector4f c2 (maxPoint.x,minPoint.y,0,1);
        
        std::cout << "corners of map in local coord sys \n";
        std::cout << "\tc1:\n" <<c1 <<"\n" ;
        std::cout << "\tc2:\n" <<c2 <<"\n";
        std::cout << "\tc3:\n" <<c3 <<"\n";
        std::cout << "\tc4:\n" <<c4 <<"\n";
        
        
        //reproject it to UTM
        Eigen::Vector4f utm_c1 = mat * c1;
        Eigen::Vector4f utm_c2 = mat * c2;
        Eigen::Vector4f utm_c3 = mat * c3;
        Eigen::Vector4f utm_c4 = mat * c4;
        
        std::cout << "corners of map in utm coord sys \n";
        std::cout << "\tc1:\n" <<utm_c1 <<"\n";
        std::cout << "\tc2:\n" <<utm_c2 <<"\n";
        std::cout << "\tc3:\n" <<utm_c3 <<"\n";
        std::cout << "\tc4:\n" <<utm_c4 <<"\n";
        
        //wsg 84
        std::cout << "utm zone" << UTM_zone << "\n";
        std::vector<pcl::PointXY> quadCoordinates(4);
        UTM::UTMtoLL(utm_c1[0], utm_c1[1], UTM_zone,quadCoordinates[0].x, quadCoordinates[0].y  );
        UTM::UTMtoLL(utm_c2[0], utm_c2[1], UTM_zone,quadCoordinates[1].x, quadCoordinates[1].y  );
        UTM::UTMtoLL(utm_c3[0], utm_c3[1], UTM_zone,quadCoordinates[2].x, quadCoordinates[2].y  );
        UTM::UTMtoLL(utm_c4[0], utm_c4[1], UTM_zone,quadCoordinates[3].x, quadCoordinates[3].y  );
        
        std::cout << quadCoordinates[3] <<"\n";
        std::cout << quadCoordinates[2] <<"\n";
        std::cout << quadCoordinates[1] <<"\n";
        std::cout << quadCoordinates[0] <<"\n";
        
        
       
        cv::imwrite("heightRamp.png" , img_heigh);
        cv::imwrite("colorRamp.png", img_color);
        
        
        writeKML(quadCoordinates, fn_kml);
        
        return 0;
}
