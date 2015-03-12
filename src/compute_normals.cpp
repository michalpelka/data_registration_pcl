#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <dataFramework\data_model.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <pcl/common/time.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


float radius =0.2;
int main (int argc, char** argv)
{
	std::cout << "Usage:\n";
	std::cout << "Modify pcd files by adding normal estimation\n";
	std::cout << argv[0] << " model.xml parameters\n";
	std::cout << " -r <radius>\t\tSet the set search radius size. Default: 0.2"<< std::endl;
	
	if(argc<2)
	{
		return -1;
	}

	std::string param_inputModel = argv[1];
	
	
	float _r =0.2;
	if(pcl::console::parse_argument (argc, argv, "-r", _r)!=-1)
	{
		radius=_r;
	}

	//Generate text from values of parameters
	
	
	std::cout << "input model " << param_inputModel <<'\n';
	std::cout << "radius      " << radius <<'\n';


	data_model inputModel;
	
	inputModel.loadFile(param_inputModel);
	
	if(!inputModel.loadFile(param_inputModel))
	{
		std::cout << "Error loading: " << param_inputModel << std::endl;
		return -2;
	}
	std::cout << "Loaded: " << param_inputModel << " correctly.\n";

	std::vector <std::string> cloud_ids;

	inputModel.getAllScansId(cloud_ids);

	for (int i=0; i < cloud_ids.size(); i++)
	{
		std::cout << "Processing scan :" << cloud_ids[i]<<"\n";
		std::string pc_path = inputModel.getFullPathOfPointcloud(cloud_ids[i]);
		pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		
		std::cout << "loading file :" << pc_path<<"\n";
	

		pcl::PointCloud<pcl::PointXYZINormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZINormal>);

		pcl::io::loadPCDFile(pc_path, *input);
		Eigen::Affine3f matrix = Eigen::Affine3f::Identity();
		inputModel.getAffine(cloud_ids[i], matrix.matrix());
		Eigen::Affine3f rotation;
		rotation = matrix.rotation();

		pcl::transformPointCloud<PointXYZI>(*input, *transformed, rotation);
		pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
		ne.setInputCloud (transformed);
		ne.setRadiusSearch(radius);
		ne.compute (*normals);
		std::cout << "normal computed\n";
		std::cout << "input->size()" << input->size() <<"\n";
		std::cout << "normals->size()" << normals->size() <<"\n";
		output->resize(normals->size() );
		for (int i=0; i< input->size(); i++)
		{
			pcl::PointXYZINormal point;
			point.normal_x = (*normals)[i].normal_x;
			point.normal_y = (*normals)[i].normal_y;
			point.normal_z = (*normals)[i].normal_z;

			point.x = (*input)[i].x;
			point.y = (*input)[i].y;
			point.z = (*input)[i].z;
			point.intensity = (*input)[i].intensity;
			(*output )[i] = point;
		}
		std::cout << "saving\n";
		pcl::io::savePCDFile(pc_path,*output);
	}
	
}