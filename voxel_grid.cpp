#include <iostream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace po = boost::program_options;

int main (int argc, char** argv)
{
  std::string file;
  std::string output;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Display help message")
    ("file,f", po::value<std::string>(&file), "The input pcd file")
    ("output,o", po::value<std::string>(&output), "The output vtk file")
  ;

  po::command_line_parser parser{argc, argv};
  parser.options(desc).allow_unregistered().style(
    po::command_line_style::default_style |
    po::command_line_style::allow_slash_for_short);
  po::parsed_options parsed_options = parser.run();

  po::variables_map vm;
  po::store(parsed_options, vm);
  po::notify(vm);

  bool nc = vm.count("nc");

  if (vm.count("help") || !vm.count("file")) {
    std::cout << desc << std::endl;
    return 0;
  }

  if (!boost::algorithm::ends_with(file, ".pcd")) {
    std::cerr << file << " should end with .pcd extension" << std::endl;
    return 1;
  }

  if (!boost::filesystem::exists(file)) {
    std::cerr << file << " not exists" << std::endl;
    return 1;
  }

  if (!vm.count("output")) {
    output = boost::replace_all_copy(file, ".pcd", "_downsample.pcd");
  }

  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read (file, *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  pcl::PCDWriter writer;
  writer.write (output, *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}

