#include <iostream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

namespace po = boost::program_options;

int main (int argc, char** argv)
{
  std::string file;
  std::string output;
  unsigned int ks;
  float sr;
  float mu;
  unsigned int mnn;
  unsigned int msa;
  unsigned int mia;
  unsigned int mxa;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Display help message")
    ("file,f", po::value<std::string>(&file), "The input pcd file")
    ("output,o", po::value<std::string>(&output), "The output vtk file")
    ("ks", po::value<unsigned int>(&ks)->default_value(20), "n.setKSearch($ks)")
    ("sr", po::value<float>(&sr)->default_value(0.1f), "gp3.setSearchRadius($sr)")
    ("mu", po::value<float>(&mu)->default_value(2.5f), "gp3.setMu($mu)")
    ("mnn", po::value<unsigned int>(&mnn)->default_value(100), "gp3.setMaximumNearestNeighbors($mnn)")
    ("msa", po::value<unsigned int>(&msa)->default_value(4), "gp3.setMaximumSurfaceAngle(M_PI / $msa)")
    ("mia", po::value<unsigned int>(&mia)->default_value(18), "gp3.setMinimumAngle(M_PI / $mia)")
    ("mxa", po::value<unsigned int>(&mxa)->default_value(3), "gp3.setMaximumAngle(2 * M_PI / $mxa)")
    ("nc", "gp3.setNormalConsistency($nc)")
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
    output = boost::replace_all_copy(file, ".pcd", ".vtk");
  }


  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile (file, cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (ks);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (sr);

  // Set typical values for the parameters
  gp3.setMu (mu);
  gp3.setMaximumNearestNeighbors (mnn);
  gp3.setMaximumSurfaceAngle(M_PI / msa); // 45 degrees
  gp3.setMinimumAngle(M_PI / mia); // 10 degrees
  gp3.setMaximumAngle(2 * M_PI / mxa); // 120 degrees
  gp3.setNormalConsistency(nc);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  pcl::io::saveVTKFile (output, triangles);

  // Finish
  return (0);
}

