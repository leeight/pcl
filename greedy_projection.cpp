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
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_subdivision.h>
#include <pcl/io/vtk_io.h>

namespace po = boost::program_options;

pcl::PolygonMesh::Ptr smoothSurface(pcl::PolygonMesh::Ptr mesh)
{
#if 0
  pcl::PolygonMesh mesh;
  pcl::PolygonMeshConstPtr trianglesPtr = triangles;
  vtk.setInputMesh(trianglesPtr);
  vtk.process(mesh);
#endif
  pcl::PolygonMesh::Ptr newMesh(new pcl::PolygonMesh);

  pcl::MeshSmoothingLaplacianVTK vtk;
  vtk.setInputMesh(mesh);
  vtk.process(*newMesh);

#if 0
  pcl::MeshSubdivisionVTK processPointCloud;
  processPointCloud.setFilterType(pcl::MeshSubdivisionVTK::LOOP);
  // processPointCloud.setFilterType();
  processPointCloud.setInputMesh(mesh);
  processPointCloud.process(*newMesh);
#endif
  // std::cout << "--------" <<std::endl;
  // std::cout << processPointCloud.getFilterType() << std::endl;
  // MeshSubdivisionVTKFilterType = LOOP;
  // std::cout << pcl::MeshSubdivisionVTK::LOOP<< std::endl;
  return newMesh;
}

int main (int argc, char** argv)
{
  std::string file;
  std::string output;

  // gp3
  unsigned int ks;
  float sr;
  float mu;
  unsigned int mnn;
  float msa;
  float mia;
  float mxa;

  // poisson
  int depth;
  int degree;
  int isoDivide;
  int minDepth;
  float scale;
  float samplesPerNode;
  int solverDivide;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Display help message")
    ("file,f", po::value<std::string>(&file), "The input pcd file")
    ("output,o", po::value<std::string>(&output), "The output vtk file")
    ("gp3", "GreedyProjectionTriangulation is an implementation of a greedy triangulation algorithm for 3D points based on local 2D projections")
    ("ks", po::value<unsigned int>(&ks)->default_value(20), "n.setKSearch($ks)")
    ("sr", po::value<float>(&sr)->default_value(0.1f), "gp3.setSearchRadius($sr)")
    ("mu", po::value<float>(&mu)->default_value(2.5f), "gp3.setMu($mu)")
    ("mnn", po::value<unsigned int>(&mnn)->default_value(100), "gp3.setMaximumNearestNeighbors($mnn)")
    ("msa", po::value<float>(&msa)->default_value(4.0f), "gp3.setMaximumSurfaceAngle(M_PI / $msa)")
    ("mia", po::value<float>(&mia)->default_value(18.0f), "gp3.setMinimumAngle(M_PI / $mia)")
    ("mxa", po::value<float>(&mxa)->default_value(1.5f), "gp3.setMaximumAngle(M_PI / $mxa)")
    ("nc", "Set the flag if the input normals are oriented consistently.")
    ("consistent_ordering", "Set the flag to order the resulting triangle vertices consistently (positive direction around normal).")
    ("poisson", "The Poisson surface reconstruction algorithm")
    ("confidence", "Enabling this flag tells the reconstructor to use the size of the normals as confidence information. When the flag is not enabled, all normals are normalized to have unit-length prior to reconstruction.")
    ("depth", po::value<int>(&depth)->default_value(8), "Running at depth d corresponds to solving on a voxel grid whose resolution is no larger than 2^d x 2^d x 2^d. Note that since the reconstructor adapts the octree to the sampling density, the specified reconstruction depth is only an upper bound.")
    ("degree", po::value<int>(&degree)->default_value(2), "Set the degree parameter")
    ("iso_divide", po::value<int>(&isoDivide)->default_value(8.0f), "Set the depth at which a block iso-surface extractor should be used to extract the iso-surface.")
    ("manifold", po::bool_switch()->default_value(true), "Enabling this flag tells the reconstructor to add the polygon barycenter when triangulating polygons with more than three vertices.")
    ("min_depth", po::value<int>(&minDepth)->default_value(5), "void pcl::Poisson< PointNT >::setMinDepth")
    ("output_polygons", "Enabling this flag tells the reconstructor to output a polygon mesh (rather than triangulating the results of Marching Cubes).")
    ("samples_per_node", po::value<float>(&samplesPerNode)->default_value(1.0f), "For noise-free samples, small values in the range [1.0 - 5.0] can be used. For more noisy samples, larger values in the range [15.0 - 20.0] may be needed to provide a smoother, noise-reduced, reconstruction.")
    ("scale", po::value<float>(&scale)->default_value(1.1f), "Set the ratio between the diameter of the cube used for reconstruction and the diameter of the samples' bounding cube.")
    ("solver_divide", po::value<int>(&solverDivide)->default_value(8), "Set the the depth at which a block Gauss-Seidel solver is used to solve the Laplacian equation.")
    ("mcr", "The marching cubes surface reconstruction algorithm, using a signed distance function based on radial basis functions.")
    ("mch", "The marching cubes surface reconstruction algorithm, using a signed distance function based on the distance from tangent planes, proposed by Hoppe et.")
    ("gp", "Grid projection surface reconstruction method.")
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

#if 0
  // Smoothing with MLS
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setInputCloud(cloud);
  mls.setComputeNormals(true);
  mls.setPolynomiaFit(true);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);
  mls.process(*mls_normals);
#endif

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
  pcl::PolygonMesh triangles;

  if (vm.count("gp3")) {
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (sr);

    // Set typical values for the parameters
    gp3.setMu (mu);
    gp3.setMaximumNearestNeighbors (mnn);
    gp3.setMaximumSurfaceAngle(M_PI / msa); // 45 degrees
    gp3.setMinimumAngle(M_PI / mia); // 10 degrees
    gp3.setMaximumAngle(M_PI / mxa); // 120 degrees
    gp3.setNormalConsistency(nc);
    gp3.setConsistentVertexOrdering(vm.count("consistent_ordering"));

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);
  }
  else if (vm.count("poisson")) {
    pcl::Poisson<pcl::PointNormal> poisson;

    poisson.setConfidence(vm.count("confidence"));
    poisson.setDegree(degree);
    poisson.setDepth(depth);
    poisson.setMinDepth(minDepth);
    poisson.setIsoDivide(isoDivide);
    poisson.setManifold(vm.count("manifold"));
    poisson.setOutputPolygons(vm.count("output_polygons"));
    poisson.setSamplesPerNode(samplesPerNode);
    poisson.setScale(scale);
    poisson.setSolverDivide(solverDivide);

    poisson.setInputCloud (cloud_with_normals);
    poisson.setSearchMethod (tree2);
    poisson.reconstruct (triangles);
  }
  else if (vm.count("mcr")) {
    pcl::MarchingCubesRBF<pcl::PointNormal> mcr;
    mcr.setInputCloud (cloud_with_normals);
    mcr.setSearchMethod (tree2);
    mcr.reconstruct (triangles);
  }
  else if (vm.count("mch")) {
    pcl::MarchingCubesHoppe<pcl::PointNormal> mch;
    mch.setInputCloud (cloud_with_normals);
    mch.setSearchMethod (tree2);
    mch.reconstruct (triangles);
  }
  else if (vm.count("gp")) {
    pcl::GridProjection<pcl::PointNormal> gp;
    gp.setInputCloud (cloud_with_normals);
    gp.setSearchMethod (tree2);
    gp.reconstruct (triangles);
  }

#if 0
  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
#endif

#if 0
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
  *mesh = triangles;
  mesh = smoothSurface(mesh);
  pcl::io::saveVTKFile (output, *mesh);
#endif
  pcl::io::saveVTKFile (output, triangles);

  std::cout << output << " successfully saved" << std::endl;

  // Finish
  return (0);
}

