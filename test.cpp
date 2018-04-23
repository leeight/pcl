#include <iostream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_subdivision.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>

namespace po = boost::program_options;

struct Gp3Options {
  double search_radius = 0.028;
  double mu = 2.5;
  double maximum_nearest_neighbors = 100;
};

struct VoxelGridOptions {
  float leaf_size = 0.01f;
};

struct LaplacianOptions {
  int num_iter = 20000;
  double convergence = 0.0f;
  double relaxation_factor = 0.01f;
  float edge_angle = 15.f;
  float feature_angle = 45.f;
};

struct SincOptions {
  int num_iter = 20000;
};

struct SubdivisionOptions {
  // linear, loop, butterfly
  std::string filter_type = "linear";
};

struct MlsOptions {
#if 0
  // ProjectionMethod { NONE, SIMPLE, ORTHOGONAL }
  std::string projection_method = "simple";
#endif
  // UpsamplingMethod { NONE, DISTINCT_CLOUD, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION }
  std::string upsampling_method = "none";
  double search_radius = 0.03;
  double upsampling_radius = 0.0;
  double upsampling_step = 0.0;
  bool compute_normals = true;
  int dilation_iterations = 0;
  float voxel_size = 1.0f;
  int point_density = 0;
  int polynomial_order = 2;
};

void greedyProjectionTriangulation(boost::shared_ptr<pcl::PolygonMesh> triangles,
                                   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals,
                                   Gp3Options &gp3,
                                   std::string &input) {
  std::cerr << ">> Begin Greedy Projection Triangulation...";
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree;
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gt;
  gt.setInputCloud(cloud_with_normals);
  gt.setSearchMethod(tree);
  gt.setSearchRadius(gp3.search_radius);
  gt.setMu(gp3.mu);
  gt.setMaximumNearestNeighbors(gp3.maximum_nearest_neighbors);
  gt.setMaximumSurfaceAngle(M_PI / 4);
  gt.setMinimumAngle(M_PI / 18);
  gt.setMaximumAngle(2 * M_PI / 3);
  gt.setNormalConsistency(false);
  gt.reconstruct(*triangles);
  std::cerr << "Done.\n";

  std::cerr << ">> Saving [1]...";
  std::string vtk_1_output = boost::replace_all_copy(input, ".pcd", "-1.vtk");
  std::string obj_1_output = boost::replace_all_copy(input, ".pcd", "-1.obj");
  pcl::io::saveVTKFile(vtk_1_output, *triangles);
  pcl::io::saveOBJFile(obj_1_output, *triangles);
  std::cerr << "Done.\n";
}

void laplacianSmoothing(boost::shared_ptr<pcl::PolygonMesh> triangles,
                        LaplacianOptions &laplacian,
                        std::string &input) {
  std::cerr << ">> Begin Laplacian Smoothing...";
  pcl::PolygonMesh output;
  pcl::MeshSmoothingLaplacianVTK vtk;
  vtk.setInputMesh(triangles);
  vtk.setNumIter(laplacian.num_iter);
  vtk.setConvergence(laplacian.convergence);
  vtk.setRelaxationFactor(laplacian.relaxation_factor);
  vtk.setFeatureEdgeSmoothing(true);
  vtk.setFeatureAngle(laplacian.feature_angle);
  vtk.setEdgeAngle(laplacian.edge_angle);
  vtk.setBoundarySmoothing(true);
  vtk.process(output);
  std::cerr << "Done.\n";

  std::cerr << ">> Saving [2]...";
  std::string vtk_2_output = boost::replace_all_copy(input, ".pcd", "-2.vtk");
  std::string obj_2_output = boost::replace_all_copy(input, ".pcd", "-2.obj");
  pcl::io::saveVTKFile(vtk_2_output, output);
  pcl::io::saveOBJFile(obj_2_output, output);
  std::cerr << "Done.\n";
}

void sincSmoothing(boost::shared_ptr<pcl::PolygonMesh> triangles,
                   SincOptions &sinc,
                   std::string &input) {
  std::cerr << ">> Begin Sinc Smoothing...";
  pcl::PolygonMesh output;
  pcl::MeshSmoothingWindowedSincVTK vtk;
  vtk.setInputMesh(triangles);
  vtk.setNumIter(sinc.num_iter);
  vtk.process(output);
  std::cerr << "Done.\n";

  std::cerr << ">> Saving [3]...";
  std::string vtk_3_output = boost::replace_all_copy(input, ".pcd", "-3.vtk");
  std::string obj_3_output = boost::replace_all_copy(input, ".pcd", "-3.obj");
  pcl::io::saveVTKFile(vtk_3_output, output);
  pcl::io::saveOBJFile(obj_3_output, output);
  std::cerr << "Done.\n";
}

void subdivisionSmoothing(boost::shared_ptr<pcl::PolygonMesh> triangles,
                          SubdivisionOptions &subd,
                          std::string &input) {
  auto filter_type = pcl::MeshSubdivisionVTK::MeshSubdivisionVTKFilterType::LINEAR;
  if (subd.filter_type.compare("loop") == 0) {
    filter_type = pcl::MeshSubdivisionVTK::MeshSubdivisionVTKFilterType::LOOP;
  }
  else if (subd.filter_type.compare("butterfly") == 0) {
    filter_type = pcl::MeshSubdivisionVTK::MeshSubdivisionVTKFilterType::BUTTERFLY;
  }
  std::cerr << ">> Begin Subdivision Smoothing...";
  pcl::PolygonMesh output;
  pcl::MeshSubdivisionVTK vtk;
  vtk.setFilterType(filter_type);
  vtk.setInputMesh(triangles);
  vtk.process(output);
  std::cerr << "Done.\n";

  std::cerr << ">> Saving [4]...";
  std::string vtk_4_output = boost::replace_all_copy(input, ".pcd", "-4.vtk");
  std::string obj_4_output = boost::replace_all_copy(input, ".pcd", "-4.obj");
  pcl::io::saveVTKFile(vtk_4_output, output);
  pcl::io::saveOBJFile(obj_4_output, output);
  std::cerr << "Done.\n";
}

pcl::PointCloud<pcl::PointNormal>::Ptr mlsSmoothing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree,
                                                    MlsOptions &mlsp) {
  std::cerr << ">> Smoothing with MLS...";
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_normals(new pcl::PointCloud<pcl::PointNormal>);

#if 0
  auto projection_method = pcl::MLSResult::SIMPLE;
  if (mlsp.projection_method.compare("none") == 0) {
    projection_method = pcl::MLSResult::NONE;
  }
  else if (mlsp.projection_method.compare("orthogonal") == 0) {
    projection_method = pcl::MLSResult::ORTHOGONAL;
  }
#endif

  auto upsampling_method = mls.NONE;
  if (mlsp.upsampling_method.compare("distinct_cloud") == 0) {
    upsampling_method = mls.DISTINCT_CLOUD;
  }
  else if (mlsp.upsampling_method.compare("sample_local_plane") == 0) {
    upsampling_method = mls.SAMPLE_LOCAL_PLANE;
  }
  else if (mlsp.upsampling_method.compare("random_uniform_density") == 0) {
    upsampling_method = mls.RANDOM_UNIFORM_DENSITY;
  }
  else if (mlsp.upsampling_method.compare("voxel_grid_dilation") == 0) {
    upsampling_method = mls.VOXEL_GRID_DILATION;
  }


  mls.setInputCloud(cloud);
  mls.setSearchMethod(tree);
  mls.setComputeNormals(mlsp.compute_normals);
  // mls.setPolynomialFit(false);
  mls.setPolynomialOrder(mlsp.polynomial_order);
  mls.setSearchRadius(mlsp.search_radius);
  mls.setUpsamplingRadius(mlsp.upsampling_radius);
  mls.setUpsamplingStepSize(mlsp.upsampling_step);
  mls.setDilationIterations(mlsp.dilation_iterations);
  mls.setDilationVoxelSize(mlsp.voxel_size);
  mls.setPointDensity(mlsp.point_density);
  // mls.setProjectionMethod(projection_method);
  mls.setUpsamplingMethod(upsampling_method);
  mls.process(*mls_normals);
  std::cerr << "Done.\n";

  return mls_normals;
}

int main(int argc, char **argv) {
  std::string input;

  Gp3Options gp3;
  VoxelGridOptions voxel;
  LaplacianOptions laplacian;
  SincOptions sinc;
  SubdivisionOptions subd;
  MlsOptions mlsp;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Display help message")
    ("file,f", po::value<std::string>(&input), "The input pcd file")

    ("enable_voxel_grid_filter", "Enable pcl::VoxelGrid filter")
    ("voxel_leaf_size",
     po::value<float>(&voxel.leaf_size)->default_value(voxel.leaf_size),
     "Set the voxel grid leaf size.")

    ("enable_mls", "Enable pcl::MovingLeastSquares")
    ("mls_compute_normals",
     po::value<bool>(&mlsp.compute_normals)->default_value(mlsp.compute_normals),
     "Set whether the algorithm should also store the normals computed")
#if 0
    ("mls_projection_method",
     po::value<std::string>(&mlsp.projection_method)->default_value(mlsp.projection_method),
     "Set the method to be used when projection the point on to the MLS surface. (none, simple, orthogonal)")
#endif
    ("mls_upsampling_method",
     po::value<std::string>(&mlsp.upsampling_method)->default_value(mlsp.upsampling_method),
     "Set the upsampling method to be used. (none, distinct_cloud, sample_local_plane, random_uniform_density, voxel_grid_dilation)")
    ("mls_search_radius",
     po::value<double>(&mlsp.search_radius)->default_value(mlsp.search_radius),
     "Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting")
    ("mls_upsampling_radius",
     po::value<double>(&mlsp.upsampling_radius)->default_value(mlsp.upsampling_radius),
     "Set the radius of the circle in the local point plane that will be sampled.")
    ("mls_upsampling_step",
     po::value<double>(&mlsp.upsampling_step)->default_value(mlsp.upsampling_step),
     "Set the step size for the local plane sampling")
    ("mls_dilation_iterations",
     po::value<int>(&mlsp.dilation_iterations)->default_value(mlsp.dilation_iterations),
     "Set the number of dilation steps of the voxel grid.")
    ("mls_voxel_size",
     po::value<float>(&mlsp.voxel_size)->default_value(mlsp.voxel_size),
     "Set the voxel size for the voxel grid")
    ("mls_point_density",
     po::value<int>(&mlsp.point_density)->default_value(mlsp.point_density),
     "Set the parameter that specifies the desired number of points within the search radius.")
    ("mls_polynomial_order",
     po::value<int>(&mlsp.polynomial_order)->default_value(mlsp.polynomial_order),
     "Set the order of the polynomial to be fit.")

    ("enable_gp3", "Enable pcl::GreedyProjectionTriangulation")
    ("gp3_search_radius",
     po::value<double>(&gp3.search_radius)->default_value(gp3.search_radius),
     "Set the sphere radius that is to be used for determining the k-nearest neighbors used for triangulating")
    ("gp3_mu",
     po::value<double>(&gp3.mu)->default_value(gp3.mu),
     "Set the multiplier of the nearest neighbor distance to obtain the final search radius for each point (this will make the algorithm adapt to different point densities in the cloud).")
    ("gp3_maximum_nearest_neighbors",
     po::value<double>(&gp3.maximum_nearest_neighbors)->default_value(gp3.maximum_nearest_neighbors),
     "Set the maximum number of nearest neighbors to be searched for")

    ("enable_laplacian", "Enable pcl::MeshSmoothingLaplacianVTK")
    ("laplacian_num_iter",
     po::value<int>(&laplacian.num_iter)->default_value(laplacian.num_iter),
     "Set the number of iterations for the smoothing filter")
    ("laplacian_convergence",
     po::value<double>(&laplacian.convergence)->default_value(laplacian.convergence),
     "Specify a convergence criterion for the iteration process")
    ("laplacian_relaxation_factor",
     po::value<double>(&laplacian.relaxation_factor)->default_value(laplacian.relaxation_factor),
     "Specify the relaxation factor for Laplacian smoothing")
    ("laplacian_feature_angle",
     po::value<float>(&laplacian.feature_angle)->default_value(laplacian.feature_angle),
     "Specify the feature angle for sharp edge identification.")
    ("laplacian_edge_angle",
     po::value<float>(&laplacian.edge_angle)->default_value(laplacian.edge_angle),
     "Specify the edge angle to control smoothing along edges (either interior or boundary).")

    ("enable_sinc", "Enable pcl::MeshSmoothingWindowedSincVTK")
    ("sinc_num_iter",
     po::value<int>(&sinc.num_iter)->default_value(sinc.num_iter),
     "Set the number of iterations for the smoothing filter")

    ("enable_subd", "Enable pcl::MeshSubdivisionVTK")
    ("subd_filter_type", po::value<std::string>(&subd.filter_type)->default_value(subd.filter_type), "Set the mesh subdivision filter type (linear, loop, butterfly).")
  ;

  po::command_line_parser parser{argc, argv};
  parser.options(desc).allow_unregistered().style(
    po::command_line_style::default_style |
    po::command_line_style::allow_slash_for_short);
  po::parsed_options parsed_options = parser.run();

  po::variables_map vm;
  po::store(parsed_options, vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("file")) {
    std::cout << desc << std::endl;
    return 0;
  }

  std::cerr << ">> Loading file...";
  pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
  pcl::io::loadPCDFile(input, *cloud_blob);
  std::cerr << "Done.\n";

  bool enable_voxel_grid_filter = vm.count("enable_voxel_grid_filter");
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
  if (enable_voxel_grid_filter) {
    std::cerr << ">> Begin VoxelGrid..." << "  " << cloud_blob->width * cloud_blob->height << " - ";
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(voxel.leaf_size, voxel.leaf_size, voxel.leaf_size);
    sor.filter(*cloud_filtered);
    std::cerr << cloud_filtered->width * cloud_filtered->height << " - ";
    std::cerr << "Done.\n";
  }

  std::cerr << ">> Transform to PointXYZ...";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(enable_voxel_grid_filter ? *cloud_filtered : *cloud_blob, *cloud);
  std::cerr << "Done.\n";

  std::cerr << ">> Create search tree...";
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
  tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
  tree->setInputCloud(cloud);
  std::cerr << "Done.\n";

  std::cerr << ">> Normal estimation...";
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  n.setInputCloud(cloud);
  n.setSearchMethod(tree);
  n.setKSearch(20);
  n.compute(*normals);
  std::cerr << "Done.\n";

  bool enable_mls = vm.count("enable_mls");
  pcl::PointCloud<pcl::PointNormal>::Ptr mls_normals = NULL;
  if (enable_mls) {
    mls_normals = mlsSmoothing(cloud, tree, mlsp);
  }

  std::cerr << ">> Concatenate XYZ and normal information...";
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  if (enable_mls) {
    std::cerr << "   " << cloud->size() << " - " << mls_normals->size() << " - ";
    // 不清楚为什么，有时候 mls_normals->size() 会比 cloud->size() 小，导致 concatenateFields 的时候出错
    // 所以补充一下，保证 size 是一致的
    int diff = cloud->size() - mls_normals->size();
    for (int i = 0; i < diff; i++) {
      mls_normals->push_back(mls_normals->back());
    }
    pcl::concatenateFields(*cloud, *mls_normals, *cloud_with_normals);
  }
  else {
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
  }
  std::cerr << "Done.\n";

  // Shared in different stages
  boost::shared_ptr<pcl::PolygonMesh> triangles(new pcl::PolygonMesh);

  bool enable_gp3 = vm.count("enable_gp3");
  if (enable_gp3) {
    greedyProjectionTriangulation(triangles, cloud_with_normals, gp3, input);
  }

  bool enable_laplacian = vm.count("enable_laplacian");
  if (enable_laplacian) {
    laplacianSmoothing(triangles, laplacian, input);
  }

  bool enable_sinc = vm.count("enable_sinc");
  if (enable_sinc) {
    sincSmoothing(triangles, sinc, input);
  }

  bool enable_subd = vm.count("enable_subd");
  if (enable_subd) {
    subdivisionSmoothing(triangles, subd, input);
  }

  return 0;
}
