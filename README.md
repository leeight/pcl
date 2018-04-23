**The Commandline Arguments**

1. `--enable_voxel_grid_filter` Use `pcl::VoxelGrid` to filter the points, reduce the points set.
2. `--enable_mls` Use `pcl::MovingLeastSquares` to smooth the points, The MLS smoothing is done before any reconstruction algorithm is applied.
3. `--enable_gp3` Use `pcl::GreedyProjectionTriangulation` to reconstruction the surface.
4. `--enable_laplacian` Use `pcl::MeshSmoothingLaplacianVTK` to smooth the surface, which is applied after the reconstruction is complete.


**Examples**

```
./test \
    --enable_voxel_grid_filter
    --enable_mls
    --enable_gp3
    --enable_laplacian
    -f xxx.pcd
```
