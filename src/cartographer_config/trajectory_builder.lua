TRAJECTORY_BUILDER = {}

TRAJECTORY_BUILDER_3D = {
  num_accumulated_range_data = 1,
  min_range = 1.0,
  max_range = 100.0,
  voxel_filter_size = 0.1,
  submaps = {
    high_resolution = 0.05,
    high_resolution_max_range = 20.0,
    low_resolution = 0.15,
  },
  ceres_scan_matcher = {
    translation_weight = 500.0,
    rotation_weight = 400.0,
  },
}

return TRAJECTORY_BUILDER
