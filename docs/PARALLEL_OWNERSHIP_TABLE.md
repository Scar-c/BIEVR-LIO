# Parallel Ownership Table (audit of the current implementation)

| Region | Parallel unit | Reads | Writes | Write ownership | Shared mutable state | Deterministic? |
|---|---|---|---|---|---|---|
| msgToPointcloud parse (conversions.h) | point | msg buffer | cloud col(i) | PER_INDEX_EXCLUSIVE | none | yes |
| filterMinMaxRange test (preprocess.h) | point | cloud | std::vector<char> mask | PER_INDEX_EXCLUSIVE | none | yes |
| transformPoints / undistortCloud | serial | cloud | cloud | serial single write | - | yes |
| intensity preprocessing (intensity_processor.cpp) | point | cloud | point_pixel_idx[i], filtered(0,i) | PER_INDEX_EXCLUSIVE | none | yes |
| voxelDownsample (preprocess.cpp) | point | cloud | entries[i] | PER_INDEX_EXCLUSIVE | none | yes |
| sampleInformed hash (preprocess.cpp) | point | cloud, map (read) | entries[i] | PER_INDEX_EXCLUSIVE | map read-only | yes |
| sampleInformed score | voxel | map (read) | voxel_scores[i] | PER_INDEX_EXCLUSIVE | map read-only | yes |
| sampleIntensityPoints hash+observed | point | cloud, map (read) | entries[i], observed_flag[i] (uint8_t) | PER_INDEX_EXCLUSIVE | map read-only | yes (post-FIX) |
| intensity downsample | point | candidates | DownsampleEntry[i] | PER_INDEX_EXCLUSIVE | none | LATENT (dist tie) |
| map integratePoints hash+sort | point | cloud | hashed_points[i] | PER_INDEX_EXCLUSIVE | none | yes |
| map per-voxel update | voxel | its group | its own Voxel images | PER_VOXEL_EXCLUSIVE | map_ read-only in parallel phase | yes |
| linearizeGeometry / linearizePhotometric | point | map (read) | accumulator | REDUCTION (TBB deterministic) | map read-only | yes |
| forEachVoxel / debug publish | voxel | map | output | serial iteration | unordered_map order | LATENT (output-only) |

Current vector<bool> race: FIXED (uint8_t observed_flag).

Latent risks (NOT fixed this round): downsample (hash,dist) equal-key ordering;
selectTopIntensityVoxels score-tie ordering; unordered_map iteration for
debug-only publishing.
