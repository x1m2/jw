// Unmodified cut_voxel from the pinned upstream source.
void cut_voxel(unordered_map<VOXEL_LOC, OCTO_TREE*>& feature_map,
               pcl::PointCloud<PointType>::Ptr feature_pts,
               Eigen::Quaterniond q, Eigen::Vector3d t, int f_head,
               int window_size, double eigen_threshold,
               bool is_base_lidar = true)
{
	uint pt_size = feature_pts->size();
	for(uint i = 0; i < pt_size; i++)
	{
		PointType& pt = feature_pts->points[i];
		Eigen::Vector3d pt_origin(pt.x, pt.y, pt.z);
		Eigen::Vector3d pt_trans = q * pt_origin + t;

		float loc_xyz[3];
		for(int j = 0; j < 3; j++)
		{
			loc_xyz[j] = pt_trans[j] / voxel_size;
			if(loc_xyz[j] < 0)
				loc_xyz[j] -= 1.0;
		}

		VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
		auto iter = feature_map.find(position);
		if(iter != feature_map.end())
		{
      if(is_base_lidar)
      {
        iter->second->baseOriginPc[f_head]->emplace_back(pt_origin);
        iter->second->baseTransPc[f_head]->emplace_back(pt_trans);
      }
      else
      {
        iter->second->refOriginPc[f_head]->emplace_back(pt_origin);
        iter->second->refTransPc[f_head]->emplace_back(pt_trans);
      }
		}
		else
		{
      OCTO_TREE* ot = new OCTO_TREE(window_size, eigen_threshold);
      if(is_base_lidar)
      {
        ot->baseOriginPc[f_head]->emplace_back(pt_origin);
        ot->baseTransPc[f_head]->emplace_back(pt_trans);
      }
      else
      {
        ot->refOriginPc[f_head]->emplace_back(pt_origin);
        ot->refTransPc[f_head]->emplace_back(pt_trans);
      }

      ot->voxel_center[0] = (0.5 + position.x) * voxel_size;
      ot->voxel_center[1] = (0.5 + position.y) * voxel_size;
      ot->voxel_center[2] = (0.5 + position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      ot->layer = 0;
      feature_map[position] = ot;
		}
	}
}

