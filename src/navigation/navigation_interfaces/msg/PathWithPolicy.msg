nav_msgs/Path path
# 与path.poses逐点对齐；同一条边生成的路径点使用相同策略。
EdgePolicy[] edge_policies
# true表示路径会随目标位置持续变化，例如UWB人员跟随路径。
bool dynamic_path
