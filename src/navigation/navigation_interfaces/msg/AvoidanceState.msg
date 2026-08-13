# 当前没有执行局部绕障，允许动态路径正常更新。
uint8 IDLE=0
# 正在等待Costmap、局部路径，或正在跟踪局部A*路径。
uint8 AVOIDING=1
# 障碍物过近或左右均不可通行，控制器保持停车。
uint8 BLOCKED=2
# 动态路径已绕过障碍，停车等待一条更新鲜的动态参考路径。
uint8 WAITING_DYNAMIC_PATH=3
# 局部规划失败，保持停车直到障碍清除。
uint8 FAILED=4

uint8 state
