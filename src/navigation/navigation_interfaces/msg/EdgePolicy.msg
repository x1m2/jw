# 是否应用该边提供的速度上限；false时由控制器使用配置最大值。
bool velocity_limit_enabled
float64 linear_velocity
float64 angular_velocity

# 1：前进；-1：倒车。
int8 motion_direction

bool obstacle_stop_enabled
bool obstacle_avoidance_enabled
float64 obstacle_avoidance_distance_m
