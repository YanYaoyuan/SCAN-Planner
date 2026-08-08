std_msgs/Header header

# 当前允许控制器执行的局部轨迹身份。traj_id 会在规划进程重启后复用，
# 因此控制器必须同时核对 trajectory_start_time，不能只比较 traj_id。
int64 traj_id
builtin_interfaces/Time trajectory_start_time

# FSM 根据真实 odom 空间投影得到的单调进度。控制器重启或晚加入时可恢复到
# 当前轨迹位置，避免从 transient B-spline 的起点重新追赶。
float64 progress_time
float64 progress_arc_length

# true 表示规划器仍存活且允许执行上述轨迹；false 要求控制器立即停车。
bool motion_allowed
