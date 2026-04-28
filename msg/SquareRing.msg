# 方环检测结果
# 用于描述穿环目标的位置信息

Header header                     # 时间戳和坐标系

geometry_msgs/Point[] corners     # 方环4个角点 (3D, 按顺时针/逆时针排列)
geometry_msgs/Point center_point  # 方环中心点
geometry_msgs/Point front_point   # 前方点 (靠近无人机一侧)
geometry_msgs/Point back_point    # 后方点 (远离无人机一侧)

float64 width                     # 方环宽度 (m)
float64 height                    # 方环高度 (m)
