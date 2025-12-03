from .scene import ScenePCD, SceneMap, SceneTrav


class SceneCommon():
    pcd = ScenePCD()
    pcd.file_name = 'scans.pcd'  # 点云文件名

    map = SceneMap()
    map.resolution = 0.08  # 栅格分辨率(米/格)，越小越精细
    map.ground_h = -0.2     # 地面高度裁剪，过滤地面以下噪点
    map.slice_dh = 2     # 高度切片间距(米)，越小层数越多、台阶更细分

    trav = SceneTrav()
    trav.kernel_size = 7       # 邻域核大小(奇数，格数)，越大统计越稳但易平滑细节
    trav.interval_min = 1   # 地面-顶面最小间隙阈值(米)，低于直接判障碍
    trav.interval_free = 0.3  # 间隙低于该值开始增加代价罚分(米)
    trav.slope_max = 0.40      # 可站立坡度上限(弧度)，越大越宽松
    trav.step_max = 0.2       # 可跨越台阶的最大单步高度差(米)，越大越易跨步
    trav.standable_ratio = 0.05  # 邻域内“可站立格”比例下限，越小越宽松
    trav.cost_barrier = 40.0   # 障碍代价值(屏障)，被判障碍的格子代价
    trav.safe_margin = 0.25    # 安全距离(米)，用于膨胀半径，越大越“加粗”障碍
    trav.inflation = 0.3      # 膨胀内核(米)，与safe_margin共同决定膨胀范围
