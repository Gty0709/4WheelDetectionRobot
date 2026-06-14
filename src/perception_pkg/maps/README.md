# Local map sessions (not in git)

建图输出保存在 `map_<时间戳>/`（栅格图、航点、rosbag 等），体积大，已通过根目录 `.gitignore` 排除。

本地建图后 `map_latest` 会指向最近一次会话；克隆仓库后需重新建图或从别处拷贝地图目录。
