# Nav-Grasp 实机程序

本目录集中存放 ROS 2 Humble 实机程序。三个子目录保留各自独立的
`src`、构建产物和启动脚本，避免 SLAM、视觉抓取和任务编排互相污染。

```text
nav_grasp_real/
└── src/
    ├── slam/          # SLAM、Nav2、底盘/雷达和机器人描述
    ├── visual_grasp/  # 相机标定、目标检测、抓取规划与机械臂执行
    └── nav_grasp/     # 导航—抓取—返航任务状态机
```

在仓库根目录构建全部实机工作区：

```bash
./scripts/build_all.sh
```

各工作区的安装、配置与运行方式见：

- [`src/slam/README.md`](src/slam/README.md)
- [`src/visual_grasp/README.md`](src/visual_grasp/README.md)
- [`src/nav_grasp/README.md`](src/nav_grasp/README.md)

> 实机程序会控制底盘和机械臂。运行前必须确认标定、工作空间和实体急停。
