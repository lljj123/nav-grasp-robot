# 贡献指南

本项目的初始版本由导航与 SLAM 负责人、视觉抓取负责人共同完成。
两个独立工作空间在完整任务中集成为总项目。

## 模块分工

| 路径 | 主要负责范围 |
| --- | --- |
| `slam_ws/` | 导航、SLAM、底盘/雷达接口及机器人 TF，由 `@lljj123` 负责 |
| `visual_grasp_ws/` | 视觉检测、手眼标定、抓取规划及机械臂执行，由 `@GGbond61` 负责 |
| `nav_grasp_ws/` | 导航—抓取—返航任务编排，由 `@lljj123` 负责 |
| `config/`、`scripts/`、`.github/` | 公共配置、构建和工程化，双方共同维护 |

## 开发流程

1. 从最新的 `main` 创建分支，不直接在 `main` 上开发。
2. 分支建议使用 `nav/`、`grasp/`、`integration/`、`fix/` 或 `docs/` 前缀。
3. 只提交源码和可公开配置；不提交地图、标定结果、实机坐标、密钥或本机路径。
4. 在本地完成与修改相关的构建和测试，然后提交 Pull Request。
5. 跨模块修改需要另一位负责人审查；实机行为变更要说明安全默认值和验证方式。

示例：

```bash
git switch main
git pull --ff-only
git switch -c nav/update-costmap

# 修改、构建和测试
git add <files>
git commit -m "nav: update costmap configuration"
git push -u origin nav/update-costmap
```

## 验证

修改提交前至少运行与改动范围相关的检查：

```bash
./scripts/build_all.sh
colcon test --base-paths visual_grasp_ws/src nav_grasp_ws/src
colcon test-result --verbose
```

若当前机器缺少 ROS 2 或硬件驱动，应在 Pull Request 中明确写出未执行的
检查及原因，不得将“未运行”记为“已通过”。

## Commit 身份

每位贡献者使用自己的 GitHub 账号邮箱或 GitHub `noreply` 邮箱提交。
只有真正共同完成的提交才添加共同作者：

```text
Co-authored-by: Name <github-email-or-noreply-address>
```

## 实机安全

- Pull Request 不得默认启用机械臂运动、夹爪豁免或未确认的自主导航。
- 实机测试先使用低速、无负载、可随时急停的配置。
- 变更 TF、TCP、工作空间、夹爪限位或地图坐标时，必须附带本机验证记录。
