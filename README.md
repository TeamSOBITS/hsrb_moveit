

# Create Workspace

1. ワークスペースを作成し，必要なパッケージを取得します．

```
mkdir -p ~/hsr_ros2_ws/src && cd ~/hsr_ros2_ws/src
git clone -b humble https://github.com/hsr-project/hsrb_common.git
git clone -b humble https://github.com/hsr-project/hsrb_manipulation.git
git clone -b humble https://github.com/hsr-project/hsrb_moveit.git
git clone -b humble https://github.com/hsr-project/hsr_common.git
git clone -b humble https://github.com/hsr-project/tmc_common.git
git clone -b humble https://github.com/hsr-project/tmc_common_msgs.git
git clone -b humble https://github.com/hsr-project/tmc_manipulation.git
git clone -b humble https://github.com/hsr-project/tmc_manipulation_base.git
git clone -b humble https://github.com/hsr-project/tmc_manipulation_planner.git
```

2. colcon buildをします.

```
cd ~/hsr_ros2_ws/
source /opt/ros/humble/setup.bash
rosdep install --from-paths . -y --ignore-src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```


# Launch Examples

シミュレータ等を起動後，demo.launch.pyを実行します．ロボットに応じた適切なlaunchファイルを利用してください．
以下は，HSR-Bの例です．

```
ros2 launch hsrb_moveit_config hsrb_demo.launch.py
```

移動機構を含めたプランニングを実行する際に重みパラメータを用途ごとに調整してください(hsrb_moveit/config/kinematics.yaml)
```
whole_body_weighted:
  kinematics_solver: hsrb_moveit_kinematics/HSRBKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3
  joint_weight: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
  #arm_lift_joint, "arm_flex_joint, arm_roll_joint, wrist_flex_joint, wrist_roll_joint, odom_x, odom_y, odom_t
```

## ライブラリの起動
demo.launch.pyを起動後,以下のコマンドでライブラリを起動するとmoveitの簡易的な以下のサービスサーバーが立ち上がります.

`hsrb/moveit_to_tf`

```
ros2 launch hsrb_moveit_config hsrb_moveit_library.launch.py
```
メッセージの型は一時的に以下のサービスを使用しています．
Requestに関しては現在`target_frame`のみに対応しています

`GetHandToTargetTF.srv`
```
# Request
string target_frame                             # Frame name to be grasped
geometry_msgs/TransformStamped tf_differential  # Differential coordinates of Target frame
---
# Result
geometry_msgs/Pose move_pose                    # Moving pose for grasping
string[] target_joint_names                     # List of joint names to move
float64[] target_joint_rad                      # List of joint angles to move
bool success                                    # Enable grasp
string message                                  # Result message
```

## C++からのMoveGroupの利用

次のコマンドで，サンプルプログラムを実行できます．

```
ros2 launch hsrb_moveit_config hsrb_example.launch.py example_name:=moveit_fk_demo
```

準備されているサンプルプログラムは以下の通りです．

* `moveit_fk_demo`
* `moveit_ik_demo`
* `moveit_gripper_demo`
* `moveit_constraints_demo`

## GUIでの操作

RvizのMotionPlanningプラグインから指令値を投げてください．
