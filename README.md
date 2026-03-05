# Bridge-inspection-drone
## 一、项目背景
面向桥梁底部结构检测任务中GNSS信号遮挡与大尺度下传统SLAM退化问题，构建融合传统SLAM与动力学约束的无人机自主导航系统，实现从GNSS覆盖区域到桥底拒止环境的连续位姿估计与稳定建图，保障检测任务可靠执行。
项目核心思路是将载体动力学模型作为物理约束引入导航系统，与视觉惯性等多源信息紧耦合融合，从而提升系统精度与鲁棒性。
## 二、项目目标
1. 构建融合飞行力学与数据驱动学习的无人机动力学模型
2. 实现无人机动力学模型与多源传感器融合的自主导航方法，并同步建图，传感器数量≥3
3. 通过多场景试验与评估，验证算法有效性
## 三、开发日志
1.9 软降落打印并安装 & 转速采集并绘制四个电机曲线
运行：
scp nvidia@192.168.8.103:/home/nvidia/rpm_test/motor2_rpm_data_ros.xlsx .
rosbag record /uav3/mavros/esc_status -O motor1_rpm
转速测量值vs转速真实值测量平台
<img width="1280" height="960" alt="image" src="https://github.com/user-attachments/assets/523b3684-6292-474f-aae5-7cd4004bf5bd" />
<img width="1280" height="640" alt="image" src="https://github.com/user-attachments/assets/c7ae3a57-77b0-4361-a076-8d397ee0fd12" />
<img width="1280" height="640" alt="image" src="https://github.com/user-attachments/assets/d81416fb-015d-4db5-9a21-5dd6004c1eff" />
<img width="1280" height="640" alt="image" src="https://github.com/user-attachments/assets/66a40894-845b-4c30-ab16-f207800efa41" />
<img width="1280" height="640" alt="image" src="https://github.com/user-attachments/assets/dd8839fa-653a-4aeb-8b40-e2416fed1e70" />
***
1.10 起飞测试并录制视频 & 分析飞行日志
PX4 官方首推、最标准的振动/状态分析工具就是 Flight Review。
网址https://logs.px4.io/
如何分析图像：https://docs.px4.io/main/en/log/flight_review
注意梯子不要开全局要不然点上传没有任何反映，我使用规则模式香港节点
对于之前[03:09:51.572 ] Critical: Accel 0 clipping, not safe to fly! Land now, and check the vehicle setup. Clipping can lead to fly-aways.报错的问题
分析acc，看起来最有问题的几张图如下
<img width="840" height="336" alt="image" src="https://github.com/user-attachments/assets/c47d95fc-b696-485b-ac8c-5a5fe7005696" />
<img width="840" height="336" alt="image" src="https://github.com/user-attachments/assets/64b05e9b-cde5-4405-8cfb-df7c80cc9b3f" />
<img width="840" height="336" alt="image" src="https://github.com/user-attachments/assets/3add86e6-fb22-4342-8a95-57211e2a9e70" />
<img width="840" height="336" alt="image" src="https://github.com/user-attachments/assets/4173bbb2-fca8-4928-bfe2-a976ddd054f8" />
看了一下acc0和acc1果然有问题，并且倒数第二张图存在明显的亮带，大概在60-80HZ左右，这种频率一般是机械振动，不像是桨叶破损引发的振动
说明一直有一个跟转速无关的稳定振动源，有两种可能，一种是铝管，一种是电机安装的问题，明天可以换电机、拆铝管试一试
***
1.12 通过QGC逐个电机进行测试并更换问题电机1 & 调试动态滤波器参数
关于动态滤波器的主要作用和参数应该怎么选：
作用：DNF（动态滤波器）更多作用于 陀螺仪引发的控制振动，对机械共振的直接振幅抑制有限
参数怎么选:PX4提供两种动态陷波滤波的方式，一种是用FFT另一种是基于转速RPM，FFT这种方法主要是用于机架弯曲、螺丝松动、电机动平衡差等原因导致的机械结构上的共振；基于转速的话是用来处理跟电机转速有关的共振。
DNF其他的参数：
IMU_GYRO_DNF_EN        Enable IMU gyro dynamic notch filtering. 0: ESC RPM, 1: Onboard FFT.
IMU_GYRO_FFT_EN        Enable onboard FFT (required if IMU_GYRO_DNF_EN is set to 1).（已废弃，被融合到第一个参数中）
IMU_GYRO_DNF_MIN        Minimum dynamic notch frequency in Hz.
IMU_GYRO_DNF_BW        Bandwidth for each notch filter in Hz.
IMU_GYRO_DNF_HMC        Number of harmonics to filter.
这些参数都要基于Flight Reviewer的图像进行确定
有个问题：动态滤波器的IMU_GYRO_DNF_EN这个参数的描述是：启用IMU陀螺仪动态陷波滤波。可见它似乎是用于陀螺仪滤波的，所以我怀疑调整这个动态滤波器之后是否对PSD图的黄带优化有用？
GPT回答：在飞行中，加速度计看到的“振动”，有很大一部分不是“纯机械输入”，
 而是“控制器 → 电机 → 机架 → 传感器”的闭环放大结果。

也就是说acc很大的原因之一是因为陀螺仪测量的信息不够纯净，夹杂了很多噪声进而影响了控制，然后控制当然也含有噪声，从而导致更大的抖动反过来影响加速度计，就产生了PSD图上的黄带

 PX4 要“优先滤 gyro，而不是 acc”

这是一个设计哲学问题：

- 控制器用 gyro
- 控制器不用 acc 做姿态控制
我发现电机在低速的的时候振动就是会很大。但是稍微高一点转速就好了
但是电机1的振动疑似有点太大了，我现在更换了电机1，重新分析日志
<img width="840" height="336" alt="image" src="https://github.com/user-attachments/assets/f509e2bb-b5d1-4280-9bba-38da68d6939b" />
<img width="840" height="336" alt="image" src="https://github.com/user-attachments/assets/94c4760b-8641-4462-896b-80bbeac6cff7" />
<img width="840" height="336" alt="image" src="https://github.com/user-attachments/assets/1bd745d2-ce83-46e7-8e81-2dacda88b67d" />
另外，如果需要更专业的分析，可以启用高速率日志记录模式（SDLOG_PROFILE参数）。
确保至少打开
3    System identification   高速 actuators + IMU
4    High rate   RC / attitude / rates / actuators
然后将日志上传到Flight reviewer就可以看到如下图表（下面图表是我没有开启动态滤波器只更换电机1并重新校准后得到的）
<img width="840" height="336" alt="image" src="https://github.com/user-attachments/assets/0d086f6c-9a18-4156-9ed8-89822e0fdbd4" />
<img width="840" height="336" alt="image" src="https://github.com/user-attachments/assets/16f3665e-0903-4a6c-aba6-dca2a950f7f0" />
现在的设置
IMU_GYRO_DNF_EN   = FFT
IMU_GYRO_DNF_MIN  = 50
 IMU_GYRO_DNF_BW   = 20
 IMU_GYRO_DNF_HMC  = 3（默认就是3）
***
1.14  fastlivo2标定
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.08 image:=/fisheye/left/image_raw --fisheye-fix-skew --fisheye-recompute-extrinsicsts --fisheye-check-conditions
左目标定结果
**** Calibrating ****
mono fisheye calibration...
D = [0.05685265889699948, 0.022598162864202978, -0.011105214423262703, 0.0012869685246281362]
K = [435.28279841838537, 0.0, 970.4100579631286, 0.0, 436.20190248022936, 586.9770333444152, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [435.28279841838537, 0.0, 970.4100579631286, 0.0, 0.0, 436.20190248022936, 586.9770333444152, 0.0, 0.0, 0.0, 1.0, 0.0]
None
# oST version 5.0 parameters

[image]

width
1920

height
1200

[narrow_stereo]

camera matrix
435.282798 0.000000 970.410058
0.000000 436.201902 586.977033
0.000000 0.000000 1.000000

distortion
0.056853 0.022598 -0.011105 0.001287

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
435.282798 0.000000 970.410058 0.000000
0.000000 436.201902 586.977033 0.000000
0.000000 0.000000 1.000000 0.000000
右目标定
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.08 image:=/fisheye/right/image_raw --fisheye-fix-skew --fisheye-recompute-extrinsicsts --fisheye-check-conditions
**** Calibrating ****
mono fisheye calibration...
D = [0.058237004740866787, 0.01885304994159972, -0.009441797510725339, 0.00025149362613982997]
K = [444.15218586439505, 0.0, 957.5603415381394, 0.0, 444.2510264832877, 611.2144408860147, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [444.15218586439505, 0.0, 957.5603415381394, 0.0, 0.0, 444.2510264832877, 611.2144408860147, 0.0, 0.0, 0.0, 1.0, 0.0]
None
# oST version 5.0 parameters

[image]

width
1920

height
1200

[narrow_stereo]

camera matrix
444.152186 0.000000 957.560342
0.000000 444.251026 611.214441
0.000000 0.000000 1.000000

distortion
0.058237 0.018853 -0.009442 0.000251

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
444.152186 0.000000 957.560342 0.000000
0.000000 444.251026 611.214441 0.000000
0.000000 0.000000 1.000000 0.000000
***
1.16 相机雷达联合标定
我在我自己的小nuc上部署了fastcalib环境
录制bag包（左左右三个位置，每个位置30s）
以中间为例（由于fastlivo2是单目，目前用的是左目）
rosbag record -O middle_place \
  /livox/lidar \
  /fisheye/right/image_rect \
  /fisheye/left/image_rect
不过我尝试了fastcalib的多视角联合标定，发现效果很差（0.14m左右）如果是单视角标定，误差大概在4mm以内，所以最后我用的还是单视角（中间哪个视角）
标定结果如下图
<img width="960" height="600" alt="image" src="https://github.com/user-attachments/assets/26d71c4a-6670-48f3-915f-05ca5626f321" />
<img width="1280" height="668" alt="image" src="https://github.com/user-attachments/assets/38650c68-981c-4ecf-8cd3-7a7330ec162c" />
<img width="733" height="479" alt="image" src="https://github.com/user-attachments/assets/c1491e22-78f6-49d4-ab2c-8262b9deaea1" />
***
1.20 根据教程重新设置动态滤波器等参数（使用RPM滤波而不是FFT）
发现一个油管上比较好的PX4 PID调节的教程：
PX4 Ultimate Tuning Guide - Part 2: Gyro Filtering - YouTube
这个博主说一般用RMP滤波，而且其实esc_status里的转速本来就是主要是滤波用的，给你上层接口只是顺手而已...
根据教程调节之后发现似乎、好像好了一点，但是现在的情况是Z轴很漂其实我感觉只要解决z轴的话倒也还好，其次的问题就是滞后比较严重，我觉得issue上那个人说的对，需要测试单帧的延迟，如果超过100ms（因为定位源是10HZ输出，不论你开不开相机），目前打算换一根线（减重并提速），然后在进行测试延迟，这个等后面再说
或许应该使用动捕作为定位源进行对比，看看z是否真的会漂
总之现在滤波器和应该设置的东西基本都设置完了，后天（明天要帮禹翔跑代码）试验一下pid自动调节
我又修改了yaml的一些参数，并取消了相机选项，但是z轴还是会漂
以下是我的最新一次相关的测试结果
<img width="840" height="336" alt="image" src="https://github.com/user-attachments/assets/500c59a6-6d36-48ca-81b3-46be3fc23564" />
另外室内的话EV_CTRL是要全部勾选比较好，因为EV_CTRL是控制外部的定位源如何影响EKF2，它没有外部的定位源的情况下自然是拿到的信息越多越好
***
1.31动捕标定
动捕设置：默认y轴是向上的，但是可以自己修改，如PX4官方教程所示：https://docs.px4.io/main/en/ros/external_position_estimation#optitrack-mocap
Specific System Setups部分https://v20.wiki.optitrack.com/index.php?title=Template:Coordinate_System，motive中是有advance network setting的，应该是在Data Streaming里
设置完成之后，现在Z轴是朝上的，带上L型矫正杆，短轴摆放的方向就是X轴的正方向（也就是说，短轴朝向就是决定了动捕系统的X的正方向）
打开动捕软件，左键拖动动捕球，右键然后好像是第一个还是第二个，然后又一个子选项卡，里面好像有个create rigid from...
roslaunch vrpn_client_ros sample.launch server:=192.168.8.246(楼下实验室的电脑连接我的小路器之后的IP)

rosrun topic_tools relay \
  /vrpn_client_node/uav1/pose \
  /mavros/vision_pose/pose
(uav1是motive里面的刚体名称，应该可以在属性里改)
rostopic echo 话题名 >>data.txt（可选）
通过上述命令，可以通过动捕给的位姿飞行了
ros包我已经上传百度网盘（vrpn_ws）
室内动捕设置（已重新标定：门口中间那个动捕摄像头是歪的，24年左右标定文件不能用）：
动捕文件使用方法，打开我保存的那个final配置文件即可。
另外设置xy方向的时候不用管它那个地面的标识是怎么标的，好像它把z给标到地面上了，无所谓你只要按要求放置（短轴从拐角点到伸长点的方向是x轴的正方向），然后在Advanced Network Settings把z轴设置成朝上就行。
***
2.1 PX4 自动调参
官方教程：
https://docs.ncnynl.com/en/px4/zh/config/autotune_fw.html
还有个视频：
https://www.youtube.com/watch?v=5xswOhhqrIQ&t=1s
可以看到只有角速度控制器和姿态控制器有自动调参这个选项，然后速度控制器和位置控制器只能手调
***
2.4 室外测试
环绕模式果然是只有在飞机起飞之后才能设置的，起飞之前左键根本没有这个选项
MC_ORBIT_RAD_MAX可以设置环绕最大半径
MC_ORBIT_YAW_MODE可以设置环绕时的机头指向
另外环绕模式会一直转圈知道你停下来，停下来的方法就是切换到别的模式，如果你不想用其他模式，可以短暂的切换到别的模式之后然后再快速切换回去，比如我是使用position模式起飞，然后在地图上点一个点，然后左键点击地图上的某个点，然后选择环绕模式，飞机就会以当前点为圆心做圆周运动，但是实际上QGC的所谓环绕模式也是用多边形去近似圆形，如果圆设置的太小的话，多边形会很明显，如下图所示：
<img width="296" height="190" alt="image" src="https://github.com/user-attachments/assets/b46301b0-9afc-4cb0-8d9a-52e406dc7f63" />
建议是使用position模式起飞，如果你想退出环绕模式，可以直接短暂的切换到高度模式或者自稳模式然后快速的切换会position模式，当你再次切换会position模式，这个时候就已经退出环绕模式了
但是注意，切换的时候要确保自己是知道机头的指向的，要不然如果距离很远的话可能看不清楚机头的朝向，导致遥控器错误的打杆
客机经过的时候终端出现的报错：
[ INFO] [1770118324.711610872]: FCU: Armed by Stick gesture     [ INFO] [1770118324.818898968]: FCU: [logger] /fs/microsd/log/2026-02-04/08_51_40.ulg   [ INFO] [1770118327.004365400]: FCU: Takeoff detected   [ERROR] [1770118365.882104920]: serial0: write: No such device
terminate called after throwing an instance of 'std::system_error'
  what():  Resource deadlock avoided
发现即使客机走了好一会儿之后还是不太行，wifi也连不上，RTK也搜索不到星，客服说确实存在干扰，过了一会儿才链接上
rosrun topic_tools relay /mavros/vision_pose/pose /uav3/mavros/vision_pose/pose
rosbag record -O outdoor_2_9_3 \
/uav3/mavros/global_position/raw/fix \
/uav3/mavros/global_position/raw/gps_vel \
/uav3/mavros/global_position/raw/satellites \
/uav3/mavros/global_position/global \
/uav3/mavros/gpsstatus/gps1/raw \
/uav3/mavros/vision_pose/pose \
/uav3/mavros/local_position/odom \
/uav3/mavros/esc_info \
/uav3/mavros/esc_status \
/uav3/mavros/imu/data_raw \
/uav3/mavros/imu/mag \
/livox/imu \
/livox/lidar \
/path \
/tf \
/tf_static
<img width="1280" height="696" alt="image" src="https://github.com/user-attachments/assets/31055414-721b-4fe3-9115-e4efec31dbe7" />
***
2.8 QGC导入plan文件 && Mission花式设置
首先确定四个点：双扭线的上下左右，以及你希望的飞行高度
然后运行/home/cyf/Apps/QGroundControl/flight_plan/gen_double.py这个代码
<img width="1280" height="782" alt="image" src="https://github.com/user-attachments/assets/4d7ca6fa-375e-4033-ac5d-cc70e2fa6258" />
***
## 四、实现效果
gazebo仿真（一键起飞+定高自动切offboard+ego-planner避障+lio2定位，并用rviz实时展示lio、真值、融合值）
（这里是视频）
实物无人机外观
<img width="1280" height="959" alt="image" src="https://github.com/user-attachments/assets/db534af0-2fcc-4d5e-bfdc-6d52097e78c5" />

室内

室外（开阔）
（这里是视频）
室外（由开阔 -> 拒止过渡环境）
（这里是视频）
速度对比
<img width="1280" height="720" alt="image" src="https://github.com/user-attachments/assets/5d5f8c3b-bb5f-414b-baeb-081e3d557bab" />
<img width="1280" height="720" alt="image" src="https://github.com/user-attachments/assets/9e025f2b-7837-49f6-98f2-582596b4d118" />
XYZ位置结果对比
<img width="640" height="480" alt="image" src="https://github.com/user-attachments/assets/700a0a69-65b3-4a4a-ad0e-04c734b8ddde" />
<img width="640" height="480" alt="image" src="https://github.com/user-attachments/assets/64330ab6-c07c-4205-9820-fdd827fa8c3a" />
<img width="640" height="480" alt="image" src="https://github.com/user-attachments/assets/26c44f6b-0301-4b7d-8ff9-540de2a91dc6" />
<img width="640" height="480" alt="image" src="https://github.com/user-attachments/assets/8eacf7a5-3416-44ff-8dd2-4e56a4408bad" />
<img width="640" height="480" alt="image" src="https://github.com/user-attachments/assets/67e25fe2-3549-4be0-8686-434aa5aad1d3" />
<img width="640" height="480" alt="image" src="https://github.com/user-attachments/assets/a59228f4-5eda-4a07-a793-38ae39869b74" />
<img width="640" height="480" alt="image" src="https://github.com/user-attachments/assets/7362b63f-b918-438f-95fb-0ab01a153420" />
<img width="640" height="480" alt="image" src="https://github.com/user-attachments/assets/a8c8cf56-3b8c-4f79-b11c-e5c26927ca0a" />

