This program is about Mr. Inoue's master's thesis "Gazed object Estimation for Obtaining Detailed Time-usage Data Using a Mobile Robot".

# How to use (current version)

## Installation

1. install nvidia-docker2 and docker-compose

## Run application

1. enter docker environment

    ```
    $ docker-compose run gazed_object_estimation bash
    ```

1. make catkin_ws

    ```
    $ catkin_make -DCMAKE_CXX_FLAGS="-DOPENCV_TRAITS_ENABLE_DEPRECATED"
    ```

1. 以下のコマンドを実行

    ```
    $ roslaunch combi_darknet_openface combi_darknet_openface.launch
    ```

# How to use (old version)

### Set up

* 機器の接続

1. Pepperくんの電源を入れて，オートインタラクションモードをオフにして，通常の姿勢にします．
1. PCとPepperをLANケーブルで接続します．
1. LRFとXtionをPCに接続します．

* 実験準備

本環境はdocker上で構築しています．

1. terminatorを起動 //terminator -l ros1207
1. Dockerを起動

```
sudo nvidia-docker-compose run gazed_object_estimation bash
terminatorの各タブで
sudo docker exec -it gazed_object_estimation_run_1 bash
終了するときはdockerを使ってないタブで
sudo nvidia-docker-compose down
```

* プログラムの起動

1. カメラのキャリブレーションファイルを移動

```
cd /calibration
cp rgb_PS1080_PrimeSense.yaml /root/.ros/camera_info
```

2. 各タブで順番にプログラムを起動

```
roslaunch pepper_bringup pepper_full.launch nao_ip:=169.254.246.15 \\ペッパーのrosパッケージ
chmod 777 /dev/ttyACM0
roslaunch urg_node urg_node.launch \\LRFのノード
roslaunch openni2_launch openni2.launch \\Xtionのノード
rosrun openface_ros openface_ros _image_topic:=/camera/rgb/image_rect_color \\顔認識のノード
roslaunch darknet_ros darknet_ros.launch \\物体認識のノード
rosrun person_tracking_kalman person_tracking_kalman_node \\人物追跡のノード
cd ~/catkin_ws/src/pioneer_2dnav/launch/
roslaunch pepper_move_base.launch \\自己位置推定のノード \\起動したRviz上でロボットのマップとLRFの点群を位置合わせしてロボットの初期位置を決定してください．
rosrun simple_navigation_goals simple_navigation_goals \\ロボットの自己位置を出力するノード
rosrun teleop_twist_keyboard teleop_twist_keyboard.py \\キーボード操作ノード
cd catkin_ws/py_ws/timeuse_test/datatimeuse/
```

3. 最終的なプログラムの実行

```
rosrun combi_darknet_openface combi_darknet_openface_node | tee -a log.txt \\最終的なプログラム
```

# Tips

OpenFace requires OpenCV 3.2.0? & dlib 19.6?

## Authors
Tomoaki Inoue/ Akishige Yuguchi / Takumi Nakamura