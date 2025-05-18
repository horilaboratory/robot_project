# 7. Python でトピックをパブリッシュするまでの流れ

## Topic（トピック）とは何か
　ROS2 の世界における **Topic（トピック）** とは、改めてどんなものなのか、ROS2 の世界を **例をもとに** 解説しましょう。。

---

　まず、ROS2 トピックとは ROS2 公式の見解のもとに解説すると以下の通りです。

> *Topic は Node（ノード）間が Message（メッセージ）をやりとりするための **通信チャンネル** であり、Node は Topic を介して Message を Publish（パブリッシュ）および Subscribe（サブスクライブ）する。*

　ノードは **プログラムそのもの** であり、人間に例えると **人間そのもの** を指します。<br>
　Topic は人間（Node）同士でやり取りするための **通信チャンネル** です。例えば Youtube にて Youtuber 「Catkin_TV」が動画を配信していたとしましょう。これを ROS2 の世界で解説するとこのようになります。

> ノード「Catkin_TV」が Topic名「Youtube」で動画データを Publish している

　Youtuber「Catkin_TV」は「Youtube」という通信チャンネル（Topic）で動画データを Publish（配信）することで、他のノードが Topic「Youtube」を Subscribe（購読）することで配信されている動画を Subscribe している他の人間（Node）が視聴することができます。
<br>
　ロボット開発において、ROS2 の Topic を活用することでさまざまな Node たちがあらゆるロボットのデータを Topic を介して Publish することで、他の Subscriber Node（Topic を Subscriber する Node）がロボットのデータを取得し、ロボットを制御することができるようになります。

---

　ROS2 において現在発行されている Topic 、つまり現在利用できるチャンネル一覧を表示するには、ターミナルにて以下のコマンドを実行します。
```bash
ros2 topic list
```
このコマンドを実行すると、例えば以下のように Topic 一覧が表示されます。
```bash
$ ros2 topic list

/cmd_vel
/odom
/scan
/parameter_events
/rosout
/clock
/robot1/image_raw
/robot1/camera_info
/robot2/joint_states
/robot2/tf
/tf_static
```
この時表示される Topic 名は1行 1 Topic で表示され、必ず `/` から始まります。そのため例えば `/robot2/joint_states` は `joint_states` という名前のトピックではなく `/robot2/joint_states` という名前であることに留意してください。

## Message（メッセージ）とは何か
　Message は Topic 内を通るデータを指します。世の中には PDF や写真、動画データなどさまざまな種類のデータがあります。ロボットからもさまざまなデータが出力されるわけで、Message を使うことで Topic を介して決まったデータを Publish、Subscriber することができます。
<br>
　Topic 内を流れる Message の種類を調べるには、以下のコマンドのように、Topic 一覧を表示するコマンド `ros2 topic list` にオプション `-t` をつけます。
```bash
ros2 topic list -t
```
　このコマンドを実行すると例えば以下のように Topic 一覧とその横に対応する Message が表示されます。
```bash
$ ros2 topic list -t

/cmd_vel               [geometry_msgs/msg/Twist]
/odom                  [nav_msgs/msg/Odometry]
/scan                  [sensor_msgs/msg/LaserScan]
/parameter_events      [rcl_interfaces/msg/ParameterEvent]
/rosout                [rcl_interfaces/msg/Log]
/clock                 [rosgraph_msgs/msg/Clock]
/robot1/image_raw      [sensor_msgs/msg/Image]
/robot1/camera_info    [sensor_msgs/msg/CameraInfo]
/robot2/joint_states   [sensor_msgs/msg/JointState]
/robot2/tf             [tf2_msgs/msg/TFMessage]
/tf_static             [tf2_msgs/msg/TFMessage]
```
　この結果からは、例えば Topic `/cmd_vel` は `geometry_msgs/msg/Twist` という種類の Message データが流れることを示しています。
 このように利用したい Topic の Message を事前に把握することでロボットを制御するプログラムを作るとき、どのようなデータが必要なのかを把握することができます。

## Publish/Subscribe
　なんらかのデータを送信することを **Publish（パブリッシュ）** といい、なんらかのデータを受け取ることを **Subscribe（サブスクライブ）** といいます。
<br>
　ROS2 では Topic を Publish するノードを **Publisher Node**、Subscribe するノードを **Subscriber Node** といいます。また、Topic を Subscriber して取得したデータを別のトピックに Publish する Pub & Sub Node という両方の機能を持つ Node も実装可能です。

## おさらい
　さて、ここでは改めて ROS2 の Node、Topic、Message、Publish、Subscribe について解説しました。改めてこれらの用語をまとめると以下のとおりです。

1. **Node（ノード）**  
   - 実行可能なプログラムの単位
   - 人間で言う「個人」に相当
   - 独立して動作し、他のノードと通信可能

2. **Topic（トピック）**  
   - ノード間の通信チャンネル
   - YouTubeのチャンネルのようにデータの伝達路として機能
   - スラッシュ(`/`)で始まる名前で識別（例: `/cmd_vel`）

3. **Message（メッセージ）**  
   - トピックを通じてやり取りされるデータ
   - 決められたフォーマット（型）がある
   - 例: `geometry_msgs/msg/Twist`（速度指令用メッセージ）

4. **Publish/Subscribeモデル**  
   - **Publish（出版）**: トピックにメッセージを送信する行為
   - **Subscribe（購読）**: トピックからメッセージを受信する行為
   - 1つのノードが複数のトピックをPublish/Subscribe可能

5. **コマンドラインツール**  
   - `ros2 topic list`: 利用可能なトピックを一覧表示
   - `ros2 topic list -t`: トピックと対応するメッセージ型を表示

　この仕組みにより、ROS2ではノード同士が疎結合になり、各コンポーネントを独立して開発・実行できるようになります。例えばロボットのセンサーデータをPublishするノードと、そのデータを使って制御指令を出すノードを別々に開発できるのです。

---

次のチュートリアル [7.1.Node を作ろう](tutorial7.1.md) では Node を プログラミング言語 Python で作成する方法を解説します。
