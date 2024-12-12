# カチャカを Docker コンテナ内で動かす方法
　WSL 内にログインし、以下のパスに移動してください。

> 以下のパスは事前に環境構築を終えていなければ到達できません。事前に環境構築を済ませましょう。

```bash
~/colcon_ws/src/erasers_kachaka/kachaka_api/tools/ros2_bridge
```
上記パスに移動したら、`ls` コマンドを実行して `docker-compose.yaml` というファイルがあるかどうか確認し、このファイルを開いてください。VScode をインストールしているならば、以下のコマンドでファイルを開くことができます。
```
code docker-compose.yaml
```
以下のコードを `docker-compose.yaml` ないのコードに置き換えてください。
```yaml
version: "2.4"

services:
  ros2_bridge:
    image: "asia-northeast1-docker.pkg.dev/kachaka-api/docker/kachaka-grpc-ros2-bridge:${TAG}"
    network_mode: "host"
    ipc: "host"
    pid: "host"
    environment:
      - API_GRPC_BRIDGE_SERVER_URI
      - ROS_DOMAIN_ID
      - ROS_LOCALHOST_ONLY
      - ROS_LOG_DIR=/tmp
      - RMW_IMPLEMENTATION
      - USER_ID
      - GROUP_ID
    volumes:
      - ./code:/code
    user: "${USER_ID}:${GROUP_ID}"
    command: >
      ros2 launch kachaka_grpc_ros2_bridge grpc_ros2_bridge.launch.xml server_uri:=${API_GRPC_BRIDGE_SERVER_URI}
  kachaka_follow:
    image: "asia-northeast1-docker.pkg.dev/kachaka-api/docker/kachaka-grpc-ros2-bridge:${TAG}"
    network_mode: "host"
    ipc: "host"
    pid: "host"
    environment:
      - ROS_DOMAIN_ID
      - ROS_LOCALHOST_ONLY
      - ROS_LOG_DIR=/tmp
      - RMW_IMPLEMENTATION
      - USER_ID
      - GROUP_ID
    user: "${USER_ID}:${GROUP_ID}"
    command: >
      ros2 run kachaka_follow follow
  kachaka_speak:
    image: "asia-northeast1-docker.pkg.dev/kachaka-api/docker/kachaka-grpc-ros2-bridge:${TAG}"
    network_mode: "host"
    ipc: "host"
    pid: "host"
    environment:
      - ROS_DOMAIN_ID
      - ROS_LOCALHOST_ONLY
      - ROS_LOG_DIR=/tmp
      - RMW_IMPLEMENTATION
      - USER_ID
      - GROUP_ID
    user: "${USER_ID}:${GROUP_ID}"
    command: >
      ros2 run kachaka_speak speak
  kachaka_smart_speaker:
    image: "asia-northeast1-docker.pkg.dev/kachaka-api/docker/kachaka-grpc-ros2-bridge:${TAG}"
    network_mode: "host"
    ipc: "host"
    pid: "host"
    environment:
      - ROS_DOMAIN_ID
      - ROS_LOCALHOST_ONLY
      - ROS_LOG_DIR=/tmp
      - RMW_IMPLEMENTATION
      - USER_ID
      - GROUP_ID
    user: "${USER_ID}:${GROUP_ID}"
    command: >
      ros2 run kachaka_smart_speaker smart_speaker
  kachaka_vision:
    image: "asia-northeast1-docker.pkg.dev/kachaka-api/docker/kachaka-grpc-ros2-bridge:${TAG}"
    network_mode: "host"
    ipc: "host"
    pid: "host"
    environment:
      - ROS_DOMAIN_ID
      - ROS_LOCALHOST_ONLY
      - ROS_LOG_DIR=/tmp
      - RMW_IMPLEMENTATION
      - USER_ID
      - GROUP_ID
    user: "${USER_ID}:${GROUP_ID}"
    command: >
      ros2 launch kachaka_vision hand_recognition_launch.py
```
このコードに置き換えたら、カレントディレクトリに `code` というディレクトリを作成してください。
```
mkdir code
```
この code デイレクトリ内にプログラムを書いてください！

## コンテナを起動する方法
　コンテナを起動するには、カレントディレクトリ内にある `start_bride.sh` を実行します。以下のようにカチャカの IP アドレスを引数に起動してください。
```bash
./start_bridge.sh $KACHAKA_IP
```
上記コマンドを実行したら、ログが大量に表示されます。ログ内に `[ERROR]` というテキストが表示されていなければ成功です。かつ、カチャカの LED ライトが一時的に点灯したら接続は成功しています。

## コンテナ内でプログラムを実行する方法
　コンテナ内にエントリーするには以下のコマンドを実行します。以下のコマンドは `docker-compose.yaml` が存在するディレクトリでしかできません。
```bash
docker compose exec ros2_bridge bash
```
するとシェルの表示が白黒になります。白黒になったら以下のコマンドを実行してください。このコマンドを実行しないとコンテナ内で ROS2 を使うことができません。
```bash
. /opt/ros/humble/setup.bash
```
次に以下のコマンドを実行して `code` ディレクトリに移動してください。すると作成したプログラムがありますので、コンテナ内でプログラムを実行すればカチャカを動かすことができます。

## プログラムを動かしているのにロボットが動かない場合
　プログラムを動かしているのにロボットが動かない場合、トピック名が異なる可能性があります。あなたの記述したコード内にある `create_publisher` 関数内に記述したトピックが、コンテナ内で発行されているトピック名と異なるとロボットは動きません。
以下のコマンドを実行し、コンテナ内で発行されているトピック名を特定しましょう。
```
ros2 topic list
```
