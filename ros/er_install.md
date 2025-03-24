# erasers_kachaka をインストールする

> [!CAUTION]
> 　このドキュメントを読む前に、[**Windows でロボット開発をはじめる**](/windows/hostsetup.md) を読んで、**「Docker をインストールする」** までの作業を終えていることを確認してください。

## Ubuntu を開く
　ターミナルを開いて、Ubuntu を起動してください。

<img src="/imgs/ros2_install.png" width=50% />

## ワークスペースを作成する
　以下のコマンドを実行してワークスペースを作成し、`src` ディレクトリに移動します。
```bash
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
```

## erasers_kachaka リポジトリの取得
　以下のコマンドを実行し erasers_kachaka リポジトリをダウンロードします。
```bash
git clone https://github.com/trcp/erasers_kachaka.git
```

## 依存関係パッケージのダウンロード
　以下のコマンドを実行して erasers_kachaka ディレクトリに移動します。
```bash
cd ~/colcon_ws/src/erasers_kachaka
```
　以下のコマンドを実行して依存関係パッケージをダウンロードします。
```bash
vcs import .. < ./setup.repos
```
## 実行ファイルのコピー
　以下のコマンドを実行して実行ファイルを所定のディレクトリにコピーします。以下のコマンドをすべてコピーして実行してください。
```bash
cp docker/Dockerfile.erk ../kachaka-api/ &&\
cp customs/grpc_ros2_bridge.trcp.launch.xml ../kachaka-api/ros2/kachaka_grpc_ros2_bridge/launch/ &&\
cp customs/dynamic_tf_bridge.cpp ~/colcon_ws/src/kachaka-api/ros2/kachaka_grpc_ros2_bridge/src/dynamic_tf_bridge.cpp &&\
cp customs/static_tf_component.cpp ~/colcon_ws/src/kachaka-api/ros2/kachaka_grpc_ros2_bridge/src/component/static_tf_component.cpp
```

## Docker イメージのビルド
　以下のコマンドを実行して Kachaka 用 Docker コンテナをビルドします。
```bash
cd ../kachaka-api && docker buildx build -t kachaka-api:erasers --target kachaka-grpc-ros2-bridge -f Dockerfile.erk . --build-arg BASE_ARCH=x86_64 --load
```

> [!NOTE]
> Docker イメージのビルドには **かなりの時間がかかります。** そのためビルド中にターミナルで新規タブから Ubuntu に入って次の作業を行ってください。

## 環境設定を追記する
　以下のコマンドを実行してテキストエディタをインストールします。
```bash
sudo apt install -y gedit
```
　以下のコマンドを実行して設定ファイル `~/.bashrc` を開きます。
```bash
gedit ~/.bashrc
```
　`~/.bashrc` の一番下の行に以下のコードを追記してください。
```bash
# kachaka
export ROS_DOMAIN_ID=1
export KACHAKA_NAME="er_kachaka"
export KACHAKA_IP=192.168.8.11
export KACHAKA_ERK_PATH=~/colcon_ws/src/erasers_kachaka
export GRPC_PORT=26400
export API_GRPC_BRIDGE_SERVER_URI="${KACHAKA_IP}:${GRPC_PORT}"
```
　テキストエディタ右上の「save」をクリックして変更を保存します。保存が完了したら右上の×を押してテキストエディタを終了します。次に以下のコマンドを実行して変更を反映します。
```bash
source ~/.bashrc
```
　作業工程は以下の動画を参考にしてください。

<img src="/imgs/erk_install.gif" />

## Docker イメージのビルドを確認する その１
　Docker イメージのビルドが完了したら、以下のコマンドを実行してイメージがあるか確認してください。
```bash
source ~/.bashrc && docker images
```
　以下のように `erasers` という名前のタグが降られているイメージが表示されていれば成功です。
```
REPOSITORY    TAG       IMAGE ID       CREATED             SIZE
kachaka-api   erasers   ea56c09c3e37   About an hour ago   1.37GB
```

## Default kachaka API コンテナをダウンロードする
　以下のコマンドを実行して erasers_kachaka/docker ディレクトリに移動します。
```bash
cd ~/colcon=ws/src/erasers_kachaka/docker
```
以下のコマンドを実行して Default kachaka api をダウンロードします。
```bash
docker compose pull default_kachaka 
```

## パッケージをビルドする
　以下のコマンドを実行して `~/colcon_ws` に移動します。
```bash
cd ~/colcon_ws
```
以下の２つのコマンドを実行して必要なパッケージを自動インストールします。
```bash
rosdep install -i -y --from-path src/kachaka-api/ros2/
```
```
rosdep install -i -y --from-path src/cartographer
```
以下のコマンドを実行してダウンロードしたパッケージをビルドします。
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
```
以下のコマンドを実行してビルドファイルが常に読み込まれるようにします。
```bash
echo ". ~/colcon_ws/install/setup.bash" >> ~/.bashrc && . ~/.bashrc
```

## map ディレクトリの作成
　最後にホームディレクトリに map という名前のディレクトリを作成してください。このファイルにロボットが作成したマップが保存されます。
```bash
mkdir ~/map
```

---

- [Windows でロボット開発をはじめるのセクション **erasers_kachaka をインストールする** に戻る](/windows/hostsetup.md/#erk)
