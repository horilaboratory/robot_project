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
　以下のコマンドを実行して依存関係パッケージをダウンロードします。
```bash
cd ~/colcon_ws/ && vcs import src < ~/colcon_ws/src/erasers_kachaka/setup.repos
```
## 実行ファイルのコピー
　以下のコマンドを実行して実行ファイルを所定のディレクトリにコピーします。以下のコマンドをすべコピーして実行してください。
```bash
cd ~/colcon_ws/src/erasers_kachaka &&\
cp docker/Dockerfile.erk ../kachaka-api/ &&\
cp customs/grpc_ros2_bridge.trcp.launch.xml ../kachaka-api/ros2/kachaka_grpc_ros2_bridge/launch/
```

## Docker イメージのビルド
　以下のコマンドを実行して Kachaka 用 Docker コンテナをビルドします。
```bash
cd ../kachaka-api && docker buildx build -t kachaka-api:erasers --target kachaka-grpc-ros2-bridge -f Dockerfile.erk . --build-arg BASE_ARCH=x86_64 --load
```

> [!NOTE]
> Docker イメージのビルドには **かなりの時間がかかります。** そのためビルド中にターミナルで新規タブから Ubuntu に入って次の作業を行ってください。

## kachaka-api のインストール
　以下のコマンドを実行して kachaka_api Python モジュールをインストールします。以下のコマンドをすべコピーして実行してください。
```bash
sudo apt install -y python3-pip && python3 -m pip install --upgrade pip &&\
pip install --break-system-packages --upgrade scipy && pip install --break-system-packages --extra-index-url https://pf-robotics.github.io/kachaka-python-packages/simple kachaka-api &&\
```
以下のコマンドを実行して何もメッセージが表示されなければ成功です。
```bash
python3 -c "import kachaka_api"
```

## パッケージをビルドする
　以下のコマンドを実行してダウンロードしたパッケージをビルドします。
```bash
cd ~/colcon_ws && colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
```
　以下のコマンドを実行してビルドファイルが常に読み込まれるようにします。
```bash
echo ". ~/colcon_ws/install/setup.bash" >> ~/.bashrc && . ~/.bashrc
```

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
export KACHAKA_NAME="er_kachaka"
export KACHAKA_IP=192.168.195.125
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

## Docker イメージのビルドが完了したら
　Docker イメージのビルドが完了したら、以下のコマンドを実行してイメージがあるか確認してください。
```bash
source ~/.bashrc && docker images
```
　以下のように `erasers` という名前のタグが降られているイメージが表示されていれば成功です。
```
REPOSITORY    TAG       IMAGE ID       CREATED             SIZE
kachaka-api   erasers   ea56c09c3e37   About an hour ago   1.37GB
```
　これで erasers_kachaka のインストールは成功です！

---

- [Windows でロボット開発をはじめるのセクション **erasers_kachaka をインストールする** に戻る](/windows/hostsetup.md/#erk)
