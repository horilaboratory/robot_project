# ROS2 jazzy をインストールする 方法
## ROS2 のインストール方法
　ターミナルを開き、以下のコマンドを実行して apt リポジトリを更新します。
```bash
sudo apt update
```
　このとき、パスワードの入力が求められるので、
**ユーザー作成時に設定したパスワード**
を入力してください。

---

　次に、以下のコマンドを実行して、ROS2 をインストールするために必要なパッケージをインストールします。
```bash
sudo apt install software-properties-common curl
```
　このとき、
`この操作後に追加で 597 kB のディスク容量が消費されます。続行しますか? [Y/n]`
とプロンプト上で尋ねられることがあるので、
**y**
を入力してエンターキを押してインストールを続行してください。

---

　次に以下のコマンドを実行して必要なリポジトリを追加します。
```bash
sudo add-apt-repository universe
```
　このとき、
`続けるには「Enter」キーを、中止するにはCtrl-cを押してください`
とプロンプト上で尋ねられるので、エンターキーを押して作業を続行してください。

---

　次に以下のコマンドを実行して ROS2 をインストールするために必要な GPG キーをダウンロードします。
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

---

　次に以下のコマンドを実行して ROS2 のリポジトリを追加します。
このコマンドを実行することで ROS2 のインストール準備が整います。
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

---

　次に、以下のコマンドを実行して先ほど追加されたリポジトリを読み込みます。
```bash
sudo apt update
```

---

　次に以下のコマンドを実行して ROS2 Jazzy と ROS2 開発用ツール一式をインストールします。
```bash
sudo apt install ros-jazzy-desktop ros-dev-tools
```
このとき、
`この操作後に追加で 2,676 MB のディスク容量が消費されます。続行しますか? [Y/n]`
とプロンプト上で尋ねられることがあるので、
**y**
を入力してエンターキを押してインストールを続行してください。

> ROS2 Jazzy のインストールには **かなりの時間がかかります。**

---

　次に以下のコマンドを実行して ROS2 が利用できるように `~/.bashrc` に読み込ませます。
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```
　次に以下のコマンドを実行して `~/.bashrc` を再読み込みしてください。
```bash
source ~/.bashrc
```

---

　以下のコマンドを実行して、`/opt/ros/jazzy/bin/ros2` と応答があれば ROS2 Jazzy のインストールは成功です！
```bash
which ros2
```

---

　これで ROS2 のインストールは完了です！
