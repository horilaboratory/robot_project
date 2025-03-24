# ROS2 Humble をインストールする

> [!CAUTION]
> 　このドキュメントを読む前に、[**Windows でロボット開発をはじめる**](/windows/hostsetup.md) を読んで、**「Ubuntu 起動時のデフォルトユーザーを変更する」** までの作業を終えていることを確認してください。

## Ubuntu を開く
　ターミナルを開いて、Ubuntu を起動してください。

<img src="/imgs/ros2_install.png" width=50% />

## ROS2 Humble をインストールする
　以下のコマンドを実行してください。
```bash
sudo apt update && sudo apt upgrade -y &&\
sudo apt install -y software-properties-common curl &&\
sudo add-apt-repository universe -y &&\
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg &&\
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null &&\
sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools &&\
echo ". /opt/ros/humble/setup.bash" >> ~/.bashrc
```

> [!TIP]
> 上記コマンドはこの動画のようにコマンドブロック右上にあるコピーボタンを押してコピーしましょう。そしてターミナル内で Control + V をすることですべてのコマンドをペーストして一括実行できます。
> <img src="/imgs/ros2_install2.gif" width=50% />

## rosdep を利用可能にする
　rosdep という ROS のパッケージを管理、インストールを支援してくれるパッケージを有効化する方法を解説します。以下のコマンドを実行してください。
```bash
sudo rosdep init
```
```
Recommended: please run

        rosdep update
```
上記のようなメッセージが表示されたら以下のコマンドを実行してください。
```bash
rosdep update
```

---

- [Windows でロボット開発をはじめるのセクション **ROS2 Humble をインストールする** に戻る](/windows/hostsetup.md/#ros2)
