# Linux に ROS2 をインストールする方法

　このセクションでは Linux に ROS2 Humble をインストールする方法を解説します．

```{note}
　Windows + WSL2 を利用されている方は本セクションを読む前に [WSL2 セットアップ方法](./wsl2) を参照してください．
```

```{warning}
　ここでは Ubuntu 22.04 の利用を前提に解説します．別のディストリビューションを利用されている場合や Docker から ROS2 を利用したい場合は [Docker をつかい ROS2 をインストールする方法](./docker.md) を参照してください．
```

1. **OS ディストリビューションの確認**<br>
    ```bash
    cat /etc/lsb-release
    ```
    上記のコマンドの実行結果に `DISTRIB_DESCRIPTION="Ubuntu 22.04.X LTS"` が表示されていることを確認してください．

1. **リポジトリのインストール**<br>
    ```bash
    sudo apt install software-properties-common -y
    ```
    ```bash
    sudo add-apt-repository universe -y
    ```

1. **インストールに必要なパッケージ `curl` をインストールする**<br>
    ```bash
    sudo apt update
    ```
    ```bash
    sudo apt install curl -y
    ```

1. **ROS2 リポジトリの追加とインストール**<br>
    ```bash
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    ```
    ```bash
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    ```
    ```bash
    sudo dpkg -i /tmp/ros2-apt-source.deb
    ```

1. **ROS2 humble と ROS2 開発パッケージ郡をインストールする**<br>
    ```bash
    sudo apt update && sudo apt upgrade -y
    ```
    　以下のコマンドを実行すると ROS2 humble と ROS2 開発用ツール郡がインストールされます．インストールには時間がかかります．
    ```bash
    sudo apt install ros-humble-desktop ros-dev-tools -y
    ``` 

1. **ROS2 の環境設定**<br>
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```
    ```bash
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
    ```
    ```bash
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
    ```
    ```bash
    source ~/.bashrc
    ```

1. **動作確認**<br>
    以下のコマンドを実行してエラーが表示されないことを確認してください．
    ```bash
    ros2 topic list
    ```