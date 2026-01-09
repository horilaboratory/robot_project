##############################################################################
ローカル環境に ROS2 をインストールする方法
##############################################################################

　お使いの OS によっては ROS2 をローカルにインストールするのが推奨されていません．

.. tabs::

    .. tab:: Ubuntu (Linux)

        #. ターミナルを開き，以下のコマンドを実行して APT リポジトリを更新します．

           .. code-block:: bash

              sudo apt update

        #. 以下のコマンドを実行して必要なパッケージを自動インストールします．

            .. code-block:: bash

                sudo apt install -y software-properties-common
            
            .. code-block:: bash

                udo add-apt-repository universe -y
        
        #. 以下のコマンドを実行して ``curl`` をインストールします．
        
            .. code-block:: bash

                sudo apt install -y curl
        
        #. 以下のコマンドを実行して ROS2 をインストールするのに必要な情報を取得，登録します．
        
            .. code-block:: bash

                export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
            
            .. code-block:: bash

                curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

            .. code-block:: bash

                sudo dpkg -i /tmp/ros2-apt-source.deb
        
        #. 以下のコマンドを実行して APT リポジトリを更新し，ROS2 Humble をインストールします．
        
            .. code-block:: bash

                sudo apt update
            
            .. code-block:: bash

                sudo apt install -y ros-humble-desktop
        
        #. インストールが完了したら，以下のコマンドを実行して ROS2 が利用できるか確認します．
        
            .. code-block:: bash

                source /opt/ros/humble/setup.bash
            
            .. code-block:: bash

                ros2 topic list
            
            このとき ``ros2 topic list`` コマンドを実行してエラーが発生しなかったり，次のログが表示されれば成功です．

            .. code-block:: console

                /parameter_events
                /rosout

    .. tab:: Windows
        
        .. warning::
        
           Windows に ROS2 をローカルにインストールする方法は推奨されていません．Docker を使ったインストール方法をお試しください．

    .. tab:: macOS

        .. warning::
        
           macOS に ROS2 をローカルにインストールする方法は推奨されていません．Docker を使ったインストール方法をお試しください．