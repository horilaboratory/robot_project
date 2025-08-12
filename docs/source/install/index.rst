########
環境構築
########

　ここではお使いのコンピュータに **ROS2 Humble (ロス２・ハンブル)** をインストールし，本ドキュメントをもとに作業を始めるために必要な環境を構築する方法を解説します．

　お使いの OS によって環境の構築方法が異なります．以下の手順を参考に環境を構築してください．

*********************************
Docker を使い環境を構築する
*********************************

Docker をインストールする
============================

　本ドキュメントでは利用しているローカル環境に影響を与えないよう **Docker** による仮装環境上での利用を推奨しています．しかしながら，本ドキュメントでは Docker の知識を問われることはほぼありません．このように環境を構築すれば良いのだなという感覚で作業を進めてください．

　また，お使いの OS が macOS の場合は **必然的に Docker をインストールする必要があります．**

.. tabs::
    .. tab:: Linux (Ubuntu)

        　以下のコマンドを **すべて** コピーし，ターミナル上にペーストして実行してください．するとお使いのコンピュータに Docker がインストールされます．

        .. hint::

            コードブロックにカーソルを当てるとコピーボタンが表示されます．これを押すとコードがすべてコピーされます．

        .. code:: bash

            sudo apt update && sudo apt install -y ca-certificates curl gnupg lsb-release &&\
            sudo mkdir -p /etc/apt/keyrings &&\
            curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg &&\
            echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
            $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null &&\
            sudo apt update && sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin &&\
            sudo usermod -aG docker $USER
        
        インストールが完了したら，以下のコマンドを実行するかコンピュータを再起動してください．もし以下のコマンドを実行して，次の手順が失敗した場合はコンピュータを再起動してください．

        .. code:: bash

            newgrp docker
        
        次に以下のコマンドを実行して Docker がインストールされているか確認します．以下のコマンドを実行してエラーが発生しなければ成功です．

        .. code:: bash

            docker version
        
        .. hint::

            もし上のコマンドを実行した時以下のエラーが発生したら，コンピュータを再起動して再度上のコマンドを試してください．

            .. code::
            
                /var/run/docker.sock: connect: permission denied


    .. tab:: Windows

        作成中

    .. tab:: macOS

        .. caution::

            本稿で記述されている手順では Rviz2 を利用することができません．どうしても macOS 上で Rviz2 を利用したい場合は VNC サーバーが利用できる Docker コンテナか VirtualBox などの仮想環境の利用を検討してください．

        以下のリンクにアクセスし，Docker Desktop をインストールしてください．

        https://www.docker.com/products/docker-desktop/


Docker イメージをダウンロードする
======================================

　Docker のインストールが完了したら，以下のコマンドを実行して ROS2 の環境を持つ Docker イメージをダウンロードします．

.. code:: bash

    docker pull gai313/ros2:humble

.. _setup-workspace-section:

ROS2 運用用ワークスペースを用意する
======================================

　Docker イメージをダウンロード中に別のターミナルを開き，**ホームディレクトリ上で** 以下のコマンドを実行して ``ros2_guidance_ws`` リポジトリをクローンしてください．

　このリポジトリは Docker 上で ROS2 ワークスペースを提供するためのコンテナを構築し，コンテナ上で作成した ROS2 ワークスペースを保存するために利用します．

.. code:: bash

    git clone https://github.com/GAI-313/ros2_guidance_ws.git

　これで環境構築は完了しました．

.. button-link:: ../boot/index.html
   :color: primary
   :shadow:

   次のステップへ進む

*******************************
ローカルに直接環境を構築する
*******************************

.. attention::

    こちらの手順は **Ubuntu 22.04 向けです．** 他のディストリビューションおよび OS では利用できません．

.. attention::

    このドキュメントでは上記の **Docker を利用した環境下** での利用を前提に書かれています．ローカル環境で作業を進める場合，手順に若干相違があることに注意してください．

ROS2 Humble をインストールする
=================================

　ローカルに直接環境を構築したい場合は，以下のコマンドをターミナル上で実行してください．

.. code:: bash

    sudo apt install software-properties-common &&\
    sudo add-apt-repository universe -y &&\
    sudo apt update && sudo apt install curl -y &&\
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') &&\
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" &&\
    sudo dpkg -i /tmp/ros2-apt-source.deb &&\
    sudo apt update && sudo apt install ros-humble-desktop -y

　インストールが完了したら以下のコマンドを実行して ROS2 を自動的に読み込むように ``~/.bashrc`` に書き込んでください．

.. code:: bash

    echo ". /opt/ros/humble/setup.bash" >> ~/.bashrc

ワークスペースを作成する
===========================

　任意のディレクトリ上で以下のコマンドを実行してワークスペース用ディレクトリを作成します．

.. code:: bash

    mkdir -p colcon_ws/src
