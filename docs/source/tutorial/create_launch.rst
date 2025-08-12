###############################
Launch ファイルを書いてみよう
###############################

　これまで Pub&Sub 通信とサービスを実装するノードの基礎的な書き方と，単体でのノードの起動方法を学びました．つぎに Launch ファイルの書き方を学びましょう．

**********************
Launch ファイルとは
**********************

 　Launch ファイル（ラウンチファイル，またはローンチファイル）は **複数のノードを一度に起動させる** 実行ファイルです．これを使うことでいちいち起動したいノードを一つづつコマンドラインで起動させる手間が省けます．

　また，従来の Launch ファイルもそうですが，複数のノードを起動する以外にもロボットのオペレーションに必要な強力な機能をたくさん有しています．

**************************
Launch ファイルの書き方
**************************

　ROS2 では，一般的に Launch ファイルを **Python で記述します．** ROS1 では XML 形式でしたから，若干の違和感があるかもしれません．しかしコツをつかめばとてもわかり易く，直感的に実装が可能になります．

　しかしながら Python というフォーマットの都合上，Launch ファイルの記述デザインは滝に及びます．ここでは **最も保守性に優れた記述方法** をもとに解説します．

Launch ディレクトリの作成
============================

　まずはじめに Launch ファイルを書く場所を用意しましょう．Launch ファイルは基本として **パッケージの launch ディレクトリ** 内部に作成します．

　現在の ros2_workshop パッケージには launch ディレクトリがありませんから，ターミナルのコマンドで新たにディレクトリを作成しましょう．

.. code:: bash

    mkdir launch

Launch ファイルの作成
========================

　ここでは前回作成したノード ``practice_publisher_node`` と ``practice_subscriber_node`` を起動する launch ファイル ``pubsub.launch.py`` を作成してみましょう．

.. important::

    Launch ファイルとして機能する Python ファイルには，**必ず launch という単語がファイル名に含まれている** 必要があります．

必要なライブラリのインポート
===============================

　Launch ファイルのスクリプトの書き方を解説します．まずはじめに以下の２つのライブラリをインポートしてください．

.. code:: python

    #! /usr/bin/env python3
    from launch import LaunchDescription
    from launch_ros.actions import Node

　``LaunchDescription`` は Launch ファイル内に記述されたノードや設定情報を内包するクラスです．ROS1 の XML Launch ファイルでいう ``<launch>`` タグのようなものだと思ってください．

　``Node`` は起動させたいノード情報を内包するクラスです．ROS1 の XML Launch ファイルでいう ``<node>`` タグのようなものだと思ってください．

Launch 関数 ``generate_launch_description``
==============================================

　``ros2 launch`` システムは launch ファイル内部の関数 ``generate_launch_description`` を読み込みます．そのため以下のように関数 ``generate_launch_description`` を作成してください．

.. code:: python

    #! /usr/bin/env python3
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():

　つぎに関数内で ``LaunchDescription`` クラスを初期化します．関数が ``LaunchDescription`` を返すようにします． この雛形が Launch ファイルの最小構成と考えてください．

.. code:: python

    #! /usr/bin/env python3
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        ld = LaunchDescription()

        return ld

起動したいノードを記述する
=============================

　``generate_launch_description`` 関数内に起動したいノードを記述しましょう．以下のように ``ld = LaunchDescription()`` 直下に ``Node`` をおいてください．``Node`` をインスタンスする変数名は起動したいノード名に合わせると良いです．

.. code:: python

    #! /usr/bin/env python3
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        ld = LaunchDescription()

        practice_publisher_node = Node()

        return ld

つぎに ``Node`` クラスの引数に必要な値をいれて起動したいノードを定義します．ここではまず ``practice_publisher_node`` を起動するための ``Node`` を定義します．

　``launch_ros.actions.Node`` は最低でも以下の引数を要求します．

.. code:: python

    Node(
        package,
        executable
    )

- ``package``

    起動させたいノードが内包されているパッケージ名を文字列で指定します．

- ``executable``

    起動させたいノード名を文字列で指定します．

　つまり，``practice_publisher_node`` を定義するにはこのように書きます．

.. code:: python

    #! /usr/bin/env python3
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        ld = LaunchDescription()

        practice_publisher_node = Node(
            package='ros2_workshop',
            executable='practice_publisher_node'
        )

        return ld

.. admonition:: やってみよう

    同じようにノード ``practice_subscriber_node`` を起動する ``Node`` も定義してみましょう．

起動したいノードを追加する
==============================

　Launch ファイルは今このようになっているかと思います．この状態はあくまで起動したい Node を定義しただけで，``LaunchDescription`` に起動させたいノード情報を渡していません．

.. code:: python

    #! /usr/bin/env python3
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        ld = LaunchDescription()

        practice_publisher_node = Node(
            ...
        )
        practice_subscriber_node = Node(
            ...
        )

        return ld

　LaunchDescription に情報を渡すにはこのように ``Node`` のインスタンス変数を ``ld.add_action()`` に代入します．

.. code:: python

    ...

     def generate_launch_description():
        ...

        ld.add_action(practice_publisher_node)

        return ld

.. caution::

    ``ld.add_action`` には **１つのオブジェクトしか代入できません．**

　これで launch ファイルの記述は完了です．

パッケージに launch ディレクトリを認識させる
=============================================

　Launch ファイルの作成は完了しましたが，このままではまだ ROS2 がパッケージに launch ファイルがあることを認識していません．そのためには **このパッケージには launch ディレクトリがある** ことを``setup.py`` に記述しなければなりません．

　そこで ``ament_python`` パッケージの ``setup.py`` に launch ファイルの存在を追記させるわけですが，今後他のファイル情報（params YAML など．．．）をかんたんに追記できるような改造を施します．

　現在の ``setup.py`` はこのようになっているかと思います．  

.. code:: python

    from setuptools import find_packages, setup

    package_name = 'ros2_workshop'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        ...
    )

　この ``setup`` 内部の引数 ``data_files`` にパッケージが内包するディレクトリ情報を追加するのですが，デフォルトのスタイルでの記述は手間がかかります．そこでまず，以下のように新たな２つのライブラリをインポートします．

.. code:: diff

    + import os
    + from glob import glob
    from setuptools import find_packages, setup

　つぎに ``package_name = 'ros2_workshop'`` 直下に以下のコードをコピペしてください．

.. code:: python

    data_files = []
    data_files.append(
        ("share/ament_index/resource_index/packages", ["resource/" + package_name])
    )
    data_files.append(("share/" + package_name, ["package.xml"]))

    def package_files(directory, data_files):
        for path, directories, filenames in os.walk(directory):
            for filename in filenames:
                data_files.append(
                    (
                        "share/" + package_name + "/" + path,
                        glob(path + "/**/*.*", recursive=True),
                    )
                )
        return data_files

そして ``setup`` の引数 ``data_files`` の値を削除して，以下のように変数 ``data_files`` に置換します．

.. code:: diff

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
    -    data_files=[
    -        ('share/ament_index/resource_index/packages',
    -            ['resource/' + package_name]),
    -        ('share/' + package_name, ['package.xml']),
    -    ],
    +    data_files=data_files,
        install_requires=['setuptools'],
        zip_safe=True,
        ...
    )

　こうすることで， ``setup()`` の↑にてパッケージ内部のリソースとして追加したいディレクトリをかんたんに設定することができるようになります．

　ここでは launch ディレクトリを追加したいので以下のように書きます．

.. code:: python

    data_files = package_files("launch", data_files)

.. note::

    ``data_files = package_files("追加したいディレクトリ名", data_files)`` と書くことで指定したデイレクトリをパッケージリソースとして追加できます．

***************************
パッケージをビルドする
***************************

　``setup.py`` を変更したので，再度パッケージをビルドする必要があります．ワークスペース直下（``/ws``）で ``colcon build`` を実行します．

.. code:: bash

    cd /ws
    colcon build --symlink-install --packages-select ros2_workshop

********************
Launch を実行する
********************

　ビルドが完了したら以下のコマンドを実行してワークスペースを読み込みましょう．するとビルドされたパッケージ情報が読み込まれます．

.. code:: bash

    source /ws/install/setup.bash

　つぎに以下のコマンドを書いてみましょう．

.. code:: bash

    ros2 launch ros2_workshop

この時点で **Tabキー** を何回か押すと作成した Launch ファイルが表示されます．

.. code::bash

    ros2 launch ros2_workshop
    .....                pub_sub.launch.py

Launch ファイルを起動してみましょう．すると２つのノードが立ち上がります．

.. code:: bash

    ros2 launch ros2_workshop pub_sub.launch.py

******************************
新しいノードを追加してみよう
*******************************
