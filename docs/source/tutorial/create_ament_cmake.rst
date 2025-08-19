#####################################################
ament_cmake パッケージで Python ノードを作ってみよう
#####################################################

これまでのチュートリアルでは，Python 専用の ``ament_python`` パッケージを作成し，その中でノードを作成する方法を学びました．ここでは，C++ と Python のコードを共存させることができる ``ament_cmake`` パッケージを作成し，その中で Python ノードを動かす方法を解説します．

************************************************
なぜ ament_cmake で Python を動かすのか？
************************************************

``ament_python`` は手軽に Python ノードを作成できる便利なビルドタイプですが，以下のようなケースでは ``ament_cmake`` を使用する必要があります．

- **C++ と Python のノードを一つのパッケージにまとめたい場合**
    C++ で書かれた高速な処理を行うノードと，Python で書かれた柔軟な処理を行うノードを連携させたい場合に便利です．
- **独自のメッセージ (msg)・サービス (srv)・アクション (action) を定義したい場合**
    これらのカスタムインターフェースを定義するには，ビルドプロセスをより細かく制御できる ``ament_cmake`` が必要になります．

このチュートリアルでは，``ament_cmake`` パッケージの基本的な構成と，Python ノードを実行するための設定方法を学びます．

*******************************
ament_cmake パッケージを作ろう
*******************************

まずは ``ament_cmake`` タイプのパッケージを作成します．ワークスペースの ``src`` ディレクトリに移動してください．

.. code:: bash

    cd /ws/src

次に，以下のコマンドを実行して，``ros2_workshop_cmake`` という名前の ``ament_cmake`` パッケージを作成します．

.. code:: bash

    ros2 pkg create ros2_workshop_cmake --build-type ament_cmake

``--build-type`` を ``ament_cmake`` に指定する点が，これまでと異なります．

.. tip::

    ここでは解説のためパッケージを作成するコマンド ``ros2 pkg create ...`` にオプション ``--build-type ament_cmake`` を表記していますが，``ament_cmae`` パッケージを作成する場合は，以下のように ``--build-type`` オプションを省いて作成することも可能です．

    .. code:: bash

        ros2 pkg create ros2_workshop_cmake

パッケージの中身について
===========================

　``ament_cmake`` パッケージはこのように構成されています．``ament_python`` とは異なり，``setup.py`` の代わりに ``CMakeLists.txt`` があるのが特徴です．

.. code::

    ros2_workshop_cmake/
    ├── CMakeLists.txt
    ├── include
    │   └── ros2_workshop_cmake
    ├── package.xml
    └── src

- ``CMakeLists.txt``
    **C/C++ のビルドルールを記述するファイル** です．``ament_cmake`` パッケージでは，Python ノードを実行するための設定もこのファイルに記述します．
- ``package.xml``
    パッケージの概要や依存関係を記述するファイルです．これは ``ament_python`` と共通です．
- ``/include``
    主に C++ のヘッダーファイル (.hpp) を格納するディレクトリです．
- ``/src``
    主に C++ のソースコード (.cpp) を格納するディレクトリです．

*************************************
Python ノードのプログラムを配置する
*************************************

``ament_python`` の時と同様に，Python スクリプトを配置するディレクトリを作成します．パッケージのルートディレクトリ (``/ws/src/ros2_workshop_cmake``) に，パッケージ名と同じ名前のディレクトリを作成してください．

.. code:: bash

    cd /ws/src/ros2_workshop_cmake
    mkdir ros2_workshop_cmake

次に作成したディレクトリ ``ros2_workshop_cmake`` 内で以下のコマンドを実行し，``__init__.py`` というファイルを作成してください．

.. code:: bash

    touch __init__.py

このディレクトリ内に，これまでのチュートリアルで作成した ``practice_publisher.py`` と ``practice_subscriber.py`` を作成 (またはコピー) してください．Python ノードのコード自体は，``ament_python`` の場合と全く同じです．

.. caution::

    Python スクリプトをまとめているディレクトリ内に ``__init__.py`` を用意しないとパッケージが Python スクリプトで構成されたノードを認識しませんので忘れずに記述しましょう．また，ament_cmake パッケージではノードとして機能する Python スクリプトを内包するディレクトリ名は **必ずパッケージ名に合わせる** 必要があります．

************************************************
ビルド設定ファイルを編集する
************************************************

ここからが ``ament_cmake`` パッケージで Python を扱う上で最も重要な部分です．``setup.py`` の代わりに ``package.xml`` と ``CMakeLists.txt`` を編集して，Python ノードを ROS2 システムに認識させます．

package.xml の編集
======================

まず，パッケージが Python の ROS2 クライアントライブラリ (``rclpy``) などに依存していることを明記します．``/ws/src/ros2_workshop_cmake/package.xml`` を開き，以下の行を追記してください．

.. code:: diff

    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
      <name>ros2_workshop_cmake</name>
      <version>0.0.0</version>
      <description>TODO: Package description</description>
      <maintainer email="root@todo.todo">root</maintainer>
      <license>TODO: License declaration</license>
    
      <buildtool_depend>ament_cmake</buildtool_depend>
    
    + <build_depend>ament_cmake_python</build_depend>
    +
    + <exec_depend>rclpy</exec_depend>
    + <exec_depend>std_msgs</exec_depend>
    
      <test_depend>ament_lint_auto</test_depend>
      <test_depend>ament_lint_common</test_depend>
    
      <export>
        <build_type>ament_cmake</build_type>
      </export>
    </package>

- ``<build_depend>ament_cmake_python</build_depend>``: ビルド時に Python 関連の CMake マクロを利用するために必要です．
- ``<exec_depend>...</exec_depend>``: パッケージの実行時に必要なライブラリを指定します．これにより，``ros2 run`` を実行した際に Python 環境が正しく設定されます．

CMakeLists.txt の編集
=========================

次に，ビルドのルールを記述する ``CMakeLists.txt`` を編集します．このファイルに，作成した Python スクリプトを「実行可能なノード」としてインストールするよう指示します．

``/ws/src/ros2_workshop_cmake/CMakeLists.txt`` を開き，以下のように編集してください．

.. code:: cmake

    cmake_minimum_required(VERSION 3.8)
    project(ros2_workshop_cmake)
    
    # find dependencies
    find_package(ament_cmake REQUIRED)
    # Pythonサポート用のパッケージを見つける
    find_package(ament_cmake_python REQUIRED)
    
    # Pythonスクリプトをインストールする設定
    install(
      PROGRAMS
      # パッケージ名/スクリプト名 の形式で指定
      ros2_workshop_cmake/practice_publisher.py
      ros2_workshop_cmake/practice_subscriber.py
      # インストール先を指定
      DESTINATION lib/${PROJECT_NAME}
    )
    
    ament_package()

- ``find_package(ament_cmake_python REQUIRED)``
    Python を扱うための CMake の機能 (マクロ) を読み込みます．
- ``install(PROGRAMS ... DESTINATION ...)``
    これが Python スクリプトをノードとして登録する核心部分です．
    
    - ``PROGRAMS``: 実行可能プログラムとしてインストールするファイルを指定します．
    
    - ``DESTINATION lib/${PROJECT_NAME}``: ファイルをどこにインストールするかを指定します．`${PROJECT_NAME}` は ``project()`` で指定したパッケージ名 (ここでは ``ros2_workshop_cmake``) に置き換えられます．``ros2 run`` コマンドは，この ``install/パッケージ名/lib/パッケージ名/`` ディレクトリにインストールされた実行ファイルを探しに行きます．

これで，ビルドの準備が整いました．

***************************
パッケージをビルドする
***************************

設定ファイルを変更したので，パッケージをビルドします．ワークスペースのルート (``/ws``) に戻り，``colcon build`` を実行してください．

.. code:: bash

    cd /ws
    colcon build --symlink-install --packages-select ros2_workshop_cmake

**************************
ノードを実行する
**************************

ビルドが完了したら，まずワークスペースの環境を読み込みます．

.. code:: bash

    source /ws/install/setup.bash

次に，``ros2 run`` コマンドでノードが実行できるか確認しましょう．
``ament_python`` パッケージの時と全く同じコマンドで実行できます．

**1つ目のターミナル** で Publisher を起動します．

.. code:: bash

    ros2 run ros2_workshop_cmake practice_publisher.py

.. hint::
    ``ament_cmake`` パッケージでは，``setup.py`` の ``entry_points`` で指定したような別名 (例: ``practice_publisher_node``) は設定していません．そのため，``ros2 run`` では Python のスクリプトファイル名を直接指定します．

**2つ目のターミナル** で Subscriber を起動します．

.. code:: bash

    ros2 run ros2_workshop_cmake practice_subscriber.py

Subscriber 側のターミナルに，Publisher からのメッセージが表示されれば成功です！

.. code::

    [INFO] [practice_subscriber]: I heard: "Hello! ROS2 count: 0"
    [INFO] [practice_subscriber]: I heard: "Hello! ROS2 count: 1"
    ...

このように，``ament_cmake`` パッケージでも ``CMakeLists.txt`` と ``package.xml`` を正しく設定することで，Python ノードを開発・実行することができます．
