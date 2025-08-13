###################################
Publisher ノードを作ってみよう
###################################

　前回作成したパッケージにトピックをパブリッシュするノードを作成してみましょう．

*****************
Publisher とは
*****************

　Publisher（パブリッシャー）とは ROS2 トピックを発行する者を指し，そのノードを **Publisher Node（パブリッシャー・ノード）** と呼びます．

******************************
ノードプログラムの書き方
******************************

　ament_python パッケージではパッケージ内部にある **パッケージ名と同じディレクトリ** 内にノードとなる Python スクリプトを作成します．ここでは ``practice_publisher.py`` というノードを作成してみましょう．

　ノードとなるプログラムの書き方には色々ありますが，ここでは **最もベーシックかつシンプルな書き方であるクラスを用いたノードの作成を解説します．**

必要なモジュールのインポート
==============================

　ROS2 では ``rclpy`` というモライブラリを利用します．ROS1 でいう ``rospy`` の代替となるライブラリです．まずは以下のように ``rclpy`` とその中にあるモジュール ``Node`` をインポートします．

.. code:: python

    #!/usr/bin/env python3
    from rclpy.node import Node
    import rclpy

.. important::

    　サンプルコードに書かれている ``#!/usr/bin/env python3`` は **シバン** といい，「このファイルは Python でしか実行できませんよ」と明示するおまじないです．

    　ROS1 ではノードとして利用するためにこれを書くことが必須ですが，ROS2 では必須ではありません．しかしここであえて書いているのはシバンはクロスプラットフォームな環境で最低限の保険として機能し，予期せぬインシデントを避けるために働くからです．

クラスを作成する
====================

　スクリプト内に任意のクラスを１つ作成しましょう．ここではスクリプトファイル名にちなんだ ``PracticePublisher`` というクラスを作成します．

　また，以下のコードのように作成するクラスには ``Node`` モジュール基クラスを継承させてください．

.. code:: python

    class PracticePublisher(Node):

.. hint::

    このように，``class XX(YY):`` のような書き方を，**クラス ``XX`` にクラス ``YY`` を継承させる** といいます．
    かんたんに説明するとクラス ``YY`` の資産をクラス ``XX`` でも利用できるようにします．

イニシャライザの作成
======================

　クラスではクラス内部で動作する機能を関数とおなじ ``def`` をつかい定義します．このクラス内部で機能する関数のことを **メソッド（Method）** といいます．

　一方，メソッドの中で **クラスが呼び出されたとき自動的に実行されるメソッド** を **イニシャライザ** といい，``__init__`` という固有名を使い定義します．

　以下のサンプルコードのように作成したクラス内部にイニシャライザを作成しましょう．メソッドを書く習慣として **必ず第１引数に self を書くことを意識してください．**

.. code:: python

    class PracticePublisher(Node):
        def __init__(self):

ノードの宣言を書く
===================

　今書いている Python スクリプトを ROS2 ノードとして機能させるためには，ROS2 ノードであることをコード内で宣言する必要があります．``rospy`` でいう ``rospy.init_node('...')`` のようなものですね．

　まずイニシャライザ内で以下のようなコードを書いてください．このコードでは「ここで ``practice_publisher`` という名前のノードを宣言する」という意味をもちます．

.. code:: python

    class PracticePublisher(Node):
        def __init__(self):
            super().__init__('practice_publisher')

Publisher を作成する
=======================
　次に Publisher を作りましょう．要はトピックをパブリッシュするためのインターフェースを用意します．ここでは ``String`` 型のメッセージを使い，``/chatter`` という名前のトピックをパブリッシュするパブリッシュあーを作成してみましょう．

　そのためには ``String`` メッセージをこの Python スクリプトにインポートしなければなりません．以下のように ``String`` メッセージモジュールをインポートしましょう．

.. code:: python

    #!/usr/bin/env python3
    from rclpy.node import Node
    import rclpy

    from std_msgs.msg import String

つぎに，イニシャライザ内で以下のようにパブリッシャーを作成します．

.. code:: python

    class PracticePublisher(Node):
        def __init__(self):
            super().__init__('practice_publisher')

            self.publisher = self.create_publisher(
                String,
                '/chatter',
                10
            )

　``self.create_puublisher`` 関数がパブリッシャーで，これを変数 ``self.publisher`` として定義しています．

この関数の引数はこのようになっています．プレフィックスのオブジェクトが ``rclpy.node.Node`` となっているのはこの関数が該当のライブラリに内包されていることを示しています．

.. code:: python

    rclpy.node.Node.create_publisher(
        msg_type,
        topic,
        qos_profile
    )

- ``msg_type``
    パブリッシュしたいメッセージ型のモジュールを代入します．ここでは ``String`` 型のメッセージを使うため ``std_msgs.msg.String`` モジュールを代入しています．

- ``topic``
    パブリッシュしたいトピック名を文字列で指定します．「？，！」などの特殊文字はトピック名に使用できません．ここでは **/chatter** という名前のトピック名を定義しています．

- ``qos_profile``
    QoS とは ``Quality of Service`` の略で，ROS1 のときよりも高度になった ROS2 の通信における信頼性や効率性を設定するパラメータです．この引数は QoS 以外にもトピック深度を変わりに定義することも可能です．

    ここでは ROS1 のときにもあった，**トピック深度（Queue Size）** を指定しています．トピック深度とはパブリッシュされたメッセージを一時的に保存しておくバッファのサイズのことです．

    **ネットワークの遅延などによりメッセージの処理が追いつかない場合，ここに指定した 10 という数の分だけメッセージをキューに溜めておくことができます．**

タイマーを作成する
======================

　前述の通り，``self.create_publisher`` は **あくまでパブリッシャーのインターフェース** を作っただけです．要はこの状態では大砲を作っただけで砲弾と砲手がいない状態なのです．

つぎに，**Timer（タイマー）** という機能を使い，一定周期で関数またはメソッドを実行し，パブリッシャーを動作させるためのインターフェースを作成します．

イニシャライザに以下のコードを追記してください．

.. code:: python

    class PracticePublisher(Node):
        def __init__(self):
            super().__init__('practice_publisher')

            self.publisher = self.create_publisher(
                ...
            )

            self.timer = self.create_timer(
                0.5,
                self.publish_callback
            )

タイマー関数 self.create_timer はこのようになっています．

.. code:: python

    rclpy.node.Node.create_timer(
        timer_period_sec,
        callback
    )

- ``timer_period_sec``

    コールバック関数が呼び出される周期を秒 [sec] で指定します．ここでは毎秒 0.5 秒でコールバック関数を呼び出し，実行するようにしています．

- ``callback``

    コールバック関数を代入します．**コールバック関数** とは呼び出されたら実行される関数を言います．ここでは ``publish_callback`` というメソッドを指定しています．
    このメソッドはこれから作成します．

コールバック関数を作成する
============================

　``PracticePublisher`` クラスに新たなメソッド ``publish_callback`` を作成してください．

.. code:: python

    class PracticePublisher(Node):
        def __init__(self):
            ...
        
        def publish_callback(self):

このメソッドは先ほど作成したタイマー関数によって 0.5 秒間隔で呼び出され，実行されます．つまりこのメソッド内にトピックをパブリッシュする機構を作ることで 0.5 秒間隔でトピックをパブリッシュする機能を実装できます．

パブリッシュするメッセージにデータを入れる
==============================================

　コールバック関数たる ``publish_callback`` メソッド内で以下のように ``String`` メッセージモジュールを任意の変数に定義してください．

.. code:: python
        
        def publish_callback(self):

            message = String()

　ROS2 メッセージにはそれぞれデータをいれるための **フィールド** があります．特定のメッセージのフィールド構造を知りたいならばターミナル上で ``ros2 interface show <メッセージ名>`` を実施します．

.. hint::
    
    ターミナルで以下のコマンドを実行して String メッセージのフィールド構造を見てみましょう．

    .. code:: bash

        ros2 interface show std_msgs/msg/String
    
    するとこのようなテキストが返ってきます．

    .. code::

        # This was originally provided as an example message.
        # It is deprecated as of Foxy
        # It is recommended to create your own semantically meaningful message.
        # However if you would like to continue using this please use the equivalent in example_msgs.

        string data
    
    Python と同じく ``#`` はコメントなので無視して構いません．注目すべきは ``string data`` です．これは「**data というフィールドに文字列（string）型のデータをいれてね**」
    という意味を持ちます．

これを Python で解釈すると ``std_msgs.msg.String.data`` という変数に文字列の値を代入すればよいのです．これを先程のコードに当てはめると，このように書きます．
    
ここではフィールドに ``'Hello! ROS2'``  という文字列を代入しています．

.. code:: python
        
        def publish_callback(self):

            message = String()
            message.data = 'Hello! ROS2'

メッセージをパブリッシュする
==============================

　コールバック関数末尾に以下のようにデータが入ったメッセージデータをパブリッシュするコードを実装しましょう．``self.publish.publish()`` を使用します．
``self.publish`` はイニシャライザで作成した ``self,create_publisher`` のインスタンス変数です．

.. code:: python
        
        def publish_callback(self):

            message = String()
            message.data = 'Hello! ROS2'

            self.publisher.publish(message)

　これでノードとして機能するクラスは完成です．つぎは今作成したノードクラス ``PracticePublisher`` を実行する部分を作成します．

実行関数 ``main`` を作成する
=================================

　クラス ``PracticePublisher`` の下に ``main`` 関数を作成してください．

.. code:: python

    class PracticePublisher(Node):
        ...
    

    def main():

つぎに main 関数内にコード ``rclpy.init()`` をはじめに書いてください．後述しますが ROS2 ではノードの実行方法が ROS1 とは少し異なります．

.. code:: python

    class PracticePublisher(Node):
        ...
    

    def main():
        rclpy.init()

.. important:: Python における ROS2 ノードの動作方法

    そもそも ``rclpy`` は **Ros Client Library for Pythpn** の略で，ROS2 を操作するクライアントとして機能します．

    ``rclpy.init()`` は実行されるスクリプト上ではじめに行わなければならない処理で，ここで ROS2 に対して「これからここでノードが動くからヨロシク」と連絡を取ります．

    そのあとに後ほど解説するノードを呼び出し，実行する処理を書くことでノードが ROS2 側に認識され，通信することができるようになります．とにかく，``rclpy.init()`` はROS2 ノードを書く上で最初に書かなければならないものだと思ってください．

    ROS1 の ``rospy`` ではこの ROS との通信の確率とノードの構築が一体化されているため，``rospy`` ライブラリでほぼすべての操作を賄うことができます．

    一方，ROS2 ではノードを動かすための ``rclpy``，ノードを構築する ``rclpy.node.Node`` と役割が明確に分けられています．ややこしいですが ROS2 のコードを書き続けるとこの習慣が自然にわかってきますので，この解説を十分に理解する必要はありません．

　``rclpy`` の初期化を書いたら次にノードのインスタンス化及びノードの実行処理を書いていきます．以下のように任意の変数を使いクラス ``PracticePublisher`` を初期化してください．

.. code:: python

    class PracticePublisher(Node):
        ...
    

    def main():
        rclpy.init()
        node = PracticePublisher()

　初期化したノードを次に ``rclpy.spin()`` に代入します．この関数が実行されると **代入されたノードを永久に実行し続けてくれます．** 要は ``while True:`` のようなものです．

.. code:: python

    class PracticePublisher(Node):
        ...
    

    def main():
        rclpy.init()
        node = PracticePublisher()
        rclpy.spin(node)

これで main 関数は完成です．

実行処理を書く
================

　最後に定義した main 関数を実行する部分を書きましょう．Python でよくみるおまじないを書いて，main 関数を実行します．

.. code:: python

    def main():
        rclpy.init()
        node = PracticePublisher()
        rclpy.spin(node)
    
    if __name__ == '__main__':
        main()

これでトピックをパブリッシュするノードスクリプトは完成です！

****************************
ノードを登録する
****************************

　先程作成したスクリプトを ROS2 ノードとして実行できるようにしましょう．一応 ``python3 practice_publisher.py`` で作成したプログラムは実行できますが，
ここでは ``ros2 run ...`` コマンドで呼び出しできるように正式なノードとしてプログラムを認識させましょう．

　ament_python パッケージにある setup.py を開いてみましょう．

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
        maintainer='root',
        maintainer_email='root@todo.todo',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
            ],
        },
    )

　このファイルを編集することでパッケージに要素を追加したり，ノードを追加したりすることができます．ノードの追加方法は ``setup`` 関数下部にある ``entry_points`` のフィールドを編集します．

　ここでは ``practice_publisher_node`` という名前で先ほど作成したプログラムをノードとして登録しましょう．以下のように ``entry_points`` を編集します．

.. code:: python

    entry_points={
        'console_scripts': [
            'practice_publisher_node = ros2_workshop.practice_publisher:main'
        ],
    },

登録したいノードはこのように ``'console_scripts': []`` 内に記述します．

この記述は，「**``practice_publisher_node`` という名前で，``ros2_workshop`` パッケージ内の ``practice_publisher.py`` スクリプトにある ``main`` 関数を実行する**」という意味になります．

- ``practice_publisher_node``
    ``ros2 run`` コマンドで呼び出す際の，このノードの実行名です．

- ``ros2_workshop.practice_publisher``
    ``パッケージ名.スクリプトファイル名`` の形式で，実行したいスクリプトを指定します．

- ``:main``
    スクリプト内で実行する関数を指定します．

　これでノードの登録が完了しました．


***************************
パッケージをビルドする
***************************

　作成したパッケージをビルドするために，コンテナ内のターミナルでワークスペース直下である ``/ws`` に移動してください．

.. code:: bash

    cd /ws

　ROS1 では ``catkin build`` や ``catkin_make`` といった ``catkin`` キャトキンというビルドツールを使ったと思いますが，ROS2 では ``colcon`` （コルコン）というビルドツールを使います．

とりあえず以下のコマンドを実行して作成したパッケージ ``ros2_workshop`` をビルドしてください．

.. code:: bash

    colcon build --symlink-install --packages-select ros2_workshop

通常は ``colcon build`` のみで問題貼りませんが，以降の引数には重要な意味を持っているためここで解説します．

- ``--symlink-install``

    ビルド時にビルド元となるパッケージのシンボリックリンクを使用してビルドします．これによりビルド後もソースコードを編集しても再度ビルドする必要がありません．
    開発中のパッケージの場合はこの引数をつけることをおすすめします．

- ``--packages-select``

    ビルド時にこのオプションで指定されたパッケージのみビルドします．ここでは ros2_workshop のみビルドするように指定しています．

**************************
ノードを実行する
**************************

　ビルドが完了したら以下のコマンドを実行してワークスペースを読み込みましょう．するとビルドされたパッケージ情報が読み込まれます．

.. code:: bash

    source /ws/install/setup.bash

　つぎに以下のコマンドを書いてみましょう．

.. code:: bash

    ros2 run ros2_workshop

この時点で **Tabキー** を何回か押すと作成したノード名が表示されます．

.. code::bash

    ros2 run ros2_workshop
    --prefix                 practice_publisher_node 

ノード名を指定して実行してみましょう．現状標準出力のコードを書いていないため何も表示されませんが，エラーが出なければ正常に実行されているはずです．

.. code::bash

    ros2 run ros2_workshop practice_publisher_node 

あらたな コンテナのターミナルを開いて，以下のコマンドを実行して利用可能なトピック一覧を取得してみましょう．

.. code:: bash

    ros2 topic list

すると以下のようにソースコード上で作成したトピック名 **/chatter** が表示されます．

.. code:: bash

   $ ros2 topic list

    /chatter
    /parameter_events
    /rosout

以下のコマンドのようにトピックを指定することで現在利用されているトピックの中身を見ることができます．このコマンドを実行するとソースコード上で指定したメッセージが出力されます．

.. code:: bash

    ros2 topic echo /chatter

また，以下のコマンドを実行すると指定したトピックでのくらいの周期でメッセージが更新されているかわかります．
このコマンドを実行すると ``average rate: 2.000`` と表示されます．これは１秒間に 2 回（2Hz）で更新されていることを示しており，
ソースコードのタイマー関数で指定した実行周期に対応しています．

.. code:: bash

    ros2 topic hz /chatter

　``practice_publisher_node`` を実行しているターミナルで 「Control + C」をしてノードを停止します．これでノードを作成し，ビルドし，実行するまでの基礎的な方法を学びました．

********************
ノードに手を加える
********************

　一応パブリッシャーノードの基礎的な作成方法から実行方法までを解説しましたが，補足事項としてノードに人手間加えみましょう．

Logger 出力を追加する
=======================

　現状のノードでは標準出力が一切ないので，ここで Logger を使った出力を実装してみましょう．

　**Logger** とは，ノードの現在の状態や処理内容をコンソールに出力（ロギング）するための ROS2 標準の仕組みです．Python の ``print()`` 関数に似ていますが，より高機能で ROS2 のシステムと統合されています．

Logger には，出力する情報の重要度に応じて **レベル（Severity Level）** が分かれています．

- ``DEBUG``: デバッグ用の非常に詳細な情報．
- ``INFO``: 通常の動作状況を示す情報．（例：「ノードを起動しました」）
- ``WARN``: 致命的ではないが注意すべき問題が発生したことを示す警告．
- ``ERROR``: 特定の処理が失敗したことを示すエラー．
- ``FATAL``: ノード全体の動作が停止するような致命的なエラー．

これらのレベルを使い分けることで，膨大なログの中から必要な情報だけをフィルタリングして確認することができます．

では，実際に ``publish_callback`` メソッドに Logger を追加してみましょう．メッセージをパブリッシュした直後に，``INFO`` レベルで「今どんなメッセージを送信したか」を出力するようにします．

ノードクラス内では ``self.get_logger()`` を使って Logger オブジェクトを取得できます．

.. code:: python

        def publish_callback(self):

            message = String()
            message.data = 'Hello! ROS2'

            self.publisher.publish(message)
            
            # INFOレベルでログを出力する
            self.get_logger().info(f'Publishing: "{message.data}"')

カウント機能を追加する
========================

　ついでにもう一手間加えましょう．現状では毎回同じ ``'Hello! ROS2'`` という文字列を送っているため，`ros2 topic echo` で見ていても新しいメッセージが来ているのか分かりにくいです．

そこで，メッセージを送信するたびに数字を1ずつ増やしていく **カウンター機能** を実装してみましょう．

まず，イニシャライザ（``__init__`` メソッド）でカウンター用の変数 ``self.count`` を初期化します．

.. code:: python

    class PracticePublisher(Node):
        def __init__(self):
            super().__init__('practice_publisher')
            ... 
            self.timer = self.create_timer(
                0.5,
                self.publish_callback
            )
            # カウンター変数を0で初期化
            self.count = 0

次に，``publish_callback`` メソッドを修正し，このカウンターをメッセージに含め，送信後に1ずつ増やすようにします．

.. code:: python

        def publish_callback(self):
            message = String()
            # メッセージに現在のカウントを含める
            message.data = f'Hello! ROS2 count: {self.count}'

            self.publisher.publish(message)
            self.get_logger().info(f'Publishing: "{message.data}"')
            
            # カウンターを1増やす
            self.count += 1

この状態でもう一度ノードを実行するとメッセージが送られるたびにカウンタが増加し，かつログが出力されるようになります．

正常にノードを終了するようにする
==================================

　現在の状態でノードを実行し，停止すると，このようなエラーが表示されます．

.. code:: 

    ^CTraceback (most recent call last):
    File "/ws/install/ros2_workshop/lib/ros2_workshop/practice_publisher_node", line 33, in <module>
        sys.exit(load_entry_point('ros2-workshop', 'console_scripts', 'practice_publisher_node')())
    File "/ws/build/ros2_workshop/ros2_workshop/practice_publisher.py", line 39, in main
        rclpy.spin(node)
    File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 226, in spin
        executor.spin_once()
    File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 751, in spin_once
        self._spin_once_impl(timeout_sec)
    File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 740, in _spin_once_impl
        handler, entity, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
    File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 723, in wait_for_ready_callbacks
        return next(self._cb_iter)
    File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 620, in _wait_for_ready_callbacks
        wait_set.wait(timeout_nsec)
    KeyboardInterrupt
    [ros2run]: Interrupt

　これは何が起きたかというと **ユーザーが終了コードを送ったことで不正にノードが止められた** ことでこのエラーが発生しています．この問題を解決するには以下のように main 関数上で ``Node.destroy_node()`` 関数を利用して正常にノードを破棄する処理が実行されるようにすることです．

.. code:: python

    def main():
        rclpy.init()
        node = PracticePublisher()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

*********************************
パブリッシャーノードの全体図
*********************************

　いかがこのセクションで扱ったソースコードの完成時の全体図です．

.. code:: python

    #!/usr/bin/env python3
    # Shebang (シバン): このスクリプトをpython3で実行することをシステムに指示するおまじない。

    # rclpy (ROS Client Library for Python) ライブラリから必要なモジュールをインポート
    from rclpy.node import Node  # Nodeクラス: ROS2のノードを作成するための基本的なクラス
    import rclpy               # rclpy: ROS2のPythonクライアントライブラリの本体

    # std_msgs (Standard Messages) パッケージからString型のメッセージ定義をインポート
    # これにより、文字列データをトピックで送受信できるようになる
    from std_msgs.msg import String


    # Nodeクラスを継承して、オリジナルのPublisherノードクラスを定義
    class PracticePublisher(Node):
            # クラスのインスタンスが作成されるときに自動的に呼び出される初期化メソッド (コンストラクタ)
            def __init__(self):
                # 親クラス (Node) のコンストラクタを呼び出し、ノード名を 'practice_publisher' として登録する
                super().__init__('practice_publisher')

                # Publisherを作成する
                # self.create_publisher() メソッドは3つの引数を取る
                self.publisher = self.create_publisher(
                    String,        # 第1引数: パブリッシュするメッセージの型 (ここではString型)
                    '/chatter',    # 第2引数: トピック名 (この名前でメッセージを配信する)
                    10             # 第3引数: QoS (Quality of Service) の設定。ここではキューサイズを10に設定。
                                #           通信が追いつかない場合に最大10個までメッセージを保持できる。
                )

                # タイマーを作成する
                # self.create_timer() メソッドは2つの引数を取る
                self.timer = self.create_timer(
                    0.5,                   # 第1引数: タイマーの周期 (秒単位)。0.5秒ごとにコールバック関数を呼び出す。
                    self.publish_callback  # 第2引数: タイマーによって呼び出されるコールバック関数名。
                )

                # 送信したメッセージの数を数えるためのカウンター変数を初期化
                self.count = 0
            
            # タイマーによって0.5秒ごとに呼び出されるコールバック関数
            def publish_callback(self):

                # 送信するString型のメッセージオブジェクトを作成
                message = String()
                # メッセージの 'data' フィールドに、カウンターを含んだ文字列を代入
                message.data = f'Hello! ROS2 count: {self.count}'

                # 作成したメッセージを実際にパブリッシュ (配信) する
                self.publisher.publish(message)
                # ターミナルにログメッセージを出力する (デバッグや動作確認に便利)
                # self.get_logger() でノード専用のロガーを取得し、.info()で情報レベルのログを出す
                self.get_logger().info(f'Publishing: "{message.data}"')

                # 次のメッセージのためにカウンターを1増やす
                self.count += 1


    # プログラムのメイン処理を定義する関数
    def main():
        # ROS2のクライアントライブラリを初期化。ノードを作成する前に必ず実行する必要がある。
        rclpy.init()
        # PracticePublisherクラスのインスタンスを作成し、ノードとして実体化させる
        node = PracticePublisher()

        # try-except-finallyブロック: Ctrl+Cでプログラムを終了した際などに、後処理を確実に行うための構文
        try:
            # rclpy.spin() はノードを実行状態に保ち、コールバック関数 (タイマーなど) を処理し続ける。
            # この関数はプログラムが終了するまでブロックされる (ここで処理が止まるように見える)。
            rclpy.spin(node)
        except KeyboardInterrupt:
            # ユーザーがCtrl+Cを押してプログラムを中断した場合の処理
            # ここでは特に何もしない (pass) が、明示的に補足することで意図しないエラーを防ぐ
            pass
        finally:
            # tryブロックを抜ける際に必ず実行される後処理
            # ノードを安全に破棄し、リソースを解放する
            node.destroy_node()


    # このスクリプトが直接実行された場合にのみ、以下のブロック内のコードが実行される
    # (他のファイルからモジュールとしてインポートされた場合は実行されない)
    if __name__ == '__main__':
        # main関数を呼び出して、プログラムを開始する
        main()
