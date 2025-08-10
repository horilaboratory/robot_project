###################################
Subscriber ノードを作ってみよう
###################################

　前回作成した Publisher ノードが発行するトピックをサブスクライブ（購読）するノードを作成してみましょう．

******************
Subscriber とは
******************

　Subscriber（サブスクライバー）とは ROS2 トピックを購読する者を指し，そのノードを **Subscriber Node（サブスクライバー・ノード）** と呼びます．Publisher が発行したメッセージを受け取り，何らかの処理を実行する役割を担います．

******************************
ノードプログラムの書き方
******************************

　Publisher の時と同様に，ament_python パッケージではパッケージ内部にある **パッケージ名と同じディレクトリ** 内にノードとなる Python スクリプトを作成します．ここでは ``practice_subscriber.py`` というノードを作成してみましょう．

　ここでも **最もベーシックかつシンプルな書き方であるクラスを用いたノードの作成を解説します．**

必要なモジュールのインポート
==============================

　Publisher と同様に ``rclpy`` とその中にあるモジュール ``Node`` をインポートします．また，購読するトピックのメッセージ型が ``String`` であるため，``std_msgs.msg`` から ``String`` もインポートしておきます．

.. code:: python

    #!/usr/bin/env python3
    from rclpy.node import Node
    import rclpy

    from std_msgs.msg import String

クラスを作成する
====================

　スクリプト内に任意のクラスを１つ作成しましょう．ここではスクリプトファイル名にちなんだ ``PracticeSubscriber`` というクラスを作成し，``Node`` モジュールを継承させます．

.. code:: python

    class PracticeSubscriber(Node):

イニシャライザの作成
======================

　クラスが呼び出されたときに自動的に実行されるイニシャライザ ``__init__`` を定義します．

.. code:: python

    class PracticeSubscriber(Node):
        def __init__(self):

ノードの宣言を書く
===================

　この Python スクリプトを ROS2 ノードとして機能させるため，イニシャライザ内で ``super().__init__()`` を使ってノード名 ``practice_subscriber`` を宣言します．

.. code:: python

    class PracticeSubscriber(Node):
        def __init__(self):
            super().__init__('practice_subscriber')

Subscriber を作成する
=======================
　次に Subscriber を作りましょう．要はトピックをサブスクライブするためのインターフェースを用意します．ここでは ``String`` 型のメッセージを使い，``/chatter`` という名前のトピックを購読するサブスクライバーを作成してみましょう．

イニシャライザ内で以下のようにサブスクライバーを作成します．

.. code:: python

    class PracticeSubscriber(Node):
        def __init__(self):
            super().__init__('practice_subscriber')

            self.subscription = self.create_subscription(
                String,
                '/chatter',
                self.listener_callback,
                10
            )

　``self.create_subscription`` 関数がサブスクライバーで，これを変数 ``self.subscription`` として定義しています．

この関数の引数はこのようになっています．

.. code:: python

    rclpy.node.Node.create_subscription(
        msg_type,
        topic,
        callback,
        qos_profile
    )

- ``msg_type``
    購読したいメッセージ型のモジュールを代入します．ここでは ``String`` 型のメッセージを使うため ``std_msgs.msg.String`` モジュールを代入しています．

- ``topic``
    購読したいトピック名を文字列で指定します．Publisher が発行しているトピック名と完全に一致させる必要があります．ここでは **/chatter** を指定しています．

- ``callback``
    メッセージを受信するたびに呼び出される **コールバック関数** を指定します．ここでは ``listener_callback`` というメソッドを指定しています．このメソッドはこれから作成します．

- ``qos_profile``
    QoS (Quality of Service) の設定です．Publisher 側と設定を合わせることが推奨されます．ここでは Publisher と同様に，**トピック深度（Queue Size）** を **10** に指定しています．

コールバック関数を作成する
============================

　トピックからメッセージを受信するたびに実行される ``listener_callback`` メソッドを ``PracticeSubscriber`` クラスに作成してください．

.. important::

    Subscriber のコールバック関数は，引数として受信したメッセージオブジェクトを受け取ります．ここでは ``msg`` という名前の引数を定義しています．この ``msg`` に ``String`` 型のメッセージデータが格納されます．

.. code:: python

    class PracticeSubscriber(Node):
        def __init__(self):
            ...
        
        def listener_callback(self, msg):

受信したメッセージを処理する
================================

　コールバック関数 ``listener_callback`` の内部で，受信したメッセージを処理するコードを記述します．ここでは，受信したメッセージの内容を Logger を使ってコンソールに出力してみましょう．

　受信したメッセージデータは，引数 ``msg`` の ``data`` フィールドに格納されています．Publisher の実装で ``message.data = '...'`` とした部分です．

.. code:: python
        
        def listener_callback(self, msg):
            self.get_logger().info(f'I heard: "{msg.data}"')

　これで，``/chatter`` トピックからメッセージを受信するたびに，その内容が ``I heard: "..."`` という形式でコンソールに表示されます．

実行関数 ``main`` を作成する
=================================

　Publisher の時と同様に，作成した ``PracticeSubscriber`` クラスを実行するための ``main`` 関数を作成します．処理の流れは全く同じです．

.. code:: python

    class PracticeSubscriber(Node):
        ...
    

    def main():
        rclpy.init()
        node = PracticeSubscriber()
        rclpy.spin(node)

実行処理を書く
================

　最後に，``main`` 関数を実行するためのおまじないを記述します．

.. code:: python

    def main():
        rclpy.init()
        node = PracticeSubscriber()
        rclpy.spin(node)
    
    if __name__ == '__main__':
        main()

これでトピックをサブスクライブするノードスクリプトは完成です！

****************************
ノードを登録する
****************************

　作成した ``practice_subscriber.py`` を ``ros2 run`` コマンドで実行できるように，``setup.py`` に登録します．

　``setup.py`` を開き，``entry_points`` の ``'console_scripts'`` リストに，新しいノードを追加します．Publisher の登録に追記する形になります．

.. code:: python

    entry_points={
        'console_scripts': [
            'practice_publisher_node = ros2_workshop.practice_publisher:main',
            'practice_subscriber_node = ros2_workshop.practice_subscriber:main'
        ],
    },

この記述は，「**``practice_subscriber_node`` という名前で，``ros2_workshop`` パッケージ内の ``practice_subscriber.py`` スクリプトにある ``main`` 関数を実行する**」という意味になります．

***************************
パッケージをビルドする
***************************

　``setup.py`` を変更したので，再度パッケージをビルドする必要があります．ワークスペース直下（``/ws``）で ``colcon build`` を実行します．

.. code:: bash

    cd /ws
    colcon build --symlink-install --packages-select ros2_workshop

**************************
ノードを実行する
**************************

　ビルドが完了したら，まずワークスペースを読み込みます．

.. code:: bash

    source /ws/install/setup.bash

次に，**2つのターミナル** を使って Publisher と Subscriber の両方を実行します．

まず，**1つ目のターミナル** で Publisher ノードを起動します．

.. code:: bash

    ros2 run ros2_workshop practice_publisher_node

次に，**2つ目のターミナル** を開いて，Subscriber ノードを起動します．

.. code:: bash

    ros2 run ros2_workshop practice_subscriber_node

すると，Subscriber ノードを実行しているターミナルに，Publisher が送信したメッセージが次々と表示されるはずです．

.. code::

    [INFO] [practice_subscriber]: I heard: "Hello! ROS2 count: 0"
    [INFO] [practice_subscriber]: I heard: "Hello! ROS2 count: 1"
    [INFO] [practice_subscriber]: I heard: "Hello! ROS2 count: 2"
    ...

これで，Publisher から Subscriber への一連の通信が確認できました．
両方のターミナルで「Control + C」を押してノードを停止してください．

正常にノードを終了するようにする
==================================

　Publisher と同様に，Subscriber も ``KeyboardInterrupt`` で終了した際にエラーが表示されます．``main`` 関数に ``try-except-finally`` 構文を追加して，ノードが正常に破棄されるようにしましょう．

.. code:: python

    def main():
        rclpy.init()
        node = PracticeSubscriber()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

*********************************
サブスクライバーノードの全体図
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


    # Nodeクラスを継承して、オリジナルのSubscriberノードクラスを定義
    class PracticeSubscriber(Node):
            # クラスのインスタンスが作成されるときに自動的に呼び出される初期化メソッド (コンストラクタ)
            def __init__(self):
                # 親クラス (Node) のコンストラクタを呼び出し、ノード名を 'practice_subscriber' として登録する
                super().__init__('practice_subscriber')

                # Subscriberを作成する
                # self.create_subscription() メソッドは4つの引数を取る
                self.subscription = self.create_subscription(
                    String,                 # 第1引数: サブスクライブするメッセージの型 (String型)
                    '/chatter',             # 第2引数: トピック名 (この名前のトピックを購読する)
                    self.listener_callback, # 第3引数: メッセージ受信時に呼び出されるコールバック関数
                    10                      # 第4引数: QoS設定 (キューサイズ)
                )

            # メッセージを受信するたびに呼び出されるコールバック関数
            # 引数 'msg' に受信したメッセージオブジェクトが格納される
            def listener_callback(self, msg):
                # ターミナルにログメッセージを出力する
                # self.get_logger() でノード専用のロガーを取得し、.info()で情報レベルのログを出す
                # msg.data で受信したメッセージの本体 (文字列) にアクセスできる
                self.get_logger().info(f'I heard: "{msg.data}"')


    # プログラムのメイン処理を定義する関数
    def main():
        # ROS2のクライアントライブラリを初期化。ノードを作成する前に必ず実行する必要がある。
        rclpy.init()
        # PracticeSubscriberクラスのインスタンスを作成し、ノードとして実体化させる
        node = PracticeSubscriber()

        # try-except-finallyブロック: Ctrl+Cでプログラムを終了した際などに、後処理を確実に行うための構文
        try:
            # rclpy.spin() はノードを実行状態に保ち、コールバック関数を処理し続ける。
            # この関数はプログラムが終了するまでブロックされる。
            rclpy.spin(node)
        except KeyboardInterrupt:
            # ユーザーがCtrl+Cを押してプログラムを中断した場合の処理
            pass
        finally:
            # tryブロックを抜ける際に必ず実行される後処理
            # ノードを安全に破棄し、リソースを解放する
            node.destroy_node()


    # このスクリプトが直接実行された場合にのみ、以下のブロック内のコードが実行される
    if __name__ == '__main__':
        # main関数を呼び出して、プログラムを開始する
        main()
