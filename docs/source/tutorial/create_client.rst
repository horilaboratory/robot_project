##########################################
Service クライアントノードを作ってみよう
##########################################

　前回作成した Service サーバーにリクエストを送信する Service クライアントノードを作成してみましょう．

********************
Service Client とは
********************

　**Service Client（クライアント）** とは，Service Server に対して処理を依頼（リクエスト）し，その結果（レスポンス）を受け取るノードです．サーバーが特定の機能を提供し，クライアントがその機能を利用するという関係になります．

　クライアントノードは，サーバーノードとは異なり，常に起動している必要はありません．多くの場合，「リクエストを送信し，レスポンスを受け取ったら終了する」という一度きりのタスクを実行するために使われます．

******************************
ノードプログラムの書き方
******************************

　これまでと同様に，``practice_service_client.py`` という名前で Python スクリプトを作成します．

必要なモジュールのインポート
==============================

　サーバーの時と同様に，``rclpy``，``Node``，そして ``AddTwoInts`` サービスをインポートします．また，今回はコマンドラインから足し算する数を受け取るため，``sys`` モジュールもインポートします．

.. code:: python

    #!/usr/bin/env python3
    import sys
    from rclpy.node import Node
    import rclpy

    from example_interfaces.srv import AddTwoInts

クラスを作成する
====================

　スクリプトファイル名にちなんだ ``PracticeServiceClient`` というクラスを作成し，``Node`` モジュールを継承させます．

.. code:: python

    class PracticeServiceClient(Node):

イニシャライザの作成
======================

　イニシャライザ ``__init__`` を定義します．サーバーとは異なり，クライアントの主な処理はイニシャライザの外で行われるため，ここではノード名とクライアントの作成のみを行います．

.. code:: python

    class PracticeServiceClient(Node):
        def __init__(self):
            super().__init__('practice_service_client')
            self.cli = self.create_client(AddTwoInts, 'add_two_ints')
            
            # サービスが利用可能になるまで待機する
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            
            # リクエストのテンプレートを作成
            self.req = AddTwoInts.Request()

　``self.create_client()`` でクライアントを作成します．引数は ``(サービス型, サービス名)`` です．サーバー側と一致させる必要があります．

　また，``self.cli.wait_for_service()`` は，指定したサービスが利用可能になるまで待機する関数です．サーバーが起動していない場合にクライアントがエラーで落ちるのを防ぎます．

リクエストを送信するメソッドを作成する
========================================

　実際にリクエストを送信し，レスポンスを待つためのメソッド ``send_request`` を作成します．

.. code:: python

    class PracticeServiceClient(Node):
        def __init__(self):
            ...

        def send_request(self, a, b):
            # リクエストに値を設定
            self.req.a = a
            self.req.b = b
            
            # 非同期でサービスを呼び出し，future (未来の結果) を受け取る
            self.future = self.cli.call_async(self.req)
            
            # futureが完了するまでノードをスピンさせる
            rclpy.spin_until_future_complete(self, self.future)
            
            # futureから結果を返す
            return self.future.result()

- ``self.cli.call_async(self.req)``
  サービスを **非同期 (async)** で呼び出します．これは，リクエストを送信した直後にプログラムの実行をブロックせず，すぐに次の処理に進むことを意味します．この関数は ``Future`` オブジェクトを返します．これは「未来に得られる結果」を表すものです．

- ``rclpy.spin_until_future_complete(self, self.future)``
  指定した ``Future`` オブジェクトが完了する（つまり，サーバーからレスポンスが返ってくる）まで，ノードをスピンさせ（処理を続け）ます．

実行関数 ``main`` を作成する
=================================

　クライアントノードの ``main`` 関数は，サーバーやサブスクライバーとは大きく異なります．``rclpy.spin()`` で無限に待ち続けるのではなく，リクエストを送信してレスポンスを受け取ったら終了するという流れになります．

.. code:: python

    def main():
        rclpy.init()

        # コマンドライン引数が正しいかチェック
        if len(sys.argv) != 3:
            print('Usage: ros2 run ros2_workshop practice_service_client_node <a> <b>')
            return

        # クライアントノードのインスタンスを作成
        client_node = PracticeServiceClient()
        
        # リクエストを送信し，レスポンスを受け取る
        # コマンドライン引数を整数に変換して渡す
        response = client_node.send_request(int(sys.argv[1]), int(sys.argv[2]))

        # レスポンスの有無で処理を分岐
        if response:
            client_node.get_logger().info(
                f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
        else:
            client_node.get_logger().info('Service call failed %r' % (client_node.future.exception(),))

        # ノードを破棄し，rclpyをシャットダウン
        client_node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

　``sys.argv`` はコマンドラインから渡された引数のリストです．``sys.argv[0]`` はスクリプト名，``sys.argv[1]`` 以降が引数に対応します．これを使って，実行時に足し算したい2つの数値をノードに渡します．

****************************
ノードを登録する
****************************

　作成した ``practice_service_client.py`` を ``setup.py`` に登録します．

.. code:: python

    entry_points={
        'console_scripts': [
            'practice_publisher_node = ros2_workshop.practice_publisher:main',
            'practice_subscriber_node = ros2_workshop.practice_subscriber:main',
            'practice_service_server_node = ros2_workshop.practice_service_server:main',
            'practice_service_client_node = ros2_workshop.practice_service_client:main'
        ],
    },

***************************
パッケージをビルドする
***************************

　``setup.py`` を変更したので，再度パッケージをビルドします．

.. code:: bash

    cd /ws
    colcon build --symlink-install --packages-select ros2_workshop

**************************
ノードを実行する
**************************

　ビルドが完了したら，まずワークスペースを読み込みます．

.. code:: bash

    source /ws/install/setup.bash

次に，**2つのターミナル** を使ってサーバーとクライアントを順に実行します．

まず，**1つ目のターミナル** で Service Server ノードを起動します．

.. code:: bash

    ros2 run ros2_workshop practice_service_server_node

次に，**2つ目のターミナル** で Service Client ノードを，引数付きで起動します．ここでは例として 5 と 8 を足してみましょう．

.. code:: bash

    ros2 run ros2_workshop practice_service_client_node 5 8

クライアント側のターミナルに，サーバーからのレスポンス（計算結果）が表示されます．

.. code::

    [INFO] [practice_service_client]: Result of add_two_ints: for 5 + 8 = 13

同時に，サーバー側のターミナルには，リクエストを受け付けて処理した際のログが表示されます．

.. code::

    [INFO] [practice_service_server]: Incoming request
    a: 5 b: 8
    Sending back response: [13]

クライアントノードはレスポンスを受け取ると自動的に終了します．

**************************************
サービスクライアントノードの全体図
**************************************

　いかがこのセクションで扱ったソースコードの完成時の全体図です．

.. code:: python

    #!/usr/bin/env python3
    # Shebang (シバン): このスクリプトをpython3で実行することをシステムに指示するおまじない。

    # sysモジュール: コマンドライン引数を扱うためにインポート
    import sys
    # rclpy (ROS Client Library for Python) ライブラリから必要なモジュールをインポート
    from rclpy.node import Node  # Nodeクラス: ROS2のノードを作成するための基本的なクラス
    import rclpy               # rclpy: ROS2のPythonクライアントライブラリの本体

    # example_interfaces パッケージから AddTwoInts というサービス定義をインポート
    from example_interfaces.srv import AddTwoInts


    # Nodeクラスを継承して、オリジナルのService Clientノードクラスを定義
    class PracticeServiceClient(Node):
            # クラスのインスタンスが作成されるときに自動的に呼び出される初期化メソッド (コンストラクタ)
            def __init__(self):
                # 親クラス (Node) のコンストラクタを呼び出し、ノード名を 'practice_service_client' として登録
                super().__init__('practice_service_client')

                # Service Clientを作成する
                # self.create_client() メソッドは2つの引数を取る
                self.cli = self.create_client(
                    AddTwoInts,       # 第1引数: サービスの型 (AddTwoInts型)
                    'add_two_ints'    # 第2引数: サービス名 (この名前のサーバーを呼び出す)
                )

                # サーバーが起動してサービスが利用可能になるまで1秒ごとに待機する
                while not self.cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')
                
                # リクエストメッセージのオブジェクトをあらかじめ生成しておく
                self.req = AddTwoInts.Request()
            
            # リクエストを送信するためのメソッド
            def send_request(self, a, b):
                # リクエストオブジェクトの 'a' と 'b' フィールドに引数の値を設定
                self.req.a = a
                self.req.b = b
                
                # サービスを非同期で呼び出し、'future' オブジェクトを受け取る
                # 'future' は、未来に結果が格納されることを約束するもの
                self.future = self.cli.call_async(self.req)

                # 'future' が完了する (サーバーからレスポンスが返ってくる) までノードの処理を待機させる
                rclpy.spin_until_future_complete(self, self.future)

                # 'future' から実際の結果 (レスポンス) を取り出して返す
                return self.future.result()


    # プログラムのメイン処理を定義する関数
    def main():
        # ROS2のクライアントライブラリを初期化。
        rclpy.init()

        # コマンドラインから渡された引数の数をチェック
        # (sys.argv[0]はスクリプト名なので、引数が2つならリストの長さは3になる)
        if len(sys.argv) != 3:
            # 引数が正しくない場合、使い方を表示して終了
            print('Usage: ros2 run ros2_workshop practice_service_client_node <a> <b>')
            return

        # PracticeServiceClientクラスのインスタンスを作成し、ノードとして実体化させる
        client_node = PracticeServiceClient()
        
        # send_requestメソッドを呼び出し、リクエストを送信する
        # コマンドライン引数は文字列なので、int()で整数に変換する
        response = client_node.send_request(int(sys.argv[1]), int(sys.argv[2]))

        # レスポンスが正常に返ってきたかチェック
        if response:
            # 成功した場合、結果をログに出力
            client_node.get_logger().info(
                f'Result of add_two_ints: for {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
        else:
            # 失敗した場合 (サーバーが落ちたなど)、エラーログを出力
            client_node.get_logger().info('Service call failed %r' % (client_node.future.exception(),))

        # ノードを安全に破棄する
        client_node.destroy_node()
        # rclpyをシャットダウンしてリソースを解放する
        rclpy.shutdown()


    # このスクリプトが直接実行された場合にのみ、以下のブロック内のコードが実行される
    if __name__ == '__main__':
        # main関数を呼び出して、プログラムを開始する
        main()
