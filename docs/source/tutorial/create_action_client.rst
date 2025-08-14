#########################################
Action クライアントノードを作ってみよう
#########################################

　前のセクションでは、時間のかかるタスクを実行する Action サーバーノードを作成しました。ここでは、そのサーバーにタスクを依頼（ゴールを送信）する **Action クライアント** ノードを作成してみましょう。

**********************
Action Client とは
**********************

　**Action Client（クライアント）** は、Action Server に対してタスクの目標（ゴール）を送信し、タスクの実行を依頼するノードです。クライアントはサーバーから送られてくる途中経過（フィードバック）を受け取り、最終的な結果（リザルト）を待つことができます。

.. image:: /_static/about_action.gif

　今回は、前回作成したフィボナッチ数列を計算する Action サーバーに対して、「5番目までの数列を計算してほしい」というゴールを送信するクライアントを作成します。

.. important::

    このチュートリアルを進めるには、前のセクションで作成した `practice_action_server_node` が必要です。

******************************
ノードプログラムの書き方
******************************

　`ros2_workshop` パッケージ内の `ros2_workshop` ディレクトリに、`practice_action_client.py` という名前で新しい Python スクリプトを作成します。

必要なモジュールのインポート
==============================

　`rclpy` と `Node` に加え、Action Clientを扱うための `rclpy.action` から `ActionClient` をインポートします。サーバーと同じく、`Fibonacci` Actionインターフェースもインポートします。

.. code:: python

    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient

    from action_tutorials_interfaces.action import Fibonacci

クラスを作成する
====================

　`PracticeActionClient` というクラスを作成し、`Node` モジュールを継承させます。

.. code:: python

    class PracticeActionClient(Node):

イニシャライザの作成
======================

　イニシャライザ `__init__` を定義し，ノード名の宣言とAction Clientの作成を行います。

.. code:: python

    class PracticeActionClient(Node):
        def __init__(self):
            super().__init__('practice_action_client')
            self.action_client = ActionClient(self, Fibonacci, 'fibonacci')

`ActionClient` の引数は以下の通りです。

* **第1引数**: ノードのインスタンス (`self`)。
* **第2引数**: Actionの型 (`Fibonacci`)。
* **第3引数**: Action名 (`fibonacci`)。この名前を持つサーバーを探しにいきます。

ゴールを送信するメソッドの作成
==================================

　次に、サーバーにゴールを送信するためのメソッド `send_goal` を作成します。

.. code:: python

    class PracticeActionClient(Node):
        def __init__(self):
            # ... (イニシャライザは省略)

        def send_goal(self, order):
            # サーバーが起動するまで待機
            self.action_client.wait_for_server()

            # 送信するゴールのメッセージを作成
            goal_msg = Fibonacci.Goal()
            goal_msg.order = order

            # ゴールを非同期で送信し、フィードバック用のコールバックを登録
            send_goal_future = self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback)

            # ゴールがサーバーに受け入れられたかどうかの結果を待つためのコールバックを登録
            send_goal_future.add_done_callback(self.goal_response_callback)

このメソッドの処理の流れは以下の通りです。

1.  `self.action_client.wait_for_server()`: 指定したAction名を持つサーバーが起動するまで、ここで処理を待ちます。
2.  `goal_msg = Fibonacci.Goal()`: `Fibonacci` ActionのGoal部分のメッセージオブジェクトを作成します。
3.  `goal_msg.order = order`: 引数で受け取った `order` の値をメッセージに設定します。
4.  `self.action_client.send_goal_async(...)`: サーバーに非同期でゴールを送信します。
    * 第1引数: 送信するゴールのメッセージ。
    * `feedback_callback`: サーバーからフィードバックが届くたびに呼び出される関数を指定します。
5.  `send_goal_future.add_done_callback(...)`: ゴールを送信した結果（サーバーに受け入れられたか、拒否されたか）が返ってきたときに呼び出される関数を登録します。

コールバック関数を作成する
============================

　Action Clientの処理は非同期で行われるため、サーバーからの応答をコールバック関数で受け取る必要があります。「ゴールの応答」「フィードバック」「最終結果」の3つに対するコールバック関数を実装します。

ゴールの応答コールバック
--------------------------

　`send_goal_async` の結果を受け取るための `goal_response_callback` を作成します。この関数は、サーバーがゴールを受け入れたか、あるいは拒否したかを知るために使われます。

.. code:: python

    class PracticeActionClient(Node):
        # ... (省略)

        def goal_response_callback(self, future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected :(')
                return

            self.get_logger().info('Goal accepted :)')

            # ゴールが受け入れられたので、最終結果を受け取る準備をする
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)

- `goal_handle = future.result()`: `send_goal_async` の未来の結果（`future`）から、ゴールハンドルを取得します。ゴールハンドルは、特定のゴールを管理するためのオブジェクトです。
- `if not goal_handle.accepted`: ゴールがサーバーに受け入れられなかった場合の処理です。
- `goal_handle.get_result_async()`: ゴールが受け入れられた場合、このメソッドを使ってサーバーからの最終結果を非同期でリクエストします。
- `get_result_future.add_done_callback(...)`: 最終結果が返ってきたときに呼び出されるコールバック関数 `get_result_callback` を登録します。

フィードバックコールバック
----------------------------

　サーバーがタスクを実行している最中に送ってくる途中経過（フィードバック）を受け取るための `feedback_callback` を作成します。

.. code:: python

    class PracticeActionClient(Node):
        # ... (省略)

        def feedback_callback(self, feedback_msg):
            feedback = feedback_msg.feedback
            self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

この関数は、`send_goal_async` の引数として渡され、サーバーからフィードバックが届くたびに自動的に呼び出されます。受け取ったフィードバックの内容をログに出力しています。

最終結果コールバック
------------------------

　タスクが完了したときの最終結果を受け取るための `get_result_callback` を作成します。

.. code:: python

    class PracticeActionClient(Node):
        # ... (省略)

        def get_result_callback(self, future):
            result = future.result().result
            self.get_logger().info(f'Result: {result.sequence}')
            rclpy.shutdown()

- `result = future.result().result`: `get_result_async` の未来の結果から、最終結果のメッセージを取得します。
- `rclpy.shutdown()`: 結果を受け取ったら、ノードを終了させるために `rclpy.shutdown()` を呼び出します。

実行関数 `main` を作成する
=================================

　最後に、作成したクラスを実行するための `main` 関数と実行ブロックを作成します。

.. code:: python

    def main():
        rclpy.init()
        node = PracticeActionClient()
        node.send_goal(5)  # 5番目までのフィボナッチ数列をリクエスト
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

    if __name__ == '__main__':
        main()

`PracticeActionClient` のインスタンスを作成した後、`send_goal` メソッドを呼び出してActionサーバーにゴールを送信します。`rclpy.spin(node)` でノードを実行状態に保ち、サーバーからのコールバックを待ち受けます。

****************************************
パッケージにノードを登録する
****************************************

　作成した `practice_action_client.py` を `ros2 run` で実行できるように、`setup.py` の `entry_points` に追記します．

.. code:: diff

    entry_points={
        'console_scripts': [
            'practice_publisher_node = ros2_workshop.practice_publisher:main',
            'practice_subscriber_node = ros2_workshop.practice_subscriber:main',
            'practice_service_server_node = ros2_workshop.practice_service_server:main',
            'practice_service_client_node = ros2_workshop.practice_service_client:main',
            'practice_action_server_node = ros2_workshop.practice_action_server:main',
    +       'practice_action_client_node = ros2_workshop.practice_action_client:main',
        ],
    },

***************************
パッケージをビルドする
***************************

　`setup.py` を変更したので，再度パッケージをビルドします．

.. code:: bash

    cd /ws
    colcon build --symlink-install --packages-select ros2_workshop

**************************
ノードを実行する
**************************

　ビルド完了後，ワークスペースを読み込み，サーバーとクライアントを**2つのターミナル**で実行します．

.. code:: bash

    source /ws/install/setup.bash

まず，**1つ目のターミナル** でAction Serverノードを起動します．

.. code:: bash

    ros2 run ros2_workshop practice_action_server_node

次に，**2つ目のターミナル** でAction Clientノードを起動します．

.. code:: bash

    ros2 run ros2_workshop practice_action_client_node

クライアント側のターミナルに、サーバーからのフィードバックと最終結果が順に表示され、処理が完了すると自動的に終了します。

.. code::

    [INFO] [practice_action_client]: Goal accepted :)
    [INFO] [practice_action_client]: Received feedback: [0, 1, 1]
    [INFO] [practice_action_client]: Received feedback: [0, 1, 1, 2]
    [INFO] [practice_action_client]: Received feedback: [0, 1, 1, 2, 3]
    [INFO] [practice_action_client]: Received feedback: [0, 1, 1, 2, 3, 5]
    [INFO] [practice_action_client]: Result: [0, 1, 1, 2, 3, 5]

同時に，サーバー側のターミナルには処理中のログが表示されます．

.. code::

    [INFO] [practice_action_server]: Executing goal...
    [INFO] [practice_action_server]: Feedback: [0, 1, 1]
    [INFO] [practice_action_server]: Feedback: [0, 1, 1, 2]
    [INFO] [practice_action_server]: Feedback: [0, 1, 1, 2, 3]
    [INFO] [practice_action_server]: Feedback: [0, 1, 1, 2, 3, 5]
    [INFO] [practice_action_server]: Returning result: [0, 1, 1, 2, 3, 5]

これで、クライアントからサーバーへタスクを依頼し、その結果を受け取る一連の流れが完成しました。

*************************************
アクションクライアントノードの全体図
*************************************

　このセクションで扱ったソースコードの完成時の全体図です．

.. code:: python

    #!/usr/bin/env python3
    # Shebang (シバン): このスクリプトをpython3で実行することをシステムに指示するおまじない．

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient

    from action_tutorials_interfaces.action import Fibonacci

    # Nodeクラスを継承して，オリジナルのAction Clientノードクラスを定義
    class PracticeActionClient(Node):

        # クラスのインスタンスが作成されるときに自動的に呼び出される初期化メソッド (コンストラクタ)
        def __init__(self):
            # 親クラス (Node) のコンストラクタを呼び出し，ノード名を 'practice_action_client' として登録
            super().__init__('practice_action_client')
            # Action Clientを作成する
            # ActionClient() メソッドは3つの引数を取る
            self.action_client = ActionClient(
                self,           # 第1引数: ノードのインスタンス
                Fibonacci,      # 第2引数: Actionの型
                'fibonacci')    # 第3引数: Action名

        # サーバーにゴールを送信するメソッド
        def send_goal(self, order):
            # サーバーが起動するまで待機する
            self.action_client.wait_for_server()

            # 送信するゴールのメッセージを作成
            goal_msg = Fibonacci.Goal()
            goal_msg.order = order

            # ゴールを非同期で送信し、フィードバック用のコールバック関数を登録
            self.get_logger().info('Sending goal request...')
            send_goal_future = self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback)

            # ゴールがサーバーに受け入れられたかどうかの結果を待つためのコールバック関数を登録
            send_goal_future.add_done_callback(self.goal_response_callback)

        # ゴールを送信した結果（受理/拒否）を受け取ったときに呼び出されるコールバック関数
        def goal_response_callback(self, future):
            # futureオブジェクトからゴールハンドルを取得
            goal_handle = future.result()
            
            # ゴールがサーバーに受け入れられなかった場合
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected :(')
                return

            # ゴールがサーバーに受け入れられた場合
            self.get_logger().info('Goal accepted :)')

            # ゴールが受け入れられたので、最終結果を非同期でリクエストし、
            # 結果を受け取るためのコールバック関数を登録
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)

        # サーバーからフィードバックが届いたときに呼び出されるコールバック関数
        def feedback_callback(self, feedback_msg):
            feedback = feedback_msg.feedback
            self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

        # サーバーから最終結果が届いたときに呼び出されるコールバック関数
        def get_result_callback(self, future):
            # futureオブジェクトから最終結果を取得
            result = future.result().result
            self.get_logger().info(f'Result: {result.sequence}')
            
            # 結果を受け取ったらROS2をシャットダウンしてプログラムを終了
            rclpy.shutdown()


    # プログラムのメイン処理を定義する関数
    def main():
        # ROS2のクライアントライブラリを初期化
        rclpy.init()

        # PracticeActionClientクラスのインスタンスを作成し，ノードとして実体化
        node = PracticeActionClient()
        
        # 5番目までのフィボナッチ数列を計算するようサーバーにリクエスト
        node.send_goal(5)

        # try-except-finallyブロック: Ctrl+Cで終了した際の後処理を確実に行う
        try:
            # rclpy.spin() はノードを実行状態に保ち，コールバック関数が呼ばれるのを待つ
            rclpy.spin(node)
        except KeyboardInterrupt:
            # ユーザーがCtrl+Cを押した場合の処理
            pass
        finally:
            # ノードを安全に破棄する
            node.destroy_node()


    # このスクリプトが直接実行された場合にのみ，main()関数を実行
    if __name__ == '__main__':
        main()
