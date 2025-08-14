#########################################
Action サーバーノードを作ってみよう
#########################################

　これまでに Pub/Sub による一方的なメッセージ通知と，Service による同期的なリクエスト／レスポンス処理を学びました．ここでは，より複雑で時間のかかるタスクを実行するための通信モデルである **Action（アクション）** のサーバーノードを作成してみましょう．

********************
Action Server とは
********************

　**Action（アクション）** とは，開始から完了までに時間がかかり，その途中経過（フィードバック）を知りたいようなタスクに使われる，非同期な通信モデルです．

.. image:: /_static/about_action.gif

- **Action Client（クライアント）**: タスクの目標（ゴール）を送信し，タスクの実行を依頼するノード．
- **Action Server（サーバー）**: ゴールを受け取ってタスクを実行し，その進捗状況をフィードバックとして随時クライアントに報告し，最終的な結果を返すノード．

　Service が「関数の呼び出し」のように一度のリクエストに対して一度のレスポンスを返すのに対し，Action は以下の3つの要素で構成されます．

* **Goal (ゴール)**: クライアントがサーバーに達成してほしい目標．（例：「ロボットアームを座標 (X, Y, Z) に動かせ」）
* **Feedback (フィードバック)** : サーバーがゴールを達成するまでの途中経過．（例：「目標まであと50%」，「現在角度30度」）
* **Result (リザルト)** : タスクが完了したときの最終結果．（例：「目標座標に到達しました」，「エラーにより失敗しました」）

eコマースでの注文に例えると，**ゴール** は「商品を注文する」こと，**フィードバック** は「注文を確認しました」「商品を発送しました」といったステータスの更新，そして**リザルト** は「商品がお手元に届きました」という最終報告に相当します．今回は，指定された数までフィボナッチ数列を計算するAction Serverを作成します．

*******************************************
Action インターフェースをインストールする
*******************************************

　このチュートリアルでは ``Fibonacci`` という Action メッセージインターフェースを使用した Action サーバーを作成します．では，以下のコマンドを実行して ``Fibonacci`` という名前の Action インターフェースがあるかどうか探してみましょう．

　以下のコマンドは現在利用可能な ROS2 Message, Service, Action で使われるインターフェース一覧を表示します．

.. code:: bash

    ros2 interface list

.. hint::

    　しかしあまりにもたくさんのインターフェースが表示されるため探したいインターフェースがあるかどうかよくわかりませんよね．以下のように末尾に ``| grep <検索したい文字列>`` のように文字列フィルタリングを適応することで探したい項目が存在するかどうか確認することができます．

    .. code:: bash

        ros2 interface list | grep Fibonacci

　もし Docker コンテナを使い ROS2 のワークショップに参加している方は Fibonacci が存在しないことを確認できるでしょう．もしローカルで ROS2 を使用している場合はもしかしたら Fibonacci インターフェースが存在するかもしれません．

　そこで ROS2 パッケージ管理でよく使う **rosdep（ロスデプ）** を使い必要なパッケージをインストールして依存関係を解決する方法を解説しましょう．

***********************************
インストールしたいパッケージを探す
***********************************

　Fibonacci インターフェースを持つパッケージをインストールするために，rosdep という依存関係解決ツールを使ってみましょう．これを使うことで，例えば他の ROS2 環境で開発したパッケージを使用するときに rosdep を使うことですぐに依存関係を解決することができます．

　Fibonacci インターフェースは ``action_tutorials_interfaces`` というパッケージに内包されています．以下のコマンドを実行して rosdep からこのパッケージがインストール可能かどうか確かめてみましょう．

　初めて rosdep を使う場合は以下のコマンドを実行して rosdep を初期化してください．しかし，Docker コンテナを利用している方はこの作業はすでに行われているためエラーが発生しますが，正常な反応なので気にする必要はありません．

.. code:: bash

    sudo rosdep init

　次に以下のコマンドを実行して rosdep を最新の状態にアップデートしてください．

.. code:: bash

    rosdep update

　次に以下のコマンドを実行してパッケージ ``action_tutorials_interfaces`` がインストールできるかどうか確認します．応答があればそのパッケージは Ubuntu のパッケージ管理ツール ``apt`` からインストール可能であることを示しています．

.. code:: bash

    rosdep db | grep action_tutorials_interfaces

.. hint::

    上記のコマンドのように ``rosdep db | grep 探したいパッケージ名`` を実行することで rosdep からインストール可能なパッケージを検索することができます．

　上記のコマンドを実行すると，以下のような応答があるでしょう．

.. code::

    action_tutorials_interfaces -> ros-humble-action-tutorials-interfaces

これの読み方は以下のとおりです．右に書かれているパッケージ名で ``apt install`` することで簡単に該当のパッケージをインストールすることができます．

.. code::

    パッケージ名 -> apt でインストールできるパッケージ名

このまま ``sudo apt install ros-humble-action-tutorials-interfaces`` を実行してもいいのですがここでは続けて ``package.xml`` と ``rosdep`` を使い依存関係となるパッケージをインストールする方法を解説します．

********************
依存関係を解決する
********************

　依存関係というのはどういうことかというと，ようはパッケージ A を動かすためにはパッケージ B が必要な場合，**パッケージ A はパッケージ B と依存関係にある** ことを示しています．
ROS2 パッケージではそんな依存関係を示す場所として ``package.xml`` が用意されています．作成した ``ros2_workshop`` パッケージの package.xml を開いてみましょう．

　``<package>`` タグ内に以下のように依存関係となるパッケージ ``action_tutorials_interfaces`` を追記しましょう．

.. code:: diff

    <package format="3">
      <name>ros2_workshop</name>
      <version>0.0.0</version>
      <description>TODO: Package description</description>
      <maintainer email="root@todo.todo">root</maintainer>
      <license>TODO: License declaration</license>

      <test_depend>ament_copyright</test_depend>
      <test_depend>ament_flake8</test_depend>
      <test_depend>ament_pep257</test_depend>
      <test_depend>python3-pytest</test_depend>

    + <depend>action_tutorials_interfaces</depend>

      <export>
        <build_type>ament_python</build_type>
      </export>
    </package>

.. hint::

    このように package.xml に依存関係を追記したい場合は以下のように書きます．

    .. code:: xml

        <depend>パッケージ名</depend>

　これで **パッケージ ros2_workshop はパッケージ action_tutorials_interfaces に依存している** という定義ができました．

　次に依存関係を解決するコマンドを実行する前に，一度以下のコマンドを実行して apt リポジトリを更新しましょう．なぜこれをしなければならないかというと，rosdep は apt をラップしているからです．

.. code:: bash

    sudo apt update

　次に，ワークスペース上の ``/src`` ディレクトリに移動してください．次に以下のコマンドを実行して依存関係を解決します．

.. code:: bash

    # ../src ディレクトリ上で行う
    rosdep install -y -i --from-path .

.. caution::

    上記コマンドを実行したとき，もし以下のようなエラーが発生したら一度コマンド ``sudo apt update`` を実行して apt リポジトリを更新してください．

    .. code::

        ERROR: the following rosdeps failed to install

　依存関係解決が成功すると，以下のようなメッセージが表示されます．

.. code::

    #All required rosdeps installed successfully

この後もう一度以下のコマンドを実行すると Fibonacci インターフェースが利用できるようになっているのが確認できるでしょう．

.. code::

    ros2 interface list | grep Fibonacci

******************************
ノードプログラムの書き方
******************************

　これまでと同様に，`ros2_workshop` パッケージ内の `ros2_workshop` ディレクトリに，`practice_action_server.py` という名前で Python スクリプトを作成します．

必要なモジュールのインポート
==============================

　`rclpy` と `Node` に加えて，Action Serverを扱うための `rclpy.action` から `ActionServer` をインポートします．また，先ほど確認した `Fibonacci` Actionもインポートします．

.. code:: python

    #!/usr/bin/env python3
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionServer

    from action_tutorials_interfaces.action import Fibonacci

クラスを作成する
====================

　`PracticeActionServer` というクラスを作成し，`Node` モジュールを継承させます．

.. code:: python

    class PracticeActionServer(Node):

イニシャライザの作成
======================

　イニシャライザ `__init__` を定義し，ノード名の宣言とAction Serverの作成を行います．

.. code:: python

    class PracticeActionServer(Node):
        def __init__(self):
            super().__init__('practice_action_server')
            self.action_server = ActionServer(
                self,
                Fibonacci,
                'fibonacci',
                self.execute_callback)

`ActionServer` の引数は以下の通りです．

* **第1引数**: ノードのインスタンス (`self`)．
* **第2引数**: Actionの型 (`Fibonacci`)．
* **第3引数**: Action名 (`fibonacci`)．クライアントはこの名前でサーバーを呼び出します．
* **第4引数**: ゴールリクエストを受け取ったときに実行されるコールバック関数 (`self.execute_callback`)．

.. important:: Actionインターフェースとフィボナッチ数列

    　コールバック関数を実装する前に，今回扱う `Fibonacci` Actionインターフェースの構造と，計算するフィボナッチ数列について理解を深めましょう．
    `ros2 interface show` コマンドで `Fibonacci` Actionの構造を確認できます．

    .. code:: bash

        ros2 interface show action_tutorials_interfaces/action/Fibonacci

    .. code::

        # Goal
        int32 order
        ---
        # Result
        int32[] sequence
        ---
        # Feedback
        int32[] partial_sequence

    この構造は `---` で3つのパートに分かれています．

    - **Goal**: クライアントからサーバーへのリクエストです．
        - `int32 order`: 「フィボナッチ数列を何番目まで計算してほしいか」を示す整数値．
    - **Result**: タスク完了後にサーバーからクライアントへ返される最終結果です．
        - `int32[] sequence`: 計算されたフィボナッチ数列全体を格納する整数の配列．
    - **Feedback**: タスク実行中にサーバーからクライアントへ送られる途中経過です．
        - `int32[] partial_sequence`: その時点までに計算されたフィボナッチ数列の部分的な配列．

.. hint:: フィボナッチ数列とは？

    　**フィボナッチ数列** とは，「前の2つの項を足し合わせると次の項になる」という規則で生成される数列です．最初の2項は 0 と 1 です．

    - 0, 1, 1, 2, 3, 5, 8, 13, 21, ...

    例えば，`order` が `5` の場合，5番目の項（3）までではなく，数列の長さが `order` に達するまで計算を進めます．今回の実装では，初期値 `[0, 1]` から始めて `order` 回の計算を行い，最終的に `[0, 1, 1, 2, 3, 5]` という数列を生成します．

実行コールバック関数を作成する
================================

　Actionの本体となる `execute_callback` メソッドを作成します．この関数はクライアントからゴールリクエストを受け取るたびに呼び出されます．

.. important::

    Actionの処理は完了までに時間がかかる可能性があるため，このコールバック関数は **非同期関数 (async function)** として定義する必要があります．これにより，重い処理の最中でも他の処理をブロックすることなく，並行してタスクを実行できます．

まず，以下のように非同期関数としてメソッドを定義しましょう．

.. code:: python

    class PracticeActionServer(Node):
        def __init__(self):
            ...

        async def execute_callback(self, goal_handle):

引数 `goal_handle` は，クライアントから送られてきたゴールに関する全ての情報と，サーバー側でそのゴールを操作するためのメソッド（成功，失敗，キャンセルなど）を持っています．

フィードバックの準備
------------------------

　次に，フィボナッチ数列の計算途中の状態をクライアントに報告（フィードバック）するためのメッセージオブジェクトを準備します．

.. code:: python

        async def execute_callback(self, goal_handle):
            self.get_logger().info('Executing goal...')

            # フィードバックメッセージのインスタンスを作成
            feedback_msg = Fibonacci.Feedback()
            # 数列の初期値を設定
            feedback_msg.partial_sequence = [0, 1]

`Fibonacci.Feedback()` でフィードバック用のオブジェクトを作成し，その `partial_sequence` フィールドにフィボナッチ数列の初期値である `[0, 1]` を設定しています．

ゴールの実行とフィードバックの送信
------------------------------------

　次に，クライアントから要求された計算（`goal_handle.request.order`）を `for` ループで実行します．ループの各回で，計算結果をフィードバックとしてクライアントに送信します．

.. code:: python

        async def execute_callback(self, goal_handle):
            ...
            feedback_msg.partial_sequence = [0, 1]

            # ゴールで指定された次数までフィボナッチ数列を計算
            for i in range(1, goal_handle.request.order):
                # フィボナッチ数列を計算して，フィードバックメッセージに追加
                feedback_msg.partial_sequence.append(
                    feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
                
                # フィードバックを送信
                self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
                goal_handle.publish_feedback(feedback_msg)
                
                # 1秒待機して，時間のかかる処理を模擬
                time.sleep(1)

- `feedback_msg.partial_sequence.append(...)`: フィボナッチ数列の次の項を計算し，リストに追加します．
- `goal_handle.publish_feedback(feedback_msg)`: 現在の計算状況（`feedback_msg`）をクライアントに送信します．
- `time.sleep(1)`: 1秒間処理を停止し，時間のかかるタスクを模擬しています．

キャンセル処理の実装
------------------------

　Actionの重要な機能の一つに，クライアントからの**キャンセル要求**への対応があります．処理の途中でクライアントがタスクの中断を要求した場合，サーバーは速やかに処理を停止する必要があります．

`for` ループの先頭で，キャンセル要求が来ていないかを確認し，もし来ていればゴールを「キャンセル済み」の状態にして処理を終了させます．

.. code:: python

        async def execute_callback(self, goal_handle):
            ...
            feedback_msg.partial_sequence = [0, 1]

            for i in range(1, goal_handle.request.order):
                # キャンセル要求があったかチェック
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    return Fibonacci.Result()
                
                # (以降の処理は省略)
                ...

- `goal_handle.is_cancel_requested`: キャンセル要求があれば `True` を返します．
- `goal_handle.canceled()`: ゴールの状態を「キャンセル済み」に設定します．
- `return Fibonacci.Result()`: 空の結果を返してコールバック関数を終了します．

最終結果の返却
--------------------

　`for` ループが無事に完了したら，それはタスクが成功したことを意味します．`goal_handle.succeed()` を呼び出してゴールが成功したことをクライアントに通知し，最終的な結果を返却します．

.. code:: python

        async def execute_callback(self, goal_handle):
            ...
            for i in range(1, goal_handle.request.order):
                ...
            
            # (ループが完了したら)
            # ゴールが成功したことをクライアントに通知
            goal_handle.succeed()

            # 結果メッセージを作成して返す
            result = Fibonacci.Result()
            result.sequence = feedback_msg.partial_sequence
            self.get_logger().info(f'Returning result: {result.sequence}')
            return result

- `goal_handle.succeed()`: ゴールの状態を「成功」に設定します．
- `result = Fibonacci.Result()`: 最終結果を格納するためのオブジェクトを作成します．
- `result.sequence = ...`: 計算結果の完全な数列を `sequence` フィールドに代入します．
- `return result`: 最終結果をクライアントに返します．

実行関数 `main` を作成する
=================================

　これまでと同様に，作成したクラスを実行するための `main` 関数と実行ブロックを作成します．

.. code:: python

    def main():
        rclpy.init()
        node = PracticeActionServer()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

    if __name__ == '__main__':
        main()

****************************************
パッケージにノードを登録する
****************************************

　作成した `practice_action_server.py` を `ros2 run` で実行できるように，`setup.py` の `entry_points` に追記します．

.. code:: python

    entry_points={
        'console_scripts': [
            'practice_publisher_node = ros2_workshop.practice_publisher:main',
            'practice_subscriber_node = ros2_workshop.practice_subscriber:main',
            'practice_service_server_node = ros2_workshop.practice_service_server:main',
            'practice_service_client_node = ros2_workshop.practice_service_client:main',
            'practice_action_server_node = ros2_workshop.practice_action_server:main',
        ],
    },

***************************
パッケージをビルドする
***************************

　`package.xml` と `setup.py` を変更したので，再度パッケージをビルドします．

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

次に，**2つ目のターミナル** で `ros2 action send_goal` コマンドを使い，サーバーにゴールを送信します．ここでは，5番目までのフィボナッチ数列を計算させてみましょう．`--feedback` オプションをつけると，途中経過が表示されます．

.. code:: bash

    ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}" --feedback

クライアント側のターミナルに，サーバーからのフィードバックと最終結果が順に表示されます．

.. code::

    Waiting for an action server to become available...
    Sending goal:
         order: 5
    Goal accepted with ID: f1d41804240842aab83935a81e93b13d

    Feedback:
    {partial_sequence: [0, 1, 1]}

    Feedback:
    {partial_sequence: [0, 1, 1, 2]}

    Feedback:
    {partial_sequence: [0, 1, 1, 2, 3]}

    Feedback:
    {partial_sequence: [0, 1, 1, 2, 3, 5]}

    Result:
    {sequence: [0, 1, 1, 2, 3, 5]}

    Goal finished with status: SUCCEEDED

同時に，サーバー側のターミナルには処理中のログが表示されます．

.. code::

    [INFO] [practice_action_server]: Executing goal...
    [INFO] [practice_action_server]: Feedback: [0, 1, 1]
    [INFO] [practice_action_server]: Feedback: [0, 1, 1, 2]
    [INFO] [practice_action_server]: Feedback: [0, 1, 1, 2, 3]
    [INFO] [practice_action_server]: Feedback: [0, 1, 1, 2, 3, 5]
    [INFO] [practice_action_server]: Returning result: [0, 1, 1, 2, 3, 5]

これで，Action通信による非同期なタスク処理が確認できました．

***********************************
アクションサーバーノードの全体図
***********************************

　このセクションで扱ったソースコードの完成時の全体図です．

.. code:: python

    #!/usr/bin/env python3
    # Shebang (シバン): このスクリプトをpython3で実行することをシステムに指示するおまじない．

    import rclpy
    import time
    from rclpy.node import Node
    from rclpy.action import ActionServer

    from action_tutorials_interfaces.action import Fibonacci

    # Nodeクラスを継承して，オリジナルのAction Serverノードクラスを定義
    class PracticeActionServer(Node):

        # クラスのインスタンスが作成されるときに自動的に呼び出される初期化メソッド (コンストラクタ)
        def __init__(self):
            # 親クラス (Node) のコンストラクタを呼び出し，ノード名を 'practice_action_server' として登録
            super().__init__('practice_action_server')

            # Action Serverを作成する
            # ActionServer() メソッドは4つの引数を取る
            self.action_server = ActionServer(
                self,                               # 第1引数: ノードのインスタンス
                Fibonacci,                          # 第2引数: Actionの型
                'fibonacci',                        # 第3引数: Action名
                self.execute_callback)              # 第4引数: ゴール受信時に実行されるコールバック関数

        # クライアントからゴールリクエストを受信した際に呼び出されるコールバック関数
        # 引数 'goal_handle' にはゴールに関する情報や操作メソッドが含まれる
        async def execute_callback(self, goal_handle):
            self.get_logger().info('Executing goal...')

            # フィードバックメッセージのオブジェクトをあらかじめ生成
            feedback_msg = Fibonacci.Feedback()
            # 数列の初期値を設定
            feedback_msg.partial_sequence = [0, 1]

            # ゴールで指定された次数 (goal_handle.request.order) までループ
            for i in range(1, goal_handle.request.order):
                # クライアントからキャンセルのリクエストがあったか確認
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled() # ゴールの状態を「キャンセル済み」に設定
                    self.get_logger().info('Goal canceled')
                    return Fibonacci.Result() # 結果を返して処理を終了

                # フィボナッチ数列を計算して，フィードバックメッセージに追加
                feedback_msg.partial_sequence.append(
                    feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
                
                # 現在の計算状況をログに出力
                self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
                # `publish_feedback()` でクライアントに途中経過を送信
                goal_handle.publish_feedback(feedback_msg)
                
                # 1秒間スリープして，時間のかかる処理を模擬
                time.sleep(1)

            # ループ完了後，ゴールの状態を「成功」に設定
            goal_handle.succeed()

            # 最終結果を格納するResultオブジェクトを作成
            result = Fibonacci.Result()
            result.sequence = feedback_msg.partial_sequence
            
            # 最終結果をログに出力
            self.get_logger().info(f'Returning result: {result.sequence}')
            
            # 最終結果をクライアントに返す
            return result


    # プログラムのメイン処理を定義する関数
    def main():
        # ROS2のクライアントライブラリを初期化．
        rclpy.init()

        # PracticeActionServerクラスのインスタンスを作成し，ノードとして実体化
        node = PracticeActionServer()

        # try-except-finallyブロック: Ctrl+Cで終了した際の後処理を確実に行う
        try:
            # rclpy.spin() はノードを実行状態に保ち，コールバック関数を処理し続ける
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
