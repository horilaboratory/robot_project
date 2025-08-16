########################################
Service サーバーノードを作ってみよう
########################################

　これまでは Publisher/Subscriber による一方通行の通信を学びました．ここでは，双方向の通信を実現する Service（サービス）のサーバーノードを作成してみましょう．

******************
Service とは
******************

　**Service（サービス）** とは，ROS2 におけるリクエスト／レスポンス型の通信モデルです．

- **Service Client（クライアント）**: 処理を依頼する（リクエストを送る）ノード．
- **Service Server（サーバー）**: 依頼を受けて処理を実行し，結果を返却する（レスポンスを返す）ノード．

以下の図のように，クライアントがリクエストを送ると，サーバーはそれに対応する処理を行い，必ず1つのレスポンスをクライアントに返します．これは，関数の呼び出しに似ており，「何かを依頼して，その結果を受け取る」という同期的なやり取りに適しています．

.. image:: /_static/about_service.gif

*******************************************
Service インターフェースをインストールする
*******************************************

　このチュートリアルでは ``example_interfaces`` パッケージに含まれる ``AddTwoInts`` という Service メッセージインターフェースを使用します．まず，このインターフェースが利用可能かどうかを確認しましょう．

　以下のコマンドは，利用可能なインターフェースの中から ``AddTwoInts`` を含むものを検索します．

.. code:: bash

    ros2 interface list | grep AddTwoInts

　もし ``example_interfaces/srv/AddTwoInts`` が表示されれば，パッケージはすでにインストールされています．何も表示されない場合は，これから説明する手順で依存関係を解決する必要があります．

　ここでは ROS2 パッケージ管理でよく使う **rosdep（ロスデプ）** を使い，必要なパッケージをインストールして依存関係を解決する方法を解説します．

***********************************
インストールしたいパッケージを探す
***********************************

　``example_interfaces`` パッケージをインストールするために，rosdep という依存関係解決ツールを使ってみましょう．これを使うことで，例えば他の ROS2 環境で開発したパッケージを使用するときに，すぐに依存関係を解決することができます．

　初めて rosdep を使う場合は，以下のコマンドを実行して rosdep を初期化してください．しかし，もし Docker コンテナを利用している場合はこの作業はすでに行われているためエラーが発生しますが，正常な反応なので気にする必要はありません．

.. code:: bash

    sudo rosdep init

　次に以下のコマンドを実行して rosdep を最新の状態にアップデートしてください．

.. code:: bash

    rosdep update

　次に，以下のコマンドを実行してパッケージ ``example_interfaces`` がインストールできるかどうか確認します．応答があればそのパッケージは Ubuntu のパッケージ管理ツール ``apt`` からインストール可能であることを示しています．

.. code:: bash

    rosdep db | grep example_interfaces

.. hint::

    上記のコマンドのように ``rosdep db | grep <探したいパッケージ名>`` を実行することで rosdep からインストール可能なパッケージを検索することができます．

　上記のコマンドを実行すると，以下のような応答があるでしょう．

.. code::

    example_interfaces -> ros-humble-example-interfaces

これの読み方は以下のとおりです．右に書かれているパッケージ名で ``apt install`` することで簡単に該当のパッケージをインストールすることができます．

.. code::

    パッケージ名 -> apt でインストールできるパッケージ名

このまま ``sudo apt install ros-humble-example-interfaces`` を実行してもいいのですが，ここでは続けて ``package.xml`` と ``rosdep`` を使い依存関係となるパッケージをインストールする方法を解説します．

********************
依存関係を解決する
********************

　依存関係というのはどういうことかというと，ようはパッケージ A を動かすためにはパッケージ B が必要な場合，**パッケージ A はパッケージ B と依存関係にある** ことを示しています．
ROS2 パッケージではそんな依存関係を示す場所として ``package.xml`` が用意されています．作成した ``ros2_workshop`` パッケージの package.xml を開いてみましょう．

　``<package>`` タグ内に以下のように依存関係となるパッケージ ``example_interfaces`` を追記しましょう．

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

    + <depend>example_interfaces</depend>

      <export>
        <build_type>ament_python</build_type>
      </export>
    </package>

.. hint::

    このように package.xml に依存関係を追記したい場合は以下のように書きます．

    .. code:: xml

        <depend>パッケージ名</depend>

　これで **パッケージ ros2_workshop はパッケージ example_interfaces に依存している** という定義ができました．

　次に依存関係を解決するコマンドを実行する前に，一度以下のコマンドを実行して apt リポジトリを更新しましょう．なぜこれをしなければならないかというと，rosdep は apt をラップしているからです．

.. code:: bash

    sudo apt update

　次に，ワークスペース上の ``/src`` ディレクトリに移動してください．次に以下のコマンドを実行して依存関係を解決します．

.. code:: bash

    # /ws/src ディレクトリ上で行う
    rosdep install -y -i --from-path .

.. caution::

    上記コマンドを実行したとき，もし以下のようなエラーが発生したら一度コマンド ``sudo apt update`` を実行して apt リポジトリを更新してください．

    .. code::

        ERROR: the following rosdeps failed to install

　依存関係解決が成功すると，以下のようなメッセージが表示されます．

.. code::

    #All required rosdeps installed successfully

この後もう一度以下のコマンドを実行すると ``AddTwoInts`` インターフェースが利用できるようになっているのが確認できるでしょう．

.. code:: bash

    ros2 interface list | grep AddTwoInts


******************************
ノードプログラムの書き方
******************************

　これまでと同様に，ament_python パッケージの **パッケージ名と同じディレクトリ** 内にノードとなる Python スクリプトを作成します．ここでは ``practice_service_server.py`` というノードを作成してみましょう．

必要なモジュールのインポート
==============================

　``rclpy`` と ``Node`` モジュールをインポートします．今回は，整数の足し算を行うサービスを作成するため，ROS2 に標準で用意されている ``example_interfaces`` パッケージの ``AddTwoInts`` というサービス定義を利用します．これをインポートしましょう．

.. code:: python

    #!/usr/bin/env python3
    from rclpy.node import Node
    import rclpy

    from example_interfaces.srv import AddTwoInts

.. hint::

    どのようなサービスが利用可能か，またその構造はどうなっているかは ``ros2 interface`` コマンドで確認できます．
    
    ターミナルで以下のコマンドを実行して ``AddTwoInts`` サービスの構造を見てみましょう．

    .. code:: bash

        ros2 interface show example_interfaces/srv/AddTwoInts
    
    するとこのようなテキストが返ってきます．

    .. code::

        int64 a
        int64 b
        ---
        int64 sum
    
    ``---`` で区切られた上が **リクエスト** のデータ構造，下が **レスポンス** のデータ構造です．
    これは「クライアントは ``a`` と ``b`` という2つの64ビット整数をリクエストとして送り，サーバーは ``sum`` という1つの64ビット整数をレスポンスとして返す」ということを意味しています．

クラスを作成する
====================

　スクリプトファイル名にちなんだ ``PracticeServiceServer`` というクラスを作成し，``Node`` モジュールを継承させます．

.. code:: python

    class PracticeServiceServer(Node):

イニシャライザの作成
======================

　クラスが呼び出されたときに自動的に実行されるイニシャライザ ``__init__`` を定義します．

.. code:: python

    class PracticeServiceServer(Node):
        def __init__(self):

ノードの宣言を書く
===================

　イニシャライザ内で ``super().__init__()`` を使ってノード名 ``practice_service_server`` を宣言します．

.. code:: python

    class PracticeServiceServer(Node):
        def __init__(self):
            super().__init__('practice_service_server')

Service Server を作成する
=========================
　次に Service Server を作りましょう．イニシャライザ内で以下のように ``self.create_service()`` を使ってサーバーを作成します．

.. code:: python

    class PracticeServiceServer(Node):
        def __init__(self):
            super().__init__('practice_service_server')

            self.srv = self.create_service(
                AddTwoInts,
                'add_two_ints',
                self.add_two_ints_callback)

　``self.create_service`` 関数の引数はこのようになっています．

.. code:: python

    rclpy.node.Node.create_service(
        srv_type,
        srv_name,
        callback
    )

- ``srv_type``
    利用するサービスの型を指定します．ここでは ``AddTwoInts`` を指定しています．

- ``srv_name``
    サービスの名称を文字列で指定します．クライアントはこの名前を使ってサーバーを呼び出します．ここでは ``add_two_ints`` としています．

- ``callback``
    クライアントからリクエストを受信した際に呼び出される **コールバック関数** を指定します．ここでは ``add_two_ints_callback`` というメソッドを指定しています．

コールバック関数を作成する
============================

　クライアントからリクエストがあった際に実行される ``add_two_ints_callback`` メソッドをクラスに作成してください．

.. important::

    サービスのコールバック関数は，引数として ``request`` と ``response`` の2つを受け取ります．
    - ``request``: クライアントから送られてきたリクエストデータが格納されています．
    - ``response``: サーバーがクライアントに返却するレスポンスデータを格納するために使います．

.. code:: python

    class PracticeServiceServer(Node):
        def __init__(self):
            ...
        
        def add_two_ints_callback(self, request, response):

リクエストを処理し，レスポンスを返す
======================================

　コールバック関数内で，リクエストを処理し，レスポンスを生成するコードを記述します．

　``AddTwoInts`` サービスでは，リクエストは ``request.a`` と ``request.b`` というフィールドを持っています．この2つの値を足し算し，その結果をレスポンスの ``sum`` フィールドに代入します．

　最後に，データを格納した ``response`` オブジェクトを ``return`` で返す必要があります．

.. code:: python
        
        def add_two_ints_callback(self, request, response):
            # リクエストから2つの整数を取得
            response.sum = request.a + request.b
            
            # 処理内容をログに出力
            self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}\n'
                                   f'Sending back response: [{response.sum}]')

            # レスポンスを返す
            return response

実行関数 ``main`` を作成する
=================================

　これまでと同様に，作成したクラスを実行するための ``main`` 関数と実行ブロックを作成します．処理の流れは全く同じです．

.. code:: python

    class PracticeServiceServer(Node):
        ...
    

    def main():
        rclpy.init()
        node = PracticeServiceServer()
        rclpy.spin(node)
    
    if __name__ == '__main__':
        main()

これでサービスサーバーのスクリプトは完成です！

****************************
ノードを登録する
****************************

　作成した ``practice_service_server.py`` を ``ros2 run`` コマンドで実行できるように，``setup.py`` に登録します．

　``entry_points`` の ``'console_scripts'`` リストに，新しいノードを追記します．

.. code:: python

    entry_points={
        'console_scripts': [
            'practice_publisher_node = ros2_workshop.practice_publisher:main',
            'practice_subscriber_node = ros2_workshop.practice_subscriber:main',
            'practice_service_server_node = ros2_workshop.practice_service_server:main'
        ],
    },

***************************
パッケージをビルドする
***************************

　``package.xml`` と ``setup.py`` を変更したので，再度パッケージをビルドします．

.. code:: bash

    cd /ws
    colcon build --symlink-install --packages-select ros2_workshop

**************************
ノードを実行する
**************************

　ビルドが完了したら，まずワークスペースを読み込みます．

.. code:: bash

    source /ws/install/setup.bash

次に，**2つのターミナル** を使ってサーバーを起動し，クライアントから呼び出します．

まず，**1つ目のターミナル** で Service Server ノードを起動します．

.. code:: bash

    ros2 run ros2_workshop practice_service_server_node

次に，**2つ目のターミナル** を開いて，``ros2 service call`` コマンドを使ってサービスを呼び出します．

.. code:: bash

    ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"

このコマンドは，「``/add_two_ints`` という名前のサービスを，``example_interfaces/srv/AddTwoInts`` 型で，リクエストデータ ``{a: 2, b: 3}`` を使って呼び出す」という意味です．

コマンドを実行すると，クライアント側のターミナルにサーバーからのレスポンスが表示されます．

.. code::

    requester: making request: example_interfaces.srv.AddTwoInts_Request(a=2, b=3)
    
    response:
    example_interfaces.srv.AddTwoInts_Response(sum=5)

同時に，サーバー側のターミナルには，リクエストを受け付けて処理した際のログが表示されます．

.. code::

    [INFO] [practice_service_server]: Incoming request
    a: 2 b: 3
    Sending back response: [5]

これで，Service による双方向通信が確認できました．
サーバーを実行しているターミナルで「Control + C」を押してノードを停止してください．

正常にノードを終了するようにする
==================================

　これまでのノードと同様に，``main`` 関数に ``try-except-finally`` 構文を追加して，ノードが正常に破棄されるようにしましょう．

.. code:: python

    def main():
        rclpy.init()
        node = PracticeServiceServer()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()

*********************************
サービスサーバーノードの全体図
*********************************

　いかがこのセクションで扱ったソースコードの完成時の全体図です．

.. code:: python

    #!/usr/bin/env python3
    # Shebang (シバン): このスクリプトをpython3で実行することをシステムに指示するおまじない。

    # rclpy (ROS Client Library for Python) ライブラリから必要なモジュールをインポート
    from rclpy.node import Node  # Nodeクラス: ROS2のノードを作成するための基本的なクラス
    import rclpy               # rclpy: ROS2のPythonクライアントライブラリの本体

    # example_interfaces パッケージから AddTwoInts というサービス定義をインポート
    # これにより、2つの整数を足し算するサービスが利用できるようになる
    from example_interfaces.srv import AddTwoInts


    # Nodeクラスを継承して、オリジナルのService Serverノードクラスを定義
    class PracticeServiceServer(Node):
        # クラスのインスタンスが作成されるときに自動的に呼び出される初期化メソッド (コンストラクタ)
        def __init__(self):
            # 親クラス (Node) のコンストラクタを呼び出し、ノード名を 'practice_service_server' として登録
            super().__init__('practice_service_server')

            # Service Serverを作成する
            # self.create_service() メソッドは3つの引数を取る
            self.srv = self.create_service(
                AddTwoInts,                 # 第1引数: サービスの型 (AddTwoInts型)
                'add_two_ints',             # 第2引数: サービス名 (この名前でクライアントから呼び出される)
                self.add_two_ints_callback  # 第3引数: リクエスト受信時に呼び出されるコールバック関数
            )

        # クライアントからリクエストを受信した際に呼び出されるコールバック関数
        # 引数 'request' にリクエストデータが、'response' にレスポンスオブジェクトが格納される
        def add_two_ints_callback(self, request, response):
            
            # リクエストの 'a' フィールドと 'b' フィールドの値を足し算し、
            # レスポンスの 'sum' フィールドに結果を代入する
            response.sum = request.a + request.b
            
            # サーバー側で処理内容をログとして出力する
            self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}\n'
                                   f'Sending back response: [{response.sum}]')

            # 処理結果が格納されたレスポンスオブジェクトをクライアントに返す
            return response


    # プログラムのメイン処理を定義する関数
    def main():
        # ROS2のクライアントライブラリを初期化。
        rclpy.init()
        # PracticeServiceServerクラスのインスタンスを作成し、ノードとして実体化させる
        node = PracticeServiceServer()

        # try-except-finallyブロック: Ctrl+Cでプログラムを終了した際などに、後処理を確実に行うための構文
        try:
            # rclpy.spin() はノードを実行状態に保ち、コールバック関数を処理し続ける。
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
