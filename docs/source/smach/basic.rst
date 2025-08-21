#############################
状態遷移を実装する（基礎編）
#############################

　``smach_ros`` パッケージをインストールしたら，早速ノードとして動作する状態遷移プログラムを実装してみましょう．

　ros2_workshop パッケージの ``ros2_workshop`` ディレクトリ内に Python プログラム ``practice_basic_state.py`` を作成しましょう．

==================================
必要なライブラリをインポートする
==================================

　必要なライブラリをコード上にインポートしましょう．まず ROS2 ノードを実装するので以下の２つのライブラリをインポートしましょう．

.. code:: python

    #!/usr/bin/env python3
    from rclpy.node import Node
    import rclpy

　次に以下のように状態遷移用ライブラリ ``smach`` をインポートします．このライブラリを利用して状態遷移を実装します．

.. code:: python

    ...
    import smach

================================
``rclpy`` とノードを初期化する
================================

　次に ``main`` 関数を作成しましょう．この中に状態遷移を実装していきます．

.. code:: python

    ...
    import smach

    def main():

　``main`` 関数のはじめに ``rclpy`` を初期化します．

.. code:: python

    ...
    def main():
        rclpy.init()

　次にノードを初期化します．今までは **ノードオブジェクトを内包したクラス** を事前に作成し，これを初期化することでノードを作成，宣言，実行していましたが，
ここでは **先にノードを作成してクラスにノードを渡す** 方法をとります．以下のようにノードを直接呼び出して変数 ``node`` を使い初期化します．

　初期化方法は ``super().__init__('node_name')`` と同じで，引数に作成したいノード名となる文字列を代入します．

.. code:: python

    def main():
        rclpy.init()
        node = Node('practice_basic_smach_node')

====================
smach を初期化する
====================

　次に状態遷移を実装するための smach を初期化します．初期化する方法は以下のとおりです．

.. code:: python

    sm = smach.StateMachine(outcomes=['final_state_name'])

　ここではクラス ``smach.StateMachine()`` を変数 ``sm`` に初期化（インスタンス化）させています．そしてクラスの引数 ``outcomes`` に配列 ``['final_state_name']`` を指定しています．

　これは **これから作成するステートは ``final_state_name`` という状態で終了する** という定義を行なっています．
作成した状態遷移の中でもし，``final_state_name`` に遷移するよう指示があった場合プログラムは終了するということです．

また，引数の型の通り，終了時の状態を複数定義することが可能です．例えば，以下のように定義したならば実装するステートマシーンは ``a`` と ``b`` のいづれかで終了するように定義します．

.. code:: python

    sm = smach.StateMachine(outcomes=['a', 'b'])
    
なのでこの時点で **これから実装する StateMachine がどの状態で終了するのかを定義しておきましょう．**

ここでは ``stop`` という状態で終了するように定義します．
これから作成するステートマシーンはユーザーからの停止（KeyboardInterrupt）が呼び出されるまでループするように実装します．

.. code:: python

    sm = smach.StateMachine(outcomes=['stop'])

==========================
State Machine を実装する
==========================

　次にステートマシーン本体を実装します．まずは初期化したコードの下に以下のようなコードを用意してください．

.. code:: python

    with sm:

    outcome = sm.execute()

　これから ``with sm:`` 内にステートマシーンを定義していきます．そのしたの ``outcome = sm.execute()`` で定義したステートマシーンを実行します．
変数 ``outcome`` にはステートマシーンが終了した時の状態名が文字列で返されます．ここでは終了時のステート名は ``stop`` のみなので，ステート終了時変数 ``outcome`` には文字列 ``'stop'`` が代入されます．

　ステートマシーンを組んでいきましょう．ステートマシーンを追加するには以下のコードを書きます．

.. code:: python

    smach.StateMachine.add('TASK_A', StateClassA(),
                            transitions={
                                'foo' : 'TASK_B',
                                'bar' : 'stop'
                            }
    )

　少々複雑ですが１つ１つみていきましょう．``smach.StateMachine.add()`` はステートマシーンを定義します．
このクラスの引数は最低でも以下の３つを要求します．

.. code:: python

    smach.StateMachine.add(
        label,
        state,
        transitions,
        remapping # option
    )

- ``label`` : 定義するステート名を文字列で定義します．**この時習慣としてステート名は大文字で書くべきです．**
- ``state`` : 実行するステートクラス，またはコールバック関数を指定します．
- ``transitions`` : 定義したステートが遷移するステート情報を辞書形式で記述します．

.. note::

    引数 ``remapping`` についてはここでは解説を省かせてください．

　ここでもう一度先ほど出した例を見てみましょう．

.. code:: python

    smach.StateMachine.add('TASK_A', StateClassA(),
                            transitions={
                                'foo' : 'TASK_B',
                                'bar' : 'stop'
                            }
    )

 これを例に見ると，ここでは ``TASK_A`` という名前のステートを定義しており，このステートではクラス ``StateClassA()`` が呼び出されます．
そして，このステートでは ``foo`` と ``bar`` という状態を返します．これらのステートが返されたらそれぞれ別のステート ``TASK_B`` と ``stop`` に遷移すると定義されています．

.. tip::

    引数 ``transitions`` は以下のように書きます．

    .. code:: python

        transitions={'今書いているスタートが返す状態の名前' : 'その状態が返された時に遷移するステート名'}

　もう一つステートを用意してみましょう．ここでは新たに ``TASK_B`` というステートを用意しました．

.. code:: python

    smach.StateMachine.add('TASK_A', StateClassA(),
                            transitions={
                                'foo' : 'TASK_B',
                                'bar' : 'stop'
                            }
    )

    smach.StateMachine.add('TASK_B', StateClassB(),
                            transitions={
                                'foo' : 'TASK_A',
                                'bar' : 'stop'
                            }
    )

これを見るとそれぞれ ``foo`` という状態になったらそれぞれのステートに遷移し，``bar`` になったら ``stop`` となることを示しています．ここで StateMachine を初期化したときに ``outcomes`` 引数に定義したものを確認してみましょう．
このように ``stop`` で状態遷移が終了するように定義されていますから，それぞれのスタートが ``bar`` を返した時，この状態遷移は終了します．

.. code:: python

        sm = smach.StateMachine(outcomes=['stop'])

        with sm:
            smach.StateMachine.add('TASK_A', StateClassA(),
                                    transitions={
                                        'foo' : 'TASK_B',
                                        'bar' : 'stop'
                                    }
            )

            smach.StateMachine.add('TASK_B', StateClassB(),
                                    transitions={
                                        'foo' : 'TASK_A',
                                        'bar' : 'stop'
                                    }
            )

==========================
ステートクラスを作成する
==========================

　ステートを定義した時，それぞれのステートにクラス ``StateClassA`` と ``StateClassB``  を用意しました．次のこの **ステートが呼び出された時に実行するクラス** を作成し，ステートの処理そのものを実装していきましょう．

ステートを編集する
====================

　先ほど作成したステート内に定義したクラスに引数 node に main 関数内に作成したインスタンス変数 node を渡すように書いてください．

.. code:: python

    smach.StateMachine.add('TASK_A', StateClassA(node=node),  # <-- node 引数を追記
                            transitions={
                                'foo' : 'TASK_B',
                                'bar' : 'stop'
                            }
    )

    smach.StateMachine.add('TASK_B', StateClassB(node=node),  # <-- node 引数を追記
                            transitions={
                                'foo' : 'TASK_A',
                                'bar' : 'stop'
                            }
    )

ステートクラスを作成する
=========================

　main 関数の上にステート内に定義したクラス ``StateClassA`` を用意しましょう．この時ステートマシーンとして動作するクラスには ``smach.State`` クラスを継承するように書きましょう．
これをしないと **smach がクラスをステートマシーンとして扱ってくれません．**

.. code:: python

    ...
    
    class StateClassA(smach.State): # <- smach.State を継承するように定義する．

    def main():
        ...

　次にステートクラスのイニシャライザを作成します．この時，引数 ``node`` を定義し，ステートマシーンを定義した時に渡される Node オブジェクトを受け取れるようにしましょう．

.. code:: python

    class StateClassA(smach.State):
        def __init__(self, node):

.. tip::

    以下のように定義した引数に型オブジェクトを ``:`` で結んで定義すると，**引数 node には Node オブジェクトの値しか代入できない** と型ロックすることができ，厳格で安全なコードを実装できます．

    .. code:: python

        class StateClassA(smach.State):
            def __init__(self, node:Node):

　次にイニシャライザ内の処理を書いていきます．イニシャライザでは **ステートマシーンとして動作するクラスがどういう状態を返すかを定義します．**

　ステートの設定はこのように書きます．

.. code:: python

    smach.State.__init__(
        self,
        outcomes=['foo', 'bar']
    )

　ここではステートマシーンとして動作する StateClassA が返す状態を引数 ``outcomes`` に定義しており，これは main 関数で定義した時の ``transitions`` 引数の状態に合わせなければなりません．


.. grid:: 2

    .. grid-item-card::  StateClassA 内

        .. code:: python
        
            class StateClassA(smach.State):
                def __init__(self, node):
                    smach.State.__init__(
                        self,
                        outcomes=['foo', 'bar']
                    )

    .. grid-item-card::  StateMachine.add 内

        .. code:: python
        
            smach.StateMachine.add( ...
                transitions={
                    'foo' : 'TASK_B',
                    'bar' : 'stop'
                }

　これでステートクラスのイニシャライザで重要な要素はかけました．ステートマシーンを組む時，よく発生するエラー ``smach.exceptions.InvalidTransitionError:`` のほとんどの原因はステートマシーンとステートクラス側の状態定義の相違です．よく確認しておきましょう．

　続けてイニシャライザで引数 ``node`` をコンストラクタに追加しましょう．後ほどこの ``node`` オブジェクトを別メソッドにて使います．

.. code:: python

    def __init__(self, node):
        ...

        self.node = node

　次にクラス内に ``execute`` メソッドを作成しましょう．必ずこの名前で定義しなければなりません．
smach はステートマシーンとして登録されたクラスの ``execute`` メソッドをステートのメインプロセスとして扱います．
また，以下のようにこのメソッドには引数 ``userdata`` を用意しましょう．この引数は smach が利用します．

.. code:: python

    StateClassA(smach.State):
        def __init__(self, node):
            ...

        def execute(self, userdata):

　execute メソッドは必ず **状態** を返さなければなりません．状態はイニシャライザで定義した ``outcomes`` 内の文字列らです．つまり **execute メソッドは文字列 "foo" または "bar" を返すように記述する必要があるのです．**
ここでは ``foo`` を返すように定義してみましょう．

.. code:: python

    StateClassA(smach.State):
        def __init__(self, node):
            ...

        def execute(self, userdata):

            return 'foo'

　これ必要最低限のステートクラスの構築は完了ですが，ダミーの処理を実装するために ``time`` モジュールをインポートし，``logger`` で適当に標準出力する機構を用意しましょう．

.. code:: python

    #!/usr/bin/env python3
    from rclpy.node import Node
    import rclpy

    import smach

    import time # 追記


    class StateClassA(smach.State):
        def __init__(self, node):
            ...

        def execute(self, userdata):
            self.node.get_logger().info('running TASK_A')

            time.sleep(1)

            return 'foo'

　次に ``TASK_B`` で定義されているクラス ``StateClassB`` を用意しましょう．以下のように ``StateClassA`` をコピペして一部書き換えて用意します．


.. code:: python

    ...

    class StateClassB(smach.State):
        def __init__(self, node):
            smach.State.__init__(
                self,
                outcomes=['foo', 'bar']
            )

            self.node = node

        def execute(self, userdata):
            self.node.get_logger().info('running TASK_B') # 出力内容を変える

            time.sleep(1)

            return 'foo'

    ...

==============
コード完成図
==============

　これでステートマシーンとして動作するコードは完成です．

.. code:: python

    #!/usr/bin/env python3
    # 必要なライブラリをインポートします
    from rclpy.node import Node
    import rclpy
    import smach
    import time # ダミー処理用のtimeモジュール

    #
    # ステートA（TASK_A）に対応するステートクラス
    # smach.Stateクラスを継承することで、smachがこのクラスをステートとして認識できるようになります
    #
    class StateClassA(smach.State):
        #
        # クラスの初期化（イニシャライザ）
        #
        def __init__(self, node):
            # smach.Stateのイニシャライザを呼び出します
            # outcomesで、このステートが返しうる全ての状態名をリストで定義します
            # この定義は、StateMachineに追加する際のtransitionsのキーと一致している必要があります
            smach.State.__init__(
                self,
                outcomes=['foo', 'bar']
            )
            # ログ出力などで使用するために、渡されたNodeオブジェクトをクラスの変数に保存しておきます
            self.node = node

        #
        # executeメソッドは、このステートが実行されたときのメイン処理です
        # smachはこの名前のメソッドを自動的に呼び出します
        # 引数userdataはsmachが必要とするため、定義しておく必要があります
        #
        def execute(self, userdata):
            # 渡されたノードを使ってログを出力します
            self.node.get_logger().info('running TASK_A')

            # 1秒間待機するダミーの処理
            time.sleep(1)

            # 必ずoutcomesで定義した状態名（文字列）のいずれかを返す必要があります
            # ここで返された値に応じて、StateMachineは次のステートに遷移します
            return 'foo'

    #
    # ステートB（TASK_B）に対応するステートクラス
    # StateClassAとほぼ同じ構造です
    #
    class StateClassB(smach.State):
        def __init__(self, node):
            # こちらも同様に、返しうる状態（'foo'または'bar'）を定義します
            smach.State.__init__(
                self,
                outcomes=['foo', 'bar']
            )
            self.node = node

        def execute(self, userdata):
            # ログの出力内容をTASK_B用に変更します
            self.node.get_logger().info('running TASK_B')

            time.sleep(1)

            # StateClassAと同様に、次の状態を返します
            return 'foo'

    #
    # メイン関数：プログラム全体のエントリーポイント
    #
    def main():
        # ROS2を初期化します
        rclpy.init()
        # ROS2ノードを作成します
        # ここではクラスにラップせず、直接ノードオブジェクトを作成しています
        node = Node('practice_basic_smach_node')

        # StateMachineのインスタンスを作成します
        # outcomesで、このステートマシン全体が終了する状態名を定義します
        # いずれかのステートが'stop'を返すと、ステートマシンは終了します
        sm = smach.StateMachine(outcomes=['stop'])

        # 'with'構文を使ってステートマシン（sm）の内部にステートを定義していきます
        with sm:
            # StateMachine.add()を使ってステートを追加します
            # 第1引数 'TASK_A': ステートの名前（ラベル）。慣習的に大文字で記述します
            # 第2引数 StateClassA(node=node): このステートが実行するクラスのインスタンス。ノードを渡します
            # 第3引数 transitions: 遷移ルールを辞書形式で定義します
            #    'foo': 'TASK_B' -> StateClassAが'foo'を返したら、'TASK_B'という名前のステートに遷移します
            #    'bar': 'stop'   -> StateClassAが'bar'を返したら、'stop'（終了状態）に遷移します
            smach.StateMachine.add('TASK_A', StateClassA(node=node),
                                    transitions={
                                        'foo' : 'TASK_B',
                                        'bar' : 'stop'
                                    }
            )

            # 同様にもう一つのステート'TASK_B'を追加します
            #    'foo': 'TASK_A' -> StateClassBが'foo'を返したら、'TASK_A'に遷移します（ループ）
            #    'bar': 'stop'   -> StateClassBが'bar'を返したら、終了します
            smach.StateMachine.add('TASK_B', StateClassB(node=node),
                                    transitions={
                                        'foo' : 'TASK_A',
                                        'bar' : 'stop'
                                    }
            )

        # 定義したステートマシンを実行します
        # この処理は、outcomesで定義された終了状態（ここでは'stop'）になるまでブロックされます
        # 終了したときの状態名が変数outcomeに代入されます
        outcome = sm.execute()

    # このスクリプトが直接実行された場合にmain()関数を呼び出す
    if __name__ == '__main__':
        main()

=============
やってみよう
=============

　これで一応動作はしますが，``bar`` 状態に遷移し，``stop`` でステートマシーンから抜ける実装をしていません．各ステート内で **KeyboardInterrupt** 例外がスローされたら ``stop`` に遷移する設計になるよう修正してみましょう．

========================
ステートマシーンの実行
========================

　ステートマシーンノードを登録しましょう．``ros2_workshop`` パッケージの ``setup.py`` に先ほど記述したプログラムをノード ``practice_basic_smach_node`` として登録し，``/ws`` 上で以下のコマンドを実行してパッケージをビルドしましょう．

.. code:: bash

    colcon build --symlink-install --packages-select ros2_workshop

　ワークスペースを読み込み，最新の情報を反映させます．

.. code:: bash

    source install/setup.bash

　以下のコマンドを実行して正常にステートマシーンが動作したら成功です．

.. code:: bash

    ros2 run ros2_workshop practice_basic_smach_node

　正常に動作すると，以下のように各ステートが１秒おきに状態遷移を繰り返します．

.. code:: 

    $ ros2 run ros2_workshop practice_basic_smach_node 
    [ DEBUG ] : Adding state (TASK_A, <ros2_workshop.practice_basic_smach.StateClassA object at 0x7f5e1471c880>, {'foo': 'TASK_B', 'bar': 'stop'})
    [ DEBUG ] : Adding state 'TASK_A' to the state machine.
    [ DEBUG ] : State 'TASK_A' is missing transitions: {}
    [ DEBUG ] : TRANSITIONS FOR TASK_A: {'foo': 'TASK_B', 'bar': 'stop'}
    [ DEBUG ] : Adding state (TASK_B, <ros2_workshop.practice_basic_smach.StateClassB object at 0x7f5e1471c910>, {'foo': 'TASK_A', 'bar': 'stop'})
    [ DEBUG ] : Adding state 'TASK_B' to the state machine.
    [ DEBUG ] : State 'TASK_B' is missing transitions: {}
    [ DEBUG ] : TRANSITIONS FOR TASK_B: {'foo': 'TASK_A', 'bar': 'stop'}
    [  INFO ] : State machine starting in initial state 'TASK_A' with userdata: 
    	[]
    [INFO] [1755739578.664720330] [practice_basic_smach_node]: running TASK_A
    [  INFO ] : State machine transitioning 'TASK_A':'foo'-->'TASK_B'
    [INFO] [1755739579.668110684] [practice_basic_smach_node]: running TASK_B
    [  INFO ] : State machine transitioning 'TASK_B':'foo'-->'TASK_A'

　これでステートマシーンの基礎的な実装方法の解説は終了です．お疲れ様でした．
