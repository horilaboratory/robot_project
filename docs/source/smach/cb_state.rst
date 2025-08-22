#######################################
状態遷移を実装する（コールバック編）
#######################################

　前回の基礎編では ``smach.State`` を継承した **クラス** を使って各ステートの処理を実装しました．
今回はよりシンプルに，**コールバック関数** をステートとして実装する方法を学びます．smach では，関数に ``@smach.cb_interface`` デコレータを付与し，それを ``smach.CBState`` でラップすることで，関数をステートとして扱うことができます．

　ここでは ``basic.rst`` で作成した ``practice_basic_smach.py`` に追記する形で，コールバック関数を用いたステートを既存のステートマシーンに追加します．まず ``practice_basic_smach.py`` をエディタで開いてください．

==================================
コールバック関数を作成する
==================================

　基礎編で作成した ``StateClassB`` の下に，新たに追加するステート ``TASK_C`` と ``TASK_D`` が呼び出すコールバック関数を追記します．

.. code:: python

    # ... StateClassB の定義の後 ...

    #
    # TASK_Cに対応するコールバック関数
    #
    @smach.cb_interface(outcomes=['foo', 'bar'])
    def task_c_cb(userdata, node):
        node.get_logger().info('running TASK_C')
        time.sleep(1)
        return 'foo'

    #
    # TASK_Dに対応するコールバック関数
    #
    @smach.cb_interface(outcomes=['foo', 'bar'])
    def task_d_cb(userdata, node):
        node.get_logger().info('running TASK_D')
        time.sleep(1)
        return 'foo'

    def main():
        ...

　関数のポイントは以下の通りです．

- **``@smach.cb_interface`` デコレータ**:

    - 関数を smach のステートとして利用可能にするための「飾り付け」です．
    - ``outcomes``: この関数が返す可能性のある状態名をリストで定義します．これは ``smach.State`` クラスの ``__init__`` で定義したものと同じ役割です．

- **関数引数**:

    - 最初の引数は ``userdata`` とする必要があります．これは smach が内部的に使用します．
    - ２つ目以降の引数（ここでは ``node``）は，後ほど ``smach.CBState`` の ``cb_kwargs`` を通じて渡されるオブジェクトを受け取ります．

- **返り値**:

    - 関数は必ず ``outcomes`` で定義した状態名のいずれかを文字列で返す必要があります．

================================
StateMachine を実装する
================================

　次に ``main`` 関数を修正します．``smach.StateMachine.add`` を呼び出す際，第２引数に ``smach.CBState(デコレータを付けた関数名, cb_kwargs={...})`` を指定します．

.. code:: python

    def main():
        # ROS2を初期化します
        rclpy.init()
        # ROS2ノードを作成します
        node = Node('practice_basic_smach_node')

        # StateMachineのインスタンスを作成します
        sm = smach.StateMachine(outcomes=['stop'])

        # 'with'構文を使ってステートマシン（sm）の内部にステートを定義していきます
        with sm:
            # TASK_A: 基礎編で作成したクラスベースのステート
            smach.StateMachine.add('TASK_A', StateClassA(node=node),
                                    transitions={
                                        'foo' : 'TASK_B',
                                        'bar' : 'stop'
                                    }
            )

            # TASK_B: 基礎編で作成したクラスベースのステート
            smach.StateMachine.add('TASK_B', StateClassB(node=node),
                                    transitions={
                                        'foo' : 'TASK_C',
                                        'bar' : 'stop'
                                    }
            )
            
            # TASK_C: 新たに追加するコールバックベースのステート
            smach.StateMachine.add('TASK_C',
                                   smach.CBState(task_c_cb,
                                                 cb_kwargs={'node': node}),
                                   transitions={'foo': 'TASK_D',
                                                'bar': 'stop'})

            # TASK_D: 新たに追加するコールバックベースのステート
            smach.StateMachine.add('TASK_D',
                                   smach.CBState(task_d_cb,
                                                 cb_kwargs={'node': node}),
                                   transitions={'foo': 'TASK_A',
                                                'bar': 'stop'})

        # 定義したステートマシンを実行します
        outcome = sm.execute()

　``smach.CBState`` の引数 ``cb_kwargs`` を使うことで，コールバック関数に ``main`` 関数内で作成した ``node`` オブジェクトを渡すことができます．辞書のキー（``'node'``）がコールバック関数の引数名に対応します．

==============
コード完成図
==============

　``practice_basic_smach.py`` を修正した後のコード全体は以下のようになります．

.. code:: python

    #!/usr/bin/env python3
    # 必要なライブラリをインポートします
    from rclpy.node import Node
    import rclpy
    import smach
    import time # ダミー処理用のtimeモジュール

    #
    # ステートA（TASK_A）に対応するステートクラス（基礎編で作成）
    #
    class StateClassA(smach.State):
        def __init__(self, node):
            smach.State.__init__(
                self,
                outcomes=['foo', 'bar']
            )
            self.node = node

        def execute(self, userdata):
            self.node.get_logger().info('running TASK_A')
            time.sleep(1)
            return 'foo'

    #
    # ステートB（TASK_B）に対応するステートクラス（基礎編で作成）
    #
    class StateClassB(smach.State):
        def __init__(self, node):
            smach.State.__init__(
                self,
                outcomes=['foo', 'bar']
            )
            self.node = node

        def execute(self, userdata):
            self.node.get_logger().info('running TASK_B')
            time.sleep(1)
            return 'foo'
            
    #
    # TASK_Cに対応するコールバック関数
    #
    @smach.cb_interface(outcomes=['foo', 'bar'])
    def task_c_cb(userdata, node):
        node.get_logger().info('running TASK_C')
        time.sleep(1)
        return 'foo'

    #
    # TASK_Dに対応するコールバック関数
    #
    @smach.cb_interface(outcomes=['foo', 'bar'])
    def task_d_cb(userdata, node):
        node.get_logger().info('running TASK_D')
        time.sleep(1)
        return 'foo'

    #
    # メイン関数：プログラム全体のエントリーポイント
    #
    def main():
        # ROS2を初期化します
        rclpy.init()
        # ROS2ノードを作成します
        node = Node('practice_basic_smach_node')

        # StateMachineのインスタンスを作成します
        sm = smach.StateMachine(outcomes=['stop'])

        # 'with'構文を使ってステートマシン（sm）の内部にステートを定義していきます
        with sm:
            # TASK_A: 基礎編で作成したクラスベースのステート
            smach.StateMachine.add('TASK_A', StateClassA(node=node),
                                    transitions={
                                        'foo' : 'TASK_B',
                                        'bar' : 'stop'
                                    }
            )

            # TASK_B: 基礎編で作成したクラスベースのステート
            smach.StateMachine.add('TASK_B', StateClassB(node=node),
                                    transitions={
                                        'foo' : 'TASK_C',
                                        'bar' : 'stop'
                                    }
            )
            
            # TASK_C: 新たに追加するコールバックベースのステート
            smach.StateMachine.add('TASK_C',
                                   smach.CBState(task_c_cb,
                                                 cb_kwargs={'node': node}),
                                   transitions={'foo': 'TASK_D',
                                                'bar': 'stop'})

            # TASK_D: 新たに追加するコールバックベースのステート
            smach.StateMachine.add('TASK_D',
                                   smach.CBState(task_d_cb,
                                                 cb_kwargs={'node': node}),
                                   transitions={'foo': 'TASK_A',
                                                'bar': 'stop'})

        # 定義したステートマシンを実行します
        outcome = sm.execute()

    # このスクリプトが直接実行された場合にmain()関数を呼び出す
    if __name__ == '__main__':
        main()

========================
ステートマシーンの実行
========================

　``practice_basic_smach.py`` は基礎編ですでにノード ``practice_basic_smach_node`` として登録済みです．ファイルを保存したら，再度パッケージをビルドして実行しましょう．

.. code-block:: bash

    # ワークスペースのルートでビルド
    colcon build --symlink-install --packages-select ros2_workshop

    # ワークスペースの読み込み
    source install/setup.bash

    # ノードの実行
    ros2 run ros2_workshop practice_basic_smach_node

　実行すると，コンソールに ``running TASK_A``, ``running TASK_B``, ``running TASK_C``, ``running TASK_D`` が1秒おきに順番に表示され，ループし続ければ成功です．
