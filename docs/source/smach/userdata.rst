#################################
ステート間でデータを共有する
#################################

　これまでのチュートリアルでは，各ステートは独立して動作していました．しかし，実際のロボットアプリケーションでは，「物体を探索するステート」が見つけた物体の位置座標を，「アームを動かすステート」に渡すといったように，ステート間でデータを共有する必要があります．

　smach ではこのデータ共有の仕組みとして **UserData** が用意されています．このセクションでは，UserData を使ってステート間でカウンターの値をやり取りする方法を学びます．

================
UserData とは
================

　**UserData** は，ステートマシン全体で共有される辞書のようなオブジェクトです．
各ステートは，この共有データに対して値の読み書きを行うことで，他のステートと情報をやり取りします．

データの流れは以下のようになります．

1.  **StateMachine に UserData の入れ物を準備する**
2.  **あるステートが UserData にデータを書き込む（Output）**
3.  **別のステートが UserData からデータを読み込む（Input）**

このデータの入出力を制御するために，各ステートでは ``input_keys`` と ``output_keys`` という特別な変数を定義します．

-   ``input_keys``: このステートが UserData から**読み込む**データのキー名を指定します．
-   ``output_keys``: このステートが UserData へ**書き込む**データのキー名を指定します．

　それでは，既存の ``practice_basic_smach.py`` を修正して，UserData の使い方を実装していきましょう．

==================================================
基本的な UserData の使い方（Remapping なし）
==================================================

　最もシンプルな UserData の使い方は，ステートクラス側で定義するキー名と，StateMachine 側で用意するキー名を **全く同じ** にすることです．
この場合，smach が自動的にキーを紐付けてくれます．

　まず，``StateClassA`` と ``StateClassB`` を修正します．両方のクラスで，共有したいデータに ``counter`` という共通のキー名を使うように定義します．
``practice_basic_smach.py`` をエディタで開いてください．

.. code:: python

    class StateClassA(smach.State):
        def __init__(self, node):
            smach.State.__init__(
                self,
                outcomes=['foo', 'bar'],
                input_keys=['counter'],  # 読み込みキーを 'counter' に
                output_keys=['counter'] # 書き込みキーも 'counter' に
            )
            self.node = node

        def execute(self, userdata):
            self.node.get_logger().info('running TASK_A')
            # userdata.counter で読み書きを行う
            new_counter_value = userdata.counter + 1
            self.node.get_logger().info(f'Incrementing counter to: {new_counter_value}')
            userdata.counter = new_counter_value
            time.sleep(1)
            return 'foo'

    class StateClassB(smach.State):
        def __init__(self, node):
            smach.State.__init__(
                self,
                outcomes=['foo', 'bar'],
                input_keys=['counter'] # 読み込みキーを 'counter' に
            )
            self.node = node

        def execute(self, userdata):
            self.node.get_logger().info('running TASK_B')
            # userdata.counter で読み込みを行う
            self.node.get_logger().info(f'Counter value is: {userdata.counter}')
            time.sleep(1)
            return 'foo'

　次に ``main`` 関数を修正します．StateMachine の UserData に ``counter`` というキーで初期値を設定します．
このキー名は，先ほどステートクラスで定義した ``input_keys`` / ``output_keys`` の名前と一致しています．

.. code:: python

    def main():
        # ...
        sm = smach.StateMachine(outcomes=['stop'])

        # StateMachineのUserDataに初期値を設定
        sm.userdata.counter = 0 

        with sm:
            # remapping引数は不要
            smach.StateMachine.add('TASK_A', StateClassA(node=node),
                                    transitions={'foo' : 'TASK_B', 'bar' : 'stop'})

            # こちらもremapping引数は不要
            smach.StateMachine.add('TASK_B', StateClassB(node=node),
                                    transitions={'foo' : 'TASK_C', 'bar' : 'stop'})
            # ...

　このように，ステートクラス側と StateMachine 側のキー名が一致していれば，``remapping`` 引数を指定しなくても smach が自動で接続してくれます．

==============================================
Remapping による柔軟なデータ共有
==============================================

　先ほどの方法はシンプルですが，問題点もあります．もし，他の人が作ったステートクラスを使いたい場合，そのクラスがどんなキー名（``input_keys`` など）を使っているかを把握し，StateMachine 側のキー名をそれに合わせなければなりません．

　そこで登場するのが **リマッピング（remapping）** です．
Remapping は，ステートクラスが内部で使うキー名と，StateMachine が全体で共有する UserData のキー名を，**自由な名前で紐付ける** ことができる強力な機能です．

　それでは，先ほどのコードを Remapping を使う形に書き換えてみましょう．
ステートクラス側は，外部のキー名を気にせず，自身の役割に合ったキー名（``counter_in``, ``counter_out`` など）を定義できるとします．

.. code:: python

    class StateClassA(smach.State):
        def __init__(self, node):
            # 内部で使うキー名を 'counter_in', 'counter_out' に変更
            smach.State.__init__(
                self,
                outcomes=['foo', 'bar'],
                input_keys=['counter_in'],
                output_keys=['counter_out']
            )
            self.node = node

        def execute(self, userdata):
            self.node.get_logger().info('running TASK_A')
            # 内部のキー名でアクセス
            new_counter_value = userdata.counter_in + 1
            self.node.get_logger().info(f'Incrementing counter to: {new_counter_value}')
            userdata.counter_out = new_counter_value
            time.sleep(1)
            return 'foo'

    class StateClassB(smach.State):
        def __init__(self, node):
            # 内部で使うキー名を 'counter_in' に変更
            smach.State.__init__(
                self,
                outcomes=['foo', 'bar'],
                input_keys=['counter_in']
            )
            self.node = node

        def execute(self, userdata):
            self.node.get_logger().info('running TASK_B')
            # 内部のキー名でアクセス
            self.node.get_logger().info(f'Counter value is: {userdata.counter_in}')
            time.sleep(1)
            return 'foo'

　ステートクラス側のキー名（``counter_in``, ``counter_out``）と，StateMachine 側のキー名（``counter``）が異なっていても問題ありません．
``main`` 関数で ``remapping`` 引数を使い，これらの異なるキー名を接続します．

.. code:: python

    def main():
        # ...
        sm.userdata.counter = 100 # 初期値を任意の値（例: 100）にしてみましょう

        with sm:
            smach.StateMachine.add('TASK_A', StateClassA(node=node),
                                    transitions={
                                        'foo' : 'TASK_B',
                                        'bar' : 'stop'
                                    },
                                    # remappingを追加
                                    remapping={
                                        'counter_in': 'counter',
                                        'counter_out': 'counter'
                                    }
            )

            smach.StateMachine.add('TASK_B', StateClassB(node=node),
                                    transitions={
                                        'foo' : 'TASK_C',
                                        'bar' : 'stop'
                                    },
                                    # remappingを追加
                                    remapping={
                                        'counter_in': 'counter'
                                    }
            )
            # ...

　``remapping`` は辞書形式で ``{'ステートクラス側のキー': 'StateMachine側のキー'}`` のように記述します．

-   **TASK_A**:
    -   ``'counter_in': 'counter'``: ``StateClassA`` の ``counter_in`` は，StateMachine の ``sm.userdata.counter`` から値を読み込みます．
    -   ``'counter_out': 'counter'``: ``StateClassA`` が ``counter_out`` に書き込んだ値は，StateMachine の ``sm.userdata.counter`` に上書きされます．
-   **TASK_B**:
    -   ``'counter_in': 'counter'``: ``StateClassB`` の ``counter_in`` は，StateMachine の ``sm.userdata.counter`` から値を読み込みます．

　このように，Remapping を使うことで，ステートクラスの再利用性が高まり，より柔軟なステートマシンの設計が可能になります．

==================================
CBState で UserData を扱う
==================================

　コールバック関数でも，同様に UserData を扱うことができます．
ここでは，``TASK_C`` がメッセージを読み込み，``TASK_D`` がメッセージを更新する処理を追加してみましょう．

　まず，コールバック関数 ``task_c_cb`` と ``task_d_cb`` の ``@smach.cb_interface`` デコレータに ``input_keys`` / ``output_keys`` を追加します．

.. code:: python

    @smach.cb_interface(outcomes=['foo', 'bar'],
                        input_keys=['message_in'])
    def task_c_cb(userdata, node):
        node.get_logger().info('running TASK_C')
        # userdataからメッセージを読み込んで表示
        node.get_logger().info(f'Message is: "{userdata.message_in}"')
        time.sleep(1)
        return 'foo'

    @smach.cb_interface(outcomes=['foo', 'bar'],
                        output_keys=['message_out'])
    def task_d_cb(userdata, node):
        node.get_logger().info('running TASK_D')
        # userdataに新しいメッセージを書き込む
        userdata.message_out = f'Hello from TASK_D at {time.time()}'
        node.get_logger().info('Message updated.')
        time.sleep(1)
        return 'foo'

　クラスベースのステートと同様に，引数 ``userdata`` を通じてデータにアクセスします．

　次に ``main`` 関数を修正します．
まず，StateMachine の UserData に新しいキー ``message`` を追加します．

.. code:: python

    def main():
        # ...
        sm.userdata.counter = 100
        # 新しいUserDataキーを追加
        sm.userdata.message = 'Initial Message'
        # ...

　最後に，``TASK_C`` と ``TASK_D`` を定義している ``smach.StateMachine.add()`` に ``remapping`` を設定します．
``input_keys`` / ``output_keys`` はデコレータで既に定義済みのため，``smach.CBState`` の引数には含めない点に注意してください．

.. code:: python
    
    def main():
        # ...
        with sm:
            # ... TASK_A, TASK_B は省略 ...
            
            smach.StateMachine.add('TASK_C',
                                   smach.CBState(task_c_cb,
                                                 cb_kwargs={'node': node}),
                                   transitions={'foo': 'TASK_D',
                                                'bar': 'stop'},
                                   remapping={'message_in': 'message'}) # remapping を追加

            smach.StateMachine.add('TASK_D',
                                   smach.CBState(task_d_cb,
                                                 cb_kwargs={'node': node}),
                                   transitions={'foo': 'TASK_A',
                                                'bar': 'stop'},
                                   remapping={'message_out': 'message'}) # remapping を追加
        # ...

　これで，コールバック関数ベースのステートでも UserData の共有ができるようになりました．

==============
コード完成図
==============

　最終的な ``practice_basic_smach.py`` のコード全体は以下のようになります（Remapping を使用したバージョン）．

.. code:: python

    #!/usr/bin/env python3
    from rclpy.node import Node
    import rclpy
    import smach
    import time

    class StateClassA(smach.State):
        def __init__(self, node):
            smach.State.__init__(
                self,
                outcomes=['foo', 'bar'],
                input_keys=['counter_in'],
                output_keys=['counter_out']
            )
            self.node = node

        def execute(self, userdata):
            self.node.get_logger().info('running TASK_A')
            new_counter_value = userdata.counter_in + 1
            self.node.get_logger().info(f'Incrementing counter to: {new_counter_value}')
            userdata.counter_out = new_counter_value
            time.sleep(1)
            return 'foo'

    class StateClassB(smach.State):
        def __init__(self, node):
            smach.State.__init__(
                self,
                outcomes=['foo', 'bar'],
                input_keys=['counter_in']
            )
            self.node = node

        def execute(self, userdata):
            self.node.get_logger().info('running TASK_B')
            self.node.get_logger().info(f'Counter value is: {userdata.counter_in}')
            time.sleep(1)
            return 'foo'
            
    @smach.cb_interface(outcomes=['foo', 'bar'],
                        input_keys=['message_in'])
    def task_c_cb(userdata, node):
        node.get_logger().info('running TASK_C')
        node.get_logger().info(f'Message is: "{userdata.message_in}"')
        time.sleep(1)
        return 'foo'

    @smach.cb_interface(outcomes=['foo', 'bar'],
                        output_keys=['message_out'])
    def task_d_cb(userdata, node):
        node.get_logger().info('running TASK_D')
        userdata.message_out = f'Hello from TASK_D at {time.time()}'
        node.get_logger().info('Message updated.')
        time.sleep(1)
        return 'foo'

    def main():
        rclpy.init()
        node = Node('practice_basic_smach_node')
        sm = smach.StateMachine(outcomes=['stop'])
        sm.userdata.counter = 100
        sm.userdata.message = 'Initial Message'

        with sm:
            smach.StateMachine.add('TASK_A', StateClassA(node=node),
                                    transitions={
                                        'foo' : 'TASK_B',
                                        'bar' : 'stop'
                                    },
                                    remapping={
                                        'counter_in': 'counter',
                                        'counter_out': 'counter'
                                    }
            )
            smach.StateMachine.add('TASK_B', StateClassB(node=node),
                                    transitions={
                                        'foo' : 'TASK_C',
                                        'bar' : 'stop'
                                    },
                                    remapping={
                                        'counter_in': 'counter'
                                    }
            )
            smach.StateMachine.add('TASK_C',
                                   smach.CBState(task_c_cb,
                                                 cb_kwargs={'node': node}),
                                   transitions={'foo': 'TASK_D',
                                                'bar': 'stop'},
                                   remapping={'message_in': 'message'})

            smach.StateMachine.add('TASK_D',
                                   smach.CBState(task_d_cb,
                                                 cb_kwargs={'node': node}),
                                   transitions={'foo': 'TASK_A',
                                                'bar': 'stop'},
                                   remapping={'message_out': 'message'})
        outcome = sm.execute()

    if __name__ == '__main__':
        main()

========================
ステートマシーンの実行
========================

　ファイルを保存したら，パッケージをビルドして実行しましょう．

.. code-block:: bash

    colcon build --symlink-install --packages-select ros2_workshop
    source install/setup.bash
    ros2 run ros2_workshop practice_basic_smach_node

　実行すると，カウンターとメッセージが各ステートで更新・表示される様子が確認できます．

.. code::

    [INFO] [practice_basic_smach_node]: running TASK_A
    [INFO] [practice_basic_smach_node]: Incrementing counter to: 101
    [INFO] [practice_basic_smach_node]: running TASK_B
    [INFO] [practice_basic_smach_node]: Counter value is: 101
    [INFO] [practice_basic_smach_node]: running TASK_C
    [INFO] [practice_basic_smach_node]: Message is: "Initial Message"
    [INFO] [practice_basic_smach_node]: running TASK_D
    [INFO] [practice_basic_smach_node]: Message updated.
    ...

　これでステート間でデータを共有する方法の解説は終了です．
