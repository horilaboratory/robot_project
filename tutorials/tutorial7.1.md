# 7.1. Node を作ろう
　ここからは実際に Python を使い Node を作成してみましょう。今から **`robot_control.py`** という名前で Python スクリプトを作成します。以下のコマンドを実行してプログラムエディタ VScode を使い robot_control.py を新規作成します。
```bash
code robot_control.py
```

## 必要なモジュールをインポートする

　これから各プログラムを ROS2 Node として扱うにはプログラム中に Node を作成する必要があります。そのためにはプログラム中に ROS2 を Python で制御するためのライブラリツール **`rclpy`** と、`rclpy` 内部にある **`Node`** ツールをインポートします。以下のコードを最初に書きましょう。
```python
from rclpy.node import Node # Node をインポート
import rclpy # rclpy をインポート
```

## Node を作成する
　Node の作成方法はいろいろありますが、最も汎用的な作成方法をここでは解説します。まずコード中に **Class（クラス）** というざっくり説明すると **複数の関数をまとめた大きな関数のようなもの** を作成します。Class についてわからない場合、とりあえずこういう感じで作っていくんだな〜程度で進んでもらって構いません。
<br>
　以下のスクリプトを続けて書いてください。以下のスクリプトは `RobotControl` という名前の Class を定義します。定義された `RobotControl` が **Class そのものです。**
```python
class RobotControl:
```
　次に作成した Class に Node を代入します。この作業は **作成した Class を ROS2 Node として扱うための重要なプロセス** です。以下のように作成したスクリプトに手を加えます。
```python
# (Node) を追加
class RobotControl(Node):
```

---

　次に Class 内部の処理を書いていきます。まずはClass 内に関数 `__init__` を作成します。
`__init__` は Class が呼び出された時にだけ１度実行される関数です。

>![TIP]
> `__init__` の `init` とは初期化を意味する `Initialize` を指し、Class が呼び出される時に自動的に実行されるとき、Class 内部で実装したい機能の初期化を行うことができます。

> [!NOTE]
> Class 内部に `def` を使い定義された関数は正確には **Method（メソッド）** と呼びます。以降 Class 内で `def` を使い定義する要素を **メソッド** と呼称します。

メソッド `__init__` は以下のように記述します。引数に必ず `self` を書くのを忘れないでください。
```python
class RobotControl(Node):
    def __init__(self):
```

---

　次に `__init__` 内部で Node の初期化を行います。以下のコードを続けて書いてください。
```python
class RobotControl(Node):
    def __init__(self):
        # Node の初期化 & Node 名の定義
        super().__init__('robot_control_node')
```
`super().__init__('robot_control_node')` は、**robot_control_node という名前の Node を作成する** という意味です。このままだと Node がちゃんと作られているかどうかよくわからないのでメソッド __init__ 内部で以下のようにターミナル上に状況を出力する処理を追加すると良いでしょう。
```python
class RobotControl(Node):
    def __init__(self):
        # Node の初期化 & Node 名の定義
        super().__init__('robot_control_node')

        #処理が行われていることを通知
        print('Node robot_control_node を作成しました。')
```

---

現在 Node を作成する Class を作っただけで作成した Class を実行する処理がまだできていません。コードの一番下に以下のコードを追記しましょう。
```python
if __name__ == '__main__':
    rclpy.init() # ROS2 をこのプログラムから制御するために rclpy モジュールを初期化する。
    node = RobotControl() # Node を宣言する処理が書かれている class を呼び出す。ここで Node が作成される
    rclpy.spin(node) # 作成された　Node を実行し続けさせる
```

> [!WARNING]
> このコードは以下のように Class 内部には書かないで、Class から退避し、かつ一番下に書きます。
> ```python
> from rclpy.node import Node # Node をインポート  
> import rclpy # rclpy をインポート  
>
>
> class RobotControl(Node):  
>     def __init__(self):  
>         # Node の初期化 & Node 名の定義  
>         super().__init__('robot_control_node')  
>
>         #処理が行われていることを通知  
>         print('Node robot_control_node を作成しました。')  
>
>
> # プログラムを実行  
> if __name__ == '__main__':  
>     rclpy.init() # ROS2 をこのプログラムから制御するために rclpy モジュールを初期化する。  
>     node = RobotControl() # Node を宣言する処理が書かれている class を呼び出す。ここで Node が作成される  
>     rclpy.spin(node) # 作成された　Node を実行し続けさせる
> ```

　条件分岐 `if __name__ == '__main__':` はメインプログラムを実行するときに書くべき **おまじない** です。なぜ書くと良いか気になる方は
[こちらのサイト](https://aiacademy.jp/media/?p=1478)
がわかりやすいです。
<br>
　次に `rclpy.init()` は、ROS2 をこの Python スクリプトから操作する時に必要な rclpy ライブラリを初期化する処理です。**必ずこれを初めに実行しないと Python 内部で Node を作成し、ROS2 操作することができません。**
<br>
　そして `node = RobotControl()` は Class RobotControl を呼び出す処理です。Class は任意の変数に格納されることを「インスタンス化」といい、変数が格納されることで Class 内部の処理が実行できるようになります。この瞬間メソッド `__init__` が実行されるため、この時点で Node が作成されるというわけです。
<br>
　インスタンス化した Class RobotControl は `node` という変数に格納されており、Node を作成した Class RobotControl は ROS2 Node と同じ存在になります。この変数を `rclpy.spin(node)` のように `rclpy.spin()` のなかに代入することで作成したノードを継続的に実行状態に保つことができます。`rclpy.spin()` はノードが終了するまで処理をブロックし、ノードをアクティブな状態に維持する重要な関数です。

## プログラムを実行してみよう
　このチュートリアルで作成したコードの全体は以下のとおりです。
```python
# ROS2のPythonライブラリをインポート
from rclpy.node import Node  # ROS2ノードを作成するための基本クラス
import rclpy  # ROS2のPythonクライアントライブラリ


# ロボット制御用のカスタムノードクラス
class RobotControl(Node):
    def __init__(self):
        """
        ノードの初期化コンストラクタ
        ノード作成時に自動的に呼び出される
        """
        
        # 親クラス(Node)の初期化 + ノード名の設定
        # 'robot_control_node'という名前でノードを登録
        super().__init__('robot_control_node')

        # デバッグ用メッセージ（ターミナルに表示）
        print('robot_control_node が起動しました')


# メインプログラム ========================================
if __name__ == '__main__':
    """
    プログラムのエントリーポイント（最初に実行される部分）
    """
    
    # ROS2通信機能の初期化（必須処理）
    rclpy.init()
    
    # RobotControlクラスからノードインスタンスを生成
    # → __init__()が自動実行され、ノードが登録される
    node = RobotControl()
    
    # ノードを継続実行（Ctrl+Cで終了するまでループ）
    # spin()はROS2ノードの「心臓部」のようなもの
    rclpy.spin(node)
```
プログラム `robot_control.py` を保存して、ターミナル上で以下のコマンドを実行して作成したプログラムを実行してみましょう。
```bash
python3 robot_control.py
```
すると、以下のようにターミナル上で `robot_control_node が起動しました` とテキストが表示されます。
```bash
$ python3 robot_control.py

robot_control_node が起動しました
```
`rclpy,spin(node)` によってプログラムは停止せず動作し続けます。この状態で新たなターミナルを開き、以下のコマンドを実行して現在実行されている ROS2 Node 一覧を表示してみましょう。
```bash
ros2 node list
```
すると、このようにプログラム中の `super()__init__()` 内で宣言したノード名が表示され、確かに作成した Node が動作中であることがわかります。
```bash
$ ros2 node list

/robot_control_node
```

---

これで Node の作成方法に関する解説は以上です。`robot_control.py` を実行しているターミナル上でプログラム停止ショートカットキー「Control + C」でプログラムを停止しましょう。

> [!NOTE]
> プログラムを停止すると、以下のように大量のエラーメッセージが表示されることがあります。これは Node を適切な方法で停止しなかったことで発生したエラーです。特に支障はありませんので気にしないでください。
> ```bash
> ^CTraceback (most recent call last):  
>   File "/ws/src/robot_control.py", line 35, in <module>  
>     rclpy.spin(node)  
>   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 226, in spin  
>     executor.spin_once()  
>   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 751, in spin_once  
>     self._spin_once_impl(timeout_sec)  
>   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 740, in _spin_once_impl  
>     handler, entity, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)  
>   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 723, in wait_for_ready_callbacks  
>     return next(self._cb_iter)  
>   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 620, in _wait_for_ready_callbacks  
>     wait_set.wait(timeout_nsec)  
> KeyboardInterrupt  
> ```

## おさらい

ここまで学んだNode作成の手順をまとめましょう。ROS2でPythonノードを作成するには以下のステップが必要です：

1. **必要なモジュールのインポート**
   ```python
   from rclpy.node import Node
   import rclpy
   ```
   - `rclpy`：ROS2のPythonクライアントライブラリ
   - `Node`：ノード作成の基本クラス

2. **ノードクラスの定義**
   ```python
   class RobotControl(Node):
       def __init__(self):
           super().__init__('robot_control_node')
           print('Nodeが起動しました')
   ```
   - `Node`クラスを継承してカスタムノードを作成
   - `super().__init__()`でノード名を登録
   - 初期化メッセージで動作確認

3. **メインプログラムの記述**
   ```python
   if __name__ == '__main__':
       rclpy.init()
       node = RobotControl()
       rclpy.spin(node)
   ```
   - `rclpy.init()`：ROS2システムの初期化（必須）
   - ノードインスタンスの生成
   - `rclpy.spin()`でノードを実行状態に維持

4. **実行と確認**
   ```bash
   python3 robot_control.py
   ros2 node list
   ```
   - プログラム実行後、`ros2 node list`でノード確認
   - Ctrl+Cで終了

### 重要なポイント
- `rclpy.init()`は必ず最初に実行
- `spin()`がないとノードは即時終了する
- クラス定義と実行コードは分離して記述

### チャレンジ
以下の作業に挑戦してみましょう。

- Node 名を変更する

---

次のチュートリアル [7.2.Publisher を実装しよう](tutorial7.2.md) では今回作成した Node に Publisher 機能を実装し、テキスト `Hello ROS2!` をパブリッシュする方法を解説します。