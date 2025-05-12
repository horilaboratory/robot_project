# 3. ROS2 を Python で扱う
　このドキュメントでは ROS2 で Python をつかいロボット（亀）を動かす方法を解説します。さっそくはじめましょう。

## turtlesim を起動しよう
　以下のコマンドを実行して **`turtlesim`** を起動しましょう。青色のウィンドウとカメが表示されたら成功です。以下のコマンドを実行してください。
```bash
ros2 run turtlesim turtlesim_node
```
<img src="https://i.imgur.com/7MyA3my.jpeg"/>

このまま起動しているシェルは待機させて、次のセクションに進んでください。

## 亀を Python で動かしてみよう
　ROS2 については前回の資料で軽く説明しました。前回ではコマンドを使い ROS2 のインターフェースに触れましたが、ここでは Python を使い ROS2 を操作してみましょう。まずは Python を書くためのワークスペースとエディタを用意します。ターミナルを開いて以下のコマンドを実行し、ホームディレクトリ上に `scripts` というディレクトリを作成します。
```bash
mkdir ~/scripts
```
　次に、以下のコマンドを実行して `turtle_control.py` という空のファイルを `scripts` ディレクトリ内に作成します。
```bash
touch ~/scripts/turtle_control.py
```
　最後に以下のコマンドを実行して、プログラムコードエディタ VScode を使い `scripts` ディレクトリを開きます。
```bash
code ~/scripts
```
　これで準備は完了です。`turtle_control.py` に亀を動かすプログラムを書いていきましょう。先に示すと、以下のコードを書くと亀が円を描きながら動いてくれます。
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def move_turtle():
    # rclpyを初期化
    rclpy.init()

    # Nodeを作成
    node = Node('turtle_controller')

    # Publisherを作成
    publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    # Twistメッセージを作成
    twist = Twist()

    # 速度を設定
    twist.linear.x = 1.0  # 前進速度
    twist.angular.z = 0.5  # 回転速度

    # ループでメッセージを送信
    try:
        while rclpy.ok():
            # メッセージをログに出力
            node.get_logger().info('Publishing: linear.x=%f, angular.z=%f' % (twist.linear.x, twist.angular.z))
            
            # メッセージを送信
            publisher.publish(twist)

            # 1秒待機
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass

    # ノードを破棄
    rclpy.shutdown()

if __name__ == '__main__':
    move_turtle()
```
実行するとこのように亀が動きます。
<img src="https://i.imgur.com/pwHslyv.gif"/>

## どうやって動かしているの？
　ここではサンプルで示した `turtle_control.py` のコードを解説します。まず注目するのは初めの 3 行です。
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
```
ここでは3つのモジュール `rclpy`、`Node`、`Twist` をインポートしています。それぞれのモジュールは以下の役割を持っています。

- **`rclpy`**
    ROS2 を制御するための Python ライブラリです。
- **`Node`**
    実行コード内で ROS2 ノードを生成し、ROS2 ネットワークに接続するための API。
- **`Twist`**
    メッセージの型です。このオブジェクトにデータを入力し指定されたトピックに Publish することでロボットを動かすことができます。

---

次に、実行部分です。ロボットを制御しているプログラムは `move_turtle` という関数にまとめられているのがわかるでしょう。
```python
def move_turtle():
    # rclpyを初期化
    rclpy.init()

    # Nodeを作成
    node = Node('turtle_controller')

    # Publisherを作成
    publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    # Twistメッセージを作成
    twist = Twist()

    # 速度を設定
    twist.linear.x = 1.0  # 前進速度
    twist.angular.z = 0.5  # 回転速度

    # ループでメッセージを送信
    try:
        while rclpy.ok():
            # メッセージをログに出力
            node.get_logger().info('Publishing: linear.x=%f, angular.z=%f' % (twist.linear.x, twist.angular.z))
            
            # メッセージを送信
            publisher.publish(twist)

            # 1秒待機
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass

    # ノードを破棄
    rclpy.shutdown()
```
関数 `move_turtle` 内の処理について解説しましょう。まず初めに
```python
rclpy.init()
```
という処理があります。この処理はこのプログラムで ROS2 を扱うので `rclpy` の初期化を行います。これを最初に書かないとプログラム内で ROS2 を扱うことができずエラーが発生してしまいます。
　次に、以下の処理が行われます。
```python
node = Node('turtle_controller')
```
これはなんなのかというと、このプログラム内で `turtle_controller` という名前のノードを宣言しています。ノードというのは ROS2 におけるプログラムの最小単位で、ノードが立つことでノード同士がデータをやり取りします。ノード同士のやり取りでロボットを動かしていくわけです。このコードは指定した名前のノードを宣言し、以降 ROS2 ネットワークに接続するために必要mなライブラリを提供してくれます。このような処置を **インスタンス化** と言います。
　次に、以下の処理が行われます。
```python
publisher = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
```
　インスタンス化された `node` には、**`create_publisher`** というメソッドが用意されています。このメソッドを使用することで指定されたトピック、メッセージ型にデータを送るためのインターフェースを作成してくれます。Publisher の設定は各引数で行われます。

- **第1引数**
    Publish するメッセージ型を指定します。ここでは `Twist` が指定されています。
- **第2引数**
    Publish するトピック名を指定します。ここでは `/turtle1/cmd_vel` が指定されています。
- **第3引数**
    **QoS** プロファイルを指定します。この段階ではなんなのかを知るとややこしくなるので `10` を入力してください。

　次に `Twist` メッセージにデータを入れるために `Twist` オブジェクトを以下の処理でインスタンス化します。
```python
twist = Twist()
```
　そして、インスタンス化されたオブジェクトにデータを入力します。
```python
# 速度を設定
twist.linear.x = 1.0  # 前進速度
twist.angular.z = 0.5  # 回転速度
```
コメントで示されている通り `Twist.linear.x` は前進速度を指定し、`Twist.angular.z` で回転速度を指定します。
　そして、以下の処理で設定した Twist データを Publish します。
```python
# ループでメッセージを送信
try:
    while rclpy.ok():
        # メッセージをログに出力
        node.get_logger().info('Publishing: linear.x=%f, angular.z=%f' % (twist.linear.x, twist.angular.z))
        
        # メッセージを送信
        publisher.publish(twist)

        # 1秒待機
        rclpy.spin_once(node, timeout_sec=1.0)
except KeyboardInterrupt:
    pass
```
`while rclpy.ok()` とは、`rclpy.ok()` が `True` の間は以下の処理を繰り返すという意味です。`rclpy.ok()` は ROS2 が動いている限り `True` を返し続けます。
　そして、以下の処理は ROS2 のロガー出力で、ログとしてメッセージを出力するコードです。
```python
node.get_logger().info('Publishing: linear.x=%f, angular.z=%f' % (twist.linear.x, twist.angular.z))
```
　そして、重要なのはここです。
```python
publisher.publish(twist)
```
ここでメッセージデータを格納した `twist` オブジェクトを `create_publisher` で指定したトピックへ Publish します。この処理によって亀が動くのです。
```python
rclpy.spin_once(node, timeout_sec=1.0)
```
　この処理は ROS2 の反映を待機する関数です。これを書かないと publisher がうまく動いてくれません。

---
## 課題
- turtle_control.py を編集して亀の動きを変えてみましょう。


---

- [**チュートリアル一覧** に戻る](./toc.md)
- [**次のチュートリアル** に移動する](./tutorial4.md)
