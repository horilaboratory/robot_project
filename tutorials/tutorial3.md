# 3. Python をつかって ROS2 をあつかってみよう
　このチュートリアルでは ROS2 で Python をつかいロボット（亀）を動かす方法を解説します。さっそくはじめましょう。

　では亀を動かしてみましょう。そのためにはプログラムを書くファイルを作らなくてはいけませんね。VScode のツールバー上部にファイルエクスプローラーがあります。ここをクリックすると <font color="blue">「フォルダーを開く」</font> というボタンが表示されるのでここをクリックしてください。
<img src="https://i.imgur.com/qQEfVCW.jpeg"/>
次にこのように上部にどのフォルダ（ディレクトリ）を選択するのかを聞かれるので `~` などと入力してホームディレクトリを指定しましょう。決定したら <font color="blue">「OK」</font> をクリックしましょう。
<img src="https://i.imgur.com/OdVRUbF.jpeg"/>
すると、このようにホームディレクトリが VScode 左辺にエクスプローラとして表示されます。このエリアにあるフォルダに + アイコンがついているボタンをクリックすると、新規フォルダ（ディレクトリ）を作成することができます。試しに `python_scripts` という名前のディレクトリを作ってみましょう。
<img src="https://i.imgur.com/bxbjVnc.jpeg"/>
フォルダアイコンをクリックすると、このように入力欄があるので、ここにディレクトリ名を入力して、エンターキーを押すと。。。
<img src="https://i.imgur.com/bhj4iXI.jpeg"/>
このようにディレクトリが作成されます！
<img src="https://i.imgur.com/4VEpQWw.jpeg"/>
今度は作成したディレクトリを指定した状態でファイルに + アイコンがついたボタンをクリックして、新規ファイルを作成しましょう。
<img src="https://i.imgur.com/mPQGg7l.jpeg"/>
　するとこのようにファイル名を入力する項目が出るので、`turtle_control.py` というファイルを作成しましょう。この得必ず拡張子が `.py` でなければなりませんので気をつけてください。
<img src="https://i.imgur.com/386k2IL.jpeg"/>
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


