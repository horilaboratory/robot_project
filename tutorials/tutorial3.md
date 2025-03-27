# 3. Python をつかって ROS2 をあつかってみよう
　このチュートリアルでは ROS2 で Python をつかいロボット（亀）を動かす方法を解説します。さっそくはじめましょう。

> [!CAUTION]
> ## チュートリアルを進める前に ...
> 前回のチュートリアル [2. ROS2 に触れてみる](tutorial2.md) を読んで ROS2 の基礎を学びましたか？
> 

## VScode から WSL 内のプログラムにアクセスできるようにする方法
　VScode から WSL 内のプログラムを直接編集、作成、実行できるようにすることで開発効率がとてもよくなります。ここでは VScode の拡張機能をインストールして WSL 内の資源を VScode 経由でアクセスできるようにする方法を解説します。

## 拡張機能をインストールする
　まずはじめに VScode を開いてください。スタートメニューから *VScode* と入力するといかのように VScode が候補に挙がりますので、「開く」をクリックして開いてください。

::: note warn
もし候補に VScode が上がらない場合は、お使いのコンピューターに VScode がインストールされていません。[**こちらの記事**](https://qiita.com/GAI-313/items/f3d4722ebeda5ea195d1) を参考に VScode をインストールしてください。
:::
<img width=10% /><img width=80% src="https://i.imgur.com/Eb5Cjy8.png" />

ツールバー横に拡張機能一覧が表示されるので、検索バーに **`Japanese`** と入力してください。

　VScode が起動すると、左側にいくつかのアイコンが縦に並んでいるところがあります。ここを **ツールバー** といいます。ツールバーの下部に **田** のようなアイコン （拡張機能） をクリックしてください。


　**拡張機能** を開いて、検索バーに ***「remote develop」*** と入力してください。すると、<font color="blue"> ***\>\<*** </font> みたいなアイコンの拡張機能 ***「Remote Development」*** をインストールしてください。
<img src="https://i.imgur.com/mFaemES.png" />
 インストールボタンが「インストール済み」になったら成功です。また、ツールバーの拡張機能の下にモニターのアイコンが追加されます。
<img src="https://i.imgur.com/j96FaTC.png" />
　このモニターアイコンをクリックすると、WSL 上に存在する Ubuntu に接続する設定が開きます。
<img src="https://i.imgur.com/PsItFxO.png" />
　**WSL TARGETS** の部分に WSL 上にインストールされている Linux 一覧が表示されています。ログインしたい Linux の横にある ***「→」*** をクリックすると、現在作業しているウィンドウで Linux にログインします。
<img src="https://i.imgur.com/bQP3XpB.png" />
　Linux に無事ログインすると、VScode ウィンドウ左下の青い部分に <font color="blue"> ***WSL Ubuntu*** </font> と表示されており、今作業している VScode で WSL Linux にログインしていることを表しています。
<img src="https://i.imgur.com/7qI7xqZ.png" />
　これで VScode をつかって WSL 上のプログラムを編集する環境が整いました！使用している VScode を WSL からログアウトしたい場合、 <font color="blue"> ***WSL Ubuntu*** </font> をクリックしてください。すると下の図のようにメニューが表示されるので、メニュー上でカーソルを下に回して一番下にある ***リモート接続を終了する*** をクリックしてください。
<img src="https://i.imgur.com/liPFJwa.png" />
　ログアウトするとVScode ウィンドウ左下の青い部分の <font color="blue"> ***WSL Ubuntu*** </font> が <font color="blue"> ***\>\<*** </font> になります。この状態が WSL からログアウトし、ホスト上にいることを示しています。

# WSL2 Ubuntu を起動する
　まずはじめに WSL2 を起動して Ubuntu にログインしましょう。PowerShell を起動して以下のコマンドを実行しましょう。
```
wsl ~
```
　PowerShell の **プロンプト** が <font color="green">緑色</font> に変化したら成功です。
<img src="https://i.imgur.com/Ozce5gB.jpeg"/>

::: note
### プロンプトとは
　コマンドプロンプト、PowerShell、ターミナルなどのウィンドウでコマンドを入力する部分のことを指します。
　プロンプトの左端に表示される文字列のことを **プロンプトストリングス（もしくは PS）** といいます。OS、使用するコマンドの種類によって PS の表示内容が異なります。ほとんどの PS は現在どこのディレクトリで作業しているかを示す **カレントディレクトリの絶対パス** が記載されます。

- **PowerShell の場合**
```
PS C:\your\path>
```
- **コマンドプロンプト の場合**
```
C:\your\path>
```
- **ターミナル の場合**
```bash
name@pc:/your/path$
```
:::

## ROS2 をかんたんにつかえるようにする Tips
　さて、ROS2 を使用するには以下のコマンドを実行しなければなりません。
 ```bash
 source /opt/ros/humble/setup.bash
 ```
 でもこれいちいちシェルと起動するたびにやるのって効率悪いですよね。ここではこのコマンドをいちいち実行しなくてもよくする方法をはじめに解説しましょう。

## bashrc
　bashrc（バッシュアールシー）とは簡単に言うと起動されるシェルの設定を書く設定ファイルです。設定ファイルといってもプログラミングコードで記述されています。以下のコマンドを実行して bashrc の中身を見てみましょう。
```
code ~/.bashrc
```
　するとこのように VScode から `~/.bashrc` の中身を見ることができます。
<img src="https://i.imgur.com/RvwoibE.jpeg"/>

　ここに書かれているプログラミング言語は Bash（バッシュ）といいますが、これは要するに **Linux で使用するコマンドを複数行かいたもの** です。Linux のコマンドさえ覚えてしまえばそんなに難しくありません。試しにですが、以下のコマンドを `~/.bashrc` の末尾に書いてみましょう。
```bash
echo "Welcome Ubuntu !"
```
　このコマンドは `""` の中にかいた文字列を出力します。このコマンドを末尾に書いて、保存してみてください。
<img src="https://i.imgur.com/nHUAZzU.jpeg"/>
そしたらシェルに戻り、以下のコマンドを実行してください。すると `~/.bashrc` が再読み込みされます。
```bash
source ~/.bashrc
```
<img src="https://i.imgur.com/Uh3pvca.jpeg"/><br>
すると、このようにシェルに先ほど入力した文字列が出力されています。
　また、一度 Ubuntu からログアウトしてもう一度ログインしてみましょう。
<img src="https://i.imgur.com/8CsdqXj.jpeg"/><br>
するとこのようにログイン時も文字列が出力されているのがわかるでしょう。このように ~/.bashrc に記述したコマンドなどは **シェルにログインされるたびに実行される** のです。これを活用すればいちいち
 ```bash
 source /opt/ros/humble/setup.bash
 ```
 を実行する必要がなくなるわけですよ（＾＾）

 ---

 まぁ要するにコマンド `source /opt/ros/humble/setup.bash` を `~/.bashrc` に書き込むことで面倒なしがらみから解放されるってことですね。VScode にこのコマンドを追記しましょう。
<img src="https://i.imgur.com/JapzfCa.jpeg"/>
そしたら以下のコマンドを実行して `~/.bashrc` を再読み込みしましょう。
```bash
source ~/.bashrc
```
つぎに ROS2 関連のコマンドを実行してみましょう。無事 ROS2 が扱えるようになるはずです。
```bash
ros2 topic list
```
これで ROS2 を簡単に扱えるようにする環境構築は完了しました。

# turtlesim を起動しよう
　以下のコマンドを実行して **`turtlesim`** を起動しましょう。青色のウィンドウとカメが表示されたら成功です。以下のコマンドを実行してください。
```bash
ros2 run turtlesim turtlesim_node
```
<img src="https://i.imgur.com/7MyA3my.jpeg"/>

このまま起動しているシェルは待機させて、次のセクションに進んでください。

# VScode で Python を書くための環境を作る
　[ここで](#bashrc) で Ubuntu から直接 VScode を起動しましたが、このセクションでは改めて Windows から VScode を起動して VScode 経由から WSL2 Ubuntu にログインする方法を解説します。この方法も覚えておくことで効率的に今後開発することができるでしょう。
<img src="https://i.imgur.com/4ZScIT6.jpeg"/>
リモートデスクトップオプションの WSL ターゲットからインストール済みの Linux ディストリビューションを指定して VScode 経由からログインすることができます。
<img src="https://i.imgur.com/yZDuHf9.jpeg"/>
左下の青い部分に  <font color="blue"> **WSL:Ubuntu** </font> と表示されていたら成功です。この状態で拡張機能を開いて **Python** 拡張機能をインストールしてください。
<img src="https://i.imgur.com/P4PDvHu.jpeg"/>
これで WSL Ubuntu で Python 開発する準備が整いました。

# 亀を Python で動かしてみよう
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


