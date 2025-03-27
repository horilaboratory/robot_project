# 2. ROS2 に触れてみる
　ROS2 はざっくり説明すると主にロボットを制御するためのメッセージデータを相互通信するシステムです。ここではその相互通信（Pub&Sub 通信：パブ、サブ通信）のサンプルを実行して動作確認してみましょう。

## サンプルプログラムを実行してトピックの通信をしてみよう
　サンプルプログラムを実行して、ROS2 の Pub&Sub 通信に触れてみましょう。以下のコマンドを実行すると、ROS2 が用意しているサンプルプログラムを実行することができます。
```
ros2 run demo_nodes_cpp talker
```
　このコマンドの意味を解説すると、**`demo_nodes` というパッケージにある `talker` というプログラムを実行します**
　ROS2 では `talker` のような１つのプログラムのことを <font color=blue>**ノード**</font> と呼びます。そして複数のノードをまとめたものを <font color=red>**パッケージ**</font> といいます。

---
サンプルプログラムを実行すると、このように以下のログが表示されます。
```
$ ros2 run demo_nodes_cpp talker
[INFO] [1727364205.649051866] [talker]: Publishing: 'Hello World: 1'
[INFO] [1727364206.649026192] [talker]: Publishing: 'Hello World: 2'
[INFO] [1727364207.649036919] [talker]: Publishing: 'Hello World: 3'
[INFO] [1727364208.648988946] [talker]: Publishing: 'Hello World: 4'
[INFO] [1727364209.649195972] [talker]: Publishing: 'Hello World: 5'
...
...
```
このままサンプルプログラムを実行しているシェルを放置して、新たなシェルを開きましょう
<img src="https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/2729782/6096e5f6-093a-3ffd-c381-246aaf7c5702.png" />

　新規 ターミナル で WSL2 Ubuntu にログインし、以下のコマンドを実行してください。
```bash
ros2 topic list
```
　すると、以下のような結果が表示されるでしょう。
```
/chatter
/parameter_events
/rosout
```
`ros2 run demo_nodes_cpp talker` を実行すると新たに **`chatter`** という名前のトピックが発行されているのがわかります。この発行されているトピックでどのようなデータが飛んでいるのか見てみましょう。
　発行されているトピック内のデータを参照するには、以下のコマンドを使います。
 ```
 ros2 topic echo <トピック名>
 ```
 ここでは `chatter` という名前のトピックを見たいので、以下のようなコマンドを実行します。
 　ここ気を付けなければならないのは、**トピック名の指定は必ず文頭に `/` を置く必要があります。** `ros2 topic echo chatter` と実行してもエラーが表示され、トピックを参照することができません。
 ```
 ros2 topic echo /chatter
 ```
 　実行すると、このようにトピック内のデータを見ることができます。
```
data: 'Hello World: 788'
---
data: 'Hello World: 789'
---
data: 'Hello World: 790'
---
data: 'Hello World: 791'
---
data: 'Hello World: 792'
---
data: 'Hello World: 793'
---
...
```
　このようにサンプルプログラムのログと比較するとリアルタイムでサンプルコードが発行しているトピックを受信していることがわかります。
<img src="https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/2729782/5b696aad-6ccc-8c47-08e8-bd3661adc92f.png" />

　このようにサンプルプログラム `talker` のように、トピックを発行するプログラムもといノードのことを <font color="red">**パブリッシャー（ $\text{Publisher}$ ：配信者）**</font> といい、パブリッシャーからのトピックを受信してトピックの中身を参照するプログラムもといノードのことを <font color="blue">**サブスクライバー（ $\text{Subscriber}$ ：購読者）**</font> といいます。
　今回コマンドを使いパブリッシャーからのトピックを参照しましたが、このようなトピックを購読、参照することを <font color="blue">**サブスクライブする（ $\text{Subscribe}$ する）**</font> といいます。一方トピックを配信することを <font color="red">**パブリッシュする（ $\text{Publish}$ する）**</font> といいます。
　今 `talker` というノードを実行してパブリッシャーを起動しています。そして先ほどはパブリッシュされているトピックをサブスクライブ（購読した）しましたよね。今度はパブリッシャーの対となるノード、サブスクライバーを実行してみましょう。サンプルプログラムでは `listener` というサブスクライバーノードがあります。以下のコマンドを **パブリッシャーが起動しているシェル以外のシェル** で実行してみましょう。
```bash
ros2 run demo_nodes_cpp listener
```
　`listener` ノードを実行すると、以下のようなログが表示されます。
```
[INFO] [1727366394.146037015] [listener]: I heard: [Hello World: 2088]
[INFO] [1727366395.146243695] [listener]: I heard: [Hello World: 2089]
[INFO] [1727366396.146081276] [listener]: I heard: [Hello World: 2090]
[INFO] [1727366397.145840356] [listener]: I heard: [Hello World: 2091]
[INFO] [1727366398.146166336] [listener]: I heard: [Hello World: 2092]
[INFO] [1727366399.145575416] [listener]: I heard: [Hello World: 2093]
[INFO] [1727366400.145775802] [listener]: I heard: [Hello World: 2094]
...
```
　このようにパブリッシャーからパブリッシュされているトピックをサブスクライブしていることがわかります。ためしにパブリッシャーノードを止めてみましょう。プログラムを停止するには、プログラムが動いているシェル上でキーボードショートカットキー **「Control + C」** を行います。普段ではこのショートカットキーはコンテンツをコピーするものですが、**プログラムが動作しているシェル上では動作しているプログラムを停止するショートカットとして機能します。**
　このように、「Control + C」を行われたタイミングで **`^C`** と表示され、プログラムが停止することがわかります。
<img src="https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/2729782/a47616e6-b1d2-e526-e534-2bb66993ada5.png" />

　パブリッシャーが停止すると、このようにサブスクライバーの更新も止まります。
<img src="https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/2729782/b952518d-08a7-8466-23b8-74c8d78b4764.png" />

<a id="4-3"></a>
## サンプルプログラムを実行してトピックの通信をしてみよう（その２）
　もうちょっと発展的な作業に取り掛かりましょう。先ほどは `demo_nodes_cpp` というサンプルコードをまとめたパッケージを使い ROS2 の基礎に触れました。このサンプルパッケージ以外にももうひとつ **`turtlesim`（タートル・シム）** というサンプルパッケージがあります。
　以下のコマンドを実行して　**turtlesim** を起動しましょう。
```bash
ros2 run turtlesim turtlesim_node
```
　起動に成功すると、このようにカメが真ん中にいる画面が表示されます。
<img src="https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/2729782/cbbe15d3-d896-695e-283f-eb4ec32c4839.png" />

> [!TIP]
> ひとによって表示されるカメのデザインが変わります。カメのデザインは起動するたびにランダムに変わります。<br>
> <img src="https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/2729782/956313bc-26dd-9d6e-6099-1a02c244059a.png" width=200 /><img src="https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/2729782/0394ee52-3b37-fe05-323a-ef186c768c57.png" width=200 /><img src="https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/2729782/088d641d-3812-d45d-f7f5-593704cef2d7.png" width=200 />

　別のシェルで現在パブリッシュされているトピックを見てみましょう。
```
ros2 topic list
```
すると、新たに **`turtle1`** から始まるトピックがいくつか表示されているのがわかるでしょう。<br>
　ここで問題が発生します。現状 `ros2 topic list` コマンドを実行しただけでは、公開されているトピックがパブリッシュされているトピックなのかサブスクライブされることを期待しているトピックなのかがわかりません。<br>
　どういうことかというと、前回は `talker` というノードを実行したら `chatter` という１つのトピックが公開されました。これは実行してわかる通りこのトピックは **明らかにパブリッシュされているトピック** だとわかります。しかしながら今回のように複数のトピックが飛ぶかう状況では **サブスクライブされることを期待しているトピック** というものが存在します。要はそのトピックを発行しているノードがそのトピックを通じてサブスクライブすることを期待しているわけですね。<br>
 　作業を始める前にこのトピックの関係を特定してみましょう。方法として、現在起動しているノードの情報を取得してなんのトピックをパブリッシュし、なんのトピックをサブスクライブしているかを特定します。<br>
　まず以下のコマンドを実行して現在どんなノードが起動しているのか確認しましょう。
```
ros2 node list
```
　実行するとこのようにノード名が表示されます。
```
/turtlesim
```
　紛らわしいことに `ros2 run turtlesim turtlesim_node` で `turtlesim_node` という名前のノードが表示されるはずですが `turtlesim` と表示されていますね。このようにコマンドで起動するノード名と `ros2 node list` で取得されるノード名が **必ずしも同じ名前であるわけではないので** 注意してください。ただ見てわかる通り明らかに `turtlesim_node` であることは名前の共通点で特定できますね。。。
　現在起動しているノード名を特定したところで以下のコマンドを実行してください。
```
ros2 node info /turtlesim
```
　このコマンドは `turtlesim` という名前のノードの情報を表示するコマンドです。このコマンドを実行すると、指定したノードが **どのようなトピックなどをパブリッシュ、サブスクライブしているか** がわかります。
 このコマンドの実行結果は以下のような構造になっています。
```
/turtlesim
  Subscribers:
    ...
  Publishers:
    ...
  Service Servers:
    ...
  Service Clients:
    ...
  Action Servers:
    ...
  Action Clients:
    ...
```
　今注目してほしいのは **`Subscribers`** と **`Publishers`** という項目です。この２つの項目の部分でなんのトピック名で購読しているのか、なんのトピック名でデータを発行しているのかがわかります。
　以下のノード情報は、前回実行した `talker` ノードの情報です。
```
/talker
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /chatter: std_msgs/msg/String
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /talker/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /talker/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /talker/get_parameters: rcl_interfaces/srv/GetParameters
    /talker/list_parameters: rcl_interfaces/srv/ListParameters
    /talker/set_parameters: rcl_interfaces/srv/SetParameters
    /talker/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```
　見てほしいのは以下の部分です。
```
/talker
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /chatter: std_msgs/msg/String
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
```
　この部分の読み方を説明すると、ここでは

**購読しているトピック**
- /parameter_events

**配信しているトピック**
- /chatter
- /parameter_events
- /rosout

ということを示しています。これら一覧から、前回触れたトピック $\text{/chatter}$ の部分だけ切り抜いてみると、このような情報が書かれています。
```
  Publishers:
    /chatter: std_msgs/msg/String
```
　$\text{/chatter}$ という名前のトピックをパブリッシュしているんだなということはわかりますが、その横にある `std_msgs/msg/String` とは何でしょうか？
　正解はそのトピックの **メッセージタイプ（型）** です。簡単に言うとそのトピックの形式を表していて、 $\text{/chatter}$ というトピックは `std_msgs/msg/String` という形式のデータである、つまり **文字列型** のデータをもつトピックであることを示しています。

> [!TIP]
> $\text{String}$ （ストリング）とは日本語で **"文字列"** を意味します。
> そして $\text{chatter}$ （チャッター）は日本語で **"おしゃべりする人"** を意味します。 $\text{chat}$ の名詞形になりますね。

---

　さて、話を戻して `turtlesim` ノードのサブスクライブトピックを見ると、`/turtle1/cmd_vel` というトピックを購読することがわかるでしょう。このトピックは `geometry_msgs/msg/Twist` というメッセージで構成されているのがわかります。この **`Twist`** 型のトピックは簡単に説明すると **ロボットを移動させるためのトピック** です。つまり、このトピックに値を入れることでロボットを動かすことができるのです。
　以下のコマンドを実行すると、このように画面上のカメが $\text{x}$ 軸方向に移動します。
```
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
```
<img src="https://i.imgur.com/uwmPujg.gif" />

　以下のコマンドのように、`x:` 以降の `1.0` を負の値にしてを実行すると、画面上のカメが $\text{x}$ 軸の負の方向に移動します。
```
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -1.0}}"
```
<img src="https://i.imgur.com/Ql8Q4xz.gif" />

　以下のコマンドのように `x:` を `y:` に変更することで、、このように画面上のカメが $\text{y}$ 軸方向に移動します。
```
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {y: 1.0}}"
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {y: -1.0}}"
```
<img src="https://i.imgur.com/eaCk9r9.gif" />

　このように、`x:` と `y:` を同時に指定してパブリッシュすることもできます。それぞれの値を変えるとカメの動きが変化します。
```
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 1.0}}"
```
<img src="https://i.imgur.com/50uKl6y.gif" />

**`linear`** を **`angular`** に変更し、`z:` に任意の値をいれて実行するとカメが旋回します。
```
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}"
```
<img src="https://i.imgur.com/lhW0cmg.gif" />


**`linear`** と **`angular`** 両方の値を入れて実行するとカメが複数のベクトルに対応した動作を行います。ここでは `linear: x:` と `angular: z:` に値をいれるとカメが円を描きます。。
```
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}"
```
<img src="https://i.imgur.com/ag3rZ9S.gif" />

### `geometry_msgs/msg/Twist` を扱うときに気を付けること
　$\text{geometry_msgs/msg/Twist}$ 形式のトピックにコマンドからデータをパブリッシュするとき、以下のようにフィールドの記述に気を付けてください。**フィールドと値の間に半角スペースがないとエラーになります。**

 - **正しいメッセージの書き方**<br>
    以下のコマンドのようにフィールド `z:` と値 `1.0` の間に半角スペースがあるとうまくいきます。
    ```
    ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}"
    ```
     以下のコマンドのようにフィールド `angular:` と値 `{z: 1.0}}` の間に半角スペースがあるとうまくいきます。
    ```
    ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}"
    ```
 - **正しくないメッセージの書き方**<br>
    以下のコマンドのようにフィールド `z:` と値 `1.0` の間に半角スペースがないとエラーが出ます。
    ```
    ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{angular: {z:1.0}}"
    ```
    以下のコマンドのようにフィールド `angular:` と値 `{z: 1.0}}` の間に半角スペースがないとうまくいきません。
    ```
    ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{angular:{z: 1.0}}"
    ```
    ```
    Failed to populate field: 'Vector3' object has no attribute 'z:1.0'
    ```

<a id="4-4"></a>
## チャレンジしてみよう
　今日学んだことをうまく活用して以下の作業に挑戦してみてください！

- `/turtle1/cmd_vel` トピックをパブリッシュして速度 $\text{0.5}$ で斜め右下へカメを動かす
- `/turtle1/cmd_vel` トピックをパブリッシュして速度 $\text{0.1}$ で反時計回りにカメを動かす。このとき **円を描くこと**。
- トピック `/turtle1/pose` の使い方を調べてみましょう。

---

- [**チュートリアル一覧** に戻る](./toc.md)
- [**次のチュートリアル** に移動する](./tutorial3.md)
