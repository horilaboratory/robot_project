# 7.3. Subscriber を実装しよう

　前章で実装したPublisherに加え、今度はSubscriberを実装してTopicを受信できるようにしましょう。**`sample_subscriber.py`** を新規作成して `/greeting` トピックをサブスクライブする機能を追加します。以下のコマンドを実行してプログラムエディタ VScode を使い sample_subscriber.py を新規作成します。
```bash
code sample_subscriber.py
```

## 必要なモジュールをインポートする
　Subscriberを実装するには、Publisherと同じMessage型をインポートする必要があります。前回は String 型の Message を使った TOpic を Publish し、今回はこれを Subscriber したいので同様の Message 型 `std_msgs/msg/String` をインポートするため、以下のコードを初めに記述しましょう。

```python
from std_msgs.msg import String  # 文字列Message型
import rclpy # rclpy をインポート
from rclpy.node import Node # Node をインポート
```

## Node を作成する
　前回の作業と同じく、新たな Class を作成し、その中で　Node を作成します。

## Subscriber を作成する
　Subscriberを作成するには、`create_subscription()`メソッドを使用します。このメソッドはNodeクラスに組み込まれており、4つの引数を指定します。

```python
self.create_subscription(
    String,                 # 受信するMessage型
    '/greeting',            # サブスクライブするTopic名
    self.greeting_callback, # メッセージ受信時に実行するコールバック関数
    10                      # QoSプロファイル
)
```

### 各引数の詳細説明
1. **Message型**  
   受信するデータの形式を指定します。Publisherで使用した`String`型と一致させる必要があります。

2. **トピック名**  
   サブスクライブするTopic名です。Publisherと同じ`/greeting`を指定します。

3. **コールバック関数**  
   Messageを受信した時に自動的に実行される関数を指定します。ここでは`self.greeting_callback`という名前のメソッドを後ほど定義します。

4. **QoSプロファイル**  
   Publisherと同じ設定（ここでは10）にします。Messageの配送信頼性を制御します。

---

　Subscriberを実装するために、Class SampleSubscriber に新しいメソッドを追加します。

```python
class SampleSubscriber(Node):
    def __init__(self):
        super().__init__('sample_subscriber_node')

        # Subscriberの作成
        self.subscriber = self.create_subscription(
            String,
            '/greeting',
            self.greeting_callback,
            10
        )
        print('/greeting トピックのSubscriberを初期化しました')

    def greeting_callback(self, msg):
        """ メッセージ受信時に実行される処理 """
        print(f'受信したメッセージ: {msg.data}')
```
`greeting_callback` メソッドにはもう一つ `msg` という引数を用意します。最初に書く `self` は Class 内でメソッドとして扱われる関数には必須です。この `msg` 引数に Subscribe した Message データが入ります。

> [!TIP]
> コールバック関数の命名規則（別に必ず追う書かなければならないわけではないがこう書いておくとわかりやすい。）：  
> 一般的に「`on_` + トピック名 + `_received`」のような名前が使われます。  
> 例：`on_image_received`, `on_scan_data`

> [!WARNING]
> コールバック関数の引数には必ず受信したMessageオブジェクトを指定してください。  
> 誤った例：`def greeting_callback():` → エラー発生

## 完全なコード
　Subscriberを実装した完全なコードは以下の通りです。

```python
from std_msgs.msg import String
from rclpy.node import Node
import rclpy

class SampleSubscriber(Node):
    def __init__(self):
        super().__init__('sample_subscriber_node')
        
        # Subscriberの設定
        self.subscriber = self.create_subscription(
            String,
            '/greeting',
            self.greeting_callback,
            10
        )
        print('/greeting トピックのSubscriberを初期化しました')

    
    def greeting_callback(self, msg):
        print(f'受信内容: {msg.data}')

if __name__ == '__main__':
    rclpy.init()
    node = SampleSubscriber()
    rclpy.spin(node)
```

## プログラムを実行してみよう
　編集したプログラムを保存し、以下の手順で動作確認します。

1. **プログラム実行**<br>
    ターミナルで Publisher プログラム `sample_publisher.py` を実行します。
    ```bash
    python3 sample_publisher.py
    ```
    新たなターミナルを開き、そこで今回作成した Subscriber プログラムを実行します。
    ```bash
    python3 sample_subscriber.py
    ```

2. **動作確認** <br>
    プログラムを実行すると、1秒ごとに以下のような出力が表示されます。
    ```bash
    Node sample_subscriber_node を作成しました。
    /greeting トピックのSubscriberを初期化しました
    送信内容: Hello ROS2!
    受信内容: Hello ROS2!
    送信内容: Hello ROS2!
    受信内容: Hello ROS2!
    ...
    ```

3. **外部ツールでの確認（任意）**  
新しいターミナルで以下のコマンドを実行し、Topicの内容を確認できます。
```bash
ros2 topic echo /greeting
```

---

## おさらい
　Subscriber実装の重要なポイントを整理しましょう：

1. **必須モジュール**  
   ```python
   from std_msgs.msg import String
   ```

2. **Subscriberの作成**  
   ```python
   self.create_subscription(Message型, Topic名, コールバック関数, QoS)
   ```

3. **コールバック関数**  
   ```python
   def callback(self, msg):
       print(msg.data)  # 受信データの処理
   ```

### 重要な注意点
- コールバック関数はメッセージ受信時に非同期で実行される
- 重い処理をコールバック関数内で行うとパフォーマンス低下の原因になる

### チャレンジ
以下の課題に挑戦してみましょう。

- 異なる Message を持つ新たなトピックをを Publish、Subscribe するプログラムを実装してみましょう。

---

次のチュートリアル [7.4. Topic を使いロボッからテキストを発話させてみよう](tutorial7.4.md) では、String 型 Message を使いロボットからテキストを発話させるプログラム `robot_speak.py` を実装します。