# 7.2. Publisher を実装しよう

## Publisherとは
　Publisher（パブリッシャー）はROS2ノードがトピックにMessageを送信（パブリッシュ）するための機能です。前章で作成したノードにPublisher機能を追加することで、他のノードへデータを共有することができるようになります。ここでは実践前としてテキストデータを Message として Topic `/greeting` へ Publish するコードを記述する方法を解説します。

## Message型のインポート
　まず、送信するMessageの型をインポートします。ここでは文字列を送信するため、`std_msgs/msg/String`型を使用します。

```python
from std_msgs.msg import String  # 文字列Message型
```

> [!NOTE]
> `std_msgs`（スタンダード・メッセージズ）はROS2の標準Messageパッケージで、基本的なデータ型が定義されています。  
> 他にも`Int32`（32bit整数）や`Float64`（64bit浮動小数点数）など様々な型があります。

## Publisherの作成方法
　Publisherを作成するには、`create_publisher()`メソッドを使用します。このメソッドはNodeクラスに組み込まれており、3つの引数を指定します。

```python
self.create_publisher(
    Message型,   # 送信するデータの型（例: String）
    トピック名,    # Messageを送信するチャンネル名
    QoSプロファイル # Messageの配送保証設定
)
```

### 各引数の詳細説明
1. **Message型**  
   送信するデータの形式を指定します。ここでは`String`型を指定しています。

2. **トピック名**  
   Messageを送信するチャンネル名です。スラッシュ(`/`)で始まる必要があります。

3. **QoSプロファイル**  
   Quality of Serviceの略で、Messageの配送信頼性を設定します。  
   - **10**：標準設定（Messageの最大キューサイズ）
   - 詳細設定も可能ですが、初心者はまず10で問題ありません

## 実際の実装コード
　これらの知識をもとに、Publisherを実装してみましょう。

```python
class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        print('Node robot_control_node を作成しました。')
        
        # Publisherの作成
        self.publisher = self.create_publisher(
            String,         # 文字列Message型
            '/greeting',    # トピック名
            10             # QoSプロファイル（キューサイズ）
        )
        print('/greeting トピックのPublisherを初期化しました')
```

> [!TIP]
> QoSプロファイルについて：  
> ロボット制御など信頼性が重要な場合は、`Reliable`（確実配送）や`TransientLocal`（最新Message保持）などの高度な設定も可能です。  
> ただし、基本的な使い方ではキューサイズ（ここでは10）のみの指定で十分です。


## Message 送信処理を実装する

### Message送信の全体フロー
　ROS2でMessageを送信するには、以下の3ステップが必要です：

1. **Messageオブジェクトの作成**
2. **データの設定**
3. **パブリッシュ実行**

### 1. Messageオブジェクトの作成
　まず、送信するMessageの「空の容器」を作成します。`String`型の場合：

```python
msg = String()  # 空のStringMessageを作成
```

> [!NOTE]
> 各Message型には専用のコンストラクタがあります。  
> 例：  
> - `Int32()` → 整数型  
> - `Twist()` → 速度指令型  
> - `Image()` → 画像データ型

### 2. データの設定
　作成したMessageオブジェクトに実際のデータを設定します。`String`型の場合は`.data`属性を使用：

```python
msg.data = 'Hello ROS2!'  # 文字列データを設定
```

#### 主要Message型のデータ設定例
| 型 | 設定方法 | 例 |
|----|---------|----|
| String | msg.data | `msg.data = "text"` |
| Int32 | msg.data | `msg.data = 100` |
| Float64 | msg.data | `msg.data = 3.14` |
| Bool | msg.data | `msg.data = True` |
| Twist | msg.linear.x | `msg.linear.x = 0.5` |

### 3. パブリッシュ実行
　設定済みのMessageを実際に送信します。以下のコードによって msg 内部で設定した Message データが `create_publisher()` で指定したトピックへ Publish されます。

```python
self.publisher.publish(msg)  # Messageを送信
```

### 完全な送信処理の実装例
　前節までのPublisherとタイマーと組み合わせた完全なコード：

```python
def publish_greeting(self):
    """ Message送信処理 """
    try:
        # 1. Messageオブジェクト作成
        msg = String()
        
        # 2. データ設定
        msg.data = 'Hello ROS2!'
        
        # 3. パブリッシュ実行
        self.publisher.publish(msg)
        
        # ログ出力
        print('Message をパブリッシュしました。')
        
    except Exception as e:
        print('エラー発生')
        print(e)
```

### よくあるエラーと対処法

1. **AttributeError（属性エラー）**  
   ```python
   AttributeError: 'String' object has no attribute 'value'
   ```
   → Message型の属性名を間違えている（正しくは`.data`）

2. **TypeError（型エラー）**  
   ```python
   TypeError: field data must be of type str
   ```
   → 指定したデータ型が不正（文字列型が必要なところに数値を指定など）

3. **Publisher未初期化エラー**  
   ```python
   RuntimeError: publisher is not initialized
   ```
   → `create_publisher()`する前に`publish()`を呼び出している

## 定期的な送信の設定
### タイマー機能とは
　ROS2ノードで定期的に処理を実行するには**タイマー**を使用します。タイマーは指定した間隔で特定の関数を自動的に呼び出す仕組みで、センサーデータの定期的な送信や制御コマンドの発行などに活用されます。

### create_timer()メソッド
　タイマーを作成するには`create_timer()`メソッドを使用します。このメソッドはNodeクラスに組み込まれており、2つの主要な引数を取ります。

```python
self.create_timer(
    間隔秒数,      # 実行間隔（秒単位）
    コールバック関数 # 定期実行する関数
)
```

#### 引数詳細説明
1. **間隔秒数**  
   - 処理を実行する間隔を秒単位で指定
   - 1.0 = 1秒間隔、0.5 = 0.5秒（500ms）間隔
   - 浮動小数点数で精密な設定が可能

2. **コールバック関数**  
   - 間隔ごとに実行したい関数を指定
   - 引数なしの関数を指定する（`self.function_name`形式）
   - 通常はクラス内のメソッドを指定

### 実際の実装例
　先ほど作成した`publish_greeting()`メソッドを1秒間隔で実行する設定例です。

```python
class RobotControl(Node):
    def __init__(self):
        # ...（前回のPublisher作成コード...）
        
        # タイマー設定（1秒間隔でpublish_greetingを実行）
        self.timer = self.create_timer(
            1.0,                   # 1秒間隔
            self.publish_greeting   # 実行するメソッド
        )
        print('1秒間隔でMessageを送信します')
```

> [!WARNING]
> **タイマーの注意点**  
> - タイマーはノードが実行中（`spin()`中）のみ動作
> - コールバック関数の処理時間が間隔時間を超えると、次の実行が遅れる
> - 高頻度（0.1秒以下）のタイマーはシステム負荷を考慮する必要あり

### タイマーのライフサイクル
　作成されたタイマーは以下のように管理されます：

1. **作成時**  
   ```mermaid
   graph LR
     A[create_timer呼び出し] --> B[内部タイマー登録]
     B --> C[次回実行時間計算]
   ```

2. **実行時**  
   ```mermaid
   graph LR
     D[時間経過] --> E[コールバック関数実行]
     E --> F[次回実行時間再計算]
   ```

3. **終了時**  
   ```mermaid
   graph LR
     G[ノード終了] --> H[全タイマー自動破棄]
   ```

## 完全なコード
```python
from rclpy.node import Node
import rclpy
from std_msgs.msg import String

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        print('Node robot_control_node を作成しました。')
        
        self.publisher = self.create_publisher(String, '/greeting', 10)
        print('/greeting トピックのPublisherを初期化しました')
        
        self.timer = self.create_timer(1.0, self.publish_greeting)
        print('1秒間隔でMessageを送信します')
    
    def publish_greeting(self):
        msg = String()
        msg.data = 'Hello ROS2!'
        self.publisher.publish(msg)
        print('送信内容:', msg.data)

if __name__ == '__main__':
    rclpy.init()
    node = RobotControl()
    rclpy.spin(node)
```

## 動作確認方法

1. **プログラム実行**
```bash
python3 robot_control.py
```

2. **トピック確認（別ターミナル）**
```bash
ros2 topic list
ros2 topic echo /greeting
```

## おさらい
- `create_publisher()`でPublisherを作成
- `create_timer()`で定期的な送信を実現
- ROS2の基本Message型は`std_msgs`モジュールに含まれる