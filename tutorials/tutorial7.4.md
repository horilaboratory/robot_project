# 7.4. Topic を使いロボットからテキストを発話させてみよう

　本チュートリアルでは、**std_msgs/msg/String** 型のメッセージをトピック **`/er_kachaka/kachaka_speak`** にパブリッシュすることで、ロボットにテキストを発話させるプログラム **`robot_speak.py`** の実装方法を解説します。ROS2のPublisher機能を活用し、ロボット制御の実践的な応用例を学びましょう。

---

## 実装手順

### 1. プログラムファイルの作成
　以下のコマンドを実行し、新しいPythonファイルを作成します。
```bash
code robot_speak.py
```

---

## 必要なモジュールをインポートする
　String型メッセージとROS2ノード機能をインポートします。
```python
from std_msgs.msg import String  # 文字列メッセージ型
import rclpy                     # ROS2 Pythonクライアント
from rclpy.node import Node      # ノード作成用クラス
```

---

## Nodeクラスの定義
　ロボット発話機能を担う `RobotSpeaker` クラスを作成します。`create_timer` を使い一定周期でテキストを Publish する機能を実装します。今夏は発話後にブランクを開けるため１０秒間隔で Publish されるようにします。
```python
class RobotSpeaker(Node):
    def __init__(self):
        super().__init__('robot_speaker_node')  # ノード名を設定
        
        # Publisherの作成
        self.speak_publisher = self.create_publisher(
            String,                        # 使用するメッセージ型
            '/er_kachaka/kachaka_speak',   # 発話用トピック名
            10                             # QoS設定
        )
        
        # タイマー設定（2秒間隔で発話）
        self.create_timer(10.0, self.speak_callback)
        print("発話ノードが起動しました！")
```

---

## 発話処理の実装
　コールバック関数でメッセージを作成・送信します。
```python
    def speak_callback(self):
        """ 定期発話処理 """
        try:
            # 1. メッセージオブジェクト作成
            msg = String()
            
            # 2. 発話内容を設定
            msg.data = "こんにちは、ROS2で発話しています"
            
            # 3. パブリッシュ実行
            self.speak_publisher.publish(msg)
            print(f"発話内容: {msg.data}")
            
        except Exception as e:
            self.get_logger().error(f"エラーが発生しました: {str(e)}")
```

> [!WARNING]
> **重要な注意点**  
> - トピック名は **`/er_kachaka/kachaka_speak`** と正確に一致させる
> - メッセージ型は `std_msgs/msg/String` を使用する
> - ロボット側でトピックをSubscribeする機能が実装されている必要あり

---

## 完全な実装コード
```python
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class RobotSpeaker(Node):
    def __init__(self):
        super().__init__('robot_speaker_node')
        
        # Publisherの初期化
        self.speak_publisher = self.create_publisher(
            String, 
            '/er_kachaka/kachaka_speak', 
            10
        )
        
        # 10秒間隔で発話
        self.timer = self.create_timer(10.0, self.speak_callback)
        print("ロボット発話ノード起動！")

    def speak_callback(self):
        msg = String()
        msg.data = "こんにちは、ROS2で発話しています"
        self.speak_publisher.publish(msg)
        print(f"発話内容: {msg.data}")

if __name__ == '__main__':
    rclpy.init()
    node = RobotSpeaker()
    rclpy.spin(node)
```

---

## 動作確認方法

### 0. erasers_kachaka の起動
 Kachaka をコンピューターに接続し、[接続方法](tutorial4.md) を参考に erasers_kachaka を起動してください。

### 1. プログラムの実行
　新しいターミナルで以下を実行します。
```bash
python3 robot_speak.py
```

### 2. 期待される出力
メッセージと一緒に Kachaka から指定したテキストが発話されます。
```bash
ロボット発話ノード起動！
発話内容: こんにちは、ROS2で発話しています
発話内容: こんにちは、ROS2で発話しています
...（10秒間隔で継続）
```

---

## 応用機能の実装例
### 対話型発話
　ユーザー入力に応じて発話内容を変更する例。
```python
def speak_callback(self):
    text = input("発話内容を入力 > ")
    msg = String()
    msg.data = text
    self.speak_publisher.publish(msg)
```

---

## トラブルシューティング
| 現象 | 対処法 |
|------|--------|
| ロボットが反応しない | erasers_kachaka を起動したか |
| メッセージ型不一致エラー | `ros2 topic info /er_kachaka/kachaka_speak` で型を確認 |

---

## おさらい
- **Publisher作成の3要素**
  1. `String` 型メッセージ
  2. `/er_kachaka/kachaka_speak` トピック
  3. QoSプロファイル（10）

- **動作フロー**
  1. メッセージオブジェクト作成
  2. `.data` にテキスト設定
  3. `publish()` で送信

### チャレンジ
- 発話内容を「今日の天気は晴れです」に変更

---

次のチュートリアル [7.5. カメラ画像を処理しよう](tutorial7.5.md) では、sensor_msgs/msg/Image 型を使った画像処理ノードの実装方法を学びます。