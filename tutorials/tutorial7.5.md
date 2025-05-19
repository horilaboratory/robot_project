# 7.5. カメラ画像を取得して表示しよう

　本チュートリアルでは、**/er_kachaka/front_camera/image_raw** トピックをサブスクライブし、OpenCVとCvBridgeを使用してカメラ画像を表示する方法を解説します。BEST_EFFORT QoSプロファイルの設定方法や画像処理の基本フローを学びましょう。

---

## 事前準備
以下のコマンドをターミナル上で実行し、画像処理用ライブラリ「Open-CV」をインストールしてください。
```bash
pip install opencv-python
```

## モジュールインポート
　まずは Node を作成するために必要な以下のモジュールをインポートしましょう。
```python
import rclpy
from rclpy.node import Node
```
次に、ここでは初登場である新たな Message 「Image」を扱うためのパッケージをインポートします。
```python
from sensor_msgs.msg import Image
```

### 画像メッセージ型 (`Image`)
- **役割**: ROS2標準の画像データ形式
- **必要性**:
  - カメラトピックのメッセージ型として使用
  - 画像データのヘッダ情報（タイムスタンプ、フレームID）を含む
  - エンコーディング形式（RGB, BGR, グレースケール等）を指定可能
- **データ構造**:
  ```python
  # 主なメンバ変数
  header.stamp       # 画像取得時刻
  header.frame_id    # カメラフレーム名
  height             # 画像高さ（ピクセル）
  width              # 画像幅（ピクセル）
  encoding           # データ形式（'bgr8'等）
  data               # 生ピクセルデータ
  ```

次に、ROS2 の Image Message データと OpenCV で使用する配列データを相互変換するために必要なライブラリ `CvBrudge` をインポートします。
```python
from cv_bridge import CvBridge
```

### CvBridge
- **役割**: ROSメッセージ⇔OpenCV画像の相互変換
- **必要性**:
  - `sensor_msgs/Image`をOpenCVが扱える`numpy.ndarray`形式に変換
  - 色空間の自動変換（RGB→BGR等）
  - エンコーディングエラーの検出
- **変換例**:
  ```python
  bridge = CvBridge()
  cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')  # ROS→OpenCV
  ros_image = bridge.cv2_to_imgmsg(cv_image, 'bgr8')  # OpenCV→ROS
  ```

そして OpenCV をインポートします。
```python
import cv2
```

### OpenCV (`cv2`)
- **役割**: 画像処理ライブラリ
- **必要性**:
  - 画像表示（`imshow()`）
  - 基本的な画像処理（フィルタリング、色変換等）
  - ウィンドウ管理
- **主要機能**:
  ```python
  cv2.imshow()      # 画像表示
  cv2.waitKey()     # キー入力待ち
  cv2.cvtColor()    # 色空間変換
  cv2.Canny()       # エッジ検出
  ```

> [!TIP]
> **モジュール間のデータフロー**  
> ```mermaid
> graph LR
>   A[ROS Imageメッセージ] -->|CvBridge| B[OpenCV画像]
>   B --> C[画像処理]
>   C --> D[画面表示/保存]
> ```

### なぜこれら全てが必要か？
1. **データ形式変換** (`CvBridge`)  
   ROSの画像形式とOpenCVの画像形式は異なるため、相互変換が必要

2. **画像処理/表示** (`OpenCV`)  
   受信した画像データを人間が確認可能な形式で表示

> [!WARNING]
> モジュール不足時のエラー例：  
> - `ImportError: No module named 'cv_bridge'`  
>   → `sudo apt install ros-<humble-cv-bridge`で解決  

---

## Nodeの定義

　カメラ画像を処理するためのノードクラス定義について、Class `CameraViewer` を作成します。
```python
class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')
```

### CvBridgeの初期化
　`__init__` 内で `self.bridge = CvBridge()` を追記してください。このように CvBridge を初期化して CvBridge を利用できるようにします。
```python
class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')

        # CvBridge の初期化
        self.bridge = CvBridge()
```

### 3. QoSプロファイル設定
次に以下のコードを `__init__` 内に追記します。
```python
        # BEST_EFFORT QoSプロファイル設定
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,  # メッセージキューサイズ
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT # BEST_EFFORT を設定
        )
```
- **Bなぜこの処理を書くのか**:
  1. **送信側の仕様整合性**  
     ロボットが`/er_kachaka/front_camera/image_raw`トピックをBEST_EFFORTでPublishしているため、受信側も同じプロファイルを使用する必要があります。ROS2ではPublisherとSubscriberのQoSプロファイルが一致しない場合、通信が確立されません。

  2. **カメラデータの特性**  
     高頻度で送られる画像データは、一部のフレーム欠落が許容される代わりに、低遅延が求められます。RELIABLEプロファイルでは再送処理が発生し、リアルタイム性が損なわれる可能性があります。

  3. **リソース効率**  
     BEST_EFFORTはバッファ使用量が少なく、システム負荷を軽減できます。

- **パラメータ詳細**:
  | パラメータ | 値 | 説明 |
  |-----------|----|------|
  | depth | 10 | 受信バッファのサイズ（メッセージ数） |
  | reliability | BEST_EFFORT | メッセージの到達保証なし |


### サブスクライバー作成
次に画像 Topic を Subscribe するための Subscriber を作成します。この時、QoS プロファイルに先ほど定義した変数 `qos_profile` を指定します。
```python
        # カメラサブスクライバーの作成
        self.subscription = self.create_subscription(
            Image,  # メッセージ型
            '/er_kachaka/front_camera/image_raw',  # トピック名
            self.image_callback,  # コールバック関数
            qos_profile  # QoS設定
        )
```
- **引数の意味**:
  1. **`Image`**  
     sensor_msgs/msg/Image型を指定。カメラ画像の標準メッセージ型です。

  2. **トピック名**  
     完全一致が必要。誤って`/front_camera/image_raw`などと省略すると受信できません。

  3. **コールバック関数**  
     メッセージ到着時に呼び出される関数。後述の`image_callback`メソッドを指定。

  4. **QoSプロファイル**  
     前述のBEST_EFFORT設定を適用。

> [!NOTE]
> **QoSプロファイル不一致時の挙動**  
> 以下の場合、通信は確立されません（`ros2 topic info`で接続ノードが表示されない）。
> - Publisher: RELIABLE / Subscriber: BEST_EFFORT
> - Publisher: BEST_EFFORT / Subscriber: RELIABLE

---

## 技術的補足：BEST_EFFORTの適応領域
| データタイプ | 推奨QoS | 理由 |
|-------------|---------|------|
| カメラ画像 | BEST_EFFORT | 高頻度・低遅延が優先 |
| 制御指令 | RELIABLE | データ欠落が危険 |
| センサデータ | SYSTEM_DEFAULT | ケースバイケース |

**BEST_EFFORTが適切なケース**:
- 30FPS以上の高フレームレートカメラ
- UDP通信を利用するネットワークカメラ
- 画像処理AIの入力データなど、一部フレーム欠落が許容される場合


`__init__` 処理がこのようになっていることを確認してください。
```python
class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')
        
        # CvBridgeの初期化（ROS⇔OpenCV変換用）
        self.bridge = CvBridge()
        
        # BEST_EFFORT QoSプロファイル設定
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # カメラサブスクライバーの作成
        self.subscription = self.create_subscription(
            Image,
            '/er_kachaka/front_camera/image_raw',
            self.image_callback,
            qos_profile
        )
        print("カメラビューアーが起動しました！")
```

## コールバック関数の実装
create_subscription 内で定義したコールバック関数 `image_callback` を作成します。サブスクライブした msg データを CvBridge を使い OpenCV が解釈可能なデータ `cv_image` に変換し、このデータを画像ウィンドウとして表示します。
```python
    def image_callback(self, msg):
        """ 画像受信時の処理 """
        try:
            # ROSメッセージ→OpenCV画像変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 画像表示
            cv2.imshow('Camera View', cv_image)
            cv2.waitKey(1)  # 描画更新
            
        except Exception as e:
            self.get_logger().error(f'画像処理エラー: {str(e)}')
```

## 完全な実装コード
以下がこのチュートリアルで解説したコードの全体図です。
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')
        
        self.bridge = CvBridge()
        
        # QoSプロファイル設定
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        )
        
        self.subscription = self.create_subscription(
            Image,
            '/er_kachaka/front_camera/image_raw',
            self.image_callback,
            qos_profile
        )
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Front Camera', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


if __name__ == '__main__':
    rclpy.init(args=args)
    node = CameraViewer()
    rclpy.spin(node)
```

---

## 動作確認手順
### 1. プログラム実行
```bash
python3 camera_viewer.py
```

### 2. 期待される動作
- 新しいウィンドウが開き、リアルタイムでカメラ画像が表示
- 終了時はCtrl+Cで終了

---

## CvBridgeの変換オプション
| エンコーディング | 用途 |
|------------------|------|
| bgr8 | OpenCV標準の色順序 |
| mono8 | グレースケール変換 |
| passthrough | 元データを保持 |

---

## 応用例
### 画像処理パイプライン
コールバック関数ないで画像処理のコードを追記することでアブスクライブした画像データの処理を行うことができます。ここではサブスクライブした画像データをグレイスケールに変換します。
```python
def image_callback(self, msg):
    cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    # グレースケール変換
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
    # エッジ検出
    edges = cv2.Canny(gray, 100, 200)
    
    cv2.imshow('Processed View', edges)
```

---

## おさらい
- **必須設定**
  - CvBridgeによる画像変換
  - BEST_EFFORT QoSプロファイル
  - OpenCVのウィンドウ管理

- **重要メソッド**
  - `imgmsg_to_cv2()`: ROS→OpenCV変換
  - `imshow()`: 画像表示

### チャレンジ
- 画像にタイムスタンプを表示
- 動体検知機能を追加
- 画像をファイル保存する機能を実装