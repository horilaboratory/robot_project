# 7.6. LiDAR センサーから距離データを取得しよう

　本チュートリアルではkachakaに搭載された LiDAR センサーのデータを取得して前方の障害物までの距離を測定する方法を解説します．

## LiDAR（ライダー）とは
　LiDAR（ $\text{Light Detection and Ranging}$ ）は **レーザー光を照射し，その反射光から対象物までの距離や形状を測定する技術** を使ったセンサーです．
自律移動ロボットや自動運転車両に使われる周囲の障害物などの環境をスキャンするために使われます．

## Kachaka の LiDAR センサーの仕様
　Kachaka には本体前面上部にある丸いリング内に LiDAR センサーが内蔵されています．

<img width=50% src='/imgs/kachaka_lidar.png'/>

Kachaka からの LiDAR センサー情報は ROS2 トピック `/er_kachaka/lidar/scan` という名前で `sensor_msgs/msg/LaserScan` というメッセージ型でパブリッシュされます．
視覚化ツール Rviz2 で確認すると，このようにロボット周囲の障害物をパーティクルとして表示してくれます．

<img width=80% src='/imgs/kachaka_lidar_scan.png'/>

　Kachaka の LiDAR センサーの仕様は少し独特て，通常の LiDAR センサーは一定の量のデータをスキャンしますが，こちらは状況に応じて取得データ量が変動します．

## Python コードでLiDAR センサーのデータを取得する方法
　Python コードを使い LiDAR センサーからの情報を取得してみましょう．LiDAR トピックをサブスクライブするワークフローは **Subscriber ノードの作成と全く同じ** です．
しかし気をつけなければならないのは今回取得するデータのメッセージ型が `sensor_msgs/msg/LaserScan` であることです．これから作成するコードに以下のように必要なモジュールと `LaserScan` メッセージをインポートしましょう．
```python
from rclpy.node import Node
import rclpy

# LiDAR センサーからのデータを取得するためのメッセージ
from sensor_msgs.msg import LaserScan
```
　以下のコードを参考にトピック名とメッセージ型を入れ替えてデータをサブスクライブできるようにしましょう．
```python
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
import rclpy

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('sample_subscriber_node')

        # 正しい QoS プロファイルを定義しないとsサブスクライブできない
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,  # メッセージキューサイズ
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT # BEST_EFFORT を設定
        )
        
        # Subscriberの設定
        self.subscriber = self.create_subscription(
            メッセージ型,
            'トピック名',
            self.sensor_callback,
            qos_profile
        )

    
    def sensor_callback(self, msg):
        print(f'受信内容: {msg}')


if __name__ == '__main__':
    rclpy.init()
    node = LidarSubscriber()
    rclpy.spin(node)
```
　これで LiDAR センサーからの情報を取得することができます．

## 前方障害物までの距離を測定するサンプルコードを作る
　LiDAR センサーからデータを取得できたとはいえ，そのデータの中身が理解できないと今後に活かせません．まずは `LaserScan` メッセージの中身を理解しましょう．
`LaserScan` トピックは次のように構成されています．

- **`header`** : メッセージのヘッダー情報
    - **`stamp`** : このデータが測定された時刻
    - **`frame_id`** : データがどの座標系に基づいているかを示す名前．簡単にいうとこのデータを飛ばしているロボットのパーツ名
- **`angle_min`** : スキャンの開始角度（ラジアン）
- **`angle_max`** : スキャンの終了角度（ラジアン）
- **`angle_increment`** : 各測定点間の角度（ラジアン）
- **`ranges`** : 各角度に対応する距離データの配列（メートル）．**この配列が最も重要**

`ranges` データのみ取得すると取得した LiDAR センサーからのすべての測定距離データが入っています．以下のように書き換えると測定データのみ取得できます．
```python
    def sensor_callback(self, msg):
        print(f'受信内容: {msg.ranges}')
```
　このままだと大量のデータが表示されてしまい，**どれがロボットの正面までの距離なのか** がわかりませんね．そこである特定の目標角度 $θ_\text{target}$ に対応する距離データが、 $\text{ranges}$ 配列の何番目のインデックス $i$ に格納されているかを求めます。

　`LaserScan` のメッセージ定義からインデックス $i$ 番目の測定点の角度 $θ_i$は、以下の一般式で表すことができます。

$`
θ_i = \text{angle\_min} + i \times \text{angle\_increment}
`$

この式は、インデックスが $0$ のときの角度が $`\text{angle\_min}`$ であり、インデックスが1つ増えるごとに $`\text{angle\_increment}`$ ずつ角度が増加することを示しています．

目標は、この角度 $θ_i$ が目標角度 $θ_\text{target}$ に一致するようなインデックス $i$ を見つけることです。つまり、

$`
θ_\text{target} = \text{angle\_min} + i \times \text{angle\_increment}
`$

となる $i$ を求めます。つまりこのようになります．

$`
i = \frac{θ_\text{target} - \text{angle\_min}}{\text{angle\_increment}}
`$

この式の $θ_\text{target}$ にロボット前方方向へ向く角度を指定することで前方の障害物を取得することができます．ここでまた　Kachaka の仕様について話を戻しますが，Kachaka の LiDAR センサーは Kachaka 本体を前面にした時 $90$ 度反時計回りに向いています．
そのためロボット前方の障害物を見るためには $θ_\text{target}$ が $-1.57$ ラジアンである必要があります．以下のコードがそれを元に作成したプログラムです．
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('sample_subscriber_node')

        # 正しい QoS プロファイルを定義しないとsサブスクライブできない
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,  # メッセージキューサイズ
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT # BEST_EFFORT を設定
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/er_kachaka/lidar/scan',
            self.scan_callback,
            10)

    def scan_callback(self, msg: LaserScan):
        # angle_incrementが0だとゼロ除算エラーになるためチェック
        if msg.angle_increment == 0.0:
            return

        # 数式 i = (θ_target - angle_min) / angle_increment を使って
        # 正面(θ_target = 0)のインデックスを動的に計算する。
        #
        # θ_target = -1.57 を代入すると、 i = -angle_min / angle_increment となる。
        # 計算結果を最も近い整数に丸めるためにround()を使用する。
        forward_index = int(round(-1.57 - msg.angle_min / msg.angle_increment))
        
        # 計算したインデックスがranges配列の有効な範囲内にあるか確認
        if forward_index < 0 or forward_index >= len(msg.ranges):
            self.get_logger().warn(
                f"Calculated forward index [{forward_index}] is out of bounds "
                f"for ranges array size [{len(msg.ranges)}].")
            return

        # 計算したインデックスを使って、正面の距離データを取得
        distance_forward = msg.ranges[forward_index]

        # 結果をログに出力
        # 距離がinf（無限遠）やnan（非数）の場合も考慮する．データが取れないとそういう表示になる．
        # inf はデータはあるが測定距離が無限大，つまり計測範囲内に障害物がないということ．
        # nan はデータが損失していることで表示される．なぜなら QoS プロファイルが 高頻度・低遅延が優先 でありデータ品質を担保しないからである．
        if np.isinf(distance_forward):
            self.get_logger().info('Forward distance: out of range (inf)')
        elif np.isnan(distance_forward):
            self.get_logger().info('Forward distance: invalid (NaN)')
        else:
            # .2f は小数点以下2桁まで表示するフォーマット
            self.get_logger().info(f'Forward distance: {distance_forward:.2f} [m]')

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector_node = ObstacleDetector()
    try:
        rclpy.spin(obstacle_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## チャレンジ
- 上記のサンプルコードでは１点のみの距離しか取っていません．取得範囲数を広げて取得範囲内の障害物の最低値を取得して，前方の障害物の認識制度をあげてみよう．
- 取得した障害物までの距離情報をもとに車輪を動かして障害物回避を実装してみよう．
