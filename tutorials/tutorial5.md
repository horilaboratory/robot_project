# 5. ROS2 の用語早見表

| 用語名           | 概要                                                                                                                                         |
|:----------------:|:--------------------------------------------------------------------------------------------------------------------------------------------|
| **ROS**         | Robot Operating Systemの略称。ロボット開発のためのミドルウェアで、通信、制御、センサ処理などの機能を提供。バージョン1系（ROS1）と、改良されたROS2が存在する。<br>[参考：ROS公式サイト](https://www.ros.org/) |
| **ROS2**        | ROS1を基にリアルタイム性、セキュリティ、マルチプラットフォーム対応を強化した次世代版。DDS（Data Distribution Service）を通信ミドルウェアに採用。産業利用や複数ロボット対応を重視している。<br>[参考：ROS2公式サイト](https://www.ros.org/) |
| **ノード（Node）** | ROSにおける基本的な処理単位。各ノードは独立して動作し、他のノードと通信して情報をやり取りする。軽量なプロセスとして複数のノードを分散実行可能。<br>[参考：ROS Wiki - Nodes](https://wiki.ros.org/ja/Nodes) |
| **トピック（Topic）** | ノード間でデータを送受信するための通信チャンネル。Publisherがデータを送信し、Subscriberが受信する。センサデータのストリームなどに適している。<br>[参考：ROS Wiki - Topics](https://wiki.ros.org/ja/Topics) |
| **Publish / Publisher** | トピックに対してデータを送信（発行）する側。センサ値や制御指令などの情報を他のノードに通知する。ノードは複数のトピックに対してPublisherを持てる。<br>[参考：ROS Wiki - Publisher](https://wiki.ros.org/ja/Publisher) |
| **Subscribe / Subscriber** | トピックからデータを受信（購読）する側。他ノードの発行した情報を受け取って処理する。センサデータの受信や、状態監視などに利用される。<br>[参考：ROS Wiki - Subscriber](https://wiki.ros.org/ja/Subscriber) |
| **サービス（Service）** | リクエストとレスポンスによる一対一の通信。ある処理を即時に呼び出し結果を得たいときに使われる。同期的な処理に向いている通信手法。<br>[参考：ROS Wiki - Services](https://wiki.ros.org/ja/Services) |
| **アクション（Action）** | 時間のかかる処理を非同期で実行する仕組み。目標を送信し、フィードバックを受け取りつつ完了を待てる。ナビゲーションなど継続的な処理に利用される。<br>[参考：ROS Wiki - Actions](https://wiki.ros.org/ja/Actions) |
| **DDS（Data Distribution Service）** | ROS2の下位通信層として使われる標準的なPub/Subミドルウェア。QoS設定により信頼性や遅延を制御可能。複数ベンダー（Fast DDS、Cyclone DDSなど）がある。<br>[参考：ROS Wiki - DDS](https://wiki.ros.org/ja/DDS) |
| **Launchファイル** | 複数のノードを一括起動するための設定スクリプト。Pythonベースで柔軟に起動順序や引数を設定できる。実験や運用時に欠かせない機能。<br>[参考：ROS Wiki - Launch](https://wiki.ros.org/ja/Launch) |
| **パッケージ（Package）** | ノードや設定ファイル、ライブラリなどをまとめた単位。ROSのプロジェクトはすべてパッケージ単位で構成される。再利用性や依存関係管理に優れる。<br>[参考：ROS Wiki - Packages](https://wiki.ros.org/ja/Packages) |
| **rclpy / rclcpp** | ROS2のPython（rclpy）およびC++（rclcpp）向けクライアントライブラリ。それぞれの言語でノードや通信処理を記述する際に使用。用途やパフォーマンス要件に応じて使い分ける。<br>[参考：ROS Wiki - rclpy](https://wiki.ros.org/ja/rclpy)、[参考：ROS Wiki - rclcpp](https://wiki.ros.org/ja/rclcpp) |



---

- [**チュートリアル一覧** に戻る](./toc.md)
- [**次のチュートリアル** に移動する](./tutorial6.md)
