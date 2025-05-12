# 5. ROS2 の用語早見表

| 用語名 | 概要 |
|:---:|:---|
| **ROS** | Robot Operating System の略称。<br>ロボット開発のためのミドルウェアで、通信、制御、センサ処理などの機能を提供。<br>バージョン1系（ROS1）と、改良されたROS2が存在する。<br>[参考：ROSとは（ロボットスタート）](https://robotstart.info/2018/03/12/ros-robot-operating-system.html) |
| **ROS2** | ROS1を基にリアルタイム性、セキュリティ、マルチプラットフォーム対応を強化した次世代版。<br>DDS（Data Distribution Service）を通信ミドルウェアに採用。<br>産業利用や複数ロボット対応を重視している。<br>[参考：ROS2とは（Zenn）](https://zenn.dev/karakuri/articles/ros2-intro) |
| **ノード（Node）** | ROSにおける基本的な処理単位。<br>各ノードは独立して動作し、他のノードと通信して情報をやり取りする。<br>軽量なプロセスとして複数のノードを分散実行可能。<br>[参考：ノードとは（Qiita）](https://qiita.com/srs/items/efc3f5b9b1e0d5c9e6c2) |
| **トピック（Topic）** | ノード間でデータを送受信するための通信チャンネル。<br>Publisherがデータを送信し、Subscriberが受信する。<br>センサデータのストリームなどに適している。<br>[参考：トピック通信（Qiita）](https://qiita.com/srs/items/efc3f5b9b1e0d5c9e6c2#%E3%83%88%E3%83%94%E3%83%83%E3%82%AF%E9%80%9A%E4%BF%A1) |
| **Publish / Publisher** | トピックに対してデータを送信（発行）する側。<br>センサ値や制御指令などの情報を他のノードに通知する。<br>ノードは複数のトピックに対してPublisherを持てる。<br>[参考：Publisher/Subscriberの実装例（Zenn）](https://zenn.dev/karakuri/articles/ros2-pub-sub-python) |
| **Subscribe / Subscriber** | トピックからデータを受信（購読）する側。<br>他ノードの発行した情報を受け取って処理する。<br>センサデータの受信や、状態監視などに利用される。<br>[参考：Publisher/Subscriberの実装例（Zenn）](https://zenn.dev/karakuri/articles/ros2-pub-sub-python) |
| **サービス（Service）** | リクエストとレスポンスによる一対一の通信。<br>ある処理を即時に呼び出し結果を得たいときに使われる。<br>同期的な処理に向いている通信手法。<br>[参考：サービス通信（Qiita）](https://qiita.com/karakuri/items/1b7e0d6d3e8a3a8e0d9f) |
| **アクション（Action）** | 時間のかかる処理を非同期で実行する仕組み。<br>目標を送信し、フィードバックを受け取りつつ完了を待てる。<br>ナビゲーションなど継続的な処理に利用される。<br>[参考：アクション通信（Qiita）](https://qiita.com/karakuri/items/1b7e0d6d3e8a3a8e0d9f#%E3%82%A2%E3%82%AF%E3%82%B7%E3%83%A7%E3%83%B3%E9%80%9A%E4%BF%A1) |
| **DDS（Data Distribution Service）** | ROS2の下位通信層として使われる標準的なPub/Subミドルウェア。<br>QoS設定により信頼性や遅延を制御可能。<br>複数ベンダー（Fast DDS、Cyclone DDSなど）がある。<br>[参考：DDSとは（Zenn）](https://zenn.dev/karakuri/articles/ros2-dds) |
| **Launchファイル** | 複数のノードを一括起動するための設定スクリプト。<br>Pythonベースで柔軟に起動順序や引数を設定できる。<br>実験や運用時に欠かせない機能。<br>[参考：Launchファイルの使い方（Qiita）](https://qiita.com/karakuri/items/1b7e0d6d3e8a3a8e0d9f#launch%E3%83%95%E3%82%A1%E3%82%A4%E3%83%AB) |
| **パッケージ（Package）** | ノードや設定ファイル、ライブラリなどをまとめた単位。<br>ROSのプロジェクトはすべてパッケージ単位で構成される。<br>再利用性や依存関係管理に優れる。<br>[参考：パッケージの作成（Zenn）](https://zenn.dev/karakuri/articles/ros2-package) |
| **rclpy / rclcpp** | ROS2のPython（rclpy）およびC++（rclcpp）向けクライアントライブラリ。<br>それぞれの言語でノードや通信処理を記述する際に使用。<br>用途やパフォーマンス要件に応じて使い分ける。<br>[参考：rclpy/rclcppとは（Zenn）](https://zenn.dev/karakuri/articles/ros2-pub-sub-python) |




---

- [**チュートリアル一覧** に戻る](./toc.md)
- [**次のチュートリアル** に移動する](./tutorial6.md)
