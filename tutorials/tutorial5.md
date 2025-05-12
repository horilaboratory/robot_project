# 5. ROS2 の用語早見表

| 用語名 | 概要 |
|:---:|:---|
| **ROS** | Robot Operating System の略称。<br>ロボット開発のためのミドルウェアで、通信、制御、センサ処理などの機能を提供。<br>バージョン1系（ROS1）と、改良されたROS2が存在する。<br>[参考：ROS公式サイト](https://www.ros.org/) |
| **ROS2** | ROS1を基にリアルタイム性、セキュリティ、マルチプラットフォーム対応を強化した次世代版。<br>DDS（Data Distribution Service）を通信ミドルウェアに採用。<br>産業利用や複数ロボット対応を重視している。<br>[参考：ROS2公式概要](https://docs.ros.org/en/humble/) |
| **ノード（Node）** | ROSにおける基本的な処理単位。<br>各ノードは独立して動作し、他のノードと通信して情報をやり取りする。<br>軽量なプロセスとして複数のノードを分散実行可能。<br>[参考：Nodes - ROS 2 Docs](https://docs.ros.org/en/humble/Concepts/Nodes.html) |
| **トピック（Topic）** | ノード間でデータを送受信するための通信チャンネル。<br>Publisherがデータを送信し、Subscriberが受信する。<br>センサデータのストリームなどに適している。<br>[参考：Topics - ROS 2 Docs](https://docs.ros.org/en/humble/Concepts/Topics.html) |
| **Publish / Publisher** | トピックに対してデータを送信（発行）する側。<br>センサ値や制御指令などの情報を他のノードに通知する。<br>ノードは複数のトピックに対してPublisherを持てる。<br>[参考：Publisher - ROS 2 Docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) |
| **Subscribe / Subscriber** | トピックからデータを受信（購読）する側。<br>他ノードの発行した情報を受け取って処理する。<br>センサデータの受信や、状態監視などに利用される。<br>[参考：Subscriber - ROS 2 Docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) |
| **サービス（Service）** | リクエストとレスポンスによる一対一の通信。<br>ある処理を即時に呼び出し結果を得たいときに使われる。<br>同期的な処理に向いている通信手法。<br>[参考：Services - ROS 2 Docs](https://docs.ros.org/en/humble/Concepts/Services.html) |
| **アクション（Action）** | 時間のかかる処理を非同期で実行する仕組み。<br>目標を送信し、フィードバックを受け取りつつ完了を待てる。<br>ナビゲーションなど継続的な処理に利用される。<br>[参考：Actions - ROS 2 Docs](https://docs.ros.org/en/humble/Concepts/Actions.html) |
| **DDS（Data Distribution Service）** | ROS2の下位通信層として使われる標準的なPub/Subミドルウェア。<br>QoS設定により信頼性や遅延を制御可能。<br>複数ベンダー（Fast DDS、Cyclone DDSなど）がある。<br>[参考：ROS2とDDS](https://design.ros2.org/articles/ros_on_dds.html) |
| **Launchファイル** | 複数のノードを一括起動するための設定スクリプト。<br>Pythonベースで柔軟に起動順序や引数を設定できる。<br>実験や運用時に欠かせない機能。<br>[参考：Launch files - ROS 2 Docs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html) |
| **パッケージ（Package）** | ノードや設定ファイル、ライブラリなどをまとめた単位。<br>ROSのプロジェクトはすべてパッケージ単位で構成される。<br>再利用性や依存関係管理に優れる。<br>[参考：Packages - ROS 2 Docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) |
| **rclpy / rclcpp** | ROS2のPython（rclpy）およびC++（rclcpp）向けクライアントライブラリ。<br>それぞれの言語でノードや通信処理を記述する際に使用。<br>用途やパフォーマンス要件に応じて使い分ける。<br>[参考：rclpy - ROS 2 Docs](https://docs.ros.org/en/humble/How-To-Guides/Using-Parameters-In-A-Class-Python.html) |


---

- [**チュートリアル一覧** に戻る](./toc.md)
- [**次のチュートリアル** に移動する](./tutorial6.md)
