# 5. ROS2 の用語早見表

| 用語名 | 概要 |
|:---:|:---|
| **ROS** | Robot Operating System の略称。<br>ロボット開発のためのミドルウェアで、通信、制御、センサ処理などの機能を提供。<br>バージョン1系（ROS1）と、改良されたROS2が存在する。<br>[参考：ROS公式サイト（英語）](https://www.ros.org/) / [日本語Wiki解説](https://ja.wikipedia.org/wiki/Robot_Operating_System) |
| **ROS2** | ROS1を基にリアルタイム性、セキュリティ、マルチプラットフォーム対応を強化した次世代版。<br>DDS（Data Distribution Service）を通信ミドルウェアに採用。<br>産業利用や複数ロボット対応を重視している。<br>[参考：ROS 2公式ドキュメント（日本語解説含む）](https://docs.ros.org/) / [ROS2設計思想](https://design.ros2.org/) |
| **ノード（Node）** | ROSにおける基本的な処理単位。<br>各ノードは独立して動作し、他のノードと通信して情報をやり取りする。<br>軽量なプロセスとして複数のノードを分散実行可能。<br>[参考：ROS 2ノード概念説明（Humble版）](https://docs.ros.org/en/humble/Concepts/About-Nodes.html) |
| **トピック（Topic）** | ノード間でデータを送受信するための通信チャンネル。<br>Publisherがデータを送信し、Subscriberが受信する。<br>センサデータのストリームなどに適している。<br>[参考：ROS 2トピック解説（Humble版）](https://docs.ros.org/en/humble/Concepts/About-Topics.html) |
| **Publish / Publisher** | トピックに対してデータを送信（発行）する側。<br>センサ値や制御指令などの情報を他のノードに通知する。<br>ノードは複数のトピックに対してPublisherを持てる。<br>[参考：Publisher実装チュートリアル（Python版）](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) |
| **Subscribe / Subscriber** | トピックからデータを受信（購読）する側。<br>他ノードの発行した情報を受け取って処理する。<br>センサデータの受信や、状態監視などに利用される。<br>[参考：Subscriber実装チュートリアル（Python版）](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) |
| **サービス（Service）** | リクエストとレスポンスによる一対一の通信。<br>ある処理を即時に呼び出し結果を得たいときに使われる。<br>同期的な処理に向いている通信手法。<br>[参考：ROS 2サービス概念説明](https://docs.ros.org/en/humble/Concepts/About-Services.html) |
| **アクション（Action）** | 時間のかかる処理を非同期で実行する仕組み。<br>目標を送信し、フィードバックを受け取りつつ完了を待てる。<br>ナビゲーションなど継続的な処理に利用される。<br>[参考：ROS 2アクション解説](https://docs.ros.org/en/humble/Concepts/About-Actions.html) |
| **DDS（Data Distribution Service）** | ROS2の下位通信層として使われる標準的なPub/Subミドルウェア。<br>QoS設定により信頼性や遅延を制御可能。<br>複数ベンダー（Fast DDS、Cyclone DDSなど）がある。<br>[参考：ROS 2とDDSの技術設計書](https://design.ros2.org/articles/ros_on_dds.html) |
| **Launchファイル** | 複数のノードを一括起動するための設定スクリプト。<br>Pythonベースで柔軟に起動順序や引数を設定できる。<br>実験や運用時に欠かせない機能。<br>[参考：Launchファイル作成チュートリアル](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html) |
| **パッケージ（Package）** | ノードや設定ファイル、ライブラリなどをまとめた単位。<br>ROSのプロジェクトはすべてパッケージ単位で構成される。<br>再利用性や依存関係管理に優れる。<br>[参考：パッケージ作成ガイド](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) |
| **rclpy / rclcpp** | ROS2のPython（rclpy）およびC++（rclcpp）向けクライアントライブラリ。<br>それぞれの言語でノードや通信処理を記述する際に使用。<br>用途やパフォーマンス要件に応じて使い分ける。<br>[参考：rclpy公式リポジトリ](https://github.com/ros2/rclpy) / [rclcpp公式リポジトリ](https://github.com/ros2/rclcpp) |

---

- [**チュートリアル一覧** に戻る](./toc.md)
- [**次のチュートリアル** に移動する](./tutorial6.md)
