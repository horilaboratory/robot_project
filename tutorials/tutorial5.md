# 5. ROS2 の用語早見表

| 用語名 | 概要 |
|:---:|:---|
| **ROS** | Robot Operating System の略称。<br>ロボット開発のためのミドルウェアで、通信、制御、センサ処理などの機能を提供。<br>バージョン1系（ROS1）と、改良されたROS2が存在する。<br>[参考：ROSとは - ライトローバーWebDoc](https://vstoneofficial.github.io/lightrover_webdoc/software/aboutROS/) |
| **ROS2** | ROS1を基にリアルタイム性、セキュリティ、マルチプラットフォーム対応を強化した次世代版。<br>DDS（Data Distribution Service）を通信ミドルウェアに採用。<br>産業利用や複数ロボット対応を重視している。<br>[参考：ROSとは - ライトローバーWebDoc](https://vstoneofficial.github.io/lightrover_webdoc/software/aboutROS/) |
| **ノード（Node）** | ROSにおける基本的な処理単位。<br>各ノードは独立して動作し、他のノードと通信して情報をやり取りする。<br>軽量なプロセスとして複数のノードを分散実行可能。<br>[参考：Nodes - ROS 2 Docs](https://docs.ros.org/en/humble/Concepts/Nodes.html) <br>**注:** 日本語の分かりやすい公式ドキュメントが見つかりませんでした。こちらは英語版の公式ドキュメントです。 |
| **トピック（Topic）** | ノード間でデータを送受信するための通信チャンネル。<br>Publisherがデータを送信し、Subscriberが受信する。<br>センサデータのストリームなどに適している。<br>[参考：Topics - ROS 2 Docs](https://docs.ros.org/en/humble/Concepts/Topics.html) <br>**注:** 日本語の分かりやすい公式ドキュメントが見つかりませんでした。こちらは英語版の公式ドキュメントです。 |
| **Publish / Publisher** | トピックに対してデータを送信（発行）する側。<br>センサ値や制御指令などの情報を他のノードに通知する。<br>ノードは複数のトピックに対してPublisherを持てる。<br>[参考：ROS（ロボットオペレーティングシステム）のインストール (ROS) - 有限会社はじめ研究所](https://hajimerobot.co.jp/ros/install/) <br>**注:** 直接的な解説ではありませんが、Publisherに言及のある日本語のページです。 |
| **Subscribe / Subscriber** | トピックからデータを受信（購読）する側。<br>他ノードの発行した情報を受け取って処理する。<br>センサデータの受信や、状態監視などに利用される。<br>[参考：C++ 参照渡しの勉強 ④ ROS2 その1 cmakeができるようにする。 subscriberのsub001.cpp](https://qiita.com/jamjam/items/28cb7fcd678b39378090) <br>**注:** Subscriberに言及のあるQiitaの記事です。 |
| **サービス（Service）** | リクエストとレスポンスによる一対一の通信。<br>ある処理を即時に呼び出し結果を得たいときに使われる。<br>同期的な処理に向いている通信手法。<br>[参考：Services - ROS 2 Docs](https://docs.ros.org/en/humble/Concepts/Services.html) <br>**注:** 日本語の分かりやすい公式ドキュメントが見つかりませんでした。こちらは英語版の公式ドキュメントです。 |
| **アクション（Action）** | 時間のかかる処理を非同期で実行する仕組み。<br>目標を送信し、フィードバックを受け取りつつ完了を待てる。<br>ナビゲーションなど継続的な処理に利用される。<br>[参考：ロボットプログラミングROS2入門 (エンジニア入門シリーズ84) | 岡田 浩之 |本 | 通販 | Amazon](https://www.amazon.co.jp/%E3%83%AD%E3%83%9C%E3%83%83%E3%83%88%E3%83%97%E3%83%AD%E3%82%B0%E3%83%A9%E3%83%9F%E3%83%B3%E3%82%B0ROS2%E5%85%A5%E9%96%80-%E3%82%A8%E3%83%B3%E3%82%B8%E3%83%8B%E3%82%A2%E5%85%A5%E9%96%80%E3%82%B7%E3%83%AA%E3%83%BC%E3%82%BA-%E5%B2%A1%E7%94%B0-%E6%B5%A9%E4%B9%8B/dp/4904774906) <br>**注:** ROS2の入門書（日本語）のAmazonリンクです。 |
| **DDS（Data Distribution Service）** | ROS2の下位通信層として使われる標準的なPub/Subミドルウェア。<br>QoS設定により信頼性や遅延を制御可能。<br>複数ベンダー（Fast DDS、Cyclone DDSなど）がある。<br>[参考：AMRの構築方法 | ROS2 DDS - ADLINK](https://www.adlinktech.com/jp/ROS2_how_do_you_build_an_amr) |
| **Launchファイル** | 複数のノードを一括起動するための設定スクリプト。<br>Pythonベースで柔軟に起動順序や引数を設定できる。<br>実験や運用時に欠かせない機能。<br>[参考：ROS2のインストール | 中部大学AIロボティクス学科](https://cu-milab.github.io/ai-robot/ps/we1emyp/) <br>**注:** Launchファイルにおける文字コードに関する記述のある日本語ページです。 |
| **パッケージ（Package）** | ノードや設定ファイル、ライブラリなどをまとめた単位。<br>ROSのプロジェクトはすべてパッケージ単位で構成される。<br>再利用性や依存関係管理に優れる。<br>[参考：Packages - ROS 2 Docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) <br>**注:** 日本語の分かりやすい公式ドキュメントが見つかりませんでした。こちらは英語版の公式ドキュメントです。 |
| **rclpy / rclcpp** | ROS2のPython（rclpy）およびC++（rclcpp）向けクライアントライブラリ。<br>それぞれの言語でノードや通信処理を記述する際に使用。<br>用途やパフォーマンス要件に応じて使い分ける。<br>[参考：C言語用ROS2クライアントライブラリrclcのmacOS Montereyビルド - Yutaka Kondo](https://www.youtalk.jp/2021/12/10/rclc-on-macos.html) <br>**注:** rclpyとrclcppに言及のある日本語のブログ記事です。 |

---

- [**チュートリアル一覧** に戻る](./toc.md)
- [**次のチュートリアル** に移動する](./tutorial6.md)
