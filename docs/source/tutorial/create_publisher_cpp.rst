#####################################################
ament_cmake パッケージで C++ ノードを作ってみよう
#####################################################

.. caution::

    この記事は現在工事中です．不足した情報が一部含まれています．

前の章では ``ament_cmake`` パッケージで Python ノードを実行する方法を学びました．この章では，同じパッケージ内で C++ を使って Publisher ノードを作成し，ビルド，実行する方法を詳しく解説します．

******************************
C++ ノードプログラムの書き方
******************************

C++ のソースコードは，パッケージ内の ``src`` ディレクトリに配置するのが一般的です．``/ws/src/ros2_workshop_cmake/src`` ディレクトリに ``practice_publisher.cpp`` という名前でファイルを作成しましょう．

必要なヘッダーファイルのインポート
==================================

C++ で ROS2 ノードを作成するには，必要な機能が定義されたヘッダーファイルをインクルードする必要があります．

.. tip::

    ヘッダファイルをインクルードするとは，Python でいう **必要なモジュールをインポートする** 作業によく似ています．これはあくまで例ですが，以下のように書き換えて表現することができます．

    .. code:: cpp

        #include "rclcpp/rclcpp.hpp"
        #include <chrono>
    
    .. code:: python

        from rclcpp import rclcpp
        import chrono

.. code:: cpp

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"

    #include <chrono>
    #include <memory>

- ``rclcpp/rclcpp.hpp``: ROS2 の C++ クライアントライブラリ (RCLCPP) の主要な機能を含みます．ノードの作成やスピン (実行待機) など，基本的な操作に必須です．

- ``std_msgs/msg/string.hpp``: 今回使用する ``String`` 型メッセージの定義です．Python で ``from std_msgs.msg import String`` としたのと同じ役割です．

- ``<chrono>``: 時間を扱うための C++ 標準ライブラリです．タイマーの周期を指定する際に使用します．

- ``<memory>``: スマートポインタ (``std::shared_ptr`` など) を使用するために必要です．RCLCPP ではノードなどをスマートポインタで管理するのが一般的です．

.. tip::

    **ポインタとスマートポインタについて**

    C++ を初めて学ぶ方にとって，「ポインタ」は最初の難関の一つかもしれません．ここではその基本的な考え方と，なぜ ROS2 (RCLCPP) で「スマートポインタ」が頻繁に使われるのかを解説します．

    **そもそもポインタとは？**

    - **変数の「住所」を入れるための変数**
        コンピュータのメモリは，よく「たくさんの小箱が並んだ棚」に例えられます．変数 (例えば ``int a = 10;``) を宣言すると，棚の一つの小箱に ``10`` というデータが格納され，その小箱には「0x1234番地」のようなユニークな **アドレス (住所)** が割り当てられます．
        **ポインタ** とは，この **アドレスそのものを値として格納する** 特殊な変数です．つまり，「データ本体」ではなく「データがどこにあるか」という場所の情報を保持します．

    **なぜポインタが必要？**

    - C++ では，ポインタを使うことでメモリを直接操作でき，非常に効率的なプログラムを書くことができます．しかし，その自由度の高さゆえに，以下のような管理が難しい問題 (バグの温床) も抱えています．
        
        - **メモリリーク**: ``new`` で確保したメモリを ``delete`` し忘れると，そのメモリ領域が解放されず，プログラムが使い続けるうちに利用可能なメモリがどんどん減ってしまう問題．
        
        - **ダングリングポインタ**: ``delete`` で解放済みのメモリ領域を，別のポインタが指し示し続けてしまい，そこへアクセスしようとして予期せぬ動作を引き起こす問題．

    **スマートポインタが解決すること**

    - **スマートポインタ** は，こうしたポインタの難しいメモリ管理を自動化し，より安全に扱えるようにした「賢い (Smart)」ポインタです．これはクラスとして実装されており，内部に生のポインタを保持します．
    
    - **自動的なメモリ解放**: スマートポインタ変数がそのスコープ (有効範囲) を抜けると，デストラクタという特殊な関数が自動的に呼び出され，保持していたメモリを ``delete`` してくれます．これにより，開発者が ``delete`` を書き忘れることによるメモリリークを劇的に減らすことができます．この仕組みを **RAII (Resource Acquisition Is Initialization)** と呼びます．
    
    - **RCLCPP でよく使われる** ``std::shared_ptr`` : スマートポインタにはいくつか種類がありますが，RCLCPP では特に ``std::shared_ptr`` (共有ポインタ) が多用されます．これは，一つのオブジェクト (例えば，作成したノード) を複数の変数で「共有」して所有できるスマートポインタです．誰もそのオブジェクトを参照しなくなった (最後の ``shared_ptr`` が破棄された) タイミングで，自動的にメモリが解放されます．ROS2 のような複雑なシステムでは，様々な場所から一つのノードにアクセスする必要があるため，この共有の仕組みが非常に便利なのです．

クラスを作成する
====================

Python の時と同様に，ノードの機能をまとめるためのクラスを作成します．``rclcpp::Node`` クラスを継承するのが基本です．

.. code:: cpp

    class PracticePublisherCpp : public rclcpp::Node
    {
    public:
        PracticePublisherCpp()
        : Node("practice_publisher_cpp"), count_(0)
        {
            // コンストラクタの中身は後で記述
        }

    private:
        // メンバ変数は後で記述
        size_t count_;
    };

.. tip::

    **コンストラクタとは**

    クラスからオブジェクトが作られる (インスタンス化される) 時に，自動的に呼び出される特殊な関数です．主な役割は，オブジェクトが作られた直後にその内部の状態 (メンバ変数) を初期化することです．Python の ``__init__`` メソッドと似た役割を持ちます．

    **メンバ変数とは**

    クラスの内部で定義される変数のことです．そのクラスから作られたオブジェクトの状態を保持するために使われます．この例では ``count_`` がメンバ変数で，メッセージを何回送信したかを記録しています．クラス内のどのメソッドからでもアクセスできます．Python でいう ``self.XXX`` のようにクラス内で使えるコンストラクタ変数のようなものです．

- ``class PracticePublisherCpp : public rclcpp::Node``: ``PracticePublisherCpp`` というクラスが ``rclcpp::Node`` を **公開継承 (public inherit)** することを意味します．これにより，``rclcpp::Node`` の機能を使えるようになります．

- ``public:``: このキーワード以降に書かれたメンバ (メソッドや変数) は，クラスの外部からアクセス可能です．

- ``PracticePublisherCpp() : Node("practice_publisher_cpp"), count_(0)``: これは **コンストラクタ** です．

    - ``Node("practice_publisher_cpp")``: 親クラスである ``rclcpp::Node`` のコンストラクタを呼び出し，ノード名を ``practice_publisher_cpp`` として登録しています．

    - ``count_(0)``: このクラスのメンバ変数である ``count_`` を 0 で初期化しています．
    
- ``private:``: このキーワード以降に書かれたメンバは，このクラスの内部からのみアクセス可能です．

Publisher とタイマーを作成する
================================

次に，コンストラクタの内部で Publisher とタイマーを作成します．

.. code:: cpp

    class PracticePublisherCpp : public rclcpp::Node
    {
    public:
        PracticePublisherCpp()
        : Node("practice_publisher_cpp"), count_(0)
        {
            // Publisherを作成
            publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);

            // 500ms周期のタイマーを作成
            using namespace std::chrono_literals;
            timer_ = this->create_wall_timer(
                500ms, std::bind(&PracticePublisherCpp::publish_callback, this));
        }

    private:
        void publish_callback()
        {
            // コールバック関数の中身は後で記述
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
    };

- ``publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);``

    - ``create_publisher`` メソッドで Publisher を作成します．

    - ``<std_msgs::msg::String>``: **テンプレート引数** と呼ばれるもので，配信するメッセージの型を指定します．
    
    - ``"/chatter"``: トピック名です．
    
    - ``10``: QoS の設定で，キューサイズ (トピック深度) を示します．
- ``timer_ = this->create_wall_timer(500ms, ...);``
    
    - ``create_wall_timer`` メソッドでタイマーを作成します．
    
    - ``500ms``: タイマーの周期です．``using namespace std::chrono_literals;`` を宣言することで，``500ms`` のような直感的な時間の書き方ができます．
    
    - ``std::bind(&PracticePublisherCpp::publish_callback, this)``: タイマーによって呼び出される **コールバック関数** を指定しています．``std::bind`` は，「``PracticePublisherCpp`` クラスの ``publish_callback`` というメンバ関数を，このインスタンス (``this``) に紐づけて呼び出してください」という意味です．

- ``private:`` セクションに，作成したタイマーと Publisher を保持するためのメンバ変数を宣言しています．``SharedPtr`` はスマートポインタの一種で，オブジェクトの寿命を自動で管理してくれます．

コールバック関数を作成する
============================

タイマーによって周期的に呼び出される ``publish_callback`` メソッドの中身を記述します．

.. code:: cpp

    private:
        void publish_callback()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello! C++ ROS2 count: " + std::to_string(count_++);
            
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            
            publisher_->publish(message);
        }

- ``auto message = std_msgs::msg::String();``: ``String`` 型のメッセージオブジェクトを作成します．``auto`` は型を自動で推論してくれるキーワードです．

- ``message.data = ...``: メッセージの ``data`` フィールドに文字列を代入します．カウンタ変数 ``count_`` を ``std::to_string`` で文字列に変換しています．

- ``RCLCPP_INFO(...)``: コンソールに情報を出力するための Logger です．Python の ``self.get_logger().info()`` に相当します．

- ``publisher_->publish(message);``: 作成したメッセージを配信します．

実行関数 ``main`` を作成する
=================================

最後に，このノードを実行するための ``main`` 関数をクラスの外に作成します．

.. code:: cpp

    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<PracticePublisherCpp>());
        rclcpp::shutdown();
        return 0;
    }

- ``int main(int argc, char * argv[])``: C++ プログラムのエントリーポイント (開始点) です．

- ``rclcpp::init(argc, argv);``: ROS2 を初期化します．Python の ``rclpy.init()`` と同じです．

- ``rclcpp::spin(...)``: ノードを実行状態に保ち，コールバック関数が呼ばれるのを待ち続けます．``std::make_shared<PracticePublisherCpp>()`` でノードクラスのインスタンスをスマートポインタとして生成し，``spin`` 関数に渡しています．

- ``rclcpp::shutdown();``: ROS2 の処理を終了します．

- ``return 0;``: プログラムが正常に終了したことを示します．

*********************************
Publisher ノードの全体図 (C++)
*********************************

このセクションで作成した C++ Publisher ノードのコード全体像です．

.. code:: cpp

    // ==========================================================================================
    // 必要なヘッダーファイルをインクルード
    // ==========================================================================================
    #include "rclcpp/rclcpp.hpp"              // ROS2 C++クライアントライブラリの基本機能
    #include "std_msgs/msg/string.hpp"      // String型メッセージの定義

    #include <chrono>                       // 時間を扱うための標準ライブラリ (500ms のような表記に必要)
    #include <memory>                       // スマートポインタ (std::make_shared など) を使うために必要

    // ==========================================================================================
    // Publisherノードの機能を実装したクラス
    // ==========================================================================================
    // rclcpp::Nodeクラスを継承して，オリジナルのPublisherノードクラスを定義
    class PracticePublisherCpp : public rclcpp::Node
    {
    public:
        // --- コンストラクタ ---
        // クラスのインスタンスが作成されるときに自動的に呼び出される
        PracticePublisherCpp()
        : Node("practice_publisher_cpp"), count_(0) // 親クラスのコンストラクタを呼び出しノード名を登録し，メンバ変数count_を0で初期化
        {
            // Publisherを作成．
            // <std_msgs::msg::String> は配信するメッセージの型．
            // "/chatter" はトピック名．
            // 10 はQoSのキューサイズ．
            publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);

            // C++で '500ms' のような時間単位を扱うためのおまじない
            using namespace std::chrono_literals;
            // 500ミリ秒ごとに publish_callback メソッドを呼び出すタイマーを作成
            timer_ = this->create_wall_timer(
                500ms, std::bind(&PracticePublisherCpp::publish_callback, this));
        }

    private:
        // --- タイマーによって呼び出されるコールバック関数 ---
        void publish_callback()
        {
            // 配信するString型のメッセージオブジェクトを作成
            auto message = std_msgs::msg::String();
            // メッセージのdataフィールドに，カウンタを含んだ文字列を代入．
            // std::to_stringで数値を文字列に変換し，count_++でカウンタを1増やす．
            message.data = "Hello! C++ ROS2 count: " + std::to_string(count_++);
            
            // 配信するメッセージの内容をコンソールにログ出力
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            
            // 作成したメッセージを実際にトピックへ配信
            publisher_->publish(message);
        }

        // --- メンバ変数 ---
        // クラス内で保持する変数
        rclcpp::TimerBase::SharedPtr timer_;                                  // 作成したタイマーを保持する変数
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;     // 作成したPublisherを保持する変数
        size_t count_;                                                      // 送信回数をカウントする変数
    };

    // ==========================================================================================
    // プログラムのエントリーポイント (ここから処理が始まる)
    // ==========================================================================================
    int main(int argc, char * argv[])
    {
        // ROS2システムを初期化
        rclcpp::init(argc, argv);
        // PracticePublisherCppノードをスピンさせ，コールバック処理などを実行可能にする
        // std::make_sharedでノードのインスタンスを作成
        rclcpp::spin(std::make_shared<PracticePublisherCpp>());
        // ROS2システムを終了
        rclcpp::shutdown();
        // プログラムの正常終了
        return 0;
    }

************************************************
ビルド設定ファイルを編集する (C++)
************************************************

C++ ノードを追加したので，``CMakeLists.txt`` と ``package.xml`` を再度編集して，ビルドシステムに C++ ノードの存在を教える必要があります．

package.xml の編集
======================

C++ の ROS2 クライアントライブラリ ``rclcpp`` への依存関係を追記します．

.. code:: diff

      <buildtool_depend>ament_cmake</buildtool_depend>
    
      <build_depend>ament_cmake_python</build_depend>
    + <build_depend>rclcpp</build_depend>
    + <build_depend>std_msgs</build_depend>
    
      <exec_depend>rclpy</exec_depend>
      <exec_depend>std_msgs</exec_depend>
    + <exec_depend>rclcpp</exec_depend>

CMakeLists.txt の編集
=========================

``CMakeLists.txt`` に，C++ ソースコードをコンパイルし，実行ファイルを作成するためのルールを追記します．

.. code:: cmake

    # ... (find_package(ament_cmake_python REQUIRED) の後) ...

    # C++ノードの実行ファイルを作成
    add_executable(practice_publisher_cpp src/practice_publisher.cpp)
    ament_target_dependencies(
      practice_publisher_cpp
      "rclcpp"
      "std_msgs"
    )

    # 実行ファイルとPythonスクリプトをインストール
    install(
      TARGETS
      practice_publisher_cpp
      DESTINATION lib/${PROJECT_NAME}
    )
    install(
      PROGRAMS
      # ... (Pythonスクリプトの記述はそのまま) ...
    )

- ``add_executable(practice_publisher_cpp src/practice_publisher.cpp)``

    - ``practice_publisher_cpp`` という名前の **実行ファイル** を，``src/practice_publisher.cpp`` というソースファイルから作成 (コンパイル＆リンク) するよう指示します．

- ``ament_target_dependencies(...)``

    - ``practice_publisher_cpp`` というターゲット (実行ファイル) が，``rclcpp`` と ``std_msgs`` というライブラリに依存していることを示します．これにより，ビルド時に必要なライブラリがリンクされます．

- ``install(TARGETS ...)``

    - ``add_executable`` で作成したターゲット (実行ファイル) をインストールします．

***************************
パッケージをビルドする
***************************

設定ファイルを変更したので，再度パッケージをビルドします．

.. code:: bash

    cd /ws
    colcon build --symlink-install --packages-select ros2_workshop_cmake

**************************
ノードを実行する
**************************

ビルドが完了したら，環境を読み込み，新しい C++ ノードを実行してみましょう．

.. code:: bash

    source /ws/install/setup.bash
    ros2 run ros2_workshop_cmake practice_publisher_cpp

Python で作成した Subscriber ノードを別のターミナルで起動すれば，C++ ノードから配信されたメッセージを受信できるはずです．

.. code:: bash

    ros2 run ros2_workshop_cmake practice_subscriber.py
