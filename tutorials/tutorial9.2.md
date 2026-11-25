# 9.1 物体認識を動かしてみよう

## 概要
ここではYOLOを用いた物体認識を動作させます．RaspberryPi上で実行する物体認識プログラムは既にRaspberryPi側に実装済みなのでこれを実行し，認識結果をサブスクライブし自分のプロジェクトに活用してください．

なお，RaspberryPi上に実装された物体認識プログラムについて詳しく知りたい場合は，[horilaboratory/robot_project_ai](https://github.com/horilaboratory/robot_project_ai)を参照してください．

## 物体認識プログラムの起動と認識結果の確認

### RaspberryPiへの接続
----
まずは，[9.1 準備](/tutorials/tutorial9.1.md)に説明されているとおり，RaspberryPiを接続し，電源を入れてください．電源投入後，起動が完了するまで待ちます（2分ほど待てば起動しているはずです）．

次に，RaspberryPi上で物体認識プログラムを起動するためにRaspberryPi環境にログインします．ログインはUbuntuの端末を開き，以下のSSHコマンドを入力します．

> [!IMPORTANT]
> 接続先のRaspberryPiにあわせてホスト名（接続コマンドの `rpai01.local` の部分）を変更してください．
> 自分が使用しているRaspberryPiの番号に応じて `rpai01.local` 〜 `rpai05.local` のいずれかに変更してください．

```bash
ssh tamagawa@rpai01.local
```

パスワードを聞かれるので，ログインパスワードを入力してください．

### 物体認識プログラムの起動
----
次に，物体認識プログラムを起動します．起動するためのプログラム（シェルスクリプト）が `~/robot_project_ai`ディレクトリに配置されていますので，そのディレクトリまで移動し，シェルスクリプトを起動します．

```bash
cd ~/robot_project_ai
./run_obj_detection.sh
```

プログラムを起動し，正常にカメラが動作していれば以下のように表示されます．以上で物体認識プログラムが起動し，その認識結果がROSのTopicとしてPublishされています．

```txt
[INFO] [1795572914.542711781] [yolo_hailo_publisher]: HEF 読み込み完了 (入力 640x640) / カメラ /dev/video0
```

> [!NOTE]
> カメラの読み込みに失敗した場合は，カメラの接続や起動スクリプトに記述されているカメラのデバイス名を確認してください．
> カメラのデバイス名は， `run_obj_detection.sh` の以下の部分です．
> ```bash
> CAMERA_DEVICE=/dev/video0
> ```
>
> カメラデバイス名を確認する場合は以下のコマンドを実行し， `C270 HD WEBCAM` と書かれている部分に記載されている `/dev/videoX` (Xは数字)を確認してください．
> ```bash
> # デバイス一覧を表示するコマンド
> v4l2-ctl --list-devices
> ```

### 認識結果の確認
----

> [!IMPORTANT]
> この部分の作業は自分のVirtualBox上のUbuntuで行いますので，これまでの作業で使った端末は開いたままにして新しく端末を開いてください．

Raspberry PiからPublishされている物体認識結果をSubscribeして確認してみましょう．これまでの作業で用いていた端末は開いたままにして新しく端末を開いてください．

新しく開いた端末で以下のコマンドを入力します．

```bash
rqt
```

すると以下のようなビューワが開きます．


## 認識結果のサブスクライブ
物体認識結果をサブスクライブし，認識結果によってカチャカを制御するプログラムを作成できます．物体認識の結果は `vision_msgs/Detection2DArray`型のメッセージが `/detection` トピックとしてパブリッシュされています．

