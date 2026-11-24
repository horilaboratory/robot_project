# 9.1 物体認識を動かしてみよう

## 概要
ここではYOLOを用いた物体認識を動作させます．RaspberryPi上で実行する物体認識プログラムは既にRaspberryPi側に実装済みなのでこれを実行し，認識結果をサブスクライブし自分のプロジェクトに活用してください．

なお，RaspberryPi上に実装された物体認識プログラムについて詳しく知りたい場合は，[horilaboratory/robot_project_ai](https://github.com/horilaboratory/robot_project_ai)を参照してください．

## 物体認識プログラムの起動と認識結果の確認
まずは，[9.1 準備](/tutorials/tutorial9.1.md)に説明されているとおり，RaspberryPiを接続し，電源を入れてください．電源投入後，起動が完了するまで待ちます（2分ほど待てば起動しているはずです）．

次に，RaspberryPi上で物体認識プログラムを起動するためにRaspberryPi環境にログインします．ログインはUbuntuの端末を開き，以下のSSHコマンドを入力します．

> [!IMPORTANT]
> 接続先のRaspberryPiにあわせてホスト名（接続コマンドの `rpai01.local` の部分）を変更してください．
> 自分が使用しているRaspberryPiの番号に応じて `rpai01.local` 〜 `rpai05.local` のいずれかに変更してください．

```bash
ssh tamagawa@rpai01.local
```

パスワードを聞かれるので，ログインパスワードを入力してください．

