# Windows におけるターミナル（Terminal）アプリについて
```{image} https://camo.githubusercontent.com/56636af374fb8ece3418d2501649bbc3f43d832cc613653730304777f2f8f08d/68747470733a2f2f75706c6f61642e77696b696d656469612e6f72672f77696b6970656469612f636f6d6d6f6e732f7468756d622f352f35312f57696e646f77735f5465726d696e616c5f6c6f676f2e7376672f36343070782d57696e646f77735f5465726d696e616c5f6c6f676f2e7376672e706e67
:width: 25%
:align: center
```

---

Windows には以下の **主要なコマンドラインインターフェース（`cli`）** があります．

```{note}
コマンドラインインターフェース（`cli` : Command Line Interface）は文字列のみで構成された命令（コマンド）を入力し，文字列のみで構成された結果を受け取るインターフェースを指します．
```

- **PowerShell**（パワーシェル）
- **Command Prompt**（コマンドプロンプト）

Windows 11 以降では PowerShell, Command Prompt が **Terminal**（ターミナル）アプリとしして統合された状態で利用できるようになりました．本ドキュメントでは基本このターミナルをつかい作業を進めていきます．

## Terminal（ターミナル）アプリの起動方法
はじめに Terminal を起動させてみましょう．この作業はこれから行う作業において一般的な作業ですので，よく覚えておきましょう．

1. **スタートメニューを開く**<br>
    スタートメニュー開き，検索欄に「terminal」または「ターミナル」と入力すると，「Terminal」アプリが表示されます．
    <br><img src="https://github.com/horilaboratory/robot_project/blob/main/imgs/open_terminal.png?raw=true" width=70% />

1. **Terminal を起動する**<br>
    「Terminal」アプリをクリックして起動してください．以下の画像のような黒いウィンドウが表示されれば成功です．
    <br><img src="https://github.com/horilaboratory/robot_project/blob/main/imgs/opened_terminal.png?raw=true" width=70% />

## Terminal の基本操作と表示内容

### 表示内容

Terminal では基本以下のレイアウトで構成されています．

- **タブ**<br>
    現在利用しているプロンプトや他のプロンプトを切り替えることができます．
- **新規プロファイル**<br>
    用意されたプロファイルをもとに新しいプロンプトを起動することができます．「+」ボタンを押すことでデフォルトのプロファイルをすぐに呼び出せます．デフォルトのプロファイルは「PowerShell」となっています．また，「+」ボタン横の「v」ボタンをクリックすると別のプロファイルを選択して起動することができます．
- **プロンプト**<br>
    実際にコマンドを入力したり，入力結果が表示されるインターフェース部分です．

### コマンドを入力する

Terminal で PowerShell を開き，以下のコマンドを実行してみましょう．上記コマンドをコピペしてエンターキーを押すと入力されたコマンドが実行されます．するとプロンプト上に「`Hello World!!`」と表示されます．
```powershell
echo "Hello World!!"
```
```{note}
このコマンドは PowerShell の基本的なコマンドの一つで，入力された文字列を表示するコマンドです．`echo` 以降に出力させたい文字列を `""` で括ることで出力したい文字列として扱われます．
```

```{danger}
Terminal 上の PowerShell で実行可能なコマンドのの中には **Windows のシステムを改変，削除，破壊する可能性のあるコマンド** も含まれています．そのため，不明なコマンドを実行することは推奨しません．
```

### コマンドをコピペする

このドキュメントでは基本 PowerShell と後述する Terminal 上で利用可能なプロンプト「WSL2 Ubuntu 22.04 Bash」を用いて作業を進めていきます．そのため多数のコマンドを扱うことになります．
<br>
一部扱うコマンドの中には長文や **タイプミスを犯すとシステムに影響を及ぼす可能性のある** コマンドもありますので，**基本ドキュメント中のコマンドを実行する際はコピペで入力してください．**
<br>
以下のコマンドをコピペして実行してみましょう．以下のコマンドブロックにマウスカーソルを当てると，コマンドブロックの右側にボタンが出現します．このボタンを押すことでコマンドブロック内のコマンドがコピー（クリップボードに保存）されます．
```powershell
Add-Type -AssemblyName System.Windows.Forms; [System.Windows.Forms.MessageBox]::Show("Hello World!!", "Robot Project", [System.Windows.Forms.MessageBoxButtons]::OK, [System.Windows.Forms.MessageBoxIcon]::Information) | Out-Null
```
Terminal（PowerShell）上でショートカットキー「Control + V」を実施して上記コマンドを入力してから実行してみましょう．すると以下のようなポップアップが表示されます．OK をクリックしてポップアップを閉じることができます．
<br><img src="https://i.imgur.com/DiU6vWQ.png"/>

### コマンドの実行結果をコピーする

プロンプト上で任意のコマンドを実行すると，実行したコマンドやプログラムの種類によって，実行結果が表示されます．実行結果には場合によってはエラーメッセージが含まれている場合もあり，トラブルシューティングにおいて実行結果を確認することは重要です．また実行結果が何を意味しているのかを調べるために，実行結果をコピーして検索することもあります．以下のコマンドを実行したときに表示される実行結果を選択してコピーして，その意味を調べてみましょう．
```powershell
Get-Item "C:\NonExistentFile"
```