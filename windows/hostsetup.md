# Windows でロボット開発をはじめる

　このドキュメントでは、お使いの Windows 上で
**ロボットを開発するための環境**
を構築する方法を解説します。

　このドキュメントを作るために使用された Windows のバージョン情報は以下の図の通りです。作業を始める前にお使いのコンピューターがこの要件を満たしているかどうか確認してください。

<img src="/imgs/win_ver.png" />

|||
|:---:|:---|
|**Windows 11 バージョン**|23H2（24H2 では動作検証を行っていません。）|
|**Windows 11 Home**|上記画像では*Windows 11 Pro* と記載されていますが、**Windows 11 Home Edition** でも大丈夫です。|

> [!CAUTION]
> **Windows10 を使用されている方は、Windows 11 にアップグレードすること強く推奨します。**<br>
> Windows 10 では動作検証を行っておらず、操作方法が異なる場合があります。

それでは、Windows の環境構築を進めていきましょう！

<img width=25%/><img src="/imgs/win_desktop.png" width=50% />

## 必要なソフトウェアとシステム一覧
　ロボット開発をはじめるには、以下のソフトウェアやシステムを導入する必要があります。

- **VScode**
- **Git**
- **WSL2**
- **Hyper-V**

　これから順番にこれらの環境を構築する方法を解説します。

## VScode をインストールする方法
### VScode とは
<img width=37.5%/><img src="/imgs/logo_vscode.png" width=25% />

　VScode（Visual Studio Code）とは、無料のプログラミングコード専門のエディタです。

- 資料制作の Word
- プレゼン制作の PowerPoint
- 表制作の Exel
- **プログラミング制作の VScode**

のような感じで、おそらく最も多く使用されているプログラムエディタでしょう。
この授業ではこの VScode を使い、プログラムコードを記述していきます。

### VScode をダウンロードする
　VScode をダウンロードするには、以下のサイトに移動してください。

- [VSCode のダウンロードサイトに移動する](https://code.visualstudio.com/download)

　そして、**Windows** と書かれているボタンをクリックして、Windows 版 VSCode インストーラーをダウンロードしてください。
このボタンをクリックするとインストーラーがダウンロードされます。

<img src="/imgs/download_vscode.png" />

## VScode をインストールする
　**エクスプローラー**
を開き、ダウンロードフォルダにダウンロードされた **VSCode インストーラー** を **ダブルクリック** するか、右クリックしたときに表示される **「開く」**
をクリックしてインストーラーを起動してください。

<img src="/imgs/download_vscode2.png" />

　起動すると、このようなセットアップ画面が表示されます。<br>
「同意する」にチェックを入れて「次へ」をクリックしてください。 

<img src="/imgs/vscode_install1.png" />

　次に進むとインストール先の指定に入りますが、ここは何も触れずに「次へ」をクリックしてください。

 <img src="/imgs/vscode_install2.png" />

　次に進むとスタートメニューを作るか否かを選択できます。ここも何も触れずに「次へ」をクリックしてください。

 <img src="/imgs/vscode_install3.png" />
 
　次に進むと VScode のインストールに関する追加機能を選択できます。ここも何も触れずに「次へ」をクリックしてください。

 <img src="/imgs/vscode_install4.png" />

　次に進むと VScode のインストール項目の確認が表示されます。ここで問題がなけらば「インストール」をクリックしてください。

 <img src="/imgs/vscode_install5.png" />

すると VScode のインストールが始まります。終了するまでまちましょう。

 <img src="/imgs/vscode_install6.png" />

　VScode のインストールが成功するとこのような画面が表示されます。「Visual Studio Code を実行する」にチェックが入っていると、「完了」ボタンをクリックすると VScode が起動します。

 <img src="/imgs/vscode_install7.png" />

　「完了」ボタンをクリックすると VScode がこのように起動します。これで VScode のインストールは完了です！

 <img width=25%/><img src="/imgs/vscode_window.png" width=50% />

## VScode の環境構築をする（日本語にする方法）

　インストールされた VScode のデフォルト言語は 
**英語**
です。日本語にするための手順を紹介します。

　まず、起動している VScode 上で、キーボードショートカット「**Control + Shift + P**」を行ってください。すると以下のように「コマンドパレット」と呼ばれるインターフェースが表示されます。<br>
 コマンドパレットの `>` の後に **`configure`** と入力してください。
 
<img src="/imgs/vscode_lang1.png" width=50% />

　すると、コマンドパレット下に候補として **`Configure Display Language`** が表示されるので、この項目をクリックしてください。

<img src="/imgs/vscode_lang2.png" width=50% />

　するとこのように言語の選択項目が表示されます。「日本語」をクリックしてください。

<img src="/imgs/vscode_lang3.png" width=50% />

　このようなポップアップが表示されます。「Restart」ボタンをクリックすると VScode が再起動します。

<img src="/imgs/vscode_lang4.png" width=50% />

　しばらくすると VScode は自動的に再起動します。再起動後の VScode が日本語になれば成功です！
 
<img src="/imgs/vscode_lang5.png" width=50% />

## VScode の環境構築をする（開発用拡張機能をインストールする）

　ロボット開発をするためには **WSL2** という仮想環境を使用します。VScode で WSL2 にアクセスし、コードを記述、編集できるようにするための拡張機能をインストールする方法を解説します。

　VScode を開き、VScode の画面左端にある
 **田みたいなマーク**
 「拡張機能」をクリックしてください。

<img src="/imgs/vscode_ext1.png" width=50% />

　拡張機能をクリックするとこのような画面が表示されます。検索バーをクリックしてください。

<img src="/imgs/vscode_ext2.png" width=50% />

　検索バーに **`Remote`** と入力してください。すると検索結果に「**Remote Development**」という項目が表示されます。

<img src="/imgs/vscode_ext3.png" width=50% />

　青い「インストール」ボタンをクリックしてこの拡張機能をインストールしてください。

<img src="/imgs/vscode_ext4.png" width=50% />

　インストールが完了すると「インストール」ボタンが消え、拡張機能アイコンの上にテレビモニターのようなアイコンが表示されます。これで拡張機能のインストールは完了です！

<img src="/imgs/vscode_ext5.png" width=50% />
