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
- **WSL2**
  - **ROS2**
  - **Docker**
  - **erasers_kachkaa** 
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

## WSL2 をインストールする方法
### WSL2 とは？
<img width=25%/> <img width=50% src='https://abrictosecurity.com/wp-content/uploads/2024/02/wsl_logo.png'/>

　Windows Subsystem for Linux 2（WSL2）とは、Windows 上で Linux OS を動作させるための仮想プラットフォームです。

### WSL2 をインストールする
　デフォルトの Windows 11 では WSL2 はインストールされていません。インストールするには PowerShell というコマンドを入力するおあれっとを使用する必要があります。
そのために、**Windows キーを押してスタートメニューを開き、`ターミナル` と検索してください。**

<img src="/imgs/open_terminal.png" width=50% />

候補に挙がった **ターミナル** を起動すると、このような黒いウィンドウが起動します。これが PowerShell で、コマンドを使いコンピューターを操作することができるインターフェースです。

<img src="/imgs/opened_terminal.png" width=50% />

　WSL2 をインストールするにはコマンドを使います。以下のコマンドをターミナル内に入力してください。コマンドを入力後エンターキーを押すと、入力されたコマンドが実行されます。
```powershell
wsl --install
```
> [!NOTE]
> エンターキーを押す前に **入力されたコマンドにタイプミスがないか確認しましょう。**

　コマンドを実行すると、以下のように管理者権限でシステムに変更を加えることを許可するかどうかのメッセージが表示されるので **「はい」** をクリックしてください。<br>
　すると右の動画のように WSL2 のインストールが開始されます。
 
<img src="https://osechi-tech.net/wp-content/uploads/2021/12/image-95.png" width=50% />
<img src="/imgs/wsl_install.gif" width=50% />

```
要求された操作は正常に終了しました。変更を有効にするには、システムを再起動する必要があります。
```
というメッセージが PowerShell 上で表示されたら、コンピューターを再起動してください。

> [!TIP]
> 以下のコマンドを実行するとコンピューターの再起動ができます。
> ```
> Restart-Computer
> ```

　再起動後ターミナルを開いて以下のコマンドを実行してみてください。以下の画像のように WSL2 の情報が表示されたら成功です。
```
wsl -v
```

> [!TIP]
> コマンド `wsl -v` はインストールされた WSL2 の情報を表示してくれます。

<img src="/imgs/wsl_check.png" width=50% />

## WSL2 に Ubuntu をインストールする
### Ubuntu とは？
　Ubuntu とは、Linux OS の一種です。ここではこの Ubuntu のバージョン 22.04（Ubuntu 22.04 Jammy Jellyfish）を使用します。

### 自動インストールされた Ubuntu を削除する
　WSL2 をインストールするとき、追加で自動的に最新の Ubuntu がインストールされます。最新の Ubuntu は **バージョン 24.04** で、この授業で使用するバージョンに対応していません。
そのためインストールされた Ubuntu をここで削除します。削除方法は簡単で、設定アプリからインストールされているアプリ一覧にある **Ubuntu** をアンインストールするだけです。

<img src="/imgs/uninstall_ubuntu.png" width=50% />

### Ubuntu 22.04 をインストールする
　Ubuntu 22.04 は **Micrrosoft Store** でアプリケーションとして用意されています。スタートメニューで **store** と検索し、候補に挙がった **Microsoft Store** を起動します。

<img src="/imgs/open_store.png" width=50% />

　ストア内の検索ボックスに **"ubuntu"** と入力してください。検索結果にインストール可能な Ubuntu 一覧が表示されます。その中にある **Ubuntu 22.04 から始まるアプリケーションをインストールしてください。**

<img src="/imgs/ubuntu_install1.png" width=50% />

　インストールが完了すると開くことができます。ストア経由で Ubuntu 22.04 を起動してください。これで Ubuntu のインストールは完了です。

<img src="/imgs/ubuntu_install2.png" width=50% />

<a id="setubuntu"></a>
### Ubuntu 内部のユーザー設定をする
　初回で Ubuntu を起動すると、このようなウィンドウが表示されます。PowerShell に似ていますがこの画面が **Ubuntu です。**<br>
 プロンプトに `root` と表示されている場合、現在 **管理者権限で Ubuntu を操作している** ことを示しており、つまり **この Ubuntu にはユーザーが存在しない** ことを示しています。

<img src="/imgs/ubuntu_setup1.png" width=50% />

> [!CAUTION]
> もし、ターミナルに上記のような画面が表示されず、以下の図のようなメッセージが表示されたら [コンピューターの仮想化を有効化する方法](./enablevm.md) を参照してください。
> ```
> Installing, this may take a few minutes...
> WslRegisterDistribution failed with error: 0x80370102
> Please enable the Virtual Machine Platform Windows feature and ensure virtualization is enabled in the BIOS.
> For information please visit https://aka.ms/enablevirtualization
> Press any key to continue...
> ```
> <br>
> <img src="/imgs/error_vm.png" width=50% />

　これから Ubuntu 内部のユーザーを作成する方法を解説します。まず、Ubuntu ターミナル上で以下のコマンドを入力してください。
```bash
adduser
```
そのあとに追加したいユーザー名を入力してください。あなたの名前であることが好ましいですが、**必ず英数表記で入力しましょう。** 日本語で入力することは強く推奨しません。例えば **`gai`** という名前のユーザーを追加する場合は、以下のように `adduser` の後にユーザー名を記述します。
```bash
adduser gai
```
入力が完了したらエンターキーを押してコマンドを実行します。するとこのように新規ユーザー用パスワード入力が求められます。覚えやすいパスワードを設定しましょう。例えば、Windows にログインするときに使う PIN コードなどを利用するのがよいです。パスワードを入力したらエンターキーを押してください。

> [!NOTE]
> パスワードは **忘れないように気を付けてください。** 忘れると再度 Ubuntu をインストールすることになる可能性があります。

> [!TIP]
> パスワードを入力しても **画面には何も表示されません。** そのため注意してパスワードを設定しましょう。

<img src="/imgs/ubuntu_setup2.png" width=50% />

　次に進むと再度パスワードの入力が求められます。先ほど入力したパスワードをもう一度入力してエンターキーを押してください。

<img src="/imgs/ubuntu_setup3.png" width=50% />

　次に進むとフルネーム入力など個人情報の入力が求められますがここは何も入力しなくても構いません。エンターキーを押して次に進みます。

<img src="/imgs/ubuntu_setup4.png" width=50% />

　ほかに追加で4つの項目について尋ねられますが、すべてエンターキーを押して無視してかまいません。最後に以下の図のように確認を求められるので `Y` を入力したらエンターキーを押して設定を確定します。

<img src="/imgs/ubuntu_setup5.png" width=47.5% /><img width=5% /><img src="/imgs/ubuntu_setup6.png" width=47.5% />

これでユーザーの追加は完了です。次に追加したユーザーに管理者権限を与えます。ユーザーに管理者権限を与えるには以下のコマンドを使います。
```bash
sudo gpasswd -a 先ほど作成したユーザー名 sudo
```
例えば、 `gai` という名前のユーザーを追加したならば、以下のように `先ほど作成したユーザー名` を `gai` に置き換えます。
```bash
sudo gpasswd -a gai sudo
```
入力したらエンターキーを押してください。すると右下の図のようにユーザーを `sudo` というグループに追加したというメッセージが表示されます。

> [!TIP]
> Super User Do（sudo）というグループに追加されたユーザーは管理者権限をもちます。ちなみに `sudo` は "スドー" ではなく "スデゥー" と呼ぶことが一般的です。<br>
> `sudo` をコマンドの文頭につけることで、管理者権限でコマンドを実行することができるようになります。

<img src="/imgs/ubuntu_setup7.png"  width=47.5% /><img width=5% /><img src="/imgs/ubuntu_setup8.png" width=47.5% />

## Ubuntu 起動時のデフォルトユーザーを変更する
　現在起動している Ubuntu ターミナルの×ボタンを押して Ubuntu を終了してください。前回 Ubuntu にユーザーを追加しましたが、再度 Ubuntu を起動しても `root` ユーザーとして起動してしまいます。
これを防ぐための方法を解説します。まずは以下のコマンドを実行して Ubuntu 22.04 が正常に存在するかどうか確認してください。以下の図のようなメッセージが表示されたら OK です。
```
wsl -l
```

<img src="/imgs/ubuntu_setup9.png" width=50% />

　次に以下のコマンドを実行して Ubuntu 起動時に前回作成したユーザーでログインされるように設定します。 `先ほど作成したユーザー名` をあなたが作成したユーザー名に置き換えてください。
```
ubuntu2204 config --default-user 先ほど作成したユーザー名
```
　以下の図では `gai` というユーザーでログインするように設定する場合です。
 
<img src="/imgs/ubuntu_setup10.png" width=50% />

　スタートメニューで Ubuntu と入力し、候補で表示された Ubuntu を起動すると、以下の図のようにユーザーとしてログインした状態で起動します。プロンプトが緑色になっていれば成功です！

<img src="/imgs/ubuntu_setup11.png" width=50% />

## ターミナルの利用について
<img width=25%/> <img width=50% src="https://upload.wikimedia.org/wikipedia/commons/thumb/5/51/Windows_Terminal_logo.svg/640px-Windows_Terminal_logo.svg.png"/>

　Ubuntu の操作、Windows 内部の設定変更やコマンドの実行に関して、本授業ではこの **ターミナル** というアプリをメインに使用します。ここではターミナルの使用方法を軽く説明します。

---

　ターミナルは Windows 上で利用できるコマンドラインインタフェース（CLI）を１つにまとめたものです。どういうことかというと、**コマンドをつかって操作する機能を一つにまとめたアプリ** と考えればいいでしょう。ターミナル内では **PowerShell**、**コマンドプロンプト** などを選択して利用できます。また、今回のように Ubuntu、WSL2 をインストールすればターミナルから Ubuntu を操作することができます。

### ターミナルを起動する
　ターミナルの起動方法は簡単です。スタートメニューからターミナルを検索して「ターミナル」を起動するだけです。ターミナルを起動すると、**デフォルトで PowerShell が起動します。**

<img src="/imgs/open_terminal.png" width=50% /><img src="/imgs/opened_terminal.png" width=50% />

### PowerShell
　PowerShell は Windows をコマンドで操作するコマンドラインインターフェース（CLI）です。ターミナルのデフォルトインターフェースで、ターミナルのロゴに **PowerShell** と表示されます。

<img src="/imgs/opened_terminal.png" width=50% />

### ターミナルから Ubuntu を起動する方法
　ターミナルから直接 Ubuntu を起動することができます。以下の動画を参考に、ターミナルのタブ横にある「+」を押して、**Ubuntu 22.04** を選択してください。ターミナルから Ubuntu を起動すると、動画にようにターミナルの画面色が紫色になり、PowerShell を開いているのか Ubuntu を開いているのか一目でわかるようになります。

<img src="/imgs/terminal1.gif" width=50% />

### デフォルトを Ubuntu にする方法
　デフォルトではターミナルを起動すると PowerShell が起動しますが、以下の動画のように設定からデフォルトの起動プロファイルを Ubuntu にすることで、ターミナル起動時にすぐに Ubuntu が利用できるようになります。

 <img src="/imgs/terminal2.gif" width=50% />

<a id="ros2"></a>
## ROS2 Humble をインストールする
　ロボット開発に必要なミドルウェア **ROS2 Humble** をインストールします。以下のリンクをクリックすると ROS2 Humble のインストール手順が書かれているため、ドキュメントの内容を読んで作業してください。作業が終わったらこのドキュメントに戻ってきてください。

- [**ROS2 Humble のインストールドキュメントに進む**](/ros/install.md)

<a id="docker"></a>
## Docker をインストールする
　ロボット開発に必要な仮想コンテナ管理ツール **Docker** をインストールします。以下のリンクをクリックすると Docker のインストール手順が書かれているため、ドキュメントの内容を読んで作業してください。作業が終わったらこのドキュメントに戻ってきてください。

- [**Docker のインストールドキュメントに進む**](/docker/install.md)

<a id="erk"></a>
## erasers_kachaka をインストールする
　Kachaka ロボットを制御するためのパッケージ **erasers_kachaka** をインストールします。以下のリンクをクリックすると erasers_kachaka のインストール手順が書かれているため、ドキュメントの内容を読んで作業してください。作業が終わったらこのドキュメントに戻ってきてください。

- [**erasers_kachaka のインストールドキュメントに進む**](/ros/er_install.md)

<a id="hyoerv"></a>
## Hyper-V をインストールする
　WSL とロボットのネットワークを接続するために必要な仮想環境管理ツール **Hyper-V** をインストールします。以下のリンクをクリックすると Hyper-V のインストール手順が書かれているため、ドキュメントの内容を読んで作業してください。作業が終わったらこのドキュメントに戻ってきてください。

- [**Hyper-V のインストールドキュメントに進む**](./hyperv.md)

<a id="setupnet"></a>
## Hyper-V でネットワーク設定を行う
　WSL とロボットのネットワークを接続するために Hyper-V を使い WSL とのネットワーク設定を行います。以下のリンクをクリックすると Hyper-V でネットワークの環境を構築する手順が書かれているため、ドキュメントの内容を読んで作業してください。作業が終わったらこのドキュメントに戻ってきてください。

- [**Hyper-V でネットワーク設定を行うに進む**](./setupnet.md)
