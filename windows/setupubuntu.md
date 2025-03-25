# Ubuntu 22.04 をセットアップする方法

> [!CAUTION]
> 　このドキュメントを読む前に、[**Windows でロボット開発をはじめる**](/windows/hostsetup.md) を読んで、**「WSL2 をインストールする方法」** までの作業を終えていることを確認してください。


## Ubuntu 22.04 をインストールする
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

しかし、`Enter new UNIX username ...` と表示されている場合、ここでユーザーの追加、パスワードの設定を行うことができます。このメッセージが表示されたら各作業を行ってください。ユーザーを無事作成できて、緑色のプロンプトが完了したらこのページの一番下までスクロールして次の作業に移ってください。

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

---

- [Windows でロボット開発をはじめるのセクション **HUbuntu 22.04 をセットアップする方法** に戻る](/windows/hostsetup.md/#ubuntu)
