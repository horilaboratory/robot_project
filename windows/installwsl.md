# WSL2 をセットアップする方法
## WSL2 とは？
<img width=25%/> <img width=50% src='https://abrictosecurity.com/wp-content/uploads/2024/02/wsl_logo.png'/>

　Windows Subsystem for Linux 2（WSL2）とは、Windows 上で Linux OS を動作させるための仮想プラットフォームです。

## WSL2 をインストールする
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
　WSL2 をインストールするとき、追加で自動的に最新の Ubuntu がインストールされることがあります。最新の Ubuntu は **バージョン 24.04** で、この授業で使用するバージョンに対応していません。
そのためもしインストールされた場合 Ubuntu をここで削除します。削除方法は簡単で、設定アプリからインストールされているアプリ一覧にある **Ubuntu** をアンインストールするだけです。もしはじめからアプリ一覧になければこの手順はスキップして次に進んでください。

<img src="/imgs/uninstall_ubuntu.png" width=50% />

---

 - [WSL2 をインストールする方法に戻る](./hostsetup.md#wsl2)
