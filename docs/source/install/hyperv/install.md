# Hyper-V をインストールする

　このセクションでは WSL2 を利用する上で必要なシステム **Hyper-V*** をインストールする手順を説明します．

　Windows 11 Home Edition などではデフォルトで Hyper-V はインストールされていません．よって以下の方法で Hyper-V をインストールします．

1. **ターミナルから PowerShell を起動して次のコマンドを実行してください**<br>
    　以下のコマンドを実行するとメモ帳「NotePad」アプリが起動します．
    ```powershell
    notepad .\setuphyperv.bat
    ```

1. **起動したメモ帳「NotePad」アプリに以下のコードをコピペしてください．**<br>
    　以下のコードはお使いの Windows 11 に Hyper-V をインストールします．
    ```{warning}
    以下のコードは必ずコピペしてください．手作業で記述することは強く推奨しません．
    ```
    ``` bat
    pushd "%~dp0"

    dir /b %SystemRoot%\servicing\Packages\*Hyper-V*.mum >hyper-v.txt

    for /f %%i in ('findstr /i . hyper-v.txt 2^>nul') do dism /online /norestart /add-package:"%SystemRoot%\servicing\Packages\%%i"
    del hyper-v.txt

    Dism /online /enable-feature /featurename:Microsoft-Hyper-V -All /LimitAccess /ALL
    ```
    　コードをコピペしたら「Control + S」でファイルを保存してください．

1. **作成したファイルをエクスプローラーから開く**<br>
    以下のコマンドを実行して咲くほど作成したファイルの保存先フォルダ「ホームディレクトリ」を開きます．
    ```powershell
    Invoke-Item ~
    ```

1. **管理者権限でプログラムを実行する**<br>
    エクスプローラーに表示されているファイル「 `setuphyperv.bat` 」を右クリックして，「管理者として実行」をクリックしてプログラムを実行します．するとコマンドプロンプトが開き，Hyper-V をインストールします．

1. **コンピューターを再起動する**<br>
    上記のプログラムを実行すると，コマンドプロンプト上にコンピューターを起動するかどうか尋ねられます．再起動しても問題ない場合「Y」キーをコマンドプロンプト上で入力してください．
    ```{warning}
    「Y」キーを入力した瞬間直ちにコンピューターは再起動します．ほかのアプリケーション上での作業が破棄される可能性があります．再起動してほしくない場合は「N」キーを入力してください．
    ```

1. **再起動後，Hyper-V を確認する**<br>
    再起動後スタートメニューで「Hyper-V」と入力して、 **Hyper-V マネージャー** というソフトがあれば Hyper-V のインストールは成功です。