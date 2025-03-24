# Hyper-V をインストールする

> [!CAUTION]
> 　このドキュメントを読む前に、[**Windows でロボット開発をはじめる**](/windows/hostsetup.md) を読んで、**「erasers_kachaka をインストールする」** までの作業を終えていることを確認してください。

## PowerShell を開く
　ターミナルを開き、PowerShell を起動してください。

## bat スクリプトファイルを作成する
　以下のコマンドを実行して VScode から Hyper-V をインストールするための bat スクリプトを作成します。以下のコマンドを実行してファイルを作成します。
```powershell
cd ~ ; code .\setupwsl.bat
```
　ファイルを作成したら以下のコードを VScode 上にコピペしてください。コピペしたらファイルを Control キー + S キー で変更を保存してください。
```bat
pushd "%~dp0"

dir /b %SystemRoot%\servicing\Packages\*Hyper-V*.mum >hyper-v.txt

for /f %%i in ('findstr /i . hyper-v.txt 2^>nul') do dism /online /norestart /add-package:"%SystemRoot%\servicing\Packages\%%i"
del hyper-v.txt

Dism /online /enable-feature /featurename:Microsoft-Hyper-V -All /LimitAccess /ALL
```
<img src="/imgs/wsl_install0.gif" />

## bat スクリプトを実行する
　VScode を閉じて、以下のコマンドを実行して bat スクリプトが保存されているファイル「ホームディレクトリ」を開きます。
```bat
Invoke-Item ~
```
先ほど作成したファイル setupwsl.bat を右クリックして「管理者として実行」をクリックしてスクリプトを実行してください。

<img src="/imgs/wsl_install1.gif" />

スクリプトの実行が完了すると、以下のように最後に再起動するかどうかを聞かれます。そしたら「Y」を押してください。 Y を入力した瞬間コンピューターが再起動します。

<img src="/imgs/wsl_install1.png" width=50% />


---

- [Windows でロボット開発をはじめるのセクション **Hyper-V をインストールする** に戻る](/windows/hostsetup.md/#hyperv)

## Hyper-V の確認
　再起動後スタートメニューで「Hyper-V」と入力して、
**Hyper-V マネージャー**
というソフトがあれば Hyper-V のインストールは成功です。

<img src="/imgs/wsl_install2.png" width=50% />
