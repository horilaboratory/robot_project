# ターミナルの利用について
<img width=25%/> <img width=50% src="https://upload.wikimedia.org/wikipedia/commons/thumb/5/51/Windows_Terminal_logo.svg/640px-Windows_Terminal_logo.svg.png"/>

　Ubuntu の操作、Windows 内部の設定変更やコマンドの実行に関して、本授業ではこの **ターミナル** というアプリをメインに使用します。ここではターミナルの使用方法を軽く説明します。

---

　ターミナルは Windows 上で利用できるコマンドラインインタフェース（CLI）を１つにまとめたものです。どういうことかというと、**コマンドをつかって操作する機能を一つにまとめたアプリ** と考えればいいでしょう。ターミナル内では **PowerShell**、**コマンドプロンプト** などを選択して利用できます。また、今回のように Ubuntu、WSL2 をインストールすればターミナルから Ubuntu を操作することができます。

## ターミナルを起動する
　ターミナルの起動方法は簡単です。スタートメニューからターミナルを検索して「ターミナル」を起動するだけです。ターミナルを起動すると、**デフォルトで PowerShell が起動します。**

<img src="/imgs/open_terminal.png" width=50% /><img src="/imgs/opened_terminal.png" width=50% />

## PowerShell
　PowerShell は Windows をコマンドで操作するコマンドラインインターフェース（CLI）です。ターミナルのデフォルトインターフェースで、ターミナルのロゴに **PowerShell** と表示されます。

<img src="/imgs/opened_terminal.png" width=50% />

## ターミナルから Ubuntu を起動する方法
　ターミナルから直接 Ubuntu を起動することができます。以下の動画を参考に、ターミナルのタブ横にある「+」を押して、**Ubuntu 22.04** を選択してください。ターミナルから Ubuntu を起動すると、動画にようにターミナルの画面色が紫色になり、PowerShell を開いているのか Ubuntu を開いているのか一目でわかるようになります。

<img src="/imgs/terminal1.gif" width=50% />

## デフォルトを Ubuntu にする方法
　デフォルトではターミナルを起動すると PowerShell が起動しますが、以下の動画のように設定からデフォルトの起動プロファイルを Ubuntu にすることで、ターミナル起動時にすぐに Ubuntu が利用できるようになります。

 <img src="/imgs/terminal2.gif" width=50% />

---

- [Windows でロボット開発をはじめるのセクション **ターミナルについて** に戻る](/windows/hostsetup.md/#terminal)
