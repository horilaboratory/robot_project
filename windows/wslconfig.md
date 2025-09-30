# WSL のネットワーク設定を行う

> [!CAUTION]
> 　このドキュメントを読む前に、[**Windows でロボット開発をはじめる**](/windows/hostsetup.md) を読んで、**「Hyper-V でネットワーク設定を行う」** までの作業を終えていることを確認してください。

## PowerShell を開く
　ターミナルを開き、PowerShell を起動してください。

## .`wslconfig` を作成する
　以下のコマンドを実行して Hyper-V で設定した内容を WSL に反映させる設定ファイル **`.wslconfig`** を作成します。以下のコマンドを実行してファイルを作成します。

> [!CAUTION]
> **`wslconfig` ではなく `.wslconfig` です。文頭に `.` をつけ忘れないでください。

```powershell
cd ~ ; code .\.wslconfig
```
　VScodeでファイルが開かれたら，以前記入した内容の次の行から以下の内容をコピペしてください。
```
networkingMode=bridged
vmSwitch=vm_ethernet
dhcp=true
localhostForwarding=True
```

コピペが完了したら以下のような内容になっていることを確認してください．(`memory=4GB`の数値はご自身の環境に合わせた値でかまいません)
```
[wsl2]
memory=4GB
swap=4GB
networkingMode=bridged
vmSwitch=vm_ethernet
dhcp=true
localhostForwarding=True
```

内容を確認できたら Control + S で変更を保存し，VSCodeを閉じてください．
次に，PowerShell で以下のコマンドを実行して WSL を再起動してください。
```bat
wsl --shutdown
```

> [!TIP]
> ### ネットワーク設定を無効にする方法
> 　`.wslconfig` で設定した項目により WSL 内で外部ネットワークにアクセスできなくなることがあります。これを回避したい場合は、再度 `.wslconfig` を編集して、ファイル上部の `[wsl2]` の文頭に `#` をつけて保存してください。
> ```
> #[wsl2]
> ...
> ```
> 　`.wslconfig` を編集したら PowerShell で WSL を再起動してください。

---

- [Windows でロボット開発をはじめるのセクション **WSL のネットワーク設定を行う** に戻る](/windows/hostsetup.md/#wslconfig)
