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
　ファイルを作成したら以下のコードを VScode 上にコピペしてください。コピペしたらファイルを Control キー + S キー で変更を保存してください。
```
[wsl2]
networkingMode=bridged
vmSwitch=vm_ethernet
dhcp=true
localhostForwarding=True
```
ファイルの編集が完了したら PowerShell で以下のコマンドを実行して WSL を再起動してください。
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
