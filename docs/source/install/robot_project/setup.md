# erasers_kachaka WSL2 イメージのセットアップ

```{note}
このセクションは **ロボットプロジェクト受講者** 向けです．
```

今後 `erasers_kachaka` イメージ内でロボット開発を進めるうえで，毎度 PowerShell 上で以下のコマンドを実行するのは面倒でしょう．
```powershell
wsl -d erasers_kachaka -u roboworks --cd ~
```
このセクションでは Terminal の「プロファイル」機能をつかいプロファイルから erasers_kachaka イメージを簡単に起動させる方法を解説します．

1. **Terminal を開く**<br>
1. **Terminal ウィンドウ上でキーボードショートカットキー「Control + ,」を実施する**<br>
    Terminal 上に以下の Terminal 設定画面が表示されます．
    <br><img src="https://i.imgur.com/nJ2P03s.png"/>
1. **設定画面サイドバーの「プロファイル」項目にある「+ 新しいプロファイルを追加します」をクリックする**<br>
    <br><img src="https://i.imgur.com/IFW2QO8.png"/>
1. **「+ 新しい空のプロファイル」をクリックする**<br>
    <br><img src="https://i.imgur.com/DduTIWW.png"/>
1. **「名前」プロパティをクリックして，「`erasers_kachaka`」と入力する**<br>
    <br><img src="https://i.imgur.com/ftbsihW.png"/>
1. **「コマンドライン」プロパティをクリックして，プロファイル呼び出し時のコマンドを定義する**<br>
    以下のコマンドをプロパティにコピペしてください．
    ```powershell
    wsl -d erasers_kachaka -u roboworks --cd ~
    ```
    <br><img src="https://i.imgur.com/k2gK5mg.png"/>
1. **アイコンを設定する（任意）**<br>
    アイコンプロパティの左側のメニューを「ファイル」にして，以下の URL をプロパティに張り付けて `erasers_kachaka` アイコンを追加できます．
    ```text
    https://github.com/horilaboratory/robot_project/blob/wsl-doc/docs/source/_static/erasers_kachaka.jpeg?raw=true
    ```
    <br><img src="https://i.imgur.com/Cp9hzin.png"/>
1. **「このプロファイルを管理者として実行する」にチェックが入っていないことを確認する**<br>
    <br><img src="https://i.imgur.com/r8MMIPq.png"/>
1. **シェルプロンプトの外観を設定する**<br>
    以下の手順でシェルの外観を Ubuntu（Linux）風にして，わかりやすくします．

    1. 追加の設定」項目にある「外観」メニューをクリックしてください．
        <br><img src="https://i.imgur.com/23y4Jv6.png"/>
    1. 「配色」をクリックします．
        <br><img src="https://i.imgur.com/KJBzzew.png"/>
    1. 配色一覧の「Ubuntu-22.04~...」を選択してください．
        <br><img src="https://i.imgur.com/F8ZwEdR.png"/>
    1. このような見た目になっていることを確認してください．
        <br><img src="https://i.imgur.com/mifcH0t.png"/>
1. **「保存」をクリックする**<br>
1. **Terminal の「+」横の「v」から `erasers_kachaka` をクリックして動作確認をする**<br>
    以下のプロンプトシェルが開けば成功です．
    <br><img src="https://i.imgur.com/pkDtCrF.png"/>