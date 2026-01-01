# erasers_kachaka WSL2 イメージのインストール

```{note}
このセクションは **ロボットプロジェクト受講者** 向けです．
```

1. **Terminal から PowerShell を開く**
1. **`wsl` ディレクトリを作成する**<br>
    以下のコマンドをコピペして実行してください．
    ```powershell
    mkdir C:\wsl
    ```
1. **`ホームディレクトリに移動する**<br>
    以下のコマンドをコピペして実行してください．
    ```powershell
    cd ~
    ```
1. **`erasers_kachaka` イメージをインポート＆インストールする**<br>
    以下のコマンドは `erasers_kachaka.tar` を展開してインストールするコマンドです．以下のコマンドをコピペして実行してください．
    ```powershell
    wsl --import erasers_kachaka C:\wsl\erasers_kachaka .\Downloads\erasers_kachaka.tar
    ``` 
1. **インストール後確認**<br>
    インストールが完了したら以下のコマンドを実行して利用可能な WSL2 イメージ一覧を取得します．
    ```powershell
    wsl -l -v
    ```
    以下の出力が表示されることを確認してください．
    ```powershell
    NAME            STATE           VERSION
    * erasers_kachaka Stopped         2
    ```
1. **動作確認**<br>
    以下のコマンドを実行すると `erasers_kachaka` イメージが起動します．
    ```powershell
    wsl -d erasers_kachaka -u roboworks --cd ~
    ```
    上記コマンドを実行すると，図のようにプロンプトの見た目が変わります．図のような変化が見られれば成功です．
    |実行前|実行後|
    |:---:|:---:|
    |<img src="https://i.imgur.com/HmZbGwh.png"/>|<img src="https://i.imgur.com/BEuNmVR.png"/>|

    変化が見られた場合は，その状態でキーボードショートカットキー「Control + D」を実施して，WSL2 イメージを停止してください．もとの PowerShell プロンプトに戻れば動作確認完了です．

    ```{error}
    もし，上記コマンド実行時に以下のエラーが発生した場合，**お使いのコンピューターの CPU の仮想化が無効化されている可能性があります．**
    ```text
    WSL2 は、現在のマシン構成ではサポートされていません。
    "仮想マシン プラットフォーム" オプション コンポーネントを有効にし、さらに、BIOS で仮想化を有効にしてください。
    "仮想マシン プラットフォーム" を有効にするには、次のコマンドを実行します: wsl.exe --install --no-distribution
    詳細については、https://aka.ms/enablevirtualization をご覧ください
    エラー コード: Wsl/Service/CreateInstance/CreateVm/HCS/HCS_E_HYPERV_NOT_INSTALLED
    Press any key to continue..
    ```