# Ubuntu 内に Docker をインストールする方法
## Docker のインストール方法
　ターミナルを開き、以下のコマンドを実行して apt リポジトリを更新します。
```bash
sudo apt update
```
　このとき、パスワードの入力が求められるので、
**ユーザー作成時に設定したパスワード**
を入力してください。

---

　次に、以下のコマンドを実行して、Docker をインストールするために必要なパッケージたちをインストールします。
```bash
sudo apt install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
```
　このとき、
`この操作後に追加で 597 kB のディスク容量が消費されます。続行しますか? [Y/n]`
とプロンプト上で尋ねられるので、
**y**
を入力してエンターキを押してインストールを続行してください。

---

　次に、以下のコマンドを実行して Docker の公式GPGキーを追加します。
このコマンドを実行することで、Docker をインストールするために必要なキー（GPGキー）を取得します。
```bash
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
```

---

　次に、以下のコマンドを実行して Docker のリポジトリを追加します。
このコマンドを実行することで Docker のインストール準備が整います。
```bash
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
```
　このとき、
`続けるには「Enter」キーを、中止するにはCtrl-cを押してください`
とプロンプト上で尋ねられるので、エンターキーを押して作業を続行してください。

---

　次に、以下のコマンドを実行して先ほど追加されたリポジトリを読み込みます。
```bash
sudo apt update
```

---

　次に、以下のコマンドを実行して Docker をインストールします。
```bash
sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin
```

### 動作確認
　これで Docker のインストールは完了です！以下のコマンドを実行すると、インストールされた Docker のバージョンを確認することができます。
<br>
　このコマンドを実行して Docker のバージョン情報が帰ってきたら成功です。
```bash
docker --version
```
　インストールが成功していると以下のメッセージが返ってきます。
`xx.y.z`
にインストールされた Docker のバージョン。
`abcdefg`
にビルド情報が表示されます。
```
Docker version xx.y.z build abcdefg
```

## 管理者権限なしで実行できるようにする方法
　インストールされた Docker は、以下のように Docker を扱うコマンドを実行するとエラーが発生します。
```bash
docker images
```
発生したエラー
```
permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Head "http://%2Fvar%2Frun%2Fdocker.sock/_ping": 
dial unix /var/run/docker.sock: connect: permission denied
```
　このエラーは、ログインしているユーザーが Docker にアクセスする権限を持っていないことが原因です。そのため、以下の手順に従ってログインしているユーザーで Docker を利用できるようにします。

1. 以下のコマンドを実行して Docker グループにログインしているユーザーを追加します。
    ```bash
    sudo usermod -aG docker $USER
    ```

2. 次に以下のコマンドを実行して設定を反映させます。<br>
    このコマンドを実行することで一度ログアウトさせなくてもグループ情報を反映させることができます。
    ```bash
    newgrp docker
    ```

　これで設定は完了です。以下のコマンドを実行して Permission Error が発生しなければ成功です！
```
docker images
```

---

　これで Docker のインストールは完了です！
