# Docker をインストールする

> [!CAUTION]
> 　このドキュメントを読む前に、[**Windows でロボット開発をはじめる**](/windows/hostsetup.md) を読んで、**「Ubuntu 起動時のデフォルトユーザーを変更する」** までの作業を終えていることを確認してください。

## Ubuntu を開く
　ターミナルを開いて、Ubuntu を起動してください。

<img src="/imgs/ros2_install.png" width=50% />

## Dockerをインストールする
　以下のコマンドを実行してください。
```bash
sudo apt update && sudo apt install -y ca-certificates curl gnupg lsb-release &&\
sudo mkdir -p /etc/apt/keyrings &&\
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg &&\
echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null &&\
sudo apt update && sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin &&\
sudo usermod -aG docker $USER
```

> [!TIP]
> 上記コマンドはこの動画のようにコマンドブロック右上にあるコピーボタンを押してコピーしましょう。そしてターミナル内で Control + V をすることですべてのコマンドをペーストして一括実行できます。
> <img src="/imgs/ros2_install2.gif" width=50% />

インストールが完了したら、ターミナルを一度閉じて Ubunutu からサインアウトしてください。そしてもう一度 Ubuntu にログインして以下のコマンドを実行してください。
```bash
docker -v
```
このコマンドを実行して、以下のように Docker のバージョンとビルド情報が表示されることを確認してください。
```
Docker version 27.5.1, build 9f9e405
```
次に以下のコマンドを実行してください。
```bash
docker images
```
以下のようなメッセージが表示されたら成功です！
```
REPOSITORY   TAG       IMAGE ID   CREATED   SIZE
```

---

- [Windows でロボット開発をはじめるのセクション **Docker をインストールする** に戻る](/windows/hostsetup.md/#docker)
