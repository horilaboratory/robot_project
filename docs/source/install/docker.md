# Docker をつかい ROS2 をインストールする方法

　このセクションでは Docker をつかい ROS2 Humble をインストールする方法を解説します．

1. **Docker をインストールする**<br>

    1. **apt リポジトリの更新**<br>
        ```bash
        sudo apt update
        ```
    
    1. **Docker をインストールするために必要なパッケージをインストールする**<br>
        ```bash
        sudo apt install -y ca-certificates curl gnupg lsb-release
        ```

    1. **認証鍵の保存先を作成**<br>
        ```bash
        sudo mkdir -p /etc/apt/keyrings
        ```
    
    1. **Docker の公式 GPG キーを取得する**<br>
        ```bash
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
        ```
        ```bash
        echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
        $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
        ```

    1. **apt リポジトリの更新**<br>
        ```bash
        sudo apt update
        ```
    
    1. **Docker をインストールする**<br>
        ```bash
        sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
        ```
    
    1. **Docker グループにユーザーを追加する**<br>
        ```bash
        sudo usermod -aG docker $USER
        ```
    
    1. **システムを再起動する**<br>
        ```bash
        sudo reboot
        ```
        ```{warning}
        WSL を使用されている方は以下のコマンドを **PowerShell** 上で実行してください．
        ```PS
        wsl --shutdown
        ```

1. **動作確認**<br>
    ```bash
    docker --version
    ```
    Docker のバージョンが表示されれば成功です．