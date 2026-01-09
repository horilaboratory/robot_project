# `erasers_kachaka` 起動方法

このチュートリアルでは，`erasers_kachaka` を起動する方法を解説します．セットアップ方法によって起動順序に相違がありますので，お使いの環境に応じて適切なタブを選択してください．

## 事前準備

```{eval-rst}

.. tabs::

    .. group-tab:: Linux

        #. **ターミナルを開く**

    .. group-tab:: WSL2

        #. **仮想インターフェースを WSL2 に適応させる**
        
            Kachaka と接続するために，WSL が Hyper-V で作成した仮想インターフェースを利用できるように設定する必要があります．

            事前に :doc:`/install/hyperv/index` を読み，``vm_ethernet`` という名前の仮想インターフェースを作成してください．

            次に，PowerShell を開き，以下のコマンドを実行して `.wslconfig` ファイルをメモ帳から開きます．

            .. code-block:: powershell

                notepad $env:USERPROFILE\.wslconfig
            
            ここに以下の内容を記述，変更してください．

            .. code-block:: ini

                [wsl2]
                networkingMode=bridged
                vmSwitch=vm_ethernet
                dhcp=true
                localhostForwarding=True
            
            変更を保存後，PowerShell で以下のコマンドを実行して，WSL2 を再起動させてください．

            .. code-block:: powershell

                wsl --shutdown
        
        #. **WSL を起動する**

            Terminal アプリのプロファイルから WSL2 のディストリビューションを選択して起動してください．
    
    .. group-tab:: Docker

        #. **ワークスペースに移動する**

            以下のコマンドを実行して ``erasers_kachaka`` ディレクトリに移動してください．

            .. code-block:: bash

                cd erasers_kachaka

```


## `erasers_kachaka` を起動する


```{eval-rst}

.. tabs::

    .. group-tab:: Linux

        #. **Kachaka と接続する**

            ``~/.bashrc`` を開き，環境変数 ``KACHAKA_IP`` を接続する Kachaka の IP アドレスに変更してください．

            .. tip::

                WI-Fi（無線）で Kachaka と接続する場合は，Kachaka にむけて「ねぇカチャカ，IP アドレスを教えて」と話しかけると IP アドレスを教えてくれます．

            .. code-block:: diff

                - export KACHAKA_IP=...
                + export KACHAKA_IP=<接続先の Kachaka の IP アドレス>

            編集後，変更を保存して以下のコマンドを実行して ``~/.bashrc`` を読み込んでください．

            .. code-block:: bash

                source ~/.bashrc

            次に，以下のコマンドを実行して接続先の Kachaka と疎通するか試してください．

            .. code-block:: bash

                ping $KACHAKA_IP

            以下のようなログが表示されれば成功を示しています．

            .. code-block:: text

                64 bytes from <Kachaka の IP アドレス>: icmp_seq=X ttl=XX time=XX ms

        #. **erasers_kachaka を起動する**

            以下のコマンドを実行して ``erasers_kachaka`` を起動します．

            .. code-block:: bash

                ros2 launch erasers_kachaka_bringup bringup.launch.py

            起動に成功すると Kachaka から「Kachaka，スタート！」と発話します．そして表示される Rviz にロボットからのステータスが表示されます．

            .. image:: https://i.imgur.com/We4FrEm.jpeg

    .. group-tab:: WSL2

        #. **Kachaka と接続する**

            ``~/.bashrc`` を開き，環境変数 ``KACHAKA_IP`` を接続する Kachaka の IP アドレスに変更してください．

            .. tip::

                WI-Fi（無線）で Kachaka と接続する場合は，Kachaka にむけて「ねぇカチャカ，IP アドレスを教えて」と話しかけると IP アドレスを教えてくれます．

            .. code-block:: diff

                - export KACHAKA_IP=...
                + export KACHAKA_IP=<接続先の Kachaka の IP アドレス>

            編集後，変更を保存して以下のコマンドを実行して ``~/.bashrc`` を読み込んでください．

            .. code-block:: bash

                source ~/.bashrc

            次に，以下のコマンドを実行して接続先の Kachaka と疎通するか試してください．

            .. code-block:: bash

                ping $KACHAKA_IP

            以下のようなログが表示されれば成功を示しています．

            .. code-block:: text

                64 bytes from <Kachaka の IP アドレス>: icmp_seq=X ttl=XX time=XX ms

        #. **erasers_kachaka を起動する**

            以下のコマンドを実行して ``erasers_kachaka`` を起動します．

            .. code-block:: bash

                ros2 launch erasers_kachaka_bringup bringup.launch.py

            起動に成功すると Kachaka から「Kachaka，スタート！」と発話します．そして表示される Rviz にロボットからのステータスが表示されます．

            .. image:: https://i.imgur.com/We4FrEm.jpeg
    
    .. group-tab:: Docker

        #. **Kachaka と接続する**

            Kachaka と疎通するか確認するために，以下のコマンドを実行して Kachaka と疎通するか試してください．

            .. tip::

                WI-Fi（無線）で Kachaka と接続する場合は，Kachaka にむけて「ねぇカチャカ，IP アドレスを教えて」と話しかけると IP アドレスを教えてくれます．

            .. code-block:: bash

                ping <接続先の Kachaka の IP アドレス>

            例えば接続先の Kachaka の IP アドレスが ``192.168.8.11`` であれば，以下のコマンドを実行してください．

            .. code-block:: bash

                ping 192.168.8.11

            以下のようなログが表示されれば成功を示しています．

            .. code-block:: text

                64 bytes from <Kachaka の IP アドレス>: icmp_seq=X ttl=XX time=XX ms

        #. **環境変数を編集する**

            ``erasers_kachaka`` ディレクトリ内にある ``kachaka_env`` ファイルを開いて，次の環境変数を編集し，接続したい Kachaka の IP アドレスに書き換えてください．

            .. code-block:: diff

                - KACHAKA_IP=...
                + KACHAKA_IP=<接続先の Kachaka の IP アドレス>

        #. **erasers_kachaka を起動する**

            以下のコマンドを実行して Docker コンテナからの GUI 出力を許可します．

            .. code-block:: bash

                xhost +

            次に，以下のコマンドを実行して ``erasers_kachaka`` を起動します．

            .. code-block:: bash

                docker compose --env-file kachaka_env up erasers_kachaka nomap_bridge

            起動に成功すると Kachaka から「Kachaka，スタート！」と発話します．そして表示される Rviz にロボットからのステータスが表示されます．

            .. image:: https://i.imgur.com/We4FrEm.jpeg

```