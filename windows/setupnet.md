# Hyper-V でネットワーク設定を行う

> [!CAUTION]
> 　このドキュメントを読む前に、[**Windows でロボット開発をはじめる**](/windows/hostsetup.md) を読んで、**「Hyper-V をインストールする」** までの作業を終えていることを確認してください。

　コンピューターに LAN ポート（Ethernet）があることを確認してください。もし、コンピューターに LAN ポート（Ethernet）がない場合、USB から LAN に変換するケーブルなどを接続して LAN が利用できるようにしてください。

<p align="center">
<img width=25% src='https://images.assetsdelivery.com/compings_v2/aayam4d/aayam4d1907/aayam4d190700850.jpg'/><img width=25% src='https://m.media-amazon.com/images/I/51y48YAw2-L._AC_UF894,1000_QL80_.jpg'/><br>
左図：LAN ポート。右図：USB から LAN に変換するポート
</p>

## Hyper-V マネージャーを開く
　スタートメニューで「Hyper-V」と入力して、
**Hyper-V マネージャー**
というソフトを起動してください。

<img src="/imgs/wsl_install2.png" width=50% />

## 仮想スイッチマネージャーを開く
　Hyper-V マネージャーの右側にある
 **「仮想スイッチマネージャー」**
 をクリックしてください。

 <img width=50% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2FPXRp8UV.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=dfbb55f2be61ff022b9f0190cc009ae5" />

 > [!TIP]
> もし上記のような項目が表示されない場合、以下のような画面になっている可能性があります。この場合、ウィドウの左側の **Hyper-V マネージャー** の項目の下にあるお使いのコンピューターの名前の項目をクリックしてください。
> <img width=80% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2FfSix43R.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=17aef10e72dea2b31293afc0b0f8400a" />

## 仮想スイッチマネージャー `vm_ethernet` を作成する
　仮想スイッチマネージャーウィンドウの「新しい仮想スイッチマネージャー」をクリックし、作成する仮想スイッチの種類を「外部」にします。
