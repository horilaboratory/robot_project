# Hyper-V のネットワーク設定を行う

```{note}
Hyper-V がインストールされていない場合は
[Hyper-V をインストールする](./install.md)
を参照してください．
```

```{warning}
コンピューターに LAN ポート（Ethernet）があることを確認してください．もし、コンピューターに LAN ポート（Ethernet）がない場合、USB から LAN に変換するケーブルなどを接続して LAN が利用できるようにしてください．

|LAN（Ethernet）ポート|USB から LAN ポートに変換するケーブル|
|:---:|:---:|
|<img src="https://camo.githubusercontent.com/18e6562cfe2583647a672d4042d8400b212313d0dc4ff276e3922dcc800ba158/68747470733a2f2f696d616765732e61737365747364656c69766572792e636f6d2f636f6d70696e67735f76322f616179616d34642f616179616d3464313930372f616179616d34643139303730303835302e6a7067"/>|<img src="https://camo.githubusercontent.com/721066899d3c6595ab22d227ae0a059c52ed02462a3d891ab7f6562792cc5f97/68747470733a2f2f6d2e6d656469612d616d617a6f6e2e636f6d2f696d616765732f492f3531793438594177322d4c2e5f41435f55463839342c313030305f514c38305f2e6a7067"/>|

```

1. **Hyper-V マネージャーを開く**<br>
    スタートメニューで「Hyper-V」と入力して、**Hyper-V マネージャー**というソフトを起動してください。

1. **仮想スイッチマネージャーを開く**<br>
    Hyper-V マネージャーの右側にある **「仮想スイッチマネージャー」** をクリックしてください。
    <br><img width=50% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2FPXRp8UV.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=dfbb55f2be61ff022b9f0190cc009ae5" />

    ```{note}
    もし上記のような項目が表示されない場合、以下のような画面になっている可能性があります。この場合、ウィドウの左側の **Hyper-V マネージャー** の項目の下にあるお使いのコンピューターの名前の項目をクリックしてください。
    <br><img width=80% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2FfSix43R.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=17aef10e72dea2b31293afc0b0f8400a" />
    ```

1. **仮想スイッチマネージャー `vm_ethernet` を作成する**<br>
    仮想スイッチマネージャーウィンドウの「新しい仮想スイッチマネージャー」をクリックし、作成する仮想スイッチの種類を「外部」にします．
    
    - 「仮想スイッチの作成」をクリックしてください。
        <br><img width=50% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2FMu4Xg3R.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=327070e4e39cceeefa20284c22d7f54b" />
    
    - 表示される「仮想スイッチプロパティ」の名前を **`vm_ethernet`** にしてください．
        <br><img width=50% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2F4sJJqG9.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=01a727e5b0453caea58433a53dd88288" />
    
    - 接続の種類の **外部ネットワーク** を選択してください．
    - このとき図のように使用するネットワークインターフェースを選択する項目があるので，この項目で以下の条件のどれかに一致するインターフェース名を選択してください．
        - **Ethernet** という名前がついている
        - **Wi-FI**、**Wireless** などの名前がついていない
    
    <img width=50% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2FZOhr4nL.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=3de9fcb9e12f9b693c8e521b17f64287" />

    - 設定が完了したら、ウィンドウ右下の「適応」をクリックしてください。すると右の図のような警告が表示されますが、「はい」をクリックして設定を完了してください。「変更を適応しています」というポップアップが消えたら「OK」をクリックして仮想スイッチマネージャーを閉じます。
    <br><img width=45% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2F6KxAo9G.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=ba7bbce369ec0421c262371eefcd1c35" /><img width=10% /><img width=45% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2FQ2kAlop.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=0f59a104083944d2473176e128cf7b85" />