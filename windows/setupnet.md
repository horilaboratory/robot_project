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
　仮想スイッチマネージャーウィンドウの「新しい仮想スイッチマネージャー」をクリックし、作成する仮想スイッチの種類を「外部」にします。そして「仮想スイッチの作成」をクリックしてください。

<img width=50% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2FMu4Xg3R.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=327070e4e39cceeefa20284c22d7f54b" />

表示される「仮想スイッチプロパティ」の名前を
**`vm_ethernet`**
にしてください。

<img width=50% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2F4sJJqG9.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=01a727e5b0453caea58433a53dd88288" />

つぎに接続の種類の
**外部ネットワーク**
を選択してください。このとき図のように使用するネットワークインターフェースを選択する項目があるので、この項目で以下の条件のどれかに一致するインターフェース名を選択してください。

- **Ethernet** という名前がついている
- **Wi-FI**、**Wireless** などの名前がついていない

<img width=50% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2FZOhr4nL.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=3de9fcb9e12f9b693c8e521b17f64287" />

設定が完了したら、ウィンドウ右下の「適応」をクリックしてください。すると右の図のような警告が表示されますが、「はい」をクリックして設定を完了してください。「変更を適応しています」というポップアップが消えたら「OK」をクリックして仮想スイッチマネージャーを閉じます。

<img width=45% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2F6KxAo9G.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=ba7bbce369ec0421c262371eefcd1c35" /><img width=10% /><img width=45% src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fi.imgur.com%2FQ2kAlop.jpeg?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=0f59a104083944d2473176e128cf7b85" />

これで仮想インターフェースの設定は完了です。

---

- [Windows でロボット開発をはじめるのセクション **Hyper-V でネットワーク設定を行う** に戻る](/windows/hostsetup.md/#setupnet)
