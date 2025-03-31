# VirtualBoxのインストール
まず，以下のサイトからVirtualBoxのインストーラをダウンロードします．Windowsを利用している方は， `Windows hosts` を選択してください．

[Download VirtualBox](https://www.virtualbox.org/wiki/Downloads)

<img src="/imgs/vbox_download.png" width=60%>

ダウンロードしたらインストーラをダブルクリックして起動します．

> [!NOTE]
> もし以下のようなエラーメッセージとともにインストーラの起動に失敗した場合は `C++2019再配布パッケージ`のインストールが必要です．
>
> <img src="/imgs/vb_install0.png" width=50%>
>
> インストールする場合は，以下のリンクからインストーラをダウンロードします．
>
> [サポートされている最新の Visual C++ 再頒布可能パッケージのダウンロード | Microsoft Learn](https://learn.microsoft.com/ja-jp/cpp/windows/latest-supported-vc-redist?view=msvc-170)
>
> このページの `最新の Microsoft Visual C++ 再頒布可能パッケージ バージョン`にある， `X64` をクリックしてインストーラをダウンロードします．
>
> <img src="/imgs/Cpp2019dist.png" width=80%>
> 
> ダウンロードしたインストーラをダブルクリックして起動し，インストールを行ってください．

インストーラが起動したら`Next`で次に進みます．

<img src="/imgs/vb_install1.png" width=50% />

使用許諾の同意画面になるので， `I accept the terms in the License Agreement`にチェックを入れて `Next`を押す．

<img src="/imgs/vb_install2.png" width=50%>

デフォルト設定のままNextを押します．

<img src="/imgs/vb_install3.png" width=50%>

インストール中にネットワーク接続が切れることがある旨の確認です．バックグラウンドで何かダウンロードなどを行っている場合は切れてしまうので注意してください．問題がなければ `Yes`を押します．

<img src="/imgs/vb_install4.png" width=50%>

PythonによるVM管理機能のインストール確認です．これを用いない場合はYesを押します．今回は `Yes`を押して進みます．

<img src="/imgs/vb_install5.png" width=50%>

ショートカット関係のチェックはお好みで設定してください．ただし，最後の `Register file associations`はチェックしておいてください．

<img src="/imgs/vb_install6.png" width=50%>

これでインストール完了です．スタートメニューからVirtualBoxを起動してみましょう．

---
- [Virtual Boxで仮想マシンをセットアップするに戻る](/windows/vbox.md)