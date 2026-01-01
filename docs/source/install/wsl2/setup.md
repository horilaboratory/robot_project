# WSL2 セットアップ
Windows ではデフォルトで WSL2 が利用できないように設定されています．以下の手順に沿って WSl2 を有効にします．

## CPU 仮想化が有効化されているかの確認
1. スタートメニューで **「タスクマネージャー」** を検索してタスクマネージャーを開くか，キーボードショートカットキー「Ctrl + Shift + Esc」を押してタスクマネージャーを開きます．

1. タスクマネージャーの「パフォーマンス」タブをクリックして，「CPU」のグラフをクリックします．

1. 「仮想化」というリストに「有効」と表示されているか確認してください．

    ```{note}
    仮想化が有効になっていない場合は，BIOS設定で CPU の仮想化を有効にしてください．お使いの PC によって BIOS の設定方法は異なりますので，Web ブラウザで以下のキーワードを参考に調べてください．
    ```text
    あなたが使用しているPCの機種名 BIOS CPU Virtualization
    ```

## Linux サブシステムを有効にする
1. Windows の設定アプリを開きます．
1. 「システム」を開きます．
1. 「システム」の「Windows の機能」を開きます．
1. 「Linux 用 Windows サブシステム」をチェックして有効化します．
    <br><img src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fqiita-image-store.s3.ap-northeast-1.amazonaws.com%2F0%2F3307766%2F5c77485b-3e37-f594-be91-97d09d308d2e.png?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=3579960dfe440360f66d454038ae3ba9" />
1. 「Hyper-V」という項目にチェックがついていない場合はこちらのチェックも入れてください．
    <br><img src="https://qiita-user-contents.imgix.net/https%3A%2F%2Fqiita-image-store.s3.ap-northeast-1.amazonaws.com%2F0%2F3307766%2F2df0cc1e-17eb-9e21-3371-5bf179d2733d.png?ixlib=rb-4.0.0&auto=format&gif-q=60&q=75&w=1400&fit=max&s=bd4417998462147fca528297ffff3f3f"/>
1. 「OK」を押して変更を保存します．<br>
    ```{warning}
    「OK」ボタンをクリックする前に誤って他のチェックを外したり新たにチェックを入れていないか確認してください．既存の機能が無効化される可能性があります．
    ```
1. 一度コンピューターを再起動してください．