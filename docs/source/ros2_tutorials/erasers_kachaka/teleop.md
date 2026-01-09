# Kachaka を手動操作する

{doc}`./bringup` を参考に起動させると，画面上に `RViz` が表示されます．このウィンドウにある `JoyyStickPanel` を使い，Kachaka を手動操作することができます．

```{image} https://i.imgur.com/rVkfEyI.png
:width: 50%
:align: center
```

このパネルは以下のように構成されています．

- 青色のプログレスバー：Kachaka の現在のバッテリー残量
- 中央のジョイスティック：Kachaka 操作用スティック
- 下部の入力欄：Kachaka に発話させたいメッセージを入力

ジョイスティックを動かすか，ジョイスティックの領域でキーボード「WASD」で，Kachaka の手動操作が可能です．また，入力欄に発話させたいメッセージを書いて，エンターキーを押すと Kachaka が発話します．
なお，この入力欄は **日本語未対応です．** 日本語を発話させたい場合はローマ字入力で入力してください

Kachaka を手動操作したい場合は，RViz2 の右下にある JoyStick Panel の「Enable Drive」をクリックします．すると Kachaka の手動操作が有効になります．

| 状態 | スクリーンショット |
| :--- | :--- |
| 手動操作が無効な場合 | <img src="https://i.imgur.com/rVkfEyI.png"/> |
| 手動操作が有効な場合（WASD キー制御有効） | <img src="https://i.imgur.com/Mf3qmVH.png"/> |
| 手動操作が有効な場合（WASD キー制御無効） | <img src="https://i.imgur.com/DV8sKvA.png"/> |

- WASD キー制御を有効にしたい場合は「Enable Drive」を２回クリックすると再度有効になります．
- WASD キー制御は RViz の他のパネルを操作すると無効になります．