# ROS2コマンド早見表
この授業で使用する主要なROS2コマンドの一覧です．

## トピック一覧を示す
```bash
ros2 topic list
```

```bash
ros2 topic list -t
```

## トピックの情報を表示する
```bash
ros2 topic info <トピック名>
```

## トピックをパブリッシュする
```bash
ros2 topic pub <トピック名> <メッセージの型> "<メッセージの値>"
```

## メッセージの構造を確認する
```bash
ros2 interface show <メッセージの型>
```