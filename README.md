# TMC-CORE1-ROS2

[CoRE](https://core.scramble-robot.org/)の1部リーグ出場を目的としたレポジトリ．以下の機能を提供する．

- ROS2-HumbleのDocker開発環境
- micro-ROSを通じたM5Stackへのコントローラデータ送信
- USBカメラの映像の表示

## 環境構築
> 事前にDocker

レポジトリのclone
```bash
> git clone --recursive https://github.com/Mittag-tech/tmc-core1-ros2.git
```

Dockerのbuild
```bash
> cd tmc-core1-ros2
> make docker-build
```

## 開発環境

```bash
> make docker-interactive
```

> ```main_ws```をマウントしているため，以下のファイルの変更はcontainer内と同期される

## 機能の実行
> M5StackとUSBカメラとシリアル通信をしているのが前提

1. M5stackをreset
1. USBカメラの接続を確認
1. launchファイルを実行
    ```bash
    > ros2 launch send_order all_system.launch.py
    ```

## 設定ファイル
M5Stackに送るための諸情報は```config.yaml```に記載

## Maintainer
> Mittag-tech