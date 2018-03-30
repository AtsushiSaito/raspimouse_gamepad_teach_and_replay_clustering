# raspimouse_gamepad_teach_and_replay_clustering
PFoEに記憶のクラスタリング機能を実装

## はじめに
PFoEでTeach-and-Replayを行うことができるROSパッケージ(ryuichiueda/raspimouse_gamepad_teach_and_replay)にクラスタリング機能を実装した。

## 動作環境
* RaspberryPi Mouse V2
* RasoberryPi3
* Ubuntu Server 16.04.3 LTS
  * https://b.ueda.tech/?post=20171224_raspi_ubuntu_image_ros

## 依存関係
クラスタリング機能を実装する際、機械学習のライブラリである`Scikit-learn`を用いている。
そのため、使用するRaspberryPiに`Scikit-learn`をインストールする必要がある。

最初に、`Scikit-learn`のインストールに必要となる`scipy`をインストールする。
```
sudo apt-get install python-scipy
```

次に`pip`経由で`Scikit-learn`をインストールします。
```
sudo pip install scikit-learn
```
## 使い方
以下のコマンドを実行します。
```
git clone git@github.com:AtsushiSaito/raspimouse_gamepad_teach_and_replay_clustering.git
```

## 解説
(準備中)
