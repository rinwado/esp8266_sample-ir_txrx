## 概要
- **時間管理**: Tickerによる定周期割り込みと、TaskSchedulerによる非ブロッキングなイベント処理。
- **タスク管理**: TaskSchedulerや割り込み内でのタスクトリガー（restart）によるタスク実行を行う。
- **赤外線受送信**: IRremoteESP8266ライブラリーを利用した、赤外線受信・送信を行うことができます。
- **ファイルシステム**: LittleFSライブラリーを利用し、赤外線データの保存・読込、赤外線出力データはJSONで管理。また、コマンドによるファイル操作ができる。
- **コマンド**: コマンド処理には、microrlライブラリーを利用し各種コマンドの処理を実装しています。

## 動作環境
- **ハードウェア**: 
  - IOT Integrated Controller V1　RRH-G101A REV-A (ESP8266搭載)
  - 一般的な ESP-WROOM-02D 4M (ESP8266搭載)
- **開発ツール**: PlatformIO (VSCode)
- **使用ライブラリ**: 
  - [TaskScheduler](https://github.com/arkhipenko/TaskScheduler)
  - Ticker (ESP8266 Core内蔵)
  - IRremoteESP8266
  - ArduinoJson
  - [microrl](https://github.com/dimmykar/microrl-remaster.git)

## 接続設定
- **LEDピン**: GPIO16 (ボード上のJP1をLED側に設定)
- **IR-Sig.**: GPIO14 (ボード上のJP2をONに設定)
- **IO I/F**: J9端子台　１番(IR-LED＋) と ２番(IR-LEDー) に赤外線ＬＥＤを接続
- **Buzzer**: GPIO13（ボード上のJP6をOFFに設定）
- **シリアル通信**: 115200 bps

## セットアップと書き込み
1. VSCodeでこのプロジェクトフォルダを開きます。
2. PlatformIOの `Upload and Monitor` を実行します。
3. 起動するとシリアルポートに、コマンドプロンプト esp8266> が出力され、コマンド受付状態になります。

## コマンドについて
- **cmd-help**: 利用可能なコマンド表示
- **cmd-ledbs**: LED LD4のブリンクスピード設定。30～1000ms の値を設定
- **cmd-irrx**: 赤外線の受信制御。赤外線受信可能にするには irrx on 、赤外線受信不可能にするには irrx off
- **cmd-irsv**: 受信した赤外線データをファイルに保存。　irsv メーカー名　機種名　機能名　を指定して保存します
- **cmd-irld**: ファイルに保存された赤外線データの読込とデータ出力確認を行う。irld メーカー名　機種名　機能名　テスト を指定してファイルから読込ます
- **cmd-ls**: ディレクトリーやファイルの一覧を表示。ls は現在のディレクトリ、ls /xxx/yyy は絶対パスでの指定
- **cmd-cat**: ファイルの内容を表示します。cat ファイル名　オプション、現在のディレクトリのファイル、または絶対パス指定。-v:制御文字可視、-J:Json整形、-x:HEX表示、その他:そのまま表示
- **cmd-rm**: ファイルの削除。現在のディレクトリのファイル、または絶対パス指定。
- **cmd-rmdir**: ディレクトリの削除。現在のディレクトリ、または絶対パス指定。
- **cmd-mkdir**: ディレクトリの作成。現在のディレクトリに、または絶対パス指定。
- **cmd-touch**: 空のファイルをz作成。現在のディレクトリに、または絶対パス指定。
- **cmd-rn**: ファイルまたはディレクトリの名前変更。現在のディレクトリの、または絶対パス指定。
- **cmd-df**: ディスクの容量を表示します。
- **cmd-pwd**: 現在のディレクトリパスを表示します。
- **cmd-cd**: カレントディレクトリーを変更します。現在のディレクトリから、または絶対パス指定。

## ライセンス
[MIT License](https://opensource.org/license/mit)