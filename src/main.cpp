/**
 * タスクスケジューラー、ティッカー、赤外線、Jsonやmicrorl ライブラリーを
 * 使った赤外線送受信や保存・読込テストのサンプルプログラム
 * for PlatformIO
 * 
 * 対応ボード：IOT Integrated Controller V1
 *            RRH-G101A REV-B
 * 
 * 2026-05-10
 * Copyright (c) 2026 rinwado
 * Licensed under the MIT License.
 * See LICENSE file in the project root for full license text.
 */

#include <Arduino.h>
#include <LittleFS.h>
#include <Ticker.h>
#include <TaskScheduler.h>
#include <sys/time.h>
#include <time.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>
#include <IRtext.h>
#include <IRrecv.h>
#include <IRac.h>
#include <IRsend.h>
#include <microrl.h>
#include <ArduinoJson.h>
#include "net_env.h"
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>



/// 定義
#define TICK_PP_MS                (3.333F)                //ティッカー周期　ｍS
#define TICK_PP_SEC               (TICK_PP_MS/1000.0F)    //ティッカー周期　ｍS
#define LED_PIN                   (16)                    //ＬＥＤピン（ボードのＪＰ１は「ＬＥＤ」側にする）
#define IR_RECV_PIN               (14)                    //赤外線信号入力ピン
#define IR_SEND_PIN               (15)                    //赤外線出力信号ピン
#define IR_CAPTURE_BUF_SIZE       (1024)                  //保存バッファ機能を有効にすると、より完全なキャプチャ範囲を確保できます。
#define IR_RECV_TIMEOUT           (50)                    //50ms, 一部のエアコンでは、プロトコルの間隔が約40ミリ秒開いている場合がある
#define IR_RECV_MIN_UNKNOWN_SIZE  (12)                    //値を非常に高く設定すると、UNKNOWNの検出を事実上無効
#define IR_TOLERANCE              (25)                    //許容誤差は通常25%
#define IR_RAW_DATA_BUFFER        (1024)                  //RAWデータの保存用バッファー数
#define WIFI_FIXED_STR_LEN_MAX    (48)                    //NULLターミネータ含めた文字数
#define JST_OFFSET                (9 * 3600)              //9時間 × 3600秒
#define NTP_SERVER                "ntp.nict.jp"
#define NOF_TICK_CNT(ms)          (ms / 3.333)            //指定したｍｓ時間が Ticker 割込みでのカウントがいくつに相当するか
#define TICK_COUNTER_PAR_30MIN    ((30*60*1000) / 3.333)  //時刻更新間隔
#define ADC_VCC_MODE              (1)                     //1:ADCの入力がチップ内部の電源ラインを読む、 0:ADCの入力が外部のA0ピンの値を読む
#define ADC_A0_1LSB_MV            (0.976562F)             //1000mV ÷ 1024
#define IO0_PIN                   (0)                     //プログラムボタンと兼用[SW2/IN1]しているので、起動時の検出はできない

/// マクロ
#define TICK_CNT_FROM_MS(t)       (t/TICK_PP_MS)          //ミリ秒からカウント数を得る


/// ESP8266 ADC 設定
#if(ADC_VCC_MODE)
ADC_MODE(ADC_VCC);                                        //ADCは内部のVCC電圧を測定するモードにする（コンパイル時の設定可能）
#define ESP8266_ADC_READ  ESP.getVcc()                    //内部のVCC電圧を読み出す場合
#else
#define ESP8266_ADC_READ  analogRead(A0)                  //通常のＡ０端子のアナログ入力を読み出す
#endif


/// 構造体
struct IRDataRecord
{ //IR保存用構造体
  decode_type_t protocol;                               //4byte
  uint16_t bits;                                        //2byte
  union {
    uint64_t value;                                     //8byte (NEC, SONYなど用)
    uint8_t state[kStateSizeMax];                       //kStateSizeMax byte (エアコンなど長い信号用)
  };
  uint16_t rawlen;                                      //2byte
  uint16_t rawData[IR_RAW_DATA_BUFFER];                 //1024*2byte
  bool isState;                                         //1byte (valueかstateかの判定フラグ)
  bool repeat;                                          //1byte
};

typedef struct
{ //コマンド名と関数を紐付ける
  const char* name;                                     //コマンド名
  void (*func)(int argc, const char* const* argv);      //実行する関数
  const char* help;                                     //ヘルプ文
} command_t;

typedef struct wifi_setting_val
{
    bool f_dhcp;
    //bool f_secure;
    char ConnectSSID[WIFI_FIXED_STR_LEN_MAX];
    char ConnectSSID_Pass[WIFI_FIXED_STR_LEN_MAX];
    IPAddress wfixIP;
    IPAddress wfixGW;
    IPAddress wfixSNM;
    IPAddress wfixDNS;
    char WiFi_mac[8];
    char WiFi_mac_str[32];
} wifi_setting_val_t;

/// 列挙型
enum wState
{	//Wi-Fi ステート
	WIFI_CSTART = 0,
	WIFI_CLIENT_PROC_ENT,
	WIFI_CONNECT_CHK,
	WIFI_TRY_CONNECT_TO_AP,
	WIFI_CONNECT_OKNG,

  WIFI_WAIT_PROCESS,
	WIFI_OHTER_PROCESS,
};

/// コマンドの処理関数 プロトタイプ宣言
void cmd_help(int argc, const char* const* argv);
void cmd_led_spd(int argc, const char* const* argv);
void cmd_ir_rcv_cntl(int argc, const char* const* argv);
void cmd_fs_dir(int argc, const char* const* argv);
void cmd_send2blynk(int argc, const char* const* argv);
void cmd_get_now_time(int argc, const char* const* argv);
void cmd_fs_remove(int argc, const char* const* argv);
void cmd_fs_remove_dir(int argc, const char* const* argv);
void cmd_fs_create_dir(int argc, const char* const* argv);
void cmd_fs_create_efile(int argc, const char* const* argv);
void cmd_fs_rename_file(int argc, const char* const* argv);
void cmd_fs_disk_info(int argc, const char* const* argv);
void cmd_fs_save_ir(int argc, const char* const* argv);
void cmd_fs_load_ir(int argc, const char* const* argv);
void cmd_fs_remove_ir(int argc, const char* const* argv);
void cmd_fs_view(int argc, const char* const* argv);
void cmd_fs_change_dir(int argc, const char* const* argv);
void cmd_fs_crr_dir_info(int argc, const char* const* argv);

/// プロトタイプ宣言
void IRAM_ATTR onTickTimerISR(void);
void MainWork_Callback(void);
void TskOneShot_LEDproc_Callback(void);
int microRL_printOut(microrl_t* mrl, const char* str);
int microRL_executeCallback(microrl_t* mrl, int argc, const char* const* argv);
void listDir(const char *dirname);
void getFSInfo(void);
void init_MFRlist_File(void);
String formatNumber(uint32_t n);

/// オブジェクト生成
Scheduler TskRunner;                                                                      //スケジューラ
Ticker tick_timer1;                                                                       //ティッカー割り込み
IRrecv irrecv(IR_RECV_PIN, IR_CAPTURE_BUF_SIZE, IR_RECV_TIMEOUT, true);                   //赤外線受信するピンやパラメータをセット
IRsend irsend(IR_SEND_PIN);                                                               //赤外線送信するピンを割り当てる
decode_results results;                                                                   //解析結果が保存される
microrl_t mrl;                                                                            //コマンドラインインターフェイス

//Task(周期ms, 実行回数[-1は無限], 実行する関数, スケジューラへのポインタ)
//Taskの宣言した順にタスク管理リストに登録され、管理リストの順に実行タイミングがチェックされる。
Task tsk_main_work(1, TASK_FOREVER, &MainWork_Callback, &TskRunner);                      //[main_work]タスクの作成
Task tsk_OneShot_01(TASK_IMMEDIATE, TASK_ONCE, &TskOneShot_LEDproc_Callback, &TskRunner); //[LEDproc]OneShotタスクの作成


/// コマンドテーブル (ここにコマンドを登録)
static const command_t commands[] = {
  {"help",    cmd_help,           "Show this help"},
  {"ledbs",   cmd_led_spd,        "LED blink speed (30...1000)"},
  {"irrx",    cmd_ir_rcv_cntl,    "IR recv control"},
  {"irsv",    cmd_fs_save_ir,     "Save IR data to FS"},
  {"irld",    cmd_fs_load_ir,     "Load IR data from FS"},
  {"irrm",    cmd_fs_remove_ir,   "Remove IR data from FS"},
  {"snd2bc",  cmd_send2blynk,     "Data send to Blynk ON/OFF"},
  {"ntim",    cmd_get_now_time,   "Show now time"},

  {"ls",      cmd_fs_dir,         "List of files"},
  {"cat",     cmd_fs_view,        "View file Contents"},
  {"rm",      cmd_fs_remove,      "Remove file"},
  {"rmdir",   cmd_fs_remove_dir,  "Remove directory"},
  {"mkdir",   cmd_fs_create_dir,  "Create directory"},
  {"touch",   cmd_fs_create_efile,"Create empty file"},
  {"rn",      cmd_fs_rename_file, "Rename file/directory"},
  {"df",      cmd_fs_disk_info,   "DISK FS Information"},
  {"pwd",     cmd_fs_crr_dir_info,"Show current directory"},
  {"cd",      cmd_fs_change_dir,  "Changing the current directory"},
  
};



/// 変数
volatile uint32_t gn_cnt1 = 0;            //汎用カウンタ１
volatile uint16_t gn_cnt2 = 0;            //汎用カウンタ２
volatile uint16_t gn_cnt3 = 0;            //汎用カウンタ３
volatile uint16_t led_speed = 0;          //LED LD4 の点滅スピード設定
volatile bool f_counter_trigger = false;  //割込みカウンタによるトリガフラグ

static IRDataRecord record;               //赤外線 受信・送信時に利用

char init_key = '\n';                     //microrl のための改行コード
bool f_littlefs_available = false;        //ファイルシステム利用可能フラグ
bool f_ir_available = false;              //赤外線利用可能フラグ
bool f_ir_rcv_enable = false;             //受信コントロールフラグ
String current_directory;                 //カレントディレクトリー保存用

bool f_cat_send_bin = false;              //バイナリーデータの表示用フラグ
bool f_send_bin_start = false;            //バイナリーデータ送出開始フラグ
unsigned long enter_time2, wait_time2;    //タイムアウトチェック用
String gbl_path;                          //ファイルパス名

wifi_setting_val_t wifi_setting;
bool f_wifi_connected = false;
enum wState WiFi_state;
enum wState WiFi_next_state;
WiFiClient* WIFI_Client;
WiFiUDP* ntpUDP_Client;
NTPClient* NTP_client;

volatile int16_t WiFi_wait_counter;
volatile int16_t WiFi_ConnectTimeOut_Counter;
int16_t wait_time;
char wifi_macAddress[16];

bool f_blynk_connected = false;
bool f_blynk_send_data = false;
int16_t blynk_reconnect_wait_time;
char blynk_auth[] = BLYNK_AUTH_TOKEN;

bool time_sync_start = false;
const char* weekd[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sta"};  //曜日の定義



/**
 * @brief Arduino setup
 * 
 */
void setup()
{
  //--- IOピン設定
  pinMode(LED_PIN, OUTPUT);
  pinMode(IO0_PIN, INPUT);

  //--- 変数初期化
  led_speed = TICK_CNT_FROM_MS(500);
  f_blynk_connected = false;
  f_blynk_send_data = false;
  blynk_reconnect_wait_time = 0;
  wait_time = 0;

  //--- シリアル
  Serial.begin(115200);
  Serial.printf_P(PSTR("\r\n[i] Flash Real Size: %u bytes\r\n"), ESP.getFlashChipRealSize()); //ESP8266に搭載されているフラッシュメモリ容量

  /*
    LittleFSには「カレントディレクトリ」という概念がない。
    Linuxのように cd で移動して「今はこのフォルダの中にいる」という状態を保持できないため、
　　ファイルを開くときは常に /（ルート）から始まるフルパス で指定する必要がある。
    本サンプルは、カレントディレクトリーを保存できるようにしている。
  */
  //--- LittleFS
  current_directory.clear();
  current_directory = "/";
  f_littlefs_available = true;
  if(!LittleFS.begin())
  { Serial.printf_P(PSTR("[e] Error LittleFS!!\r\n"));
    f_littlefs_available = false;
  }
  else
  { init_MFRlist_File();   //赤外線リモコンメーカリストファイル確認
  }

  //--- microrl (コマンドライン)
  microrl_init(&mrl, microRL_printOut, microRL_executeCallback);  //シリアル出力とコマンド実行の登録
  microrl_set_prompt(&mrl, (char*)"esp8266> ");                   //プロンプト設定
  microrl_processing_input(&mrl, &init_key, 1);                   //プロンプト設定では「>」のみ表示なので、ＬＦを送って「esp8266>」を出力させる

  //--- 赤外線
  if(0 == irutils::lowLevelSanityCheck()) //コンパイラが期待どおりにビットフィールドのパッキングを行い、エンディアンも期待どおりであることを確認
  { //OK
    Serial.printf_P(PSTR("\r\n[i] Waiting for IR message on Pin %d\r\n"), IR_RECV_PIN);
    microrl_processing_input(&mrl, &init_key, 1);                 //ＬＦを送ってプロンプト「esp8266>」を出力させる
    irrecv.setUnknownThreshold(IR_RECV_MIN_UNKNOWN_SIZE);         //信号波形のパルス数が設定値未満のメッセージは無視する。
    irrecv.setTolerance(IR_TOLERANCE);                            //許容誤差の設定(再度)
    //irrecv.enableIRIn();                                          //赤外線受信準備
    irsend.begin();                                               //赤外線送信準備
    f_ir_available = true;
  }
  else
  { //NG
    Serial.printf_P(PSTR("\r\n[e] ERR: Low level sanity check!\r\n"), IR_RECV_PIN);
    f_ir_available = false;
  }

  //Wi-Fi パラメータ初期化
  f_wifi_connected = false;
  wifi_setting.f_dhcp = false;
  memset(wifi_setting.ConnectSSID, 0, sizeof(wifi_setting.ConnectSSID));
  strcpy(wifi_setting.ConnectSSID, WIFI_SSID);
  memset(wifi_setting.ConnectSSID_Pass, 0, sizeof(wifi_setting.ConnectSSID_Pass));
  strcpy(wifi_setting.ConnectSSID_Pass, WIFI_VERI_WORD);
  wifi_setting.wfixIP.fromString(WIFI_FIX_IP);    //IPAddress のメンバ関数[fromString]でストリングからIPアドレスに変換
  wifi_setting.wfixGW.fromString(WIFI_FIX_GW);
  wifi_setting.wfixSNM.fromString(WIFI_FIX_SNM);
  wifi_setting.wfixDNS.fromString(WIFI_FIX_DNS);

  WiFi_state = WIFI_CSTART;
  WiFi_next_state = WIFI_CSTART;
  WIFI_Client = NULL;
  ntpUDP_Client = NULL;
  NTP_client = NULL;

  //Blynk パラメータ初期化
  f_blynk_connected = false;
  Blynk.config(blynk_auth);

  //--- ティッカーの開始 秒数で指定(0.003333 = 3.333ms, onTimerISR関数をセット)
  tick_timer1.attach(TICK_PP_SEC, onTickTimerISR);
  //--- タスクを有効化
  tsk_main_work.enable();
  //tsk_OneShot_01.enable();  //TASK_ONCE　なので、ここではenableにしない
}



/**
 * @brief Arduino loop
 * 
 */
void loop()
{
  //--- スケジューラを回す
  TskRunner.execute();

  //--- Wi-Fi/NTP 処理
  static uint32_t now_free_heap_size = 0;
  switch(WiFi_state)
  { //Wi-Fi 接続・切断・再接続の処理ステート
    case WIFI_CSTART:
      f_wifi_connected = false;
      now_free_heap_size = ESP.getFreeHeap();
      Serial.printf_P(PSTR("[i] Free Heap  size: %d\r\n"), (int)now_free_heap_size);
      Serial.printf_P(PSTR("[i] WiFiClient size: %d\r\n"), (int)sizeof(WiFiClient));
      Serial.printf_P(PSTR("[i] WiFiUDP    size: %d\r\n"), (int)sizeof(WiFiUDP));
      Serial.printf_P(PSTR("[i] NTPClient  size: %d\r\n"), (int)sizeof(NTPClient));

      if(8192 < now_free_heap_size)
      { //Wi-Fi 関係で 8192byte は、通信バッハーなどでこの先利用するだろうと予測して数値を設定している
        if(NULL == WIFI_Client)
        { WIFI_Client = new WiFiClient();
          if(NULL != WIFI_Client)
          { //OK
              WiFi.mode(WIFI_STA);
              Serial.printf_P(PSTR("[i] WiFi-Mode:STA\r\n"));
          }
        }

        if(NULL == ntpUDP_Client)
        { ntpUDP_Client = new WiFiUDP();
        }

        if((NULL == NTP_client) && (NULL != ntpUDP_Client))
        { NTP_client = new NTPClient(*ntpUDP_Client, NTP_SERVER, JST_OFFSET);
        }

        if((NULL == WIFI_Client) || (NULL == ntpUDP_Client) || (NULL == NTP_client))
        { //NG
          Serial.printf_P(PSTR("[e] Failed to create the instance. Will retry in 1 second.\r\n"));
          wait_time = NOF_TICK_CNT(1000); //1000ms
          WiFi_wait_counter = 0;
          WiFi_state = WIFI_WAIT_PROCESS;
          WiFi_next_state = WIFI_CSTART;
        }
        else
        { //OK
          NTP_client->setTimeOffset(32400);       //日本の＋９時間分の秒数
          now_free_heap_size = ESP.getFreeHeap();
          Serial.printf_P(PSTR("[i] Free Heap  size: %d\r\n"), (int)now_free_heap_size);            
          WiFi_state = WIFI_CLIENT_PROC_ENT;
          WiFi_next_state = WIFI_CLIENT_PROC_ENT;
        }
      }
      else
      { Serial.printf_P(PSTR("[?] There may not be enough heap memory. Will retry in 1 second.\r\n"));
        wait_time = NOF_TICK_CNT(1000); //1000ms
        WiFi_wait_counter = 0;
        WiFi_state = WIFI_WAIT_PROCESS;
        WiFi_next_state = WIFI_CSTART;
      }
    break;

    case WIFI_CLIENT_PROC_ENT:
      WiFi.disconnect(true);        //WiFi.disconnect(true, true); wifioff	trueを指定すると、ステーションモードを終了する。省略時はfalse。 eraseap	trueを指定すると、WiFiの設定情報を削除する。省略時はfalse。
      WiFi.setAutoConnect(false);		//電源再投入時に最後に接続されたAPに自動的に接続するか否か：自動接続しない
      WiFi.setAutoReconnect(false); //APの接続が切れた場合、自動的に再接続するか否か：自動接続しない（既に接続が切れている時に実行してもAPへの再接続はされない）

      if(WiFi.status() != WL_CONNECTED)
      {	//接続が切れているのを確認
        f_wifi_connected = false;
        WiFi_state = WIFI_TRY_CONNECT_TO_AP;
        WiFi_next_state = WIFI_TRY_CONNECT_TO_AP;
      }
      else
      { wait_time = NOF_TICK_CNT(50); //50ms
        WiFi_wait_counter = 0;
        WiFi_state = WIFI_WAIT_PROCESS;
        WiFi_next_state = WIFI_CLIENT_PROC_ENT;
      }
    break;     

    case WIFI_TRY_CONNECT_TO_AP:
        //ルーター（ＡＰ）への接続（WiFi.begin：デフォルトはDHCP）
        Serial.printf_P(PSTR("[i] WiFi Try connect.\r\n"));
        if(!wifi_setting.f_dhcp)
        { //DHCPでない場合
            WiFi.config(wifi_setting.wfixIP, wifi_setting.wfixGW, wifi_setting.wfixSNM, wifi_setting.wfixDNS);
        }
        WiFi.begin(wifi_setting.ConnectSSID, wifi_setting.ConnectSSID_Pass);

        WiFi_ConnectTimeOut_Counter = NOF_TICK_CNT(30000); //接続タイムアウト時間 30s セット
        WiFi_state = WIFI_CONNECT_OKNG;
        WiFi_next_state = WIFI_CONNECT_OKNG;

        now_free_heap_size = ESP.getFreeHeap();
        Serial.printf_P(PSTR("[i] Free Heap  size: %d\r\n"), (int)now_free_heap_size);   
    break;

    case WIFI_CONNECT_OKNG:
      if(WiFi.status() == WL_CONNECTED)
      {	//接続された
          WiFi.macAddress((uint8_t*)wifi_setting.WiFi_mac);
          memset(wifi_macAddress, 0, sizeof(wifi_macAddress));
          sprintf(wifi_macAddress, "%02X%02X%02X%02X%02X%02X",
                                              wifi_setting.WiFi_mac[5], wifi_setting.WiFi_mac[4], wifi_setting.WiFi_mac[3],
                                              wifi_setting.WiFi_mac[2], wifi_setting.WiFi_mac[1], wifi_setting.WiFi_mac[0]);
          sprintf(wifi_setting.WiFi_mac_str, "%02X:%02X:%02X:%02X:%02X:%02X",
                                              wifi_setting.WiFi_mac[5], wifi_setting.WiFi_mac[4], wifi_setting.WiFi_mac[3],
                                              wifi_setting.WiFi_mac[2], wifi_setting.WiFi_mac[1], wifi_setting.WiFi_mac[0]);
          Serial.printf_P(PSTR("\r\n[i] WiFi connected.\r\n"));
          Serial.printf_P(PSTR("[i] MAC address    : %s\r\n"), wifi_setting.WiFi_mac_str);
          Serial.printf_P(PSTR("[i] IP address     : %s\r\n"), WiFi.localIP().toString().c_str());
          Serial.printf_P(PSTR("[i] Default Gateway: %s\r\n"), WiFi.gatewayIP().toString().c_str());
          Serial.printf_P(PSTR("[i] Subnetmask     : %s\r\n"), WiFi.subnetMask().toString().c_str());
          Serial.printf_P(PSTR("[i] DNS Server1    : %s\r\n"), WiFi.dnsIP(0).toString().c_str());
          Serial.printf_P(PSTR("[i] DNS Server2    : %s\r\n"), WiFi.dnsIP(1).toString().c_str());

          WiFi.setAutoConnect(false);		//電源再投入時に最後に接続されたAPに自動的に接続するか否か：自動接続しない
          WiFi.setAutoReconnect(false); //APの接続が切れた場合、自動的に再接続するか否か：自動接続しない（既に接続が切れている時に実行してもAPへの再接続はされない）

          NTP_client->begin();  //NTP Client 開始

          WiFi_state = WIFI_CONNECT_CHK;
          WiFi_next_state = WIFI_CONNECT_CHK;
          f_wifi_connected = true;
          time_sync_start = true;

          f_blynk_connected = false;                                  //Wi-Fi 接続開始後に接続されているので、blynk は接続されていないはず。
          gn_cnt3 = blynk_reconnect_wait_time = NOF_TICK_CNT(10000);  //10000ms (１０秒) 初回すぐに接続開始させるため
          microrl_processing_input(&mrl, &init_key, 1);               //プロンプト設定では「>」のみ表示なので、ＬＦを送って「esp8266>」を出力させる
      }
      else
      {	//接続タイムアウトチェック
        if(0 >= WiFi_ConnectTimeOut_Counter)
        {   //タイムアウト
          Serial.printf_P(PSTR("[i] WiFi connect timeout(30s)!\r\n"));
          wait_time = NOF_TICK_CNT(200); //200ms
          WiFi_wait_counter = 0;

          WiFi_state = WIFI_WAIT_PROCESS;
          WiFi_next_state = WIFI_CLIENT_PROC_ENT;
          f_wifi_connected = false;
          time_sync_start = false;
        }
      }        
    break;

    case WIFI_CONNECT_CHK:
      if(WiFi.status() != WL_CONNECTED)
      {	//接続が切れた
        Serial.printf_P(PSTR("[i] Now Wi-Fi Disconnected!\r\n"));
        NTP_client->end();              //NTP Client を終了
        WIFI_Client->stop();            //TCP通信停止

        wait_time = NOF_TICK_CNT(1000); //1000ms
        WiFi_wait_counter = 0;

        WiFi_state = WIFI_WAIT_PROCESS;
        WiFi_next_state = WIFI_CLIENT_PROC_ENT;
        f_wifi_connected = false;
        time_sync_start = false;

        now_free_heap_size = ESP.getFreeHeap();
        Serial.printf_P(PSTR("[i] Free Heap  size: %d\r\n"), (int)now_free_heap_size);   
      }
    break;

    case WIFI_WAIT_PROCESS:
      if(wait_time <= WiFi_wait_counter)
          WiFi_state = WiFi_next_state;
    break;     

    default:
    break;
  } //switch(WiFi_state)


  if(f_wifi_connected)
  {
    if(!f_blynk_connected)
    { //Blynk 未接続状態
      if(!Blynk.connected())
      { //Blynk cloud に未接続状態
        if(blynk_reconnect_wait_time <= gn_cnt3)
        { //接続トライ
          Serial.printf_P(PSTR("\r\n[i] Try connecting to Blynk Cloud.\r\n"));
          Serial.printf_P(PSTR("[!] Program will block here until a Blynk device is connected or a timeout occurs.\r\n"));
          if(Blynk.connect(8000UL))  //８秒のタイムアウトの設定で　Blynk　接続トライ（ブロッキングされます）
          { //接続された
            Serial.printf_P(PSTR("[i] Connected to the Blynk cloud...\r\n"));
            f_blynk_connected = true;
          }
          else
          { //接続失敗かタイムアウト発生 (１０秒後再接続)
            Serial.printf_P(PSTR("[e] Failed to connect to the Blynk cloud or a timeout occurred. We will retry in 10 seconds.\r\n"));
            gn_cnt3 = 0;
            f_blynk_connected = false;
          }

          microrl_processing_input(&mrl, &init_key, 1);               //プロンプト設定では「>」のみ表示なので、ＬＦを送って「esp8266>」を出力させる
        }
      }
      else
      { //既に Blynk cloud に接続状態
          f_blynk_connected = true;
      }
    }
    else
    { //Blynk 接続状態
      if(!Blynk.connected())
      { //Blynk cloud と切断された
        Serial.printf_P(PSTR("[!] Blynk Cloud connection has been lost. We will retry in 10 seconds.\r\n"));
        gn_cnt3 = 0;        
        f_blynk_connected = false;

        microrl_processing_input(&mrl, &init_key, 1);               //プロンプト設定では「>」のみ表示なので、ＬＦを送って「esp8266>」を出力させる
      }
      else
      { //Blynk cloud 接続維持中
        Blynk.run();
      }
    }    
  }
} //void loop()



/**
 * @brief ティッカーによる割込み処理
 *        3.333mS ごとに処理
 *        ここには長い処理やdelay()などは書かない、１ｍｓ以下で処理が完了するような内容が望ましい
 */
void IRAM_ATTR onTickTimerISR(void)
{
  gn_cnt1++;
  gn_cnt2++;
  gn_cnt3++;

  //ESP8266の時刻更新用
  if(TICK_COUNTER_PAR_30MIN <= gn_cnt1)
  { //main_work 用のトリガフラグ
    gn_cnt1 = 0;
    f_counter_trigger = true;
  }

  if(led_speed <= gn_cnt2)
  { //ＬＥＤ処理、ＡＤＣサンプルとデジタルデータリード　タスク
    gn_cnt2 = 0;
    tsk_OneShot_01.restart();
  }

  //---
  WiFi_wait_counter++;
  WiFi_ConnectTimeOut_Counter--;
}



/**
 * @brief TaskScheduler コールバック関数
 *        １ｍｓ 間隔で実行される
 */
void MainWork_Callback(void)
{
  if(!f_cat_send_bin)
  { //cat コマンドでバイナリー表示のフラグが立っていない場合
    // シリアルに受信データあるか確認
    int available = Serial.available();
    if(available > 0)
    { //受信文字あり
      char buffer[32];                                                          //入力バッファー、一度に読み込む最大サイズ 
      int to_read;                                                              //実際に読み込む文字数
      
      to_read = (available > (int)sizeof(buffer))? sizeof(buffer) : available;  //バッファサイズを超えないように、読み込む文字数を決定
      int n = Serial.readBytes(buffer, to_read);                                //文字の読込み
      microrl_processing_input(&mrl, buffer, n);                                //第３引数に「読み込んだ文字数 n」指定してmicrorlに渡す
    }
  }
  else
  { //cat コマンドでバイナリー表示の要求がある場合
    if(0 >= Serial.available())
    { //シリアルから受信文字なし
      if((millis() - enter_time2) >= wait_time2)
      { //送信タイムアウト      
        Serial.printf_P(PSTR("Timeout for binary output using the `cat` command.\r\n"));
        gbl_path.clear();
        f_send_bin_start = false;
        f_cat_send_bin = false;
        microrl_processing_input(&mrl, &init_key, 1); //ＬＦを送ってプロンプト「esp8266>」を出力させる
      }  
    }
    else
    { //シリアルから受信文字あり
      int c = Serial.read();
      if('\n' == c)
      { //LF コードが確認された
        while(0 < Serial.available())
          Serial.read();                              //受信データ空になるまで、読み捨てる
        f_send_bin_start = true;
      }
    }

    if(f_send_bin_start)
    { //バイナリーデータの送出
      File file;
      #if(0)
      Serial.printf_P(PSTR("Out put file = %s\r\n"), gbl_path.c_str());
      #endif

      //ファイルを開く
      file = LittleFS.open(gbl_path, "r");
      if(!file)
      { //オープン失敗
        Serial.printf_P(PSTR("[e] Failed to open file(%s)!\r\n"), gbl_path.c_str());
        microrl_processing_input(&mrl, &init_key, 1);                 //ＬＦを送ってプロンプト「esp8266>」を出力させる
      }
      else
      { //オープン成功
        uint8_t c;
        while(file.available())
        { c = file.read();
          Serial.printf("%c", c);
        }
        file.close();
      }

      gbl_path.clear();
      f_cat_send_bin = false;
      f_send_bin_start = false;
      #if(0)
      microrl_processing_input(&mrl, &init_key, 1);                 //ＬＦを送ってプロンプト「esp8266>」を出力させる
      #endif
    }
  }

  //ESP8266 内部の時間設定用
  if(f_wifi_connected && (time_sync_start || f_counter_trigger))
  {
    f_counter_trigger = false;
    time_sync_start = false;

    if(NTP_client->update())  //時刻をサーバーに問合せ。update()は、最大６０秒１回なので同期されるまで最大で６０秒待たされる。
    { //サーバーと同期がとれた
      struct timeval time_value;
      struct tm* now_time;
      time_t epoch = NTP_client->getEpochTime();
      time_value.tv_sec = epoch;
      time_value.tv_usec = 0;
      settimeofday(&time_value, NULL); //ESP8266 に時間をセット

      //セット後にESP8266から時刻取得し表示
      if(0 == gettimeofday(&time_value, NULL))
      { //成功
        epoch = time_value.tv_sec;
        now_time = localtime(&epoch);
        Serial.printf_P(PSTR("[Time sync] %04d/%02d/%02d (%s) %02d:%02d:%02d\r\n"),
                      now_time->tm_year + 1900, //1900年からの経過年
                      now_time->tm_mon + 1,     //0(1月)〜11(12月)
                      now_time->tm_mday,
                      weekd[now_time->tm_wday],
                      now_time->tm_hour,
                      now_time->tm_min,
                      now_time->tm_sec);
      }
      else
      { //失敗
        Serial.printf_P(PSTR("[e] Read operation failed after the time was set.\r\n"));
      }

      microrl_processing_input(&mrl, &init_key, 1); //プロンプト設定では「>」のみ表示なので、ＬＦを送って「esp8266>」を出力させる
    }
  }

  //赤外線の受信データがあるか確認
  if(irrecv.decode(&results) && f_ir_available)
  { //受信データあり
    yield();
    Serial.printf_P(PSTR("\r\n[i] IR Recved ------\r\n"));
    if(results.overflow)                                                        //キャプチャバッファの容量を超えるサイズのIRメッセージを受信したか
      Serial.printf(D_WARN_BUFFERFULL "\r\n", IR_CAPTURE_BUF_SIZE);
    Serial.printf(D_STR_LIBRARY "   : v" _IRREMOTEESP8266_VERSION_STR "\r\n");
    Serial.printf(D_STR_TOLERANCE " : %d%%\r\n", IR_TOLERANCE);

    Serial.printf_P(PSTR("[i] Basic info ------\r\n"));
    Serial.print(resultToHumanReadableBasic(&results));                         //受信内容の基本的な出力を表示します

    Serial.printf_P(PSTR("[i] A/C info ------\r\n"));
    String description = IRAcUtils::resultAcToString(&results);
    if(description.length())                                                    //追加のエアコン情報があれば、それを表示してください。
      Serial.println(D_STR_MESGDESC ": " + description);
    yield();                                                                    //出力には時間がかかる場合があるため

    //結果をソースコードとして出力する
    Serial.printf_P(PSTR("[i] Source Code info ------\r\n"));
    Serial.println(resultToSourceCode(&results));
    Serial.println();
    yield();

    irrecv.resume();                                                            //次の値を受け取りを継続させる
    microrl_processing_input(&mrl, &init_key, 1);                               //ＬＦを送ってプロンプト「esp8266>」を出力させる
  }
}


/**
 * @brief ＬＥＤの点滅、ＡＤＣサンプルとデジタルデータリード処理タスク
 *        ティックタイマー割込みの「リスタート」で処理が行われる
 *        処理が終わるとタスクは、disable（休止状態）
 */
void TskOneShot_LEDproc_Callback(void)
{
  static uint8_t led_onoff_cnt = 0;
  static int8_t  add_cnt = 0;
  static uint32_t adc_dfata = 0;
  static uint8_t cnt1 = 0;
  static int send_adc_data = 0;
  static int send_di_io0 = 0;

  //ＬＥＤ点滅
  digitalWrite(LED_PIN, (uint8_t)(led_onoff_cnt & 0x01));
  led_onoff_cnt++;

  //ＡＤコンバート、ＳＷ２ボタン読込
  if(40 <= ++cnt1)
  { //４０サイクルに１回変換 （500msの設定の場合、20秒に１回、３回の平均は１分かかる）
    adc_dfata += (uint32_t)ESP8266_ADC_READ;
    add_cnt++;
    if(3 <= add_cnt)
    { //３回サンプリングの平均
      #if(ADC_VCC_MODE)
      send_adc_data = (int)(adc_dfata / 3);
      #else
      send_adc_data = (int)((((float)adc_dfata / 3.0F) * ADC_A0_1LSB_MV) + 0.5F);
      #endif
      send_di_io0 = (digitalRead(IO0_PIN))? 0 : 1;

      //Blynk Cloud にデータ送信
      if(f_blynk_connected)
      { //Blynk Cloud に接続されている状態
        #if(0)
        Serial.printf("\r\n[i] ESP-WROOM A0: %d[mV], GPIO0-IN:%d\r\n", send_adc_data, send_di_io0);
        #endif
        if(f_blynk_send_data)
        { Serial.printf("\r\n[i] ESP-WROOM A0: %d[mV], GPIO0-IN:%d\r\n", send_adc_data, send_di_io0);
          Serial.printf_P(PSTR("[i] Send data to Blynk Cloud.\r\n"));
          Blynk.virtualWrite(V10, send_adc_data);
          Blynk.virtualWrite(V11, send_di_io0);

          microrl_processing_input(&mrl, &init_key, 1); //プロンプト設定では「>」のみ表示なので、ＬＦを送って「esp8266>」を出力させる
        }
      }
      #if(0)
      else
      { //Blynk Cloud に未接続
        Serial.printf_P(PSTR("[!] Data will not be sent because it is not connected to the Blynk Cloud.\r\n"));
      }
      #endif
      add_cnt = 0;
      adc_dfata = 0;
    }
    cnt1 = 0;
  }
}


//--- microrlから呼ばれる シリアル出力用コールバック
int microRL_printOut(microrl_t* mrl, const char* str)
{ return Serial.print(str);
}


//--- microrlから呼ばれる コマンド実行コールバック
int microRL_executeCallback(microrl_t* mrl, int argc, const char* const* argv)
{ if (argc == 0) return 0;
  // テーブル内を検索
  for(size_t i = 0; i < sizeof(commands)/sizeof(command_t); i++)
  { if(strcmp(argv[0], commands[i].name) == 0)
    { commands[i].func(argc, argv);                                       //一致した関数呼び出し
      return 0;
    }
  }
  Serial.printf_P(PSTR("Unknown command. Type 'help' for info.\r\n"));   //コマンド一致するものがなかった
  return 0;
}

/**
 * @brief Construct a new blynk write object
 *        V2ピンに動きがあった時に実行される
 */
BLYNK_WRITE(V2)
{
  //受け取ったデータを変数に保存
  int value = param.asInt();                      //整数として取得 (0か1、またはスライダーの値)

  if(value == 1)
  { //赤外線データを出力する
    Serial.printf_P(PSTR("[i] IR Send: SONY TV Power...\r\n"));
    irsend.send(SONY, 0x0A90, 12, 0U);
  }
  else
  { //何もしない
    Serial.printf_P(PSTR("[i] V2 Button OFF!\r\n"));
  }

  microrl_processing_input(&mrl, &init_key, 1); //プロンプト設定では「>」のみ表示なので、ＬＦを送って「esp8266>」を出力させる
}



//----コマンド処理関数 ---------------------------------------------------------------------------------
/**
 * @brief 利用可能なコマンドのリスト
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンドのみ
 */
void cmd_help(int argc, const char* const* argv)
{
  if(argc != 1)
  { Serial.printf_P(PSTR("Usage: help\r\n"));
    return;
  }

  Serial.printf_P(PSTR("Available commands:\r\n"));
  for(size_t i = 0; i < sizeof(commands)/sizeof(command_t); i++)
  { Serial.printf_P(PSTR("  %-10s : %s\r\n"), commands[i].name, commands[i].help);
  }
}

/**
 * @brief ＬＥＤ　ＬＤ４の点滅スピード
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド、スピード[30...1000]
 */
void cmd_led_spd(int argc, const char* const* argv)
{
  uint16_t _bs;
  if(argc != 2)
  { Serial.printf_P(PSTR("Usage: ledbs <blink speed> (30...1000)\r\n"));
    return;
  }

  _bs = atoi(argv[1]);
  if((30 > _bs) || (1000 < _bs))
  { Serial.printf_P(PSTR("The value is out of range. <blink speed> (30...1000)\r\n"));
  }
  else
  { led_speed = _bs;
    digitalWrite(LED_PIN, LOW);
  }
}

/**
 * @brief 赤外線受信の制御
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド、受信する「on」しない「off」
 */
void cmd_ir_rcv_cntl(int argc, const char* const* argv)
{
  if(argc != 2)
  { Serial.printf_P(PSTR("Usage: irrx <on/off>\r\n"));
    return;
  }

  if(!f_ir_available)
  { Serial.printf_P(PSTR("The infrared interface is currently unavailable.\r\n"));
    return;
  }

  if(0 == strcmp(argv[1], "on"))
  { Serial.printf_P(PSTR("Infrared reception will be enabled.\r\n"));
    irrecv.enableIRIn();                  //受信を再開
    f_ir_rcv_enable = true;
  } else
  if(0 == strcmp(argv[1], "off"))
  { Serial.printf_P(PSTR("Infrared reception will no longer be possible.\r\n"));
    irrecv.disableIRIn();                 //回り込み防止で一時受信を停止
    f_ir_rcv_enable = false;
  }
  else
  { Serial.printf_P(PSTR("Parameter error! irrx <on/off>\r\n"));
  }
}

/**
 * @brief ディレクトリ、ファイルの表示
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド、絶対パスまたは相対パス（対カレントディレクトリ）
 */
void cmd_fs_dir(int argc, const char* const* argv)
{
  String path;
  File fh;
  bool isdir;

  if(!f_littlefs_available)
  { //ファイルシステムが利用不可
    Serial.printf_P(PSTR("[x] not available LittleFS\r\n"));
    return;
  }

  if((0 == argc) || (2 < argc))
  { //パラメータエラー
    Serial.printf_P(PSTR("[i] Usage: ls <dirname>\r\n"));
    return;
  }

  if(1 == argc)
  { path = current_directory;
  }
  else
  { if('/' == argv[1][0])
    { //先頭文字が"/"からの場合は絶対パスと見る
      path = argv[1];
      while((path.length()>1) && (path.startsWith("//")))
      { //argv[1]が２文字以上で、かつ「/」が２個連続している
        path.remove(0, 1);  //最初の「/」を削除する
      }
    }
    else
    { //相対パス
      if(current_directory.endsWith("/"))
        path = current_directory + argv[1];         //カレントディレクトリーの最後が"/"の場合は、そのまま入力文字を足す
      else
        path = current_directory + "/" + argv[1];   //カレントディレクトリーの最後が"/"でない場合は"/"を付加して入力文字を足す
    }
    if((path.length() > 1) && path.endsWith("/"))
    { //結合したパスの末尾が"/"の場合は削除
      path.remove(path.length() - 1); //index=長さ-1 から、最後まで（＝結果的に1文字）削除
    }
  }

  isdir = false;
  if(LittleFS.exists(path))
  { //ディレクトリまたはファイルが存在する
    fh = LittleFS.open(path, "r");
    if(fh)
    { isdir = fh.isDirectory();                         //オープン成功し、できれクトリか？　チェックしてクローズする
      fh.close();
    }
    else
    { Serial.printf_P(PSTR("[e] Failed to open!\r\n")); //オープン失敗しら、戻る
      return;
    }
    
    if(isdir)
    { //ディレクトリーである
      current_directory = path; //カレントディレクトリーを更新
      listDir(path.c_str());    //指定された階層の情報を表示
    }
    else
    { //ディレクトリではない
      Serial.printf_P(PSTR("[e] Is not a directory!\r\n"));
    }
  }
  else
  { //ディレクトリまたはファイルが無い、カレントディレクトリの更新しないで戻る
    Serial.printf_P(PSTR("[e] Directory not found!\r\n"));
  }
}

/**
 * @brief File system ドライブ情報表示
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド
 */
void cmd_fs_disk_info(int argc, const char* const* argv)
{
  if(argc != 1)
  { Serial.printf_P(PSTR("Usage: df\r\n"));
    return;
  }
  getFSInfo();  //空き容量や使用量の情報
}

/**
 * @brief カレントディレクトリーの表示
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド
 */
void cmd_fs_crr_dir_info(int argc, const char* const* argv)
{
  if(!f_littlefs_available)
  { //ファイルシステムが利用不可
    Serial.printf_P(PSTR("[x] not available LittleFS\r\n"));
    return;
  }

  if(1 > argc)
  { //パラメータエラー
    Serial.printf_P(PSTR("[i] Usage: pwd\r\n"));
    return;
  }
  Serial.printf_P(PSTR(" %s\r\n"), current_directory.c_str());
}

/**
 * @brief カレントディレクトリーの変更
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド　絶対パスまたは相対パス（対カレントディレクトリー）
 */
void cmd_fs_change_dir(int argc, const char* const* argv)
{
  String path;
  File fh;
  bool isdir;

  if(!f_littlefs_available)
  { //ファイルシステムが利用不可
    Serial.printf_P(PSTR("[x] not available LittleFS\r\n"));
    return;
  }

  if(2 != argc)
  { //パラメータエラー
    Serial.printf_P(PSTR("[i] Usage: cd <dirname>\r\n"));
    return;
  }

  if('/' == argv[1][0])
  { //先頭文字が"/"からの場合は絶対パスと見る
    path = argv[1];
    while((path.length()>1) && (path.startsWith("//")))
    { //argv[1]が２文字以上で、かつ「/」が２個連続している
      path.remove(0, 1);  //最初の「/」を削除する
    }
  }
  else
  { //相対パス
    if(current_directory.endsWith("/"))
      path = current_directory + argv[1];         //カレントディレクトリーの最後が"/"の場合は、そのまま入力文字を足す
    else
      path = current_directory + "/" + argv[1];   //カレントディレクトリーの最後が"/"でない場合は"/"を付加して入力文字を足す
  }
  if((path.length() > 1) && path.endsWith("/"))
  { //結合したパスの末尾が"/"の場合は削除
    path.remove(path.length() - 1); //index=長さ-1 から、最後まで（＝結果的に1文字）削除
  }

  isdir = false;
  if(LittleFS.exists(path))
  { //ディレクトリまたはファイルが存在する
    fh = LittleFS.open(path, "r");
    if(fh)
    { //オープン成功し、できれクトリか？　チェックしてクローズする
      isdir = fh.isDirectory();
      fh.close();
    }
    else
    { //オープン失敗しら、戻る
      Serial.printf_P(PSTR("[e] Open error for verification!\r\n"));
      return;
    }
    
    if(isdir)
    { //ディレクトリーである
      current_directory = path; //カレントディレクトリーを更新
    }
    else
    { //ディレクトリではない
      Serial.printf_P(PSTR("[e] Is not a directory!\r\n"));
    }
  }
  else
  { //ディレクトリまたはファイルが無い、カレントディレクトリの更新しないで戻る
    Serial.printf_P(PSTR("[e] Directory not found!\r\n"));
  }
}

/**
 * @brief ファイルシステムに赤外線データを保存
 *        {
 *           "next_id":1,
 *           "list":[
 *              {"id":1, "model":"xxx", "function":"power", "ffpath":"/IRCdata/ir0001.ird"},
 *              ・・・・
 *           ]
 *        }
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド、メーカー名、機種名、ボタン名
 */
void cmd_fs_save_ir(int argc, const char* const* argv)
{
  File file;
  JsonDocument jdoc;
  JsonObject root;
  JsonArray array;
  JsonObject ir_new_item;
  DeserializationError jerr;
  String file_path;
  String ird_file_path;
  int id;
  bool found;

  if(argc < 4)
  { Serial.printf_P(PSTR("Usage: irsv <maker name> <model name> <button name>\r\n"));
    return;
  }

  if(!f_littlefs_available)
  { //ファイルシステムの準備ができていない
    Serial.printf_P(PSTR("[e] Not available the LittleFS!!\r\n"));
    return;
  }

  if(!LittleFS.exists("/IRCdata/mfrlist.json"))
  { //ファイルが存在しない
    Serial.printf_P(PSTR("[e] File(/IRCdata/mfrlist.json) could not be found!\r\n"));
    return;
  }

  //メーカー名リストチェック
  file = LittleFS.open("/IRCdata/mfrlist.json", "r");
  if(!file)
  { //赤外線メーカーリストファイルがない
    Serial.printf_P(PSTR("[e] Failed to open file(/IRCdata/mfrlist.json)!\r\n"));
    return;
  }
  else
  { //ファイルがある
    jerr = deserializeJson(jdoc, file); //ファイルからJSONファイルを読込
    file.close();

    if(DeserializationError::Ok != jerr.code())
    { //有効なJSONファイルでない
      Serial.printf_P(PSTR("[e] IR-mfr deserialization failed! %s\r\n"), jerr.f_str());
      return;
    }
    Serial.printf_P(PSTR("[i] IR-mfr deserialization succeeded.\r\n"));
  }

  //JsonArrayとして取り込む
  array = jdoc.as<JsonArray>(); 
  if(array.isNull())
  { //[] もない全く空の状態
    array = jdoc.to<JsonArray>();  //配列化する
  }

  //配列からメーカー名があるか確認する
  found = false;
  for(const char* value : array)
  { if(0 == strcmp(argv[1], value))
    { found = true;
      break;
    }
  }
  if(!found)
  { //argv[1]と同じ内容はなかった
    array.add(argv[1]); //メーカー名追加
    Serial.printf_P(PSTR("[i] Added to mfrlist.json (%s)\r\n"), argv[1]);
    file = LittleFS.open("/IRCdata/mfrlist.json", "w");
    if(!file)
    { //赤外線メーカーリストファイル オープンエラー
      Serial.printf_P(PSTR("[e] Failed to open file(/IRCdata/mfrlist.json)!\r\n"));
      return;
    }
    else
    { //追加したので、更新したJSONを保存してクローズ
      serializeJson(jdoc, file);
      file.close();
    }
  }
  else
  { //メーカー名は既に登録あり
    Serial.printf_P(PSTR("[i] %s already exists in mfrlist.json.\r\n"), argv[1]);
  }
  
  //赤外線データの保存用リスト読込と確認
  file_path = "/IRCdata/" + String(argv[1]) + ".lst"; //赤外線データの保存用リストファイルパス
  if(!LittleFS.exists(file_path))
  { //ファイルが存在しない
    file = LittleFS.open(file_path, "w");
    if(!file)
    { //赤外線データ保存用リストファイル作成に失敗
      Serial.printf_P(PSTR("[e] Failed to open(w) file(%s)!\r\n"), file_path.c_str());
      return;
    }

    //オープン成功：最初のデータ登録になるので、Jsonフォーマットを作成そ保存する
    jdoc.clear();                         //JSON doc クリア
    root = jdoc.to<JsonObject>();         //オブジェクト化する, {}
    root["next_id"] = 1;                  //{"next_id":1} ID は１から始まる
    array = root["list"].to<JsonArray>(); //{"next_id":1, list:[]}
    serializeJson(jdoc, file);            //JSON doc をファイルに保存
    file.close();
  }

  //赤外線データ保存用リストファイルを読込、更新時に再度書込む
  file = LittleFS.open(file_path, "r");
  if(!file)
  { //赤外線データの保存用リストファイル オープン失敗
    Serial.printf_P(PSTR("[e] Failed to open file(%s)!\r\n"), file_path.c_str());
    return;
  }
  else
  { //ファイルオープン成功
    jdoc.clear();                         //JSON doc クリア
    jerr = deserializeJson(jdoc, file);   //ファイルからJSONデータ読み込む
    file.close();                         //読み終わったら一旦閉じる

    if(DeserializationError::Ok != jerr.code())
    { //有効なJSONデータでない
      Serial.printf_P(PSTR("[e] IR-list deserialization failed! %s\r\n"), jerr.f_str());
      return;
    }
    Serial.printf_P(PSTR("[i] IR-list deserialization succeeded.\r\n"));
  }

  //jdoc に JSON ファイル読込済みの状態
  root = jdoc.as<JsonObject>();           //JsonObjectとして取り込む
  id = root["next_id"];                   //IDを取得
  array = root["list"].as<JsonArray>();   //JsonArrayとして取り込む

  //すでに登録あるかの確認する
  found = false;
  //配列内をループで回して検索
  for(JsonObject ir_item : array)
  { //model と function の両方が一致するかチェック
    if((0 == strcmp(ir_item["model"], argv[2])) && (0 == strcmp(ir_item["function"], argv[3])))
    { //一致するデータがあった ffpath を取得
      ird_file_path.clear();
      ird_file_path = ir_item["ffpath"].as<String>();
      found = true;
      break;
    }
  }

  if(!found)
  { //新規追加
    //新しいリモコンデータ（オブジェクト）を追加
    ir_new_item = array.add<JsonObject>();  //配列に新たにオブジェくと追加してハンドル取得
    ir_new_item["id"] = id;                 //idセット
    ir_new_item["model"] = argv[2];         //argv[2]のモデル名セット
    ir_new_item["function"] = argv[3];      //argv[3]のボタン名セット
    char idstr[6];                          //IDをリモコンデータ保存用に使うため
    sprintf_P(idstr, PSTR("%04d"), id);
    ird_file_path.clear();
    ird_file_path = "/IRCdata/" + String(argv[1]) + String(idstr) + ".ird"; //赤外線データの保存用ファイル名とファイルパス
    ir_new_item["ffpath"] = ird_file_path;  //リモコンデータ保存用ファイル名とパス
    root["next_id"] = id + 1;               //次のリモコンデータIDのためにm更新

    //書き込みモードで開き直して保存
    file = LittleFS.open(file_path, "w");
    if(!file)
    { //赤外線データの保存用リストファイル オープン失敗
      Serial.printf_P(PSTR("[e] Failed to open file(%s)!\r\n"), file_path.c_str());
      return;
    }
    else
    { serializeJson(jdoc, file);
      file.close();
      Serial.printf_P(PSTR("[i] List file has been updated, and next ID:%d\r\n"), id);
    }
  }
  else
  { //上書き
    Serial.printf_P(PSTR("[i] IR data has already been registered.\r\n"));
    Serial.printf_P(PSTR("[!] Overwrite the IR data: %s\r\n"), ird_file_path.c_str());
  }


  //保存用構造体に取得した赤外線データを入れる
  memset((void*)&record, 0x00, sizeof(IRDataRecord));     //保存用構造体クリア
  record.protocol  = results.decode_type;                 //プロトコルのコピー
  record.bits = results.bits;                             //ビット数のコピー
  record.repeat = results.repeat;                         //リピートのコピー
  record.rawlen = results.rawlen - 1;                     //生データ数の保存

  //64bit超か判別して保存先を分ける
  if(results.bits > 64)
  { //６４ビットを超えているデータ
    record.isState = true;
    memcpy(record.state, results.state, kStateSizeMax);   //サイズはkStateSizeMax等に合わせて調整
  }
  else
  { //６４ビットを超えたデータ
    record.isState = false;
    record.value = results.value;
  }

  //Raw Data のコピー
  if(IR_RAW_DATA_BUFFER < record.rawlen)
  { Serial.printf_P(PSTR("The received RAW data exceeded the number of destination buffers. (%d)\r\n"), (int)record.rawlen);
    Serial.printf_P(PSTR("The process will be stopped.\r\n"));
    return;
  }
  else
  { Serial.printf_P(PSTR("Length of RAW data(uint16_t) [%d].\r\n"), (int)record.rawlen);
    for(uint16_t r=0; r<record.rawlen; r++)
    { record.rawData[r] = results.rawbuf[r+1] * kRawTick; //results.rawbuf[0] は使われていない為＋１がスタート
    }
  }

  //赤外線データの構造体をそのままファイルに保存
  file= LittleFS.open(ird_file_path, "w");
  if(file)
  { //オープン成功
    size_t WBytes = file.write((uint8_t*)&record, sizeof(IRDataRecord));
    file.close();

    if(WBytes == sizeof(record))
    { Serial.printf_P(PSTR("[i] IR data saved to %s (%d bytes)\r\n"), ird_file_path.c_str(), (int)WBytes);
    }
    else
    { Serial.printf_P(PSTR("[e] Write mismatch! %d / %d\r\n"), (int)WBytes, (int)sizeof(record));
    }
  }
  else
  { //オープン失敗
    Serial.printf_P(PSTR("[e] File could not be opened in write mode.\r\n"));
  }
}

/**
 * @brief ファイルの削除
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド、ファイルの名前
 */
void cmd_fs_remove(int argc, const char* const* argv)
{
  String path;

  if(argc != 2)
  { Serial.printf_P(PSTR("Usage: rm <file name>\r\n"));
    return;
  }

  if(!f_littlefs_available)
  { Serial.printf_P(PSTR("[e] Not available the LittleFS!!\r\n"));
    return;
  }

  //[current_directory]と結合してパスを生成
  if('/' == argv[1][0])
  { //先頭文字が"/"からの場合は絶対パスと見る
    path = argv[1];
    while((path.length()>1) && (path.startsWith("//")))
    { //argv[1]が２文字以上で、かつ「/」が２個連続している
      path.remove(0, 1);  //最初の「/」を削除する
    }
  }
  else
  { //相対パス
    if(current_directory.endsWith("/"))
      path = current_directory + argv[1];         //カレントディレクトリーの最後が"/"の場合は、そのまま入力文字を足す
    else
      path = current_directory + "/" + argv[1];   //カレントディレクトリーの最後が"/"でない場合は"/"を付加して入力文字を足す
  }
  if((path.length() > 1) && path.endsWith("/"))
  { //結合したパスの末尾が"/"の場合は削除
    path.remove(path.length() - 1);               //index=長さ-1 から、最後まで（＝結果的に1文字）削除
  }

  Serial.printf_P(PSTR("[i] Deleting file: %s\r\n"), path);
  if(LittleFS.remove(path))
  { Serial.printf_P(PSTR("   deleted.\r\n"));
  }
  else
  { Serial.printf_P(PSTR("   failed!\r\n"));
  }
}

/**
 * @brief ディレクトリーの削除
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド、ディレクトリの名前
 */
void cmd_fs_remove_dir(int argc, const char* const* argv)
{
  String path;

  if(argc != 2)
  { Serial.printf_P(PSTR("Usage: rmdir <file name>\r\n"));
    return;
  }

  if(!f_littlefs_available)
  { Serial.printf_P(PSTR("[e] Not available the LittleFS!!\r\n"));
    return;
  }

  //[current_directory]と結合してパスを生成
  if('/' == argv[1][0])
  { //先頭文字が"/"からの場合は絶対パスと見る
    path = argv[1];
    while((path.length()>1) && (path.startsWith("//")))
    { //argv[1]が２文字以上で、かつ「/」が２個連続している
      path.remove(0, 1);  //最初の「/」を削除する
    }
  }
  else
  { //相対パス
    if(current_directory.endsWith("/"))
      path = current_directory + argv[1];         //カレントディレクトリーの最後が"/"の場合は、そのまま入力文字を足す
    else
      path = current_directory + "/" + argv[1];   //カレントディレクトリーの最後が"/"でない場合は"/"を付加して入力文字を足す
  }
  if((path.length() > 1) && path.endsWith("/"))
  { //結合したパスの末尾が"/"の場合は削除
    path.remove(path.length() - 1);               //index=長さ-1 から、最後まで（＝結果的に1文字）削除
  }

  Serial.printf_P(PSTR("[i] Deleting directory: %s\r\n"), path);
  if(LittleFS.rmdir(path))
  { Serial.printf_P(PSTR("   deleted.\r\n"));
  }
  else
  { Serial.printf_P(PSTR("   failed!\r\n"));
  }
}

/**
 * @brief ディレクトリーの作成
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド、ディレクトリの名前
 */
void cmd_fs_create_dir(int argc, const char* const* argv)
{
  String path;

  if(argc != 2)
  { Serial.printf_P(PSTR("Usage: mkdir <file name>\r\n"));
    return;
  }

  if(!f_littlefs_available)
  { Serial.printf_P(PSTR("[e] Not available the LittleFS!!\r\n"));
    return;
  }

  //[current_directory]と結合してパスを生成
  if('/' == argv[1][0])
  { //先頭文字が"/"からの場合は絶対パスと見る
    path = argv[1];
    while((path.length()>1) && (path.startsWith("//")))
    { //argv[1]が２文字以上で、かつ「/」が２個連続している
      path.remove(0, 1);  //最初の「/」を削除する
    }
  }
  else
  { //相対パス
    if(current_directory.endsWith("/"))
      path = current_directory + argv[1];         //カレントディレクトリーの最後が"/"の場合は、そのまま入力文字を足す
    else
      path = current_directory + "/" + argv[1];   //カレントディレクトリーの最後が"/"でない場合は"/"を付加して入力文字を足す
  }
  if((path.length() > 1) && path.endsWith("/"))
  { //結合したパスの末尾が"/"の場合は削除
    path.remove(path.length() - 1);               //index=長さ-1 から、最後まで（＝結果的に1文字）削除
  }

  Serial.printf_P(PSTR("[i] Creating directory: %s\r\n"), path);
  if(LittleFS.mkdir(path))
  { Serial.printf_P(PSTR("   created.\r\n"));
  }
  else
  { Serial.printf_P(PSTR("   failed!\r\n"));
  }
}

/**
 * @brief ファイルを変更する
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド、ファイル名前１、ファイル名前２
 */
void cmd_fs_rename_file(int argc, const char* const* argv)
{
  String path1, path2;

  if(argc != 3)
  { Serial.printf_P(PSTR("Usage: rn <name1> <name2>\r\n"));
    return;
  }

  if(!f_littlefs_available)
  { Serial.printf_P(PSTR("[e] Not available the LittleFS!!\r\n"));
    return;
  }

  //[current_directory]と結合してパス１を生成
  if('/' == argv[1][0])
  { //先頭文字が"/"からの場合は絶対パスと見る
    path1 = argv[1];
    while((path1.length()>1) && (path1.startsWith("//")))
    { //argv[1]が２文字以上で、かつ「/」が２個連続している
      path1.remove(0, 1);  //最初の「/」を削除する
    }
  }
  else
  { //相対パス
    if(current_directory.endsWith("/"))
      path1 = current_directory + argv[1];         //カレントディレクトリーの最後が"/"の場合は、そのまま入力文字を足す
    else
      path1 = current_directory + "/" + argv[1];   //カレントディレクトリーの最後が"/"でない場合は"/"を付加して入力文字を足す
  }
  if((path1.length() > 1) && path1.endsWith("/"))
  { //結合したパスの末尾が"/"の場合は削除
    path1.remove(path1.length() - 1);               //index=長さ-1 から、最後まで（＝結果的に1文字）削除
  }

  //[current_directory]と結合してパス２を生成
  if('/' == argv[2][0])
  { //先頭文字が"/"からの場合は絶対パスと見る
    path2 = argv[2];
    while((path2.length()>1) && (path2.startsWith("//")))
    { //argv[1]が２文字以上で、かつ「/」が２個連続している
      path2.remove(0, 1);  //最初の「/」を削除する
    }
  }
  else
  { //相対パス
    if(current_directory.endsWith("/"))
      path2 = current_directory + argv[2];         //カレントディレクトリーの最後が"/"の場合は、そのまま入力文字を足す
    else
      path2 = current_directory + "/" + argv[2];   //カレントディレクトリーの最後が"/"でない場合は"/"を付加して入力文字を足す
  }
  if((path2.length() > 1) && path2.endsWith("/"))
  { //結合したパスの末尾が"/"の場合は削除
    path2.remove(path2.length() - 1);               //index=長さ-1 から、最後まで（＝結果的に1文字）削除
  }

  Serial.printf_P(PSTR("[i] Renaming: %s to %s\r\n"), path1, path2);
  if(LittleFS.rename(path1, path2))
  { Serial.printf_P(PSTR("   renamed.\r\n"));
  }
  else
  { Serial.printf_P(PSTR("   failed!\r\n"));
  }
}

/**
 * @brief 空のファイルを作成する
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド、ファイル名前
 */
void cmd_fs_create_efile(int argc, const char* const* argv)
{
  String path;

  if(argc < 2)
  { Serial.printf_P(PSTR("Usage: touch <file name>\r\n"));
    return;
  }

  if(!f_littlefs_available)
  { Serial.printf_P(PSTR("[e] Not available the LittleFS!!\r\n"));
    return;
  }

  //[current_directory]と結合してパスを生成
  if('/' == argv[1][0])
  { //先頭文字が"/"からの場合は絶対パスと見る
    path = argv[1];
    while((path.length()>1) && (path.startsWith("//")))
    { //argv[1]が２文字以上で、かつ「/」が２個連続している
      path.remove(0, 1);  //最初の「/」を削除する
    }
  }
  else
  { //相対パス
    if(current_directory.endsWith("/"))
      path = current_directory + argv[1];         //カレントディレクトリーの最後が"/"の場合は、そのまま入力文字を足す
    else
      path = current_directory + "/" + argv[1];   //カレントディレクトリーの最後が"/"でない場合は"/"を付加して入力文字を足す
  }
  if((path.length() > 1) && path.endsWith("/"))
  { //結合したパスの末尾が"/"の場合は削除
    path.remove(path.length() - 1);               //index=長さ-1 から、最後まで（＝結果的に1文字）削除
  }

  Serial.printf_P(PSTR("[i] Creating file: %s\r\n"), path.c_str());
  File _fs = LittleFS.open(path, "w");
  if(_fs)
  { Serial.printf_P(PSTR("   created.\r\n"));
    _fs.close();
  }
  else
  { Serial.printf_P(PSTR("   failed!\r\n"));
  }
}

/**
 * @brief ファイルを表示する
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド、ファイルパス、オプション
 */
void cmd_fs_view(int argc, const char* const* argv)
{
  File file;
  JsonDocument jdoc;
  DeserializationError jerr;
  String path;
  bool f_opt, f_end_wait;
  bool f_json_type_file;

  if(argc < 2)
  { Serial.printf_P(PSTR("Usage: cat <file path> <option: -v, -j, -x, -b> <end wait: 1..10S/20S>\r\n"));
    return;
  }
  if(argc >= 3) f_opt = true;      else f_opt = false;
  if(argc == 4) f_end_wait = true; else f_end_wait = false;

  if(!f_littlefs_available)
  { //ファイルシステムの準備ができていない
    Serial.printf_P(PSTR("[e] Not available the LittleFS!!\r\n"));
    return;
  }

  //[current_directory]と結合してパスを生成
  if('/' == argv[1][0])
  { //先頭文字が"/"からの場合は絶対パスと見る
    path = argv[1];
    while((path.length()>1) && (path.startsWith("//")))
    { //argv[1]が２文字以上で、かつ「/」が２個連続している
      path.remove(0, 1);  //最初の「/」を削除する
    }
  }
  else
  { //相対パス
    if(current_directory.endsWith("/"))
      path = current_directory + argv[1];         //カレントディレクトリーの最後が"/"の場合は、そのまま入力文字を足す
    else
      path = current_directory + "/" + argv[1];   //カレントディレクトリーの最後が"/"でない場合は"/"を付加して入力文字を足す
  }
  if((path.length() > 1) && path.endsWith("/"))
  { //結合したパスの末尾が"/"の場合は削除
    path.remove(path.length() - 1);               //index=長さ-1 から、最後まで（＝結果的に1文字）削除
  }

  //ファイルが存在するか確認
  if(!LittleFS.exists(path))
  { //ファイルが存在しない
    Serial.printf_P(PSTR("[e] File(%s) could not be found!\r\n"), path.c_str());
    return;
  }

  //ファイルを開く
  file = LittleFS.open(path, "r");
  if(!file)
  { //オープン失敗
    Serial.printf_P(PSTR("[e] Failed to open file(%s)!\r\n"), path.c_str());
    return;
  }
  else
  { //オープン成功
    if(file.isDirectory())
    { //ディレクトリーである
      Serial.printf_P(PSTR("[!] This name is not a file.(%s)!\r\n"), path);
      file.close();
      return;
    }

    //指定パスはファイルである
    f_json_type_file = false;
    if(f_opt && (0 == strcmp("-j", argv[2])))
    { //JSON形式の表示要求あり
      jerr = deserializeJson(jdoc, file);
      if(DeserializationError::Ok != jerr.code())
      { //JSON形式のファイルではない
        Serial.printf_P(PSTR("[!] not a JSON file! (%s)\r\n"), jerr.f_str());
      }
      else
      { f_json_type_file = true;
      }
    }

    file.seek(0);
    if(f_opt && (0 == strcmp("-v", argv[2])))
    { //制御文字（見えない文字）を ^ 記号を使って可視化
      while(file.available())
      { uint8_t c = file.read();
        if(c == '\n')
        { //改行はそのまま実行
          Serial.println();
        } else
        if(c == '\t')
        { //タブはそのまま表示（または \t と出すなら修正）
          Serial.print('\t');
        } else
        if(c < 32)
        { //ASCII 0-31 の制御文字を ^A, ^B... 形式にする。例）13 (CR) は ^M
          Serial.print("^");
          Serial.print((char)(c + 64)); 
        } else
        if(c == 127)
        { //DELキー
          Serial.print("^?");
        } else
        if(c > 127)
        { //拡張ASCII（バイナリ）は M- (Meta) 形式にするのが cat -v の標準
          Serial.print("M-");
          uint8_t low = c & 0x7F;
          if(low < 32)
          { Serial.print("^");
            Serial.print((char)(low + 64));
          }
          else
          { Serial.print((char)low);
          }
        }
        else
        { //通常の表示可能文字
          Serial.print((char)c);
        }
      }
      Serial.println();
    } else

    if(f_opt && (0 == strcmp("-x", argv[2])))
    { //HEX ダンプ
      int count;
      unsigned int addr;
      uint8_t c;
      char s[20];

      addr = count = 0;
      memset(s, 0, sizeof(s));
      while(file.available())
      { 
        c = file.read();
        if((c < 0x20) || (c == 0x7F) || (c > 0x7F))
          s[count] = '.';
        else
          s[count] = c;
        
        if(0 == count) Serial.printf_P(PSTR(" %08X: "), addr);
        Serial.printf_P(PSTR("%02X"), c);
        count++;
        if(16 > count)
        { Serial.printf_P(PSTR("-"));
        }
        else
        { //１６データ表示した
          Serial.printf_P(PSTR(" [%s]\r\n"), s);
          memset(s, 0, sizeof(s));
          addr += 16;
          count = 0;
        }
      }

      if(0 != count)
      { //残りの表示処理
        while(1)
        { s[count] = 0x20;  //空白
          Serial.printf_P(PSTR("  "));
          count++;
          if(16 > count)
          { Serial.printf_P(PSTR("-"));
          }
          else
          { //１６データ表示した
            Serial.printf_P(PSTR(" [%s]\r\n"), s);
            break;
          }
        }
      }
    }
    else

    if(f_opt && (0 == strcmp("-b", argv[2])))
    { //すべてをそのまま出力するための段取り
      f_send_bin_start = false;
      f_cat_send_bin = true;
      gbl_path = path;          //ファイルのパスをコピー（バイナリ送信時に利用する）
      wait_time2 = 0;
      enter_time2 = millis();   //現在の時間をミリ秒で取得
    } else

    if(f_opt && (0 == strcmp("-j", argv[2])) && f_json_type_file)
    { //JSON 整形された形式で表示
      Serial.printf_P(PSTR("--- Json >>>\r\n"));
      serializeJsonPretty(jdoc, Serial);
      Serial.printf_P(PSTR("\r\n<<< Json ---\r\n"));
    }
    else
    { //その他
      int count;
      uint8_t c;

      count = 0;
      while(file.available())
      { c = file.read();
        if((0x20 > c) || (0x7F < c) || (0x7F == c))
        { if(('\r' == c) || ('\n' == c) /*|| ('\t' == c)*/)
          { //これらはそのまま出力
            Serial.print((char)c);
            count++;
            if(('\n' == c)) count = 0;
          }
          else
          { Serial.print('.');
            count++;
          }
        }
        else
        { Serial.print((char)c);
          count++;
        }
        if(80 <= count)
        { count = 0;
          Serial.println();
        }
      }
      Serial.println();
    }
    file.close();

    //終了前に一定時間を待って終了するか否か（バイナリーデータをＰＣなどで取得するとき、ログなどを終了するまでの猶予時間に使える）
    if(f_end_wait && !f_cat_send_bin && (0 < atoi(argv[3])) && (11 > atoi(argv[3])))
    { //コマンド終了待ち時間あり
      unsigned long enter_time, wait_time;
      
      wait_time = atoi(argv[3]) * 1000; //待ち時間をミリ秒にする
      enter_time = millis();            //現在の時間をミリ秒で取得
      while((millis() - enter_time) < wait_time)
      { yield(); 
      }
    } else
    if(f_end_wait && f_cat_send_bin && (0 < atoi(argv[3])) && (21 > atoi(argv[3])))
    {
      wait_time2 = atoi(argv[3]) * 1000; //タイムアウト時間をミリ秒にする
      Serial.printf_P(PSTR("Binary output will occur either upon receiving the character code “LF” or after %s seconds.\r\n"), argv[3]);
    } else
    if(f_cat_send_bin)
    {
      wait_time2 = 15000; //デフォルト値タイムアウト時間をミリ秒にする
      Serial.printf_P(PSTR("Binary output will occur either upon receiving the character code “LF” or after 15 seconds.\r\n"));
    }

    Serial.println();
  }
}

/**
 * @brief ファイルシステムから赤外線データを読込、データの出力テストを行う
 *  {
 *     "next_id":1,
 *     "list":[
 *        {"id":1, "model":"xxx", "function":"power", "ffpath":"/IRCdata/ir0001.ird"},
 *        ・・・・
 *     ]
 *  }
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド、メーカー名、機種名、ボタン名、テストIR出力(raw, pro, 他は出力しない)
 */
void cmd_fs_load_ir(int argc, const char* const* argv)
{
  File file;
  JsonDocument jdoc;
  JsonArray array;
  DeserializationError jerr;
  String file_path;

  if(argc < 5)
  { Serial.printf_P(PSTR("Usage: irld <maker> <model> <button> <irtest>\r\n"));
    return;
  }

  if(!f_littlefs_available)
  { //ファイルシステムの準備ができていない
    Serial.printf_P(PSTR("[e] Not available the LittleFS!!\r\n"));
    return;
  }

  file_path.clear();
  file_path = "/IRCdata/" + String(argv[1]) + ".lst";

  if(!LittleFS.exists(file_path))
  { //ファイルが存在しない
    Serial.printf_P(PSTR("[e] File(%s) could not be found!\r\n"), file_path.c_str());
    return;
  }

  file = LittleFS.open(file_path, "r");
  if(!file)
  { //指定のメーカー赤外線データリストファイルがない
    Serial.printf_P(PSTR("[e] Failed to open file(%s)!\r\n"), file_path.c_str());
    return;
  }
  else
  { //ファイルオープン成功
    if(file.isDirectory())
    { Serial.printf_P(PSTR("[!] This name is not a file.(%s)!\r\n"), file_path.c_str());
      file.close();
      return;
    }
    
    //ファイルからJSONデータを取得
    jerr = deserializeJson(jdoc, file);
    file.close();

    if(DeserializationError::Ok != jerr.code())
    { Serial.printf_P(PSTR("[e] IR data table deserialization failed! %s\r\n"), jerr.f_str());
      return;
    }
    Serial.printf_P(PSTR("[i] IR data table deserialization succeeded.\r\n"));
  }

  array = jdoc["list"].as<JsonArray>();  //"list"をJsonArrayとして取り込む
  //配列内をループで回して検索
  for(JsonObject ir_item : array)
  { //model と function の両方が一致するかチェック
    if((0 == strcmp(ir_item["model"], argv[2])) && (0 == strcmp(ir_item["function"], argv[3])))
    { //一致するデータがあった ffpath を取得
      file_path.clear();
      file_path = ir_item["ffpath"].as<String>();
      break;
    }
  }

  file = LittleFS.open(file_path, "r");
  if(!file)
  { //指定のメーカー赤外線データファイルがない
    Serial.printf_P(PSTR("[e] Failed to open file(%s)!\r\n"), file_path.c_str());
    return;
  }
  else
  { //ファイルオープン成功（構造体にデータサイズ分を読み込む）
    memset((void*)&record, 0x00, sizeof(IRDataRecord));
    size_t read_byte = file.read((uint8_t*)&record, sizeof(IRDataRecord));
    file.close();

    //読み込んだバイト数が構造体のサイズと一致するか確認
    if(read_byte == sizeof(IRDataRecord))
    {
      Serial.printf_P(PSTR("[i] IR data loaded successfully. (%d bytes)\r\n"), (int)read_byte);
      Serial.printf_P(PSTR("    IR Protocol: %s\r\n"), typeToString(record.protocol).c_str());
      Serial.printf_P(PSTR("    IR Bits    : %d\r\n"), (int)record.bits);
      Serial.printf_P(PSTR("    IR Repeat  : %d\r\n"), (int)record.repeat);
      Serial.printf_P(PSTR("    IR RawLen  : %d\r\n"), (int)record.rawlen);
      if(record.isState)
      { //長いリモコンデータ
        int count, tcount;
        count = 0;
        tcount = 0;
        Serial.printf_P(PSTR("    IR DATA (state)"));
        while(1)
        {
          if(0 == count) Serial.printf_P(PSTR("    "));
          Serial.printf_P(PSTR("%02X"), record.state[tcount]);
          count++; tcount++;
          if((16 > count) && (kStateSizeMax >= tcount))
          {
            if(kStateSizeMax <= tcount)
              break;
            Serial.printf_P(PSTR("-"));
          }
          else
          { //１６データ表示した
            Serial.printf_P(PSTR("\r\n"));
            if(kStateSizeMax <= tcount)
              break;
            count = 0;
          }
        }
        Serial.println();
      }
      else
      { //短いリモコンデータ
        Serial.printf_P(PSTR("    IR DATA    : 0x%016" PRIX64 "\r\n"), record.value); //PRIX64 というマクロは、内部で "llX" という「文字列」として定義
      }

      if(0 != record.rawlen)
      {
        int count, tcount;
        count = 0;
        tcount = 0;
        Serial.printf_P(PSTR("    IR RawData :\r\n"));
        while(1)
        {
          if(0 == count) Serial.printf_P(PSTR("    "));
          Serial.printf_P(PSTR("%10d"), record.rawData[tcount]);
          count++; tcount++;
          if((10 > count) && (1024 >= tcount))
          {
            if((record.rawlen <= tcount) || (1024 <= tcount))
              break;
            Serial.printf_P(PSTR(","));
          }
          else
          { //１０データ表示した
            Serial.printf_P(PSTR("\r\n"));
            if((record.rawlen <= tcount) || (1024 <= tcount))
              break;
            count = 0;
          }
        }
        Serial.println();
      }

      if(f_ir_rcv_enable) irrecv.disableIRIn();   //回り込み防止で一時受信を停止

      if(0 == strcmp("raw", argv[4]))
      { Serial.printf_P(PSTR("Outputs infrared data in raw format.\r\n"));
        irsend.sendRaw(record.rawData, record.rawlen, 38000);
      } else

      if(0 == strcmp("pro", argv[4]))
      {
        if(record.isState)
        { //データが６４ビットを超えている
          Serial.printf_P(PSTR("Output infrared signals as protocol data(>64).\r\n"));
          irsend.send(record.protocol, record.state, (uint16_t)record.bits/8);
        }
        else
        { //データが６４ビット以下である
          Serial.printf_P(PSTR("Output infrared signals as protocol data(<=64).\r\n"));
          irsend.send(record.protocol, record.value, record.bits, record.repeat? 1U : 0U);
        }
      }
      else
      { Serial.printf_P(PSTR("Do not output infrared light.\r\n"));        
      }
      if(f_ir_rcv_enable) irrecv.enableIRIn();    //受信を再開
    }
    else
    {
      Serial.printf_P(PSTR("[e] Load mismatch! %d / %d\r\n"), (int)read_byte, (int)sizeof(IRDataRecord));
    }
  }
}




/**
 * @brief ファイルシステムから赤外線データを削除
 *  {
 *     "next_id":1,
 *     "list":[
 *        {"id":1, "model":"xxx", "function":"power", "ffpath":"/IRCdata/ir0001.ird"},
 *        ・・・・
 *     ]
 *  }
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド、メーカー名、機種名、ボタン名
 */
void cmd_fs_remove_ir(int argc, const char* const* argv)
{
  File file;
  JsonDocument jdoc;
  JsonArray array;
  DeserializationError jerr;
  String file_path, file_maker_path;
  int found_id, match_count;
  bool found, optional, id_match, exec_del_file;

  if(argc < 4)
  { Serial.printf_P(PSTR("Usage: irrm <maker> <model> <function> <ID:Optional>\r\n"));
    return;
  }

  optional = false;
  if(argc == 5) optional = true;

  if(!f_littlefs_available)
  { //ファイルシステムの準備ができていない
    Serial.printf_P(PSTR("[e] Not available the LittleFS!!\r\n"));
    return;
  }

  file_maker_path.clear();
  file_maker_path = "/IRCdata/" + String(argv[1]) + ".lst";

  if(!LittleFS.exists(file_maker_path))
  { //ファイルが存在しない
    Serial.printf_P(PSTR("[e] File(%s) could not be found!\r\n"), file_maker_path.c_str());
    return;
  }

  file = LittleFS.open(file_maker_path, "r");
  if(!file)
  { //指定のメーカー赤外線データリストファイルがない
    Serial.printf_P(PSTR("[e] Failed to open file(%s)!\r\n"), file_maker_path.c_str());
    return;
  }
  else
  { //ファイルオープン成功
    if(file.isDirectory())
    { Serial.printf_P(PSTR("[!] This name is not a file.(%s)!\r\n"), file_maker_path.c_str());
      file.close();
      return;
    }
    
    //ファイルからJSONデータを取得
    jerr = deserializeJson(jdoc, file);
    file.close();

    if(DeserializationError::Ok != jerr.code())
    { Serial.printf_P(PSTR("[e] IR data table deserialization failed! %s\r\n"), jerr.f_str());
      return;
    }
    Serial.printf_P(PSTR("[i] IR data table deserialization succeeded.\r\n"));
  }

  found_id = 0;
  found = false;
  match_count = 0;
  array = jdoc["list"].as<JsonArray>();  //"list"をJsonArrayとして取り込む
  //配列内をループで回して検索
  for(JsonObject ir_item : array)
  { //model と function の両方が一致するかチェック
    if(ir_item["id"].is<int>())
    { //id というキーが存在し数値である
      id_match = (optional)? (atoi(argv[4]) == ir_item["id"].as<int>()) : true;   //ID チェックオプションがある場合　IDの照合を行う
      if((0 == strcmp(ir_item["model"], argv[2])) && (0 == strcmp(ir_item["function"], argv[3])) && id_match)
      { //一致するデータがあった ffpath を取得
        file_path.clear();
        file_path = ir_item["ffpath"].as<String>();
        found_id = ir_item["id"].as<int>();
        found = true;
        match_count++;
        Serial.printf_P(PSTR("[i] Match ID(%d): model(%s), function(%s), file(%s)\r\n"), found_id, argv[2], argv[3], file_path.c_str());
      }
    }
  }
  Serial.println();

  if(!found)
  { //リモコン登録がない
    Serial.printf_P(PSTR("[e] model(%s), function(%s) could not be found!\r\n"), argv[2], argv[3]);
  }
  else
  { //リモコン登録がある
    if(1 != match_count)
    { //一致した項目が複数ある
      Serial.printf_P(PSTR("[!] There are multiple matching items.\r\n"));
      Serial.printf_P(PSTR("[!] Please include the ID of the item you want to delete when executing this command.\r\n"));
      Serial.printf_P(PSTR("    Usage: irrm <maker> <model> <function> <ID>\r\n"));
    }
    else
    { //一致した項目が１個の場合
      Serial.printf_P(PSTR("[i] Registration has been found. Remote control data and registration will be deleted.\r\n"));
      Serial.printf_P(PSTR("[i] Delete ID(%d): model(%s), function(%s), file(%s)\r\n"), found_id, argv[2], argv[3], file_path.c_str());
      Serial.printf_P(PSTR("[!] If you confirm, please press the (Y) key within 10 seconds.\r\n"));

      bool tmout;
      unsigned long enter_time = millis();  //現在の時間をミリ秒で取得
      tmout = true;
      while((millis() - enter_time) < 10000)
      {
        if(0 < Serial.available())
        { //シリアルから受信文字あり
          int c = Serial.read();
          if('Y' == c)
          { //LF コードが確認された
            while(0 < Serial.available())
              Serial.read();                //受信データ空になるまで、読み捨てる
            tmout = false;
            break;
          }
        }
        yield(); 
      }
      Serial.println();

      exec_del_file = false;
      if(tmout)
      { //タイムアウト発生
        Serial.printf_P(PSTR("[!] (Y) could not be verified. Deletion process will be aborted.\r\n"));
      }
      else
      { //JSON オブジェクトの削除と対象のIRデータファイル削除
        for(size_t r=0; r<array.size(); r++)
        { //配列内のオブジェクトを確認
          if(found_id == array[r]["id"])
          { //ターゲットの登録ＩＤが一致
            if(file_path == array[r]["ffpath"].as<String>())
            { //１次チェック時のファイルパスと、今回のパスが一致
              Serial.printf_P(PSTR("[!] Deleted object index(%d): ID(%d), model(%s), function(%s), file(%s)\r\n"), r, found_id,
                (array[r]["model"].as<String>()).c_str(), (array[r]["function"].as<String>()).c_str(), (array[r]["ffpath"].as<String>()).c_str());
              array.remove(r);

              //赤外線データの更新保存
              if(!LittleFS.exists(file_maker_path))
              { //ファイルが存在しない
                Serial.printf_P(PSTR("[e] File(%s) could not be found!\r\n"), file_maker_path.c_str());
                return;
              }
              else
              { //ファイルがある
                file = LittleFS.open(file_maker_path, "w"); //ファイル上書き
                if(!file)
                { //赤外線データ保存用リストファイルオープン失敗
                  Serial.printf_P(PSTR("[e] Failed to open(w) file(%s)!\r\n"), file_maker_path.c_str());
                  return;
                }

                //オープン成功：Json 更新データを保存する
                serializeJson(jdoc, file);  //JSON doc をファイルに保存
                file.close();
                exec_del_file = true;
              }
            }
            else
            { //１次チェック時のファイルパスと、今回のパスが「不一致」
              Serial.printf_P(PSTR("[e] Deletion criteria do not match, so the process has been interrupted.\r\n"));
              exec_del_file = false;
            }
            break;  //削除後抜ける
          }
        }

        if(exec_del_file)
        { //ファイルの削除処理
          Serial.printf_P(PSTR("[i] Deleting file: %s\r\n"), file_path.c_str());
          if(LittleFS.remove(file_path.c_str()))
          { Serial.printf_P(PSTR("[!] Deletion process is complete.\r\n"));
          }
          else
          { Serial.printf_P(PSTR("[e] Deletion process is failed!\r\n"));
          }
        }
      }
    }
  }
}

/**
 * @brief Blynk Cloud にデータを送信するか否か
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド、ON/OFF
 */
void cmd_send2blynk(int argc, const char* const* argv)
{
  if(argc != 2)
  { Serial.printf_P(PSTR("Usage: snd2bc <on/off>\r\n"));
    Serial.printf_P(PSTR("The transmission interval is 120x the LED blink setting value.\r\n"));
    return;
  }

  if(0 == (strcmp("on", argv[1])))
  { Serial.printf_P(PSTR("Enabled data uploads to the Blynk cloud.\r\n"));
    Serial.printf_P(PSTR("The transmission interval is 120x the LED blink setting value.\r\n"));
    f_blynk_send_data = true;
  } else

  if(0 == (strcmp("off", argv[1])))
  { Serial.printf_P(PSTR("Disabled data uploads to the Blynk cloud.\r\n"));
    f_blynk_send_data = false;
  }
  else
  { Serial.printf_P(PSTR("Usage: snd2bc <on/off>\r\n"));
    Serial.printf_P(PSTR("The transmission interval is 120x the LED blink setting value.\r\n"));
  }
}

/**
 * @brief 現在の時刻表示
 * 
 * @param argc コマンドとパラメーターの数
 * @param argv コマンド
 */
void cmd_get_now_time(int argc, const char* const* argv)
{
  struct timeval time_value;
  struct tm* now_time;
  time_t epoch;

  if(argc != 1)
  { Serial.printf_P(PSTR("Usage: ntim\r\n"));
    return;
  }

  //ESP8266から時刻取得し表示
  if(0 == gettimeofday(&time_value, NULL))
  { //成功
    epoch = time_value.tv_sec;
    now_time = localtime(&epoch);
    Serial.printf_P(PSTR("[Now Time] %04d/%02d/%02d (%s) %02d:%02d:%02d\r\n"),
                  now_time->tm_year + 1900, //1900年からの経過年
                  now_time->tm_mon + 1,     //0(1月)〜11(12月)
                  now_time->tm_mday,
                  weekd[now_time->tm_wday],
                  now_time->tm_hour,
                  now_time->tm_min,
                  now_time->tm_sec);
  }
  else
  { //失敗
    Serial.printf_P(PSTR("[e] Operation failed!\r\n"));
  }
}


//---- 処理関数 -------------------------------------------------------------------------------------
/**
 * @brief リモコン保存用のホルダ及びメーカリストJSONファイルがあるか確認
 * 
 */
void init_MFRlist_File(void)
{
  File _fs;
  bool _isDir;

  _fs = LittleFS.open("/IRCdata", "r"); 
  if(_fs)
  {
    _isDir = _fs.isDirectory();
    _fs.close();
    
    if(!_isDir) LittleFS.mkdir("/IRCdata");
  }
  else
  { LittleFS.mkdir("/IRCdata");
  }

  if(!LittleFS.exists("/IRCdata/mfrlist.json"))
  { //ファイルが存在しないので作成
    _fs = LittleFS.open("/IRCdata/mfrlist.json", "w");
    if(_fs)
    { //空のJSONファイルを作成
      if(2 == _fs.print("[]"))
        Serial.printf_P(PSTR("[i] Created file [mfrlist.json]\r\n"));
      else
        Serial.printf_P(PSTR("[e] Failed to create the [mfrlist.json].\r\n"));
      _fs.close();
    }
  }
  else
  { //ファイルが存在する
    Serial.printf_P(PSTR("[i] Confirmed the existence of the file(/IRCdata/mfrlist.json).\r\n"));
  }
}

/**
 * @brief ファイルシステムの空き、使用量表示
 * 
 */
void getFSInfo(void)
{
  FSInfo fs_info;
  LittleFS.info(fs_info);

  uint32_t totalBytes = fs_info.totalBytes;     //全体の容量
  uint32_t usedBytes  = fs_info.usedBytes;      //使用中の容量
  uint32_t freeBytes  = totalBytes - usedBytes; //空き容量

  Serial.printf_P(PSTR("Total space: %10u bytes\r\n"), totalBytes);
  Serial.printf_P(PSTR("Used  space: %10u bytes\r\n"), usedBytes);
  Serial.printf_P(PSTR("Free  space: %10u bytes\r\n"), freeBytes);
}

/**
 * @brief 指定されたパスのディレクトリ、ファイルを表示する
 *        注意：タイムスタンプも表示できるようになっていますが、ESP8266 モジュールのRTCを日時刻をセットしていない場合は
 *             リセット
 * 
 * @param dirname 絶対パス
 */
void listDir(const char *dirname)
{
  bool isEmpty = false;
  Dir root;

  root = LittleFS.openDir(dirname);
  Serial.printf_P(PSTR("Listing: %s\r\n"), dirname);
  isEmpty = true;
  while (root.next())
  { isEmpty = false;
    File file = root.openFile("r");
    if(file)
    { time_t lw = file.getLastWrite();          //更新タイム、作成タイムの場合は「time_t cr = file.getCreationTime();」
      if(file.isDirectory())
        Serial.printf_P(PSTR(" directory: "));  //directory
      else
        Serial.printf_P(PSTR(" File     : "));  //file
      struct tm* tmstruct = localtime(&lw);
      Serial.printf_P(PSTR("%d/%02d/%02d %02d:%02d:%02d "), (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
      String fsz = formatNumber((uint32_t)file.size());
      Serial.printf_P(PSTR("%10s[Byte] "), fsz.c_str());
      Serial.printf_P(PSTR("%s\r\n"), root.fileName().c_str());
      file.close();
    }
  }
  if(isEmpty) Serial.printf_P(PSTR("This directory is empty.\r\n"));
}

/**
 * @brief 数字を３桁ごとに「,」を入れる
 * 
 * @param n 正の数値
 * @return String 
 */
String formatNumber(uint32_t n)
{
  String s = String(n);
  int len = s.length();
  if(len <= 3) return s;

  String res = "";
  int count = 0;
  for(int i=len-1; i>=0; i--)
  { res = s[i] + res;
    count++;
    if((count%3 == 0) && (i != 0))
    { res = "," + res;
    }
  }
  return res;
}




