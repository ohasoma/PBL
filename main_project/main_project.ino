#include <FrontEnd.h>
#include <OutputMixer.h>
#include <MemoryUtil.h>
#include <arch/board/board.h>

FrontEnd* theFrontEnd;
OutputMixer* theMixer;

static const int32_t channel_num = AS_CHANNEL_STEREO;
static const int32_t bit_length = AS_BITLENGTH_16;
static const int32_t frame_sample = 240;
static const int32_t frame_size = frame_sample * (bit_length / 8) * channel_num;

static uint8_t proc_buffer[frame_size];

bool isCaptured = false;
bool isEnd = false;

/* マイク入力コールバック */
static void frontend_pcm_callback(AsPcmDataParam pcm) {
  if (!pcm.is_valid || pcm.size == 0) {
    memset(proc_buffer, 0, frame_size);
  } else {
    memcpy(proc_buffer, pcm.mh.getPa(), pcm.size);
  }

  if (pcm.is_end) {
    isEnd = true;
  }

  isCaptured = true;
}

/* Mixer 送信コールバック（何もしない） */
static void outmixer_send_callback(int32_t id, bool end_flag) {
  UNUSED(id);
  UNUSED(end_flag);
}

/* 1フレーム処理（加工なし） */
bool execute_aframe() {
  AsPcmDataParam pcm_param;

  /* 出力バッファ確保 */
  while (pcm_param.mh.allocSeg(S0_REND_PCM_BUF_POOL, frame_size) != ERR_OK) {
    delay(1);
  }

  pcm_param.identifier = OutputMixer0;
  pcm_param.callback = 0;
  pcm_param.bit_length = bit_length;
  pcm_param.size = frame_size;
  pcm_param.sample = frame_sample;
  pcm_param.is_valid = true;
  pcm_param.is_end = isEnd;

  /* 加工なし → そのままコピー */
  memcpy(pcm_param.mh.getPa(), proc_buffer, frame_size);

  int err = theMixer->sendData(OutputMixer0, outmixer_send_callback, pcm_param);
  return (err == OUTPUTMIXER_ECODE_OK);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  initMemoryPools();
  createStaticPools(MEM_LAYOUT_RECORDINGPLAYER);

  memset(proc_buffer, 0, frame_size);

  theFrontEnd = FrontEnd::getInstance();
  theMixer = OutputMixer::getInstance();

  theFrontEnd->begin();
  theMixer->begin();

  theMixer->create();

  theFrontEnd->setCapturingClkMode(FRONTEND_CAPCLK_NORMAL);

  theFrontEnd->activate();
  theMixer->activate(OutputMixer0, outmixer_send_callback);

  delay(100);

  AsDataDest dst;
  dst.cb = frontend_pcm_callback;

  theFrontEnd->init(channel_num, bit_length, frame_sample, AsDataPathCallback, dst);

  theMixer->setVolume(0, 0, 0); //ボリューム調整

  board_external_amp_mute_control(false);

  theFrontEnd->start();

  Serial.println("Audio passthrough started.");
}




void loop()
{
  if (isCaptured) {
    isCaptured = false;

    // ① 可変抵抗の値を読む
    int valL = analogRead(A0);  // 0〜1023
    int valR = analogRead(A1);  // 0〜1023

    // ② 0〜1023 → ゲインにマッピング（例：0.0〜2.0倍）
    float gainL = (float)valL / 1023.0f * 2.0f;
    float gainR = (float)valR / 1023.0f * 2.0f;

    // ③ 左右の音量を反映
    volume_lr((int16_t*)proc_buffer, frame_sample * 2, gainL, gainR);

    // ④ 出力
    execute_aframe();
  }
}

//左右で音の大きさを変える関数
void volume_lr(int16_t* pcm, int samples, float gainL, float gainR)
{
  for (int i = 0; i < samples; i += 2) {
    // 左チャンネル
    int32_t L = pcm[i] * gainL;
    if (L > 32767) L = 32767;
    if (L < -32768) L = -32768;
    pcm[i] = L;

    // 右チャンネル
    int32_t R = pcm[i + 1] * gainR;
    if (R > 32767) R = 32767;
    if (R < -32768) R = -32768;
    pcm[i + 1] = R;
  }
}

