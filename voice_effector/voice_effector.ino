#include <FrontEnd.h>
#include <OutputMixer.h>
#include <MemoryUtil.h>
#include <arch/board/board.h>
#include <math.h>
#include <Audio.h>
#define max_delay 4800  // 最大遅延 100ms (48kHz)

FrontEnd* theFrontEnd;
OutputMixer* theMixer;

static const int32_t channel_num = AS_CHANNEL_STEREO;
static const int32_t bit_length = AS_BITLENGTH_16;
static const int32_t frame_sample = 240;
static const int32_t frame_size = frame_sample * (bit_length / 8) * channel_num;  //(=7680)

static const int32_t proc_size = frame_size;
static uint8_t proc_buffer[proc_size];  //ここにPCMデータが格納

bool isCaptured = false;
bool isEnd = false;
bool ErrEnd = false;

//ohara_filterで使う変数↓-----------------

//--parameter_setting--
const int pin_x = A0;
const int pin_y = A1;
const float av = 0.00025;

//--bias_valume_filter--
static float n;

//--LR_volume_filter--
static float nL, nR;

//--delay_filter--
static bool flag;
static int delay_sample;
static int16_t delayBufL[max_delay];
static int16_t delayBufR[max_delay];
static int writePos = 0;

//--bandstop_filter--
typedef struct {
  float a0, a1, a2;
  float b1, b2;
  float x1, x2;
  float y1, y2;
} BandstopFilter;
static BandstopFilter nfL;
static BandstopFilter nfR;
float fL, fR;
float QL, QR;
float fs = 48000.0f;  // サンプリング周波数

//-----------------------------------------
void signal_process(int16_t* ptr, int size) {
  main_filter(ptr, size);
}
//--------------------------------------------------------------------------------
void saito_filter(int16_t* ptr, int size) {
  //加工処理
}
void ohara_filter(int16_t* ptr, int size) {
  parameter_setting();
  bias_volume_filter(ptr, size);
  LR_volume_filter(ptr, size);
  run_bandstop_filter(ptr, size);
  delay_filter(ptr, size);
}

void main_filter(int16_t* ptr, int size) {
  force_mono(ptr, size);  //LをRにコピー
  saito_filter(ptr, size);
  ohara_filter(ptr, size);
}

void parameter_setting() {
  //座標計算
  static float x_f = 510.0f;
  static float y_f = 510.0f;

  x_f = 0.95f * x_f + 0.05f * analogRead(pin_x);
  y_f = 0.95f * y_f + 0.05f * (1023 - analogRead(pin_y));

  float x = x_f - 510.0f;
  float y = y_f - 510.0f;

  float r = sqrt(pow(x, 2) + pow(y, 2));  //距離
  float rad = atan2(x, y);                //角度
  //bias倍率nを計算
  n = max(0.0f, 4.0f - r / 100.0f);

  //flag 判定
  if (rad >= 0) {
    flag = true;
  } else {
    flag = false;
  }
  //delay計算(ウッドワースの公式を使う)
  delay_sample = static_cast<int>(av * (fabsf(rad) - sinf(fabsf(rad))) * 48000);

  //bandstop_filter
  if (rad >= 0) {
    fR = 9000 - 4500 * sin(abs(rad));
    fL = 9000 + 2000 * sin(abs(rad));
  } else {
    fL = 9000 - 4500 * sin(abs(rad));
    fR = 9000 + 2000 * sin(abs(rad));
  }
  QL = fL / 1000;
  QR = fR / 1000;

  init_bandstop(&nfL, fs, fL, QL);
  init_bandstop(&nfR, fs, fR, QR);

  //LR_volume_filter
  float pan = rad / (M_PI / 2.0f);
  if (pan < -1) pan = -1;
  if (pan > 1) pan = 1;
  float k = 0.7f;  // 耳の遮蔽を表す係数
  nL = (1.0f - k * max(0.0f, pan));
  nR = (1.0f - k * max(0.0f, -pan));
}

void bias_volume_filter(int16_t* ptr, int size) {
  for (int i = 0; i < size; i += 2) {
    int16_t L = ptr[i] * n;
    int16_t R = ptr[i + 1] * n;

    if (L > 32767) L = 32767;
    if (L < -32768) L = -32768;
    if (R > 32767) R = 32767;
    if (R < -32768) R = -32768;

    ptr[i] = L;
    ptr[i + 1] = R;
  }
}
void LR_volume_filter(int16_t* ptr, int size) {
  for (int i = 0; i < size; i += 2) {
    int16_t L = ptr[i] * nL;
    int16_t R = ptr[i + 1] * nR;

    if (L > 32767) L = 32767;
    if (L < -32768) L = -32768;
    if (R > 32767) R = 32767;
    if (R < -32768) R = -32768;

    ptr[i] = L;
    ptr[i + 1] = R;
  }
}

void delay_filter(int16_t* ptr, int size) {
  if (delay_sample < 0) delay_sample = 0;
  if (delay_sample >= max_delay) delay_sample = max_delay - 1;

  for (int i = 0; i < size; i += 2) {

    int16_t L = ptr[i];
    int16_t R = ptr[i + 1];

    int readPos = writePos - delay_sample;
    if (readPos < 0) readPos += max_delay;

    int16_t outL = L;
    int16_t outR = R;

    if (flag) {
      // ★ 左だけ遅延
      outL = delayBufL[readPos];
    } else {
      // ★ 右だけ遅延
      outR = delayBufR[readPos];
    }

    // 出力
    ptr[i] = outL;
    ptr[i + 1] = outR;

    // 現在のサンプルをバッファに保存
    delayBufL[writePos] = L;
    delayBufR[writePos] = R;

    // 書き込み位置を進める
    writePos++;
    if (writePos >= max_delay) writePos = 0;
  }
}

void init_bandstop(BandstopFilter* nf, float fs, float fc, float Q) {

  float w0 = 2.0f * M_PI * fc / fs;
  float alpha = sinf(w0) / (2.0f * Q);

  float b0 = 1.0f;
  float b1 = -2.0f * cosf(w0);
  float b2 = 1.0f;
  float a0 = 1.0f + alpha;
  float a1 = -2.0f * cosf(w0);
  float a2 = 1.0f - alpha;

  nf->a0 = b0 / a0;
  nf->a1 = b1 / a0;
  nf->a2 = b2 / a0;
  nf->b1 = a1 / a0;
  nf->b2 = a2 / a0;
}

float bandstop_process(BandstopFilter* nf, float x) {

  float y = nf->a0 * x + nf->a1 * nf->x1 + nf->a2 * nf->x2
            - nf->b1 * nf->y1 - nf->b2 * nf->y2;

  nf->x2 = nf->x1;
  nf->x1 = x;
  nf->y2 = nf->y1;
  nf->y1 = y;

  return y;
}

void run_bandstop_filter(int16_t* ptr, int size) {

  // ステレオ処理（L/R 交互）
  for (int i = 0; i < size; i += 2) {

    float L = (float)ptr[i];
    float R = (float)ptr[i + 1];

    L = bandstop_process(&nfL, L);
    R = bandstop_process(&nfR, R);

    ptr[i] = (int16_t)L;
    ptr[i + 1] = (int16_t)R;
  }
}

void force_mono(int16_t* ptr, int size) {

  for (int i = 0; i < size; i += 2) {
    ptr[i + 1] = ptr[i];
  }
}

//--------------------------------------------------------------------------------

/**
 * @brief clip function
 *
 * @param [in] val   source value
 * @param [in] peak  clip point value
 */
inline int16_t clip(int32_t val, int32_t peak) {
  return (val > 0) ? ((val < peak) ? val : peak) : ((val > (-1 * peak)) ? val : (-1 * peak));
}

/**
 * @brief Frontend attention callback
 *
 * When audio internal error occurs, this function will be called back.
 */

void frontend_attention_cb(const ErrorAttentionParam* param) {
  puts("Attention!");

  if (param->error_code >= AS_ATTENTION_CODE_WARNING) {
    ErrEnd = true;
  }
}

/**
 * @brief OutputMixer attention callback
 *
 * When audio internal error occurs, this function will be called back.
 */
void mixer_attention_cb(const ErrorAttentionParam* param) {
  puts("Attention!");

  if (param->error_code >= AS_ATTENTION_CODE_WARNING) {
    ErrEnd = true;
  }
}

/**
 * @brief Frontend done callback procedure
 *
 * @param [in] event        AsMicFrontendEvent type indicator
 * @param [in] result       Result
 * @param [in] sub_result   Sub result
 *
 * @return true on success, false otherwise
 */

static bool frontend_done_callback(AsMicFrontendEvent ev, uint32_t result, uint32_t sub_result) {
  UNUSED(ev);
  UNUSED(result);
  UNUSED(sub_result);
  return true;
}

/**
 * @brief Mixer done callback procedure
 *
 * @param [in] requester_dtq    MsgQueId type
 * @param [in] reply_of         MsgType type
 * @param [in,out] done_param   AsOutputMixDoneParam type pointer
 */
static void outputmixer_done_callback(MsgQueId requester_dtq,
                                      MsgType reply_of,
                                      AsOutputMixDoneParam* done_param) {
  UNUSED(requester_dtq);
  UNUSED(reply_of);
  UNUSED(done_param);
  return;
}

/**
 * @brief Pcm capture on FrontEnd callback procedure
 *
 * @param [in] pcm          PCM data structure
 */
static void frontend_pcm_callback(AsPcmDataParam pcm) {
  if (!pcm.is_valid) {
    puts("Invalid data !");
    memset(proc_buffer, 0, frame_size);
  } else {
    if (pcm.size > frame_size) {
      puts("Capture size is too big!");
      pcm.size = frame_size;
    }

    if (pcm.size == 0) {
      memset(proc_buffer, 0, frame_size);
    } else {
      memcpy(proc_buffer, pcm.mh.getPa(), pcm.size);
    }
  }

  if (pcm.is_end) {
    isEnd = true;
  }

  isCaptured = true;

  return;
}

/**
 * @brief Mixer data send callback procedure
 *
 * @param [in] identifier   Device identifier
 * @param [in] is_end       For normal request give false, for stop request give true
 */
static void outmixer0_send_callback(int32_t identifier, bool is_end) {
  /* Do nothing, as the pcm data already sent in the main loop. */
  UNUSED(identifier);
  UNUSED(is_end);
  return;
}

/**
 * @brief Execute signal processing for one frame
 */
bool execute_aframe() {
  isCaptured = false;
  signal_process((int16_t*)proc_buffer, proc_size);

  AsPcmDataParam pcm_param;

  /* Alloc MemHandle */
  while (pcm_param.mh.allocSeg(S0_REND_PCM_BUF_POOL, frame_size) != ERR_OK) {
    delay(1);
  }

  if (isEnd) {
    pcm_param.is_end = true;
  } else {
    pcm_param.is_end = false;
  }

  /* Set PCM parameters */
  pcm_param.identifier = OutputMixer0;
  pcm_param.callback = 0;
  pcm_param.bit_length = bit_length;
  pcm_param.size = frame_size;
  pcm_param.sample = frame_sample;
  pcm_param.is_valid = true;

  memcpy(pcm_param.mh.getPa(), proc_buffer, pcm_param.size);

  int err = theMixer->sendData(OutputMixer0,
                               outmixer0_send_callback,
                               pcm_param);

  if (err != OUTPUTMIXER_ECODE_OK) {
    printf("OutputMixer send error: %d\n", err);
    return false;
  }

  return true;
}

/**
 * @brief Setup Audio Objects
 *
 * Set input device to Mic <br>
 * Initialize frontend to capture stereo and 48kHz sample rate <br>
 */
void setup() {
  /* Initialize serial */
  Serial.begin(115200);
  while (!Serial)
    ;

  /* Initialize memory pools and message libs */
  initMemoryPools();
  createStaticPools(MEM_LAYOUT_RECORDINGPLAYER);

  /* Clear the buffer for singnal processing */
  memset(proc_buffer, 0, proc_size);

  /* Begin objects */
  theFrontEnd = FrontEnd::getInstance();
  theMixer = OutputMixer::getInstance();

  theFrontEnd->begin(frontend_attention_cb);
  theMixer->begin();

  puts("begin FrontEnd and OutputMixer");

  /* Create Objects */
  theMixer->create(mixer_attention_cb);

  /* Set capture clock */
  theFrontEnd->setCapturingClkMode(FRONTEND_CAPCLK_NORMAL);

  /* Activate objects */
  theFrontEnd->activate(frontend_done_callback);
  theMixer->activate(OutputMixer0, outputmixer_done_callback);

  usleep(100 * 1000); /* waiting for Mic startup */

  /* Initialize each objects */
  AsDataDest dst;
  dst.cb = frontend_pcm_callback;

  theFrontEnd->init(channel_num, bit_length, frame_sample, AsDataPathCallback, dst);

  /* Set rendering volume */
  theMixer->setVolume(0, 0, 0);

  /* Unmute */
  board_external_amp_mute_control(false);

  theFrontEnd->start();
}

/**
 * @brief audio loop
 */
void loop() {
  if (ErrEnd) {
    puts("Error End");
    theFrontEnd->stop();
    goto exitCapturing;
  }

  if (isCaptured) {
    if (!execute_aframe()) {
      printf("Rendering error!\n");
      goto exitCapturing;
    }
  }

  if (isEnd && !isCaptured) {
    isEnd = false;
    goto exitCapturing;
  }

  return;

exitCapturing:
  board_external_amp_mute_control(true);
  theFrontEnd->deactivate();
  theMixer->deactivate(OutputMixer0);
  theFrontEnd->end();
  theMixer->end();

  puts("Exit.");
  exit(1);
}
