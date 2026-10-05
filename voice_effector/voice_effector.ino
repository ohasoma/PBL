// Spresense Arduino Libraries
#include <FrontEnd.h>
#include <OutputMixer.h>
#include <MemoryUtil.h>
#include <arch/board/board.h>
#include <Audio.h>
#include <stdint.h>
#define max_delay 4800  // 最大遅延 100ms (48kHz)

//Utilities
#include <math.h>

FrontEnd* theFrontEnd;
OutputMixer* theMixer;

static const int32_t channel_num = AS_CHANNEL_STEREO;
static const int32_t bit_length = AS_BITLENGTH_16;
static const int32_t frame_sample = 240;
static const int32_t frame_size = frame_sample * (bit_length / 8) * channel_num;  //(=960)

static const int32_t proc_size = frame_size;
static uint8_t proc_buffer[proc_size];  //ここにPCMデータが格納

bool isCaptured = false;
bool isEnd = false;
bool ErrEnd = false;

struct ProcessConfig {
  bool gain_amp_enabled;
  bool dynamics_modifier_enabled;
  bool soft_crip_enabled;
  bool serial_send_enabled;
  bool delay_enabled;
  bool ohara_serial;
  bool all_serial;
  bool echo_enabled;
};

static ProcessConfig processConfig;

//---------------------saito global variables--------------------------
static const int32_t delay_buffer_size = frame_size * 100;
static int16_t delayedBuffer[delay_buffer_size];
static int accessPos = 0;
static int16_t lpfy = 0;
static float lpl0, lpl1, lpl2, lpl3, lpl4, lpl5, lpl6, lpl7, lpl8, lpl9;
static float lpr0, lpr1, lpr2, lpr3, lpr4, lpr5, lpr6, lpr7, lpr8, lpr9;
//---------------------------------------------------------------------
//ohara_filterで使う変数↓-----------------

//--parameter_setting--
const int pin_x = A0;
const int pin_y = A1;
const float av = 0.00025;
float x, y, r, rad;
unsigned long current = 0;
volatile int adc_x = 510;
volatile int adc_y = 510;
float nmax = 3.0f;

//--bias_valume_filter--
float n;

//--LR_volume_filter--
float nL, nR;

//--delay_filter--
bool flag;
static int delay_sample;
static int16_t delayBufL[max_delay];
static int16_t delayBufR[max_delay];
int writePos = 0;

//--ピーキングEQ--
typedef struct {
  float a0, a1, a2;
  float b1, b2;
  float x1, x2;
  float y1, y2;
} PeakingEQ;
static PeakingEQ peL, peR;
float fL, fR;
float QL, QR;
float dBL, dBR;
float fs = 48000.0f;  // サンプリング周波数

//--------------------------------------------------------------------------------
//テンプレート
// void templete(int16_t* ptr, int size){
//   int16_t *ls = ptr;
//   int16_t *rs = ls + 1;
//   //変数定義など
//     for (int32_t cnt = 0; cnt < size; cnt += 4) {
//     //ここに処理
//     ls += 2;
//     rs += 2;
//   }
// }

//--------------------------------------------------------------------------------
//シリアル通信
void serial_recieve() {
  if (Serial.available() > 0) {
    // シリアルデータの受信 (改行まで)
    String data = Serial.readStringUntil('\n');

    if (data == "amp") {
      processConfig.gain_amp_enabled = !processConfig.gain_amp_enabled;
      Serial.print("gain_amp: ");
      Serial.println(processConfig.gain_amp_enabled);
    }
    if (data == "dynamics") {
      processConfig.dynamics_modifier_enabled = !processConfig.dynamics_modifier_enabled;
      Serial.print("dynaimcs_modifier: ");
      Serial.println(processConfig.dynamics_modifier_enabled);
    }
    if (data == "softcrip") {
      processConfig.soft_crip_enabled = !processConfig.soft_crip_enabled;
      Serial.print("soft_crip: ");
      Serial.println(processConfig.soft_crip_enabled);
    }
    if (data == "delay") {
      processConfig.delay_enabled = !processConfig.delay_enabled;
      Serial.print("delay: ");
      Serial.println(processConfig.delay_enabled);
    }
    if (data == "serial") {
      processConfig.serial_send_enabled = !processConfig.serial_send_enabled;
      Serial.print("serial_send: ");
      Serial.println(processConfig.serial_send_enabled);
    }
    if (data == "ohara_serial") {
      processConfig.ohara_serial = !processConfig.ohara_serial;
      Serial.print("ohara_serial: ");
      Serial.println(processConfig.ohara_serial);
    }
    if (data == "all_serial") {
      processConfig.all_serial = !processConfig.all_serial;
      Serial.print("all_serial: ");
      Serial.println(processConfig.all_serial);
    }
    if (data == "echo") {
      processConfig.echo_enabled = !processConfig.echo_enabled;
      Serial.print("echo: ");
      Serial.println(processConfig.echo_enabled);
    }
    if (data == "status") {
      Serial.println("-----------status------------");
      Serial.print("amp: ");
      Serial.println(processConfig.gain_amp_enabled);
      Serial.print("dynamics: ");
      Serial.println(processConfig.dynamics_modifier_enabled);
      Serial.print("softcrip: ");
      Serial.println(processConfig.soft_crip_enabled);
      Serial.print("delay: ");
      Serial.println(processConfig.delay_enabled);
      Serial.print("serial: ");
      Serial.println(processConfig.serial_send_enabled);
      Serial.print("ohara_serial: ");
      Serial.println(processConfig.ohara_serial);
      Serial.print("all_serial: ");
      Serial.println(processConfig.all_serial);
      Serial.print("echo: ");
      Serial.println(processConfig.echo_enabled);
      Serial.println("-----------------------------");
    }
  }
}




/**
 * @brief Sample singnal Processing function
 *
 * @param [in] uint16_t   ptr
 * @param [in] int        size
 */
void signal_process(int16_t* ptr, int size) {
  main_filter(ptr, size);
}

//--------------------------------------------------------------------------------
void saito_filter(int16_t* ptr, int size) {
  //加工処理
  avoid_noise(ptr, size);
  if (processConfig.gain_amp_enabled) {
    gain_amp(ptr, size);
  }
  if (processConfig.dynamics_modifier_enabled) {
    //dynamics_modifier(ptr, size);
    lpf(ptr, size, 0.1f);
  }
  if (processConfig.soft_crip_enabled) {
    soft_crip(ptr, size);
  }
  if (processConfig.delay_enabled) {
    delay(ptr, size, 500, 0.25f, 0.9f, &lpl9, &lpr9);
  }
  if (processConfig.echo_enabled) {
    pre_reverb(ptr, size);
    echo(ptr, size);
  }
  if (processConfig.serial_send_enabled) {
    Serial.print(-16384);  // Your absolute minimum visible Y-value
    Serial.print(",");
    Serial.print(16384);  // Your absolute maximum visible Y-value
    Serial.print(",");
    Serial.println(*ptr);
  }

  if (processConfig.all_serial) {
    all_data_serial_send(ptr, size);
  }
}

//-----------------------------加工処理の関数--------------------------------------

void avoid_noise(int16_t* ptr, int size) {
  int16_t* s = ptr;
  int16_t peak = 1;
  for (int cnt = 0; cnt < size; cnt += 2) {
    int16_t a = abs(*s);
    if (a > peak) peak = a;
    s += 1;
  }

  if (peak > 1500) {
    return;
  } else {
    s = ptr;
    for (int cnt = 0; cnt < size; cnt += 2) {
      *s = 0;
      s += 1;
    }
  }
}

void gain_amp(int16_t* ptr, int size) {
  int16_t* ls = ptr;
  int16_t* rs = ls + 1;
  int16_t gain_std = 15000;
  int16_t peak = 1;
  for (int cnt = 0; cnt < size; cnt += 4) {
    int16_t a = abs(*ls);
    int16_t b = abs(*rs);
    if (a > peak) peak = a;
    if (b > peak) peak = b;
    ls += 2;
    rs += 2;
  }

  if (peak < 1500) return;

  float amp = gain_std / peak;
  ls = ptr;
  rs = ls + 1;
  for (int32_t cnt = 0; cnt < size; cnt += 4) {
    int32_t tmp;

    tmp = *ls;
    *ls = int16_t(tmp * amp);
    tmp = *rs;
    *rs = int16_t(tmp * amp);

    ls += 2;
    rs += 2;
  }
}

void dynamics_modifier(int16_t* ptr, int size) {
  float freq = 1.0;
  int16_t* ls = ptr;
  int16_t* rs = ls + 1;
  float gain = (1.0f + sinf(0.002f * PI * freq * float(millis()))) / 2.0f;

  for (int32_t cnt = 0; cnt < size; cnt += 4) {
    int32_t tmp;

    tmp = float(*ls);
    *ls = int16_t(tmp * gain);
    tmp = float(*rs);
    *rs = int16_t(tmp * gain);

    ls += 2;
    rs += 2;
  }
}

void soft_crip(int16_t* ptr, int size) {
  int16_t thresholdplus = 8000;
  int16_t thresholdminus = -8000;
  int16_t* ls = ptr;
  int16_t* rs = ls + 1;

  for (int32_t cnt = 0; cnt < size; cnt += 4) {
    if (*ls > thresholdplus) {
      *ls = thresholdplus;
    }

    if (*ls < thresholdminus) {
      *ls = thresholdminus;
    }

    if (*rs > thresholdplus) {
      *rs = thresholdplus;
    }

    if (*rs < thresholdminus) {
      *rs = thresholdminus;
    }
  }
}

void delay(int16_t* ptr, int size, int delay_time_ms, float mlt, float a, float* lpStatel, float* lpStater) {
  int16_t* ls = ptr;
  int16_t* rs = ls + 1;
  int delay_buffer_diff = delay_time_ms * 96;

  for (int32_t cnt = 0; cnt < size; cnt += 4) {
    int16_t tmp;
    int fb;

    int fetchPos = (((accessPos - delay_buffer_diff) % delay_buffer_size) + delay_buffer_size) % delay_buffer_size;

    tmp = *ls;
    fb = delayedBuffer[fetchPos];
    *lpStatel += a * (fb - *lpStatel);
    delayedBuffer[accessPos] = tmp + *lpStatel * mlt;
    *ls = delayedBuffer[accessPos];

    tmp = *rs;
    fb = delayedBuffer[fetchPos + 1];
    *lpStater += a * (fb - *lpStater);
    delayedBuffer[accessPos + 1] = tmp + *lpStater * mlt;
    *rs = delayedBuffer[accessPos + 1];

    accessPos += 2;

    if (accessPos >= delay_buffer_size) accessPos = 0;

    ls += 2;
    rs += 2;
  }
}

void pre_reverb(int16_t* ptr, int size) {
  delay(ptr, size, 7, 0.15f, 0.1f, &lpl0, &lpr0);
  delay(ptr, size, 19, 0.1f, 0.1f, &lpl1, &lpr1);
  delay(ptr, size, 29, 0.07f, 0.1f, &lpl2, &lpr2);
  delay(ptr, size, 37, 0.05f, 0.1f, &lpl3, &lpr3);
}

void echo(int16_t* ptr, int size) {
  delay(ptr, size, 100, 0.3f, 0.1f, &lpl4, &lpr4);
  delay(ptr, size, 170, 0.3f, 0.1f, &lpl5, &lpr5);
  delay(ptr, size, 290, 0.3f, 0.1f, &lpl6, &lpr6);
  delay(ptr, size, 370, 0.2f, 0.1f, &lpl7, &lpr7);
  delay(ptr, size, 410, 0.2f, 0.1f, &lpl8, &lpr8);
}

void lpf(int16_t* ptr, int size, float a) {
  int16_t* ls = ptr;
  int16_t* rs = ls + 1;

  for (int32_t cnt = 0; cnt < size; cnt += 4) {
    lpfy = lpfy + a * (*ls - lpfy);
    *ls = lpfy;
    *rs = lpfy;
    ls += 2;
    rs += 2;
  }
}

void all_data_serial_send(int16_t* ptr, int size) {
  int16_t* ls = ptr;

  for (int32_t cnt = 0; cnt < size; cnt += 4) {
    Serial.println(*ls);
    ls += 16;
  }
}
//--------------------------------------------------------------------------------
void ohara_filter(int16_t* ptr, int size) {
  parameter_setting();
  LR_volume_filter(ptr, size);
  run_peaking_eq_filter(ptr, size);
  delay_filter(ptr, size);
}

void parameter_setting() {
  //座標計算
  static float x_f = 510.0f;
  static float y_f = 510.0f;

  x_f = 0.95f * x_f + 0.05f * adc_x;
  y_f = 0.95f * y_f + 0.05f * adc_y;

  x = x_f - 510.0f + 10.0f;
  y = -(y_f - 510.0f) - 43.0f;

  if (x < 15 && x > -15) x = 0;
  if (y < 15 && y > -15) y = 1;

  r = sqrtf(x * x + y * y);  //距離
  rad = atan2(x, y);         //角度
  //bias倍率nを計算
  n = (279.0f * nmax) / (r * (nmax - 1.0f) + (280.0f - nmax));

  //flag 判定
  if (rad >= 0) {
    flag = true;
  } else {
    flag = false;
  }
  //delay計算(ウッドワースの公式を使う)
  delay_sample = static_cast<int>(av * (fabsf(rad) - sinf(fabsf(rad))) * 48000);

  //peaking_eq
  float abs_rad = fabsf(rad);

  if (rad >= 0) {
    fR = 9000 - 4500 * sinf(abs_rad);
    fL = 9000 + 2000 * sinf(abs_rad);
  } else {
    fL = 9000 - 4500 * sinf(abs_rad);
    fR = 9000 + 2000 * sinf(abs_rad);
  }

  if (abs_rad > (M_PI / 2.0f)) {
    // 【後ろにいるとき】
    // 1kHz付近を強調（山）し、高音域（fL, fR）を大きくデシベルで削る（谷）
    // 後ろに回り込むほど（abs_radがπに近づくほど）効果を強くする
    float rear_factor = (abs_rad - (M_PI / 2.0f)) / (M_PI / 2.0f);  // 0.0(真横) 〜 1.0(真後ろ)

    // 後ろからの音は高音(fL, fR)をガッツリ削る (例: 最大 -12dB の谷)
    dBL = -12.0f * rear_factor;
    dBR = -12.0f * rear_factor;

    // ※もし「1kHzの強調」も同時にやりたい場合は、直列にもう一つEQを繋ぐのが理想ですが、
    // まずはこの高音のノッチ（谷）を再現するだけでも、劇的に「後ろ感」が出ます。

    // 後ろの音は耳介で遮られて少しマイルドに変化するため、Q値を少し低め（広め）にする
    QL = (fL / 500.0f) * 0.5f;
    QR = (fR / 500.0f) * 0.5f;

  } else {
    // 【前にいるとき】
    // 前方からの音はクッキリ聴こえるため、高音域をわずかに強調するか、フラット(0dB)にする
    float front_factor = 1.0f - (abs_rad / (M_PI / 2.0f));  // 1.0(真前) 〜 0.0(真横)

    dBL = 2.0f * front_factor;  // 前方にいるときは少し高音をシャープに強調 (+2dBの山)
    dBR = 2.0f * front_factor;

    QL = fL / 500.0f;
    QR = fR / 500.0f;
  }

  // 安全のためQ値が小さくなりすぎないようにガード
  if (QL < 0.5f) QL = 0.5f;
  if (QR < 0.5f) QR = 0.5f;

  // 新しいピーキングEQ関数を呼び出し (AL, AR は dB値 として渡されます)
  init_peaking_eq(&peL, fs, fL, QL, dBL);
  init_peaking_eq(&peR, fs, fR, QR, dBR);

  //LR_volume_filter
  float pan = rad / (M_PI / 2.0f);
  if (pan < -1) pan = -1;
  if (pan > 1) pan = 1;
  float k = 0.7f;  // 耳の遮蔽を表す係数
  nL = n * (1.0f - k * max(0.0f, pan));
  nR = n * (1.0f - k * max(0.0f, -pan));
}

void bias_volume_filter(int16_t* ptr, int size) {

  int16_t* L = ptr;
  int16_t* R = ptr + 1;

  for (int i = 0; i < size; i += 4) {

    int32_t l = (*L) * n;
    int32_t r = (*R) * n;

    if (l > 32767) l = 32767;
    if (l < -32768) l = -32768;

    if (r > 32767) r = 32767;
    if (r < -32768) r = -32768;

    *L = (int16_t)l;
    *R = (int16_t)r;

    L += 2;
    R += 2;
  }
}

void LR_volume_filter(int16_t* ptr, int size) {

  int16_t* L = ptr;
  int16_t* R = ptr + 1;

  for (int i = 0; i < size; i += 4) {

    int32_t l = (*L) * nL;
    int32_t r = (*R) * nR;

    if (l > 32767) l = 32767;
    if (l < -32768) l = -32768;
    if (r > 32767) r = 32767;
    if (r < -32768) r = -32768;

    *L = (int16_t)l;
    *R = (int16_t)r;

    L += 2;
    R += 2;
  }
}

void delay_filter(int16_t* ptr, int size) {

  if (delay_sample < 0) delay_sample = 0;
  if (delay_sample >= max_delay) delay_sample = max_delay - 1;

  int16_t* Lp = ptr;
  int16_t* Rp = ptr + 1;

  for (int i = 0; i < size; i += 4) {

    int16_t L = *Lp;
    int16_t R = *Rp;

    int readPos = writePos - delay_sample;
    if (readPos < 0) readPos += max_delay;

    int16_t outL = L;
    int16_t outR = R;

    if (flag) {
      outL = delayBufL[readPos];
    } else {
      outR = delayBufR[readPos];
    }

    *Lp = outL;
    *Rp = outR;

    delayBufL[writePos] = L;
    delayBufR[writePos] = R;

    writePos++;
    if (writePos >= max_delay) writePos = 0;

    Lp += 2;
    Rp += 2;
  }
}

void init_peaking_eq(PeakingEQ* eq, float fs, float fc, float Q, float dB) {

  float A = powf(10.0f, dB / 40.0f);

  float w0 = 2.0f * (float)M_PI * fc / fs;
  float alpha = sinf(w0) / (2.0f * Q);

  float b0 = 1.0f + alpha * A;
  float b1 = -2.0f * cosf(w0);
  float b2 = 1.0f - alpha * A;
  float a0 = 1.0f + alpha / A;
  float a1 = -2.0f * cosf(w0);
  float a2 = 1.0f - alpha / A;

  eq->a0 = b0 / a0;
  eq->a1 = b1 / a0;
  eq->a2 = b2 / a0;
  eq->b1 = a1 / a0;
  eq->b2 = a2 / a0;
}

float peaking_eq_process(PeakingEQ* eq, float x) {
  float y = eq->a0 * x + eq->a1 * eq->x1 + eq->a2 * eq->x2
            - eq->b1 * eq->y1 - eq->b2 * eq->y2;

  eq->x2 = eq->x1;
  eq->x1 = x;
  eq->y2 = eq->y1;
  eq->y1 = y;

  return y;
}




void run_peaking_eq_filter(int16_t* ptr, int size) {

  int16_t* L = ptr;
  int16_t* R = ptr + 1;

  for (int i = 0; i < size; i += 4) {

    float l = (float)(*L);
    float r = (float)(*R);

    l = peaking_eq_process(&peL, l);
    r = peaking_eq_process(&peR, r);

    if (l > 32767.0f) l = 32767.0f;
    if (l < -32768.0f) l = -32768.0f;
    if (r > 32767.0f) r = 32767.0f;
    if (r < -32768.0f) r = -32768.0f;

    *L = (int16_t)l;
    *R = (int16_t)r;

    L += 2;
    R += 2;
  }
}

void force_mono(int16_t* ptr, int size) {
  int16_t* L = ptr;
  int16_t* R = ptr + 1;

  for (int i = 0; i < size; i += 4) {
    *R = *L;  // L の値を R にコピー
    L += 2;
    R += 2;
  }
}


void main_filter(int16_t* ptr, int size) {
  force_mono(ptr, size);  //LをRにコピー
  saito_filter(ptr, size);
  ohara_filter(ptr, size);
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

  //---------------------------------------------------------
  memset(delayedBuffer, 0, delay_buffer_size);
  //---------------------------------------------------------

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

  /* process config initialize*/
  processConfig.dynamics_modifier_enabled = false;
  processConfig.soft_crip_enabled = false;
  processConfig.gain_amp_enabled = false;
  processConfig.serial_send_enabled = false;
  processConfig.delay_enabled = false;
  processConfig.ohara_serial = false;
  processConfig.all_serial = false;
}

/**
 * @brief audio loop
 */
void loop() {
  if (processConfig.ohara_serial) {
    Serial.print(x);
    Serial.print(" ");
    Serial.print(y);
    Serial.print(" ");
    Serial.print(r);
    Serial.print(" ");
    Serial.println(n);
    /*
  Serial.print(" ");
  Serial.println(nL);
  Serial.print(" ");
  Serial.println(QL);
  */
  }

  if (millis() - current > 100) {
    adc_x = analogRead(A0);
    adc_y = analogRead(A1);
    current = millis();
  }
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

  //changed
  serial_recieve();
  //till here

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
