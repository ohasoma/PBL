// Spresense Arduino Libraries
#include <FrontEnd.h>
#include <OutputMixer.h>
#include <MemoryUtil.h>
#include <arch/board/board.h>

//Utilities
#include <math.h>

FrontEnd *theFrontEnd;
OutputMixer *theMixer;

static const int32_t channel_num = AS_CHANNEL_STEREO;
static const int32_t bit_length = AS_BITLENGTH_16;
static const int32_t frame_sample = 240;
static const int32_t frame_size = frame_sample * (bit_length / 8) * channel_num;  //(=960)

static const int32_t proc_size = frame_size;
static uint8_t proc_buffer[proc_size];  //ここにPCMデータが格納

//---------------------saito global variables--------------------------
static const int32_t delay_buffer_size = frame_size * 100;
static uint16_t delayedBuffer[delay_buffer_size];
static int writePos = 0;
//---------------------------------------------------------------------

bool isCaptured = false;
bool isEnd = false;
bool ErrEnd = false;

struct ProcessConfig {
  bool gain_amp_enabled;
  bool dynamics_modifier_enabled;
  bool soft_crip_enabled;
  bool serial_send_enabled;
  bool delay_enabled;
};

ProcessConfig processConfig;

/**
 * @brief Sample singnal Processing function
 *
 * @param [in] uint16_t   ptr
 * @param [in] int        size
 */
void signal_process(int16_t *ptr, int size) {
  main_filter(ptr, size);
}

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
      Serial.println("-----------------------------");
    }
  }
}

//--------------------------------------------------------------------------------
void saito_filter(int16_t *ptr, int size) {
  //加工処理
  avoid_noise(ptr, size);
  if (processConfig.gain_amp_enabled) {
    gain_amp(ptr, size);
  }
  if (processConfig.dynamics_modifier_enabled) {
    dynamics_modifier(ptr, size);
  }
  if (processConfig.soft_crip_enabled) {
    soft_crip(ptr, size);
  }
  if (processConfig.delay_enabled) {
    delay(ptr, size);
  }
  if (processConfig.serial_send_enabled) {
    Serial.println(*ptr);
  }
}

//-----------------------------加工処理の関数--------------------------------------

void avoid_noise(int16_t *ptr, int size) {
  int16_t *s = ptr;
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

void gain_amp(int16_t *ptr, int size) {
  int16_t *ls = ptr;
  int16_t *rs = ls + 1;
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

void dynamics_modifier(int16_t *ptr, int size) {
  float freq = 1.0;
  int16_t *ls = ptr;
  int16_t *rs = ls + 1;
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

void soft_crip(int16_t *ptr, int size) {
  int16_t thresholdplus = 8000;
  int16_t thresholdminus = -8000;
  int16_t *ls = ptr;
  int16_t *rs = ls + 1;

  for (int32_t cnt = 0; cnt < size; cnt += 4) {
    if (*ls > thresholdplus) {
      *ls = thresholdplus * 2;
    }

    if (*ls < thresholdminus) {
      *ls = thresholdminus * 2;
    }

    if (*rs > thresholdplus) {
      *rs = thresholdplus * 2;
    }

    if (*rs < thresholdminus) {
      *rs = thresholdminus * 2;
    }

    ls += 2;
    rs += 2;
  }
}

void delay(int16_t *ptr, int size) {
  int16_t *ls = ptr;
  int16_t *rs = ls + 1;
  //変数定義など
  for (int32_t cnt = 0; cnt < size; cnt += 4) {
    int16_t tmp;

    tmp = *ls;
    delayedBuffer[writePos] = tmp + delayedBuffer[writePos] / 4;
    *ls = delayedBuffer[writePos];

    tmp = *rs;
    delayedBuffer[writePos + 1] = tmp + delayedBuffer[writePos + 1] / 4;
    *rs = delayedBuffer[writePos + 1];

    writePos += 2;

    if (writePos >= delay_buffer_size) writePos = 0;

    ls += 2;
    rs += 2;
  }
}




//--------------------------------------------------------------------------------
void ohara_filter(int16_t *ptr, int size) {

  //パラメータ定義
  float magnification_L = 1;
  float magnification_R = 1;
  float delay_L = 0;
  float delay_R = 0;

  int16_t *ls = ptr;
  int16_t *rs = ptr + 1;
  for (int32_t cnt = 0; cnt < size; cnt += 4) {

    //音量調整
    int32_t r = (*rs) * magnification_R;
    int32_t l = (*ls) * magnification_L;
    if (r > 32767) r = 32767;
    if (r < -32768) r = -32768;
    if (l > 32767) l = 32767;
    if (l < -32768) l = -32768;

    *rs = (int16_t)r;
    *ls = (int16_t)l;

    ls += 2;
    rs += 2;
  }
}

void force_mono(int16_t *ptr, int size) {
  int16_t *L = ptr;
  int16_t *R = ptr + 1;

  for (int i = 0; i < size; i += 4) {
    *R = *L;  // L の値を R にコピー
    L += 2;
    R += 2;
  }
}

void main_filter(int16_t *ptr, int size) {
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

void frontend_attention_cb(const ErrorAttentionParam *param) {
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
void mixer_attention_cb(const ErrorAttentionParam *param) {
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
                                      AsOutputMixDoneParam *done_param) {
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
  signal_process((int16_t *)proc_buffer, proc_size);

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
