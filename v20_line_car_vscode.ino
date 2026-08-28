#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

/*
 * ============ 四路红外巡线 + 超声横移避障 v20（横移/直行之间加0.8秒停顿） ============
 *
 * 基于用户实测 v19，唯一新增功能：
 *   左移完成 -> 停止0.8秒 -> 编码器定距直行
 *   直行完成 -> 停止0.8秒 -> 右移找线
 *
 * 实现方式：新增两个纯停顿状态 OBS_PAUSE_TO_FORWARD / OBS_PAUSE_TO_RIGHT，
 * 停顿期间三轮全部停止、状态机只倒计时，不做任何判断。
 * ★★★ 巡线、避障、转弯、终点和 TFT 控制策略均未修改 ★★★
 *
 * [v21] 越障直行同步增强（只动 OBS_FORWARD 这一段）：
 *   A/D独立基础速度 + 同步增益12->8 + 修正上限8->14 + 修正下限38，
 *   新增直行同步诊断串口输出（计数差/修正量/均值），用于标定基础速度。
 *   巡线、横移、转弯、终点逻辑未动。
 *
 * [v22] 检测到障碍后先停顿0.8秒，再开始左移：
 *   复用原 OBS_BRAKE 停稳状态，时长 100ms -> 800ms。
 *   停顿期间三轮全停、不判线不判距；编码器横移起点在停顿结束后记录，
 *   停顿期间的蠕动不计入左移距离。其余流程全部不变。
 *
 * [v23] 越障直行结束判据改为"任一轮到达目标即两轮同拍停"：
 *   消除先到轮急停、落后轮独自滑行造成的尾部姿态偏移。
 *
 * [v24] 结束判据进一步改为"两轮都到达目标才同拍停"：
 *   实测A轮编码器在较高PWM下会间歇性虚计数（同一次运行里左移时
 *   A/D计数1%内吻合，直行时却15:1，右移时B轮计数也爆炸过），
 *   "任一轮到达"会被A轮假计数误触发，车只挪一小段就提前停车。
 *   改为两轮都到达后，单轮假计数骗不过判据；两轮仍在同一控制周期
 *   一起停，v23的防尾部偏航效果保留。
 *   另加3秒超时兜底：编码器若彻底失效（计数不动），强制结束直行。
 *
 * 屏幕架构说明（为什么不在 loop() 里刷屏）：
 *   - [v25] ST7735 改走硬件SPI（GPIO矩阵路由到SCK=9/MOSI=1，接线不变），
 *     一帧<10ms，CPU占用相比软SPI下降一个数量级
 *   - 辅助任务跑核心0：超声任务(优先级1) + TFT任务(优先级0) + 主循环(核心1)
 *   - TFT 任务每 50ms 自动读取全局状态刷新，与控制完全解耦
 *
 * 硬件接线与 v19 相同：
 *   TFT: CS=8 RST=3 DC=2 SCK=9 MOSI=1，VCC/BLK=3.3V（[v25]硬件SPI）
 *   编码器: A轮 48/47, B轮 40/39, D轮 42/41
 *   超声: TRIG=19 ECHO=20，红外: GPIO4~7，电机: 10~18/21，灯: 38
 *
 * 运行方式：烧录后自动运行；RESET = 重新进入 3 秒准备期
 * ==============================================================
 */

/* 传感器物理顺序（已实车确认）：OUT1=最左 .. OUT4=最右 */
#define SENSOR_FLIP false

// ---------- 传感器引脚 ----------
const uint8_t SENSOR_PINS[4] = {4, 5, 6, 7};   // OUT1..OUT4 -> GPIO4..GPIO7
const uint8_t BLACK_LEVEL    = LOW;             // 实测：黑线输出 0

// 采样滤波：9 次采样，至少 6 票相同才采用
const uint8_t  SAMPLE_COUNT         = 9;
const uint8_t  BLACK_VOTE_THRESHOLD = 6;
const uint16_t SAMPLE_INTERVAL_US   = 1000;

// ---------- 电机引脚 ----------
const int STBY_PIN = 10;

const int PWMA_PIN = 11;                        // A 左前
const int AIN1_PIN = 12;
const int AIN2_PIN = 13;

const int PWMB_PIN = 14;                        // B 后（巡线不参与，保持停转）
const int BIN1_PIN = 15;
const int BIN2_PIN = 16;

const int PWMD_PIN = 17;                        // D 右前
const int DIN1_PIN = 18;
const int DIN2_PIN = 21;

const int PWM_FREQUENCY  = 10000;
const int PWM_RESOLUTION = 8;                   // 8 位：速度范围 0~255

// ---------- 板载 RGB 状态灯（WS2812, GPIO38） ----------
const int RGB_LED_PIN = 38;
// 红闪=准备期  绿色=巡线中  橙色=丢线找线中  青色=找回确认期

// ---------- HC-SR04 超声波传感器 ----------
const uint8_t TRIG_PIN = 19;
const uint8_t ECHO_PIN = 20;

// ==================== [v15] TFT屏幕引脚 ====================
#define TFT_CS     8
#define TFT_RST    3
#define TFT_DC     2
#define TFT_SCK    9
#define TFT_MOSI   1
#define TFT_REFRESH_MS 50    /* [v25] 硬件SPI后一帧<10ms，50ms刷新基本实时 */

// [v25] 硬件SPI：GPIO矩阵把FSPI路由到SCK=9/MOSI=1，接线不变，一帧<10ms。
// 旧写法(传引脚号)是软SPI，CPU逐位翻转GPIO，一帧要几十毫秒。
Adafruit_ST7735 tft(&SPI, TFT_CS, TFT_DC, TFT_RST);

const uint8_t TFT_LINE_H    = 14;
const uint8_t TFT_DATA_Y0   = 22;
const uint8_t TFT_ROW_MAX   = 6;
const uint8_t TFT_LINE_PAD  = 26;

// ---------- A/B/D轮霍尔编码器 ----------
// 当前只统计A相的上升沿和下降沿，B相用于判断方向（AB正交x2计数）。
const uint8_t ENCODER_A_A_PIN = 48;
const uint8_t ENCODER_A_B_PIN = 47;
const uint8_t ENCODER_B_A_PIN = 40;
const uint8_t ENCODER_B_B_PIN = 39;
const uint8_t ENCODER_D_A_PIN = 42;
const uint8_t ENCODER_D_B_PIN = 41;

volatile int32_t encoderACount = 0;
volatile int32_t encoderBCount = 0;
volatile int32_t encoderDCount = 0;
int32_t forwardStartACount = 0;
int32_t forwardStartDCount = 0;

// [v21] 直行同步诊断：修正量累计与最近一次修正值（仅供串口标定，不参与控制）
int32_t forwardSyncCorrSum = 0;
uint32_t forwardSyncCorrSamples = 0;
int lastForwardSyncCorr = 0;

// 仅供横移日志使用，不参与任何电机控制或状态判断。
int32_t lateralStartACount = 0;
int32_t lateralStartBCount = 0;
int32_t lateralStartDCount = 0;
int32_t lateralLastACount = 0;
int32_t lateralLastBCount = 0;
int32_t lateralLastDCount = 0;

// ---------- 横移避障参数（v19 用户实测标定值，原样保留） ----------
const float SOUND_SPEED_CM_PER_US = 0.0343f;
const float OBSTACLE_TRIGGER_CM   = 10.0f; // 连续测得小于阈值，启动避障
const float OBSTACLE_CLEAR_CM     = 30.0f; // 超时无回波或大于30cm，视为前方已清空
const uint8_t OBSTACLE_CLEAR_COUNT   = 1;
const uint16_t ULTRASONIC_INTERVAL_MS = 50;
// 恢复已经在实车上验证有效的30ms回波等待时间。
const uint32_t ULTRASONIC_TIMEOUT_US  = 30000;
// [v22] 检测到障碍后的停顿时长：原100ms停稳，现停0.8s再开始左移
const uint16_t OBSTACLE_BRAKE_MS      = 800;
const uint16_t OBSTACLE_MIN_SIDE_MS   = 200;
const uint16_t OBSTACLE_CLEAR_EXTRA_LEFT_MS = 0;   // 清空后无需额外续移（实测调整）
const int OBSTACLE_LATERAL_A_SPEED    = 35;
const int OBSTACLE_LATERAL_B_SPEED    = 57;
const int OBSTACLE_LATERAL_D_SPEED    = 35;
// 左移实测的编码器绝对计数比例，作为横移三轮目标比例。
// 右移使用相同绝对比例，但三个轮子的计数方向全部取反。
const int32_t LATERAL_RATIO_A_COUNTS = 220;
const int32_t LATERAL_RATIO_B_COUNTS = 440;
const int32_t LATERAL_RATIO_D_COUNTS = 220;
// 霍尔比例同步只小幅修正A/D，B保持原基准速度，避免改变已验证的横移动力。
const int LATERAL_SYNC_DIVISOR = 6;
const int LATERAL_SYNC_MAX     = 6;
const int LATERAL_MIN_SPEED    = 35; // 防止霍尔修正把A/D压入实测无法起步区
// [v21] A/D独立基础速度：先同值，按实测修正均值标定后分开
const int OBSTACLE_FORWARD_A_SPEED     = 66;
const int OBSTACLE_FORWARD_D_SPEED     = 34;
const int32_t OBSTACLE_FORWARD_TARGET_COUNTS = 420;
// [v24] 直行超时兜底：正常走完约0.3~1秒，3秒还没到说明编码器失效，强制结束
const uint32_t OBSTACLE_FORWARD_TIMEOUT_MS = 1000;
// A/D累计计数同步：领先侧降速、落后侧提速，减少越障直行偏航。
// [v21] 增益 12->8、上限 8->14，同步修正更快、余量更大
const int OBSTACLE_FORWARD_SYNC_DIVISOR = 8;
const int OBSTACLE_FORWARD_SYNC_MAX     = 14;
// [v21] 修正下限：不许把落后轮压进实测起步死区（最低A/D PWM约33）
const int OBSTACLE_FORWARD_MIN_A        = 38;
const int OBSTACLE_FORWARD_MIN_D        = 38;
const int OBSTACLE_RIGHT_A_SPEED       = 34;
const int OBSTACLE_RIGHT_B_SPEED       = 49;
const int OBSTACLE_RIGHT_D_SPEED       = 34;
const uint16_t OBSTACLE_MIN_RIGHT_MS   = 200;
const uint8_t OBSTACLE_LINE_CONFIRM_COUNT = 3;
const uint16_t OBSTACLE_COOLDOWN_MS    = 1000;

/* [v20新增] 横移→直行、直行→右移之间的停顿时长 */
const uint16_t OBSTACLE_STOP_PAUSE_MS = 800;

/* --------------------------------------------------------------
 * 【巡线参数】调车只改这几个数
 * -------------------------------------------------------------- */
const int BASE_SPEED       = 48;
const int GAIN_NEAR        = 12;
const int GAIN_FAR         = 6;
const int CURVE_SLOWDOWN   = 4;
const int TURN_MAX         = 60;
const int SEARCH_SPEED     = 44;
const int SEARCH_REVERSE_SPEED = 44;
const int REACQUIRE_CYCLES = 12;
const int REACQUIRE_SPEED  = 40;
const int REACQUIRE_GAIN   = 8;
const int RAMP_STEP        = 3;
const int REVERSE_RAMP_STEP = 18;
const int MOTOR_START_SPEED = 40;

// 统一大角度转弯：采用不对称轮速，旋转时保留少量沿弯道前进的趋势
const int LOCKED_TURN_SPEED       = 48;
const int LOCKED_REVERSE_SPEED    = 40;
const uint16_t TURN_MIN_MS        = 120;
const uint16_t TURN_MAX_LOCK_MS   = 900;
const uint8_t SPECIAL_PATTERN_CONFIRM = 2;
const uint8_t NEW_PATH_CONFIRM    = 3;
const int PENDING_FORWARD_SPEED   = 37;
const uint8_t LOST_TO_TURN_CONFIRM = 1;
const uint8_t PENDING_CANCEL_CONFIRM = 15;

// 探头位置权重：线在哪，误差就偏向哪（负=偏左，正=偏右）
const int WEIGHT_x10[4] = {-30, -10, +10, +30};   // [最左 中左 中右 最右]

const uint32_t READY_MS = 3000;
const uint16_t PRINT_MS = 200;

// ---------- 运行状态 ----------
bool lineBlack[4] = {false, false, false, false};

int  lastErr_x10 = 0;
int  lastDir = 0;
int  err_x10 = 0;

bool wasLost = false;
uint8_t reacquireCnt = 0;

int curLeft  = 0;
int curRight = 0;
// [v25] B轮当前PWM（只有横移时非0），供TFT实时显示；正=右移 负=左移
int curBack  = 0;

uint32_t lastPrint = 0;
const char* actionName = "";

enum LockedTurnState {
  TURN_NONE,
  TURN_LEFT,
  TURN_RIGHT
};

LockedTurnState turnState = TURN_NONE;
LockedTurnState pendingTurn = TURN_NONE;
uint32_t turnStartMs = 0;
uint8_t newPathConfirmCnt = 0;
uint8_t leftTurnEntryCnt = 0;
uint8_t rightTurnEntryCnt = 0;
bool originalDeviationCleared = false;
uint8_t pendingLostCnt = 0;
uint8_t pendingCenterCnt = 0;
bool pendingSawFullBlack = false;

enum ObstacleState {
  OBS_FOLLOW_LINE,
  OBS_BRAKE,
  OBS_MOVE_LEFT,
  OBS_PAUSE_TO_FORWARD,   // [v20新增] 左移完成后的0.8秒停顿
  OBS_FORWARD,
  OBS_PAUSE_TO_RIGHT,     // [v20新增] 直行完成后的0.8秒停顿
  OBS_MOVE_RIGHT
};

ObstacleState obstacleState = OBS_FOLLOW_LINE;
uint32_t obstacleStateStartMs = 0;
float lastDistanceCm = 400.0f;
bool lastDistanceValid = false;
uint8_t obstacleClearCount = 0;
bool obstacleClearExtraActive = false;
uint32_t obstacleClearExtraStartMs = 0;
uint8_t obstacleLineConfirmCount = 0;
bool rightMoveSawNoLine = false;
uint32_t obstacleCooldownUntilMs = 0;
bool obstacleRouteCompleted = false;
bool stoppedAtFinalCross = false;

// [v20新增] 停顿状态起始时刻（两个停顿状态共用一个变量即可）
uint32_t obstaclePauseStartMs = 0;

// 从外侧看顺时针时实测计数为负：A前进为逆时针，D前进为顺时针。
void IRAM_ATTR onEncoderAChange() {
  if (digitalRead(ENCODER_A_A_PIN) == digitalRead(ENCODER_A_B_PIN)) {
    encoderACount++;
  } else {
    encoderACount--;
  }
}

void IRAM_ATTR onEncoderBChange() {
  if (digitalRead(ENCODER_B_A_PIN) == digitalRead(ENCODER_B_B_PIN)) {
    encoderBCount++;
  } else {
    encoderBCount--;
  }
}

void IRAM_ATTR onEncoderDChange() {
  if (digitalRead(ENCODER_D_A_PIN) == digitalRead(ENCODER_D_B_PIN)) {
    encoderDCount++;
  } else {
    encoderDCount--;
  }
}

void readAllEncoderCounts(int32_t &aCount, int32_t &bCount, int32_t &dCount) {
  noInterrupts();
  aCount = encoderACount;
  bCount = encoderBCount;
  dCount = encoderDCount;
  interrupts();
}

void readEncoderCounts(int32_t &aCount, int32_t &dCount) {
  noInterrupts();
  aCount = encoderACount;
  dCount = encoderDCount;
  interrupts();
}

void saveForwardEncoderStart() {
  readEncoderCounts(forwardStartACount, forwardStartDCount);
}

void startLateralEncoderLog() {
  readAllEncoderCounts(lateralStartACount,
                       lateralStartBCount,
                       lateralStartDCount);
  lateralLastACount = lateralStartACount;
  lateralLastBCount = lateralStartBCount;
  lateralLastDCount = lateralStartDCount;
}

void printLateralEncoderLog() {
  int32_t currentACount;
  int32_t currentBCount;
  int32_t currentDCount;
  readAllEncoderCounts(currentACount, currentBCount, currentDCount);

  Serial.print("[横移霍尔] 方向=");
  Serial.print(obstacleState == OBS_MOVE_LEFT ? "左移" : "右移");
  Serial.print(" 时间=");
  Serial.print(millis() - obstacleStateStartMs);
  Serial.print("ms | A:AB=");
  Serial.print(digitalRead(ENCODER_A_A_PIN));
  Serial.print(digitalRead(ENCODER_A_B_PIN));
  Serial.print(" 相对=");
  Serial.print(currentACount - lateralStartACount);
  Serial.print(" 增量=");
  Serial.print(currentACount - lateralLastACount);
  Serial.print(" | B:AB=");
  Serial.print(digitalRead(ENCODER_B_A_PIN));
  Serial.print(digitalRead(ENCODER_B_B_PIN));
  Serial.print(" 相对=");
  Serial.print(currentBCount - lateralStartBCount);
  Serial.print(" 增量=");
  Serial.print(currentBCount - lateralLastBCount);
  Serial.print(" | D:AB=");
  Serial.print(digitalRead(ENCODER_D_A_PIN));
  Serial.print(digitalRead(ENCODER_D_B_PIN));
  Serial.print(" 相对=");
  Serial.print(currentDCount - lateralStartDCount);
  Serial.print(" 增量=");
  Serial.println(currentDCount - lateralLastDCount);

  lateralLastACount = currentACount;
  lateralLastBCount = currentBCount;
  lateralLastDCount = currentDCount;
}

void calculateLateralEncoderSpeeds(bool moveLeft,
                                   int &aSpeed,
                                   int &bSpeed,
                                   int &dSpeed) {
  int32_t currentACount;
  int32_t currentBCount;
  int32_t currentDCount;
  readAllEncoderCounts(currentACount, currentBCount, currentDCount);

  int32_t aProgress;
  int32_t bProgress;
  int32_t dProgress;

  if (moveLeft) {
    // 左移实测方向：A负、B正、D负。
    aProgress = -(currentACount - lateralStartACount);
    bProgress =  (currentBCount - lateralStartBCount);
    dProgress = -(currentDCount - lateralStartDCount);
  } else {
    // 右移方向完全取反：A正、B负、D正。
    aProgress =  (currentACount - lateralStartACount);
    bProgress = -(currentBCount - lateralStartBCount);
    dProgress =  (currentDCount - lateralStartDCount);
  }

  // 启动瞬间可能出现一个反向毛刺，不允许负进度参与比例计算。
  if (aProgress < 0) aProgress = 0;
  if (bProgress < 0) bProgress = 0;
  if (dProgress < 0) dProgress = 0;

  // 以B轮为基准，按目标比例计算此刻A、D应达到的累计计数。
  int32_t expectedA = bProgress * LATERAL_RATIO_A_COUNTS /
                      LATERAL_RATIO_B_COUNTS;
  int32_t expectedD = bProgress * LATERAL_RATIO_D_COUNTS /
                      LATERAL_RATIO_B_COUNTS;

  int aCorrection = (int)((expectedA - aProgress) / LATERAL_SYNC_DIVISOR);
  int dCorrection = (int)((expectedD - dProgress) / LATERAL_SYNC_DIVISOR);
  aCorrection = constrain(aCorrection, -LATERAL_SYNC_MAX, LATERAL_SYNC_MAX);
  dCorrection = constrain(dCorrection, -LATERAL_SYNC_MAX, LATERAL_SYNC_MAX);

  int baseASpeed = moveLeft ? OBSTACLE_LATERAL_A_SPEED
                            : OBSTACLE_RIGHT_A_SPEED;
  int baseBSpeed = moveLeft ? OBSTACLE_LATERAL_B_SPEED
                            : OBSTACLE_RIGHT_B_SPEED;
  int baseDSpeed = moveLeft ? OBSTACLE_LATERAL_D_SPEED
                            : OBSTACLE_RIGHT_D_SPEED;

  aSpeed = constrain(baseASpeed + aCorrection, LATERAL_MIN_SPEED, 255);
  bSpeed = baseBSpeed;
  dSpeed = constrain(baseDSpeed + dCorrection, LATERAL_MIN_SPEED, 255);
}

void printForwardEncoderCalibration(uint32_t elapsedMs) {
  int32_t currentACount;
  int32_t currentDCount;
  readEncoderCounts(currentACount, currentDCount);

  int32_t aForwardCounts = currentACount - forwardStartACount;
  int32_t dForwardCounts = -(currentDCount - forwardStartDCount);
  int32_t averageCounts = (aForwardCounts + dForwardCounts) / 2;

  Serial.println();
  Serial.println("========== 越障直行编码器标定 ==========");
  Serial.print("直行时间(ms): ");
  Serial.println(elapsedMs);
  Serial.print("A轮前进计数: ");
  Serial.println(aForwardCounts);
  Serial.print("D轮前进计数: ");
  Serial.println(dForwardCounts);
  Serial.print("A/D平均计数: ");
  Serial.println(averageCounts);
  Serial.println("========================================");
}

/* ======== [v21] 直行同步诊断打印（随 printStatus 每200ms一次） ========
 * 读法：修正均值持续为正 -> A轮天生慢，A_SPEED+=均值、D_SPEED-=均值；
 *       修正均值持续为负 -> D轮天生慢，反向调整。
 *       把均值搬进基础速度后，±14的修正权限全部留给动态扰动。
 * ===================================================================== */
void printForwardSyncLog() {
  int32_t currentACount;
  int32_t currentDCount;
  readEncoderCounts(currentACount, currentDCount);

  int32_t aForwardCounts = currentACount - forwardStartACount;
  int32_t dForwardCounts = -(currentDCount - forwardStartDCount);

  int meanCorr = (forwardSyncCorrSamples > 0)
                     ? (int)(forwardSyncCorrSum /
                             (int32_t)forwardSyncCorrSamples)
                     : 0;

  Serial.print("[直行同步] A=");
  Serial.print(aForwardCounts);
  Serial.print(" D=");
  Serial.print(dForwardCounts);
  Serial.print(" 计数差(D-A)=");
  Serial.print(dForwardCounts - aForwardCounts);
  Serial.print(" 本次修正=");
  Serial.print(lastForwardSyncCorr);
  Serial.print(" 修正均值=");
  Serial.println(meanCorr);
}

// 三次阻塞式测距放到ESP32-S3另一核心，避免阻塞约10ms一次的巡线循环。
portMUX_TYPE ultrasonicMux = portMUX_INITIALIZER_UNLOCKED;
volatile float ultrasonicTaskDistanceCm = 400.0f;
volatile bool ultrasonicTaskDistanceValid = false;
volatile uint32_t ultrasonicSampleSequence = 0;

/* ================= 传感器：带投票滤波的读取 ================= */
void readSensors() {
  uint8_t blackVotes[4] = {0, 0, 0, 0};

  for (uint8_t sample = 0; sample < SAMPLE_COUNT; sample++) {
    for (uint8_t i = 0; i < 4; i++) {
      if (digitalRead(SENSOR_PINS[i]) == BLACK_LEVEL) {
        blackVotes[i]++;
      }
    }
    delayMicroseconds(SAMPLE_INTERVAL_US);
  }

  for (uint8_t i = 0; i < 4; i++) {
    bool isBlack = (blackVotes[i] >= BLACK_VOTE_THRESHOLD);
    uint8_t pos  = SENSOR_FLIP ? (3 - i) : i;
    lineBlack[pos] = isBlack;
  }
}

/* ================= 电机底层：单电机方向+速度 ================= */
void setMotor(int pwmPin, int in1Pin, int in2Pin, int dir, int spd) {
  if (dir > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    ledcWrite(pwmPin, spd);
  } else if (dir < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    ledcWrite(pwmPin, spd);
  } else {
    ledcWrite(pwmPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}

/* ============ 差速驱动：正=前进，负=后退，0=停 ============ */
void drive(int leftSpd, int rightSpd) {
  if (leftSpd > 0)      setMotor(PWMA_PIN, AIN1_PIN, AIN2_PIN,  1,  leftSpd);
  else if (leftSpd < 0) setMotor(PWMA_PIN, AIN1_PIN, AIN2_PIN, -1, -leftSpd);
  else                  setMotor(PWMA_PIN, AIN1_PIN, AIN2_PIN,  0, 0);

  if (rightSpd > 0)      setMotor(PWMD_PIN, DIN1_PIN, DIN2_PIN,  1,  rightSpd);
  else if (rightSpd < 0) setMotor(PWMD_PIN, DIN1_PIN, DIN2_PIN, -1, -rightSpd);
  else                   setMotor(PWMD_PIN, DIN1_PIN, DIN2_PIN,  0, 0);
}

/* ================= HC-SR04 单次测距 ================= */
bool readUltrasonicDistance(float &distanceCm) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  uint32_t duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) {
    distanceCm = 400.0f;
    return false;
  }

  distanceCm = duration * SOUND_SPEED_CM_PER_US / 2.0f;
  if (distanceCm > 400.0f) distanceCm = 400.0f;
  return true;
}

bool readFilteredUltrasonicDistance(float &distanceCm, uint8_t samples = 3) {
  float sum = 0.0f;
  uint8_t validCount = 0;

  for (uint8_t i = 0; i < samples; i++) {
    float sampleDistance = 400.0f;
    if (readUltrasonicDistance(sampleDistance)) {
      sum += sampleDistance;
      validCount++;
    }
    delay(10);
  }

  if (validCount == 0) {
    distanceCm = 400.0f;
    return false;
  }

  distanceCm = sum / validCount;
  return true;
}

void ultrasonicTask(void *parameter) {
  (void)parameter;

  while (true) {
    float filteredDistance = 400.0f;
    bool filteredValid = readFilteredUltrasonicDistance(filteredDistance, 3);

    portENTER_CRITICAL(&ultrasonicMux);
    ultrasonicTaskDistanceCm = filteredDistance;
    ultrasonicTaskDistanceValid = filteredValid;
    ultrasonicSampleSequence++;
    portEXIT_CRITICAL(&ultrasonicMux);

    vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_INTERVAL_MS));
  }
}

bool updateUltrasonic() {
  static uint32_t consumedSequence = 0;
  uint32_t currentSequence;
  float currentDistance;
  bool currentValid;

  portENTER_CRITICAL(&ultrasonicMux);
  currentSequence = ultrasonicSampleSequence;
  currentDistance = ultrasonicTaskDistanceCm;
  currentValid = ultrasonicTaskDistanceValid;
  portEXIT_CRITICAL(&ultrasonicMux);

  if (currentSequence == consumedSequence) return false;

  consumedSequence = currentSequence;
  lastDistanceCm = currentDistance;
  lastDistanceValid = currentValid;
  return true;
}

void stopAllMotors() {
  drive(0, 0);
  setMotor(PWMB_PIN, BIN1_PIN, BIN2_PIN, 0, 0);
  curLeft = 0;
  curRight = 0;
  curBack = 0;
}

void clearLineControlState() {
  turnState = TURN_NONE;
  pendingTurn = TURN_NONE;
  newPathConfirmCnt = 0;
  leftTurnEntryCnt = 0;
  rightTurnEntryCnt = 0;
  originalDeviationCleared = false;
  pendingLostCnt = 0;
  pendingCenterCnt = 0;
  pendingSawFullBlack = false;
  wasLost = false;
  reacquireCnt = 0;
}

void startObstacleAvoidance() {
  clearLineControlState();
  stopAllMotors();
  obstacleState = OBS_BRAKE;
  obstacleStateStartMs = millis();
  obstacleClearCount = 0;
  obstacleClearExtraActive = false;
  obstacleClearExtraStartMs = 0;
  actionName = "确认障碍 -> 停顿0.8s再左移";
  Serial.print("检测到障碍，距离=");
  Serial.print(lastDistanceCm, 1);
  Serial.println("cm，准备向左横移");
}

/* ======== [v20] 避障状态机：仅插入两处停顿状态，其余逻辑不变 ========
 * 新流程：
 *   巡线 -> 停稳 -> 左移 --(清空确认)--> 停0.8s --> 直行 --(计数达标)-->
 *   停0.8s --> 右移找线 --(见线确认)--> 接线巡线
 * ===================================================================== */
void handleObstacleAvoidance(bool newDistanceSample) {
  uint32_t now = millis();

  switch (obstacleState) {
    case OBS_FOLLOW_LINE:
      break;

    case OBS_BRAKE:
      stopAllMotors();
      neopixelWrite(RGB_LED_PIN, 40, 0, 40);
      actionName = "避障 -> 停顿0.8s再左移";

      if (now - obstacleStateStartMs >= OBSTACLE_BRAKE_MS) {
        startLateralEncoderLog();
        obstacleState = OBS_MOVE_LEFT;
        obstacleStateStartMs = now;
        obstacleClearCount = 0;
        obstacleClearExtraActive = false;
        obstacleClearExtraStartMs = 0;
        Serial.println("开始向左横移");
      }
      break;

    case OBS_MOVE_LEFT: {
      int lateralASpeed;
      int lateralBSpeed;
      int lateralDSpeed;
      calculateLateralEncoderSpeeds(true,
                                    lateralASpeed,
                                    lateralBSpeed,
                                    lateralDSpeed);
      setMotor(PWMA_PIN, AIN1_PIN, AIN2_PIN, -1, lateralASpeed);
      setMotor(PWMB_PIN, BIN1_PIN, BIN2_PIN, -1, lateralBSpeed);
      setMotor(PWMD_PIN, DIN1_PIN, DIN2_PIN,  1, lateralDSpeed);
      curLeft = -lateralASpeed;
      curRight = lateralDSpeed;
      curBack = -lateralBSpeed;
      neopixelWrite(RGB_LED_PIN, 40, 20, 0);
      actionName = "避障 -> 向左横移";

      uint32_t sideElapsed = now - obstacleStateStartMs;

      if (!obstacleClearExtraActive &&
          newDistanceSample &&
          sideElapsed >= OBSTACLE_MIN_SIDE_MS) {
        bool frontClear = !lastDistanceValid ||
                          lastDistanceCm > OBSTACLE_CLEAR_CM;
        if (frontClear) {
          if (obstacleClearCount < OBSTACLE_CLEAR_COUNT) obstacleClearCount++;
        } else {
          obstacleClearCount = 0;
        }
      }

      if (!obstacleClearExtraActive &&
          obstacleClearCount >= OBSTACLE_CLEAR_COUNT) {
        obstacleClearExtraActive = true;
        obstacleClearExtraStartMs = now;
        Serial.println("超声连续确认前方无遮挡");
      }

      if (obstacleClearExtraActive &&
          now - obstacleClearExtraStartMs >= OBSTACLE_CLEAR_EXTRA_LEFT_MS) {
        // [v20修改] 左移完成后不再直接进入直行，先停下来0.8秒
        printLateralEncoderLog();   // 离开左移状态前记录最终计数
        stopAllMotors();            // 三轮全停（原来只停B轮，现在彻底停稳）
        neopixelWrite(RGB_LED_PIN, 40, 0, 40);   // 紫：停顿
        obstaclePauseStartMs = now;
        obstacleState = OBS_PAUSE_TO_FORWARD;
        actionName = "左移完成 -> 停顿0.8s后直行";
        Serial.println("左移完成，停止0.8秒后开始编码器定距前进");
      }
      break;
    }

    // [v20新增] 左移→直行之间的停顿状态
    case OBS_PAUSE_TO_FORWARD:
      stopAllMotors();   // 整个停顿期间保持三轮全停
      neopixelWrite(RGB_LED_PIN, 40, 0, 40);
      actionName = "左移完成 -> 停顿0.8s";

      if (now - obstaclePauseStartMs >= OBSTACLE_STOP_PAUSE_MS) {
        saveForwardEncoderStart();   // 停顿结束后才记录直行起点
        forwardSyncCorrSum = 0;      // [v21] 重置直行同步诊断统计
        forwardSyncCorrSamples = 0;
        lastForwardSyncCorr = 0;
        obstacleState = OBS_FORWARD;
        obstacleStateStartMs = now;
        curLeft = 0;
        curRight = 0;
        Serial.println("停顿结束，B轮保持停止，开始编码器定距前进");
      }
      break;

    case OBS_FORWARD: {
      int32_t currentACount;
      int32_t currentDCount;
      readEncoderCounts(currentACount, currentDCount);

      int32_t aForwardCounts = currentACount - forwardStartACount;
      int32_t dForwardCounts = -(currentDCount - forwardStartDCount);
      if (aForwardCounts < 0) aForwardCounts = 0;
      if (dForwardCounts < 0) dForwardCounts = 0;

      int syncCorrection = (int)((dForwardCounts - aForwardCounts) /
                                 OBSTACLE_FORWARD_SYNC_DIVISOR);
      syncCorrection = constrain(syncCorrection,
                                 -OBSTACLE_FORWARD_SYNC_MAX,
                                  OBSTACLE_FORWARD_SYNC_MAX);

      // [v21] 记录修正量供串口标定（不参与控制）
      lastForwardSyncCorr = syncCorrection;
      forwardSyncCorrSum += syncCorrection;
      forwardSyncCorrSamples++;

      // [v21] A/D独立基础速度；下限压在起步阈值上，防止落后轮被压停
      int aSpeed = constrain(OBSTACLE_FORWARD_A_SPEED + syncCorrection,
                             OBSTACLE_FORWARD_MIN_A, 255);
      int dSpeed = constrain(OBSTACLE_FORWARD_D_SPEED - syncCorrection,
                             OBSTACLE_FORWARD_MIN_D, 255);

      // [v24] 结束判据改为"两轮都到达才同拍停"：
      // "任一轮到达"会被A轮间歇性虚计数误触发（车挪一小段就提前停）。
      // 两轮都到达后，单轮假计数骗不过判据；两轮仍在同一控制周期内
      // 一起停，v23的防尾部偏航效果保留。
      bool bothReached =
          aForwardCounts >= OBSTACLE_FORWARD_TARGET_COUNTS &&
          dForwardCounts >= OBSTACLE_FORWARD_TARGET_COUNTS;
      // 超时兜底：编码器若彻底失效（计数不动），强制结束防止直行不止
      bool forwardTimedOut =
          now - obstacleStateStartMs >= OBSTACLE_FORWARD_TIMEOUT_MS;

      if (bothReached || forwardTimedOut) {
        printForwardEncoderCalibration(now - obstacleStateStartMs);
        stopAllMotors();
        neopixelWrite(RGB_LED_PIN, 35, 0, 35);   // 紫：停顿
        obstaclePauseStartMs = now;
        obstacleState = OBS_PAUSE_TO_RIGHT;
        actionName = "直行完成 -> 停顿0.8s后右移";
        if (forwardTimedOut) {
          Serial.println("直行超时强制结束（编码器可能失效），0.8秒后向右找线");
        } else {
          Serial.println("两轮均到达目标计数，两轮同拍停车，0.8秒后向右找线");
        }
      } else {
        setMotor(PWMA_PIN, AIN1_PIN, AIN2_PIN, aSpeed > 0 ? 1 : 0, aSpeed);
        setMotor(PWMB_PIN, BIN1_PIN, BIN2_PIN, 0, 0);
        setMotor(PWMD_PIN, DIN1_PIN, DIN2_PIN, dSpeed > 0 ? 1 : 0, dSpeed);
        curLeft = aSpeed;
        curRight = dSpeed;
        neopixelWrite(RGB_LED_PIN, 0, 40, 40);
        actionName = "避障 -> 编码器定距直行";
      }
      break;
    }

    // [v20新增] 直行→右移之间的停顿状态
    case OBS_PAUSE_TO_RIGHT:
      stopAllMotors();   // 整个停顿期间保持三轮全停
      neopixelWrite(RGB_LED_PIN, 35, 0, 35);
      actionName = "直行完成 -> 停顿0.8s";

      if (now - obstaclePauseStartMs >= OBSTACLE_STOP_PAUSE_MS) {
        // 注意顺序：先切状态再记录起点，让OBS_MOVE_RIGHT的首帧读到
        // 全新的lateralStart（停顿期间的微小蠕动不计入右移距离）
        obstacleState = OBS_MOVE_RIGHT;
        obstacleStateStartMs = now;
        startLateralEncoderLog();
        obstacleLineConfirmCount = 0;
        rightMoveSawNoLine = false;
        Serial.println("停顿结束，开始向右横移寻找黑线");
      }
      break;

    case OBS_MOVE_RIGHT: {
      int lateralASpeed;
      int lateralBSpeed;
      int lateralDSpeed;
      calculateLateralEncoderSpeeds(false,
                                    lateralASpeed,
                                    lateralBSpeed,
                                    lateralDSpeed);
      setMotor(PWMA_PIN, AIN1_PIN, AIN2_PIN,  1, lateralASpeed);
      setMotor(PWMB_PIN, BIN1_PIN, BIN2_PIN,  1, lateralBSpeed);
      setMotor(PWMD_PIN, DIN1_PIN, DIN2_PIN, -1, lateralDSpeed);
      curLeft = lateralASpeed;
      curRight = -lateralDSpeed;
      curBack = lateralBSpeed;
      neopixelWrite(RGB_LED_PIN, 35, 0, 35);
      actionName = "避障 -> 向右找线";

      uint8_t blackCount = lineBlack[0] + lineBlack[1] +
                           lineBlack[2] + lineBlack[3];
      uint32_t rightElapsed = now - obstacleStateStartMs;

      if (blackCount == 0) {
        rightMoveSawNoLine = true;
        obstacleLineConfirmCount = 0;
      } else if (rightMoveSawNoLine &&
                 rightElapsed >= OBSTACLE_MIN_RIGHT_MS) {
        if (obstacleLineConfirmCount < OBSTACLE_LINE_CONFIRM_COUNT) {
          obstacleLineConfirmCount++;
        }
      } else {
        obstacleLineConfirmCount = 0;
      }

      if (obstacleLineConfirmCount >= OBSTACLE_LINE_CONFIRM_COUNT) {
        printLateralEncoderLog();   // 离开右移状态前记录最终计数
        stopAllMotors();
        clearLineControlState();
        obstacleState = OBS_FOLLOW_LINE;
        reacquireCnt = REACQUIRE_CYCLES;
        obstacleCooldownUntilMs = now + OBSTACLE_COOLDOWN_MS;
        obstacleRouteCompleted = true;
        actionName = "重新找到黑线 -> 进入接线巡线";
        Serial.println("连续3帧找到黑线，停止横移并恢复巡线");
      }
      break;
    }
  }
}

/* ============ 轮速斜坡（原逻辑不变） ============ */
int rampOneWheel(int currentSpeed, int targetSpeed) {
  const bool changingDirection =
      (currentSpeed > 0 && targetSpeed < 0) ||
      (currentSpeed < 0 && targetSpeed > 0);

  if (changingDirection) {
    return 0;
  }

  if (currentSpeed == 0 && targetSpeed > 0) {
    return (targetSpeed < MOTOR_START_SPEED)
               ? targetSpeed
               : MOTOR_START_SPEED;
  }

  const int step = (targetSpeed < 0) ? REVERSE_RAMP_STEP : RAMP_STEP;
  int difference = targetSpeed - currentSpeed;

  if (difference > step) difference = step;
  if (difference < -step) difference = -step;

  return currentSpeed + difference;
}

void rampTo(int targetLeft, int targetRight) {
  curLeft = rampOneWheel(curLeft, targetLeft);
  curRight = rampOneWheel(curRight, targetRight);
}

/* ============== 统一大角度转弯的方向触发（原逻辑不变） ============== */
bool isStrongLeftDeviation() {
  return lineBlack[0] && lineBlack[1] && !lineBlack[3];
}

bool isStrongRightDeviation() {
  return !lineBlack[0] && lineBlack[2] && lineBlack[3];
}

bool isSpecialLeftPattern() {
  return lineBlack[0] && !lineBlack[1] && lineBlack[2] && !lineBlack[3]; // 1010
}

bool isSpecialRightPattern() {
  return !lineBlack[0] && lineBlack[1] && !lineBlack[2] && lineBlack[3]; // 0101
}

void startLockedTurn(LockedTurnState direction) {
  turnState = direction;
  pendingTurn = TURN_NONE;
  turnStartMs = millis();
  newPathConfirmCnt = 0;
  leftTurnEntryCnt = 0;
  rightTurnEntryCnt = 0;
  originalDeviationCleared = false;
  pendingLostCnt = 0;
  pendingCenterCnt = 0;
  pendingSawFullBlack = false;
  wasLost = false;
  reacquireCnt = 0;

  if (direction == TURN_LEFT) {
    lastErr_x10 = -30;
    lastDir = -1;
  } else {
    lastErr_x10 = +30;
    lastDir = +1;
  }
}

void startPendingTurn(LockedTurnState direction) {
  pendingTurn = direction;
  pendingLostCnt = 0;
  pendingCenterCnt = 0;
  pendingSawFullBlack = false;
  leftTurnEntryCnt = 0;
  rightTurnEntryCnt = 0;

  if (direction == TURN_LEFT) {
    lastErr_x10 = -30;
    lastDir = -1;
  } else {
    lastErr_x10 = +30;
    lastDir = +1;
  }
}

/* ================= 巡线决策（原逻辑一字未改） ================= */
void followLine() {
  bool farLeft  = lineBlack[0];
  bool midLeft  = lineBlack[1];
  bool midRight = lineBlack[2];
  bool farRight = lineBlack[3];
  uint8_t blackCount = farLeft + midLeft + midRight + farRight;
  bool strongLeftDeviation = isStrongLeftDeviation();
  bool strongRightDeviation = isStrongRightDeviation();
  bool specialLeftPattern = isSpecialLeftPattern();
  bool specialRightPattern = isSpecialRightPattern();
  bool leftTurnEvidence = strongLeftDeviation || specialLeftPattern;
  bool rightTurnEvidence = strongRightDeviation || specialRightPattern;

  int targetLeft, targetRight;

  if (stoppedAtFinalCross ||
      (obstacleRouteCompleted && blackCount == 4)) {
    if (!stoppedAtFinalCross) {
      Serial.println("避障后检测到十字路口1111，任务完成并停车");
    }
    stoppedAtFinalCross = true;
    clearLineControlState();
    stopAllMotors();
    err_x10 = 0;
    actionName = "避障后遇到1111 -> 终点停车";
    neopixelWrite(RGB_LED_PIN, 40, 0, 0);
    return;
  }

  if (turnState == TURN_NONE &&
      pendingTurn == TURN_NONE &&
      reacquireCnt == 0) {
    if (strongLeftDeviation) {
      leftTurnEntryCnt = SPECIAL_PATTERN_CONFIRM;
      rightTurnEntryCnt = 0;
    } else if (strongRightDeviation) {
      rightTurnEntryCnt = SPECIAL_PATTERN_CONFIRM;
      leftTurnEntryCnt = 0;
    } else if (specialLeftPattern) {
      if (leftTurnEntryCnt < SPECIAL_PATTERN_CONFIRM) leftTurnEntryCnt++;
      rightTurnEntryCnt = 0;
    } else if (specialRightPattern) {
      if (rightTurnEntryCnt < SPECIAL_PATTERN_CONFIRM) rightTurnEntryCnt++;
      leftTurnEntryCnt = 0;
    } else {
      leftTurnEntryCnt = 0;
      rightTurnEntryCnt = 0;
    }
  }

  if (turnState == TURN_LEFT || turnState == TURN_RIGHT) {
    uint32_t turnElapsed = millis() - turnStartMs;
    bool originalDeviationStillPresent =
        (turnState == TURN_LEFT) ? leftTurnEvidence : rightTurnEvidence;

    if (!originalDeviationStillPresent) originalDeviationCleared = true;

    if (turnState == TURN_LEFT) {
      err_x10 = -30;
      targetLeft = LOCKED_TURN_SPEED;
      targetRight = -LOCKED_REVERSE_SPEED;
      actionName = "大转弯方向锁定 -> 持续左找";
    } else {
      err_x10 = +30;
      targetLeft = -LOCKED_REVERSE_SPEED;
      targetRight = LOCKED_TURN_SPEED;
      actionName = "大转弯方向锁定 -> 持续右找";
    }
    neopixelWrite(RGB_LED_PIN, 35, 0, 35);

    bool stableCenteredPath = !farLeft &&
                              (midLeft || midRight) &&
                              !farRight;
    if (turnElapsed >= TURN_MIN_MS &&
        originalDeviationCleared &&
        !originalDeviationStillPresent &&
        stableCenteredPath) {
      newPathConfirmCnt++;
      if (newPathConfirmCnt >= NEW_PATH_CONFIRM) {
        turnState = TURN_NONE;
        newPathConfirmCnt = 0;
        originalDeviationCleared = false;
        leftTurnEntryCnt = 0;
        rightTurnEntryCnt = 0;
        reacquireCnt = REACQUIRE_CYCLES;
        wasLost = false;

        int captureSum = 0;
        if (farLeft)  captureSum += WEIGHT_x10[0];
        if (midLeft)  captureSum += WEIGHT_x10[1];
        if (midRight) captureSum += WEIGHT_x10[2];
        if (farRight) captureSum += WEIGHT_x10[3];
        err_x10 = captureSum / blackCount;

        lastErr_x10 = err_x10;
        if (err_x10 <= -5)      lastDir = -1;
        else if (err_x10 >= +5) lastDir = +1;

        int captureAbsErr = (err_x10 < 0) ? -err_x10 : err_x10;
        int captureDelta = (REACQUIRE_GAIN * captureAbsErr) / 10;
        if (err_x10 < 0) {
          targetLeft  = REACQUIRE_SPEED + captureDelta;
          targetRight = REACQUIRE_SPEED - captureDelta;
        } else if (err_x10 > 0) {
          targetLeft  = REACQUIRE_SPEED - captureDelta;
          targetRight = REACQUIRE_SPEED + captureDelta;
        } else {
          targetLeft = targetRight = REACQUIRE_SPEED;
        }
        targetLeft  = constrain(targetLeft,  0, TURN_MAX);
        targetRight = constrain(targetRight, 0, TURN_MAX);

        actionName = "弯道新路线已确认 -> 当帧P修正接线";
        neopixelWrite(RGB_LED_PIN, 0, 20, 40);
      }
    } else {
      newPathConfirmCnt = 0;
    }

    if (turnState != TURN_NONE && turnElapsed >= TURN_MAX_LOCK_MS) {
      turnState = TURN_NONE;
      newPathConfirmCnt = 0;
      originalDeviationCleared = false;
      leftTurnEntryCnt = 0;
      rightTurnEntryCnt = 0;
      wasLost = (blackCount == 0);
      reacquireCnt = (blackCount > 0) ? REACQUIRE_CYCLES : 0;
      actionName = "弯道锁定超时 -> 交回普通跟线";
      neopixelWrite(RGB_LED_PIN, 0, 20, 40);
    }
  }
  else if (pendingTurn == TURN_LEFT || pendingTurn == TURN_RIGHT) {
    err_x10 = (pendingTurn == TURN_LEFT) ? -30 : +30;
    targetLeft = targetRight = PENDING_FORWARD_SPEED;
    actionName = (pendingTurn == TURN_LEFT)
                     ? "已记左转 -> 低速直行等待0000"
                     : "已记右转 -> 低速直行等待0000";
    neopixelWrite(RGB_LED_PIN, 35, 20, 0);

    if (blackCount == 4) pendingSawFullBlack = true;

    if (blackCount == 0) {
      pendingCenterCnt = 0;
      if (pendingLostCnt < LOST_TO_TURN_CONFIRM) pendingLostCnt++;

      if (pendingLostCnt >= LOST_TO_TURN_CONFIRM) {
        LockedTurnState confirmedDirection = pendingTurn;
        startLockedTurn(confirmedDirection);

        if (confirmedDirection == TURN_LEFT) {
          err_x10 = -30;
          targetLeft = LOCKED_TURN_SPEED;
          targetRight = -LOCKED_REVERSE_SPEED;
          actionName = "0000连续确认 -> 按记忆锁定左转";
        } else {
          err_x10 = +30;
          targetLeft = -LOCKED_REVERSE_SPEED;
          targetRight = LOCKED_TURN_SPEED;
          actionName = "0000连续确认 -> 按记忆锁定右转";
        }
        neopixelWrite(RGB_LED_PIN, 35, 0, 35);
      }
    } else {
      pendingLostCnt = 0;

      bool stableCenterBeforeLoss = !farLeft &&
                                    (midLeft || midRight) &&
                                    !farRight;
      if (stableCenterBeforeLoss) {
        if (pendingCenterCnt < PENDING_CANCEL_CONFIRM) pendingCenterCnt++;
      } else {
        pendingCenterCnt = 0;
      }

      if (!pendingSawFullBlack &&
          pendingCenterCnt >= PENDING_CANCEL_CONFIRM) {
        pendingTurn = TURN_NONE;
        pendingCenterCnt = 0;
        pendingSawFullBlack = false;
        leftTurnEntryCnt = 0;
        rightTurnEntryCnt = 0;
        err_x10 = 0;
        targetLeft = targetRight = BASE_SPEED;
        actionName = "待转期间稳定回中 -> 取消转弯记忆";
        neopixelWrite(RGB_LED_PIN, 0, 40, 0);
      }
    }
  }
  else if (reacquireCnt == 0 &&
           leftTurnEvidence &&
           leftTurnEntryCnt >= SPECIAL_PATTERN_CONFIRM) {
    startPendingTurn(TURN_LEFT);
    err_x10 = -30;
    targetLeft = targetRight = PENDING_FORWARD_SPEED;
    actionName = specialLeftPattern
                     ? "1010确认 -> 记左转并等待0000"
                     : "左侧图案 -> 记左转并等待0000";
    neopixelWrite(RGB_LED_PIN, 35, 20, 0);
  }
  else if (reacquireCnt == 0 &&
           rightTurnEvidence &&
           rightTurnEntryCnt >= SPECIAL_PATTERN_CONFIRM) {
    startPendingTurn(TURN_RIGHT);
    err_x10 = +30;
    targetLeft = targetRight = PENDING_FORWARD_SPEED;
    actionName = specialRightPattern
                     ? "0101确认 -> 记右转并等待0000"
                     : "右侧图案 -> 记右转并等待0000";
    neopixelWrite(RGB_LED_PIN, 35, 20, 0);
  }
  else if (blackCount == 0) {
    wasLost = true;
    reacquireCnt = 0;
    err_x10 = 0;

    int searchDir;
    if (lastErr_x10 < 0)      searchDir = -1;
    else if (lastErr_x10 > 0) searchDir = +1;
    else if (lastDir != 0)    searchDir = lastDir;
    else                      searchDir = -1;

    if (searchDir < 0) {
      targetLeft = SEARCH_SPEED; targetRight = -SEARCH_REVERSE_SPEED;
      actionName = "丢线 -> 按最后方向持续左找";
    } else if (searchDir > 0) {
      targetLeft = -SEARCH_REVERSE_SPEED; targetRight = SEARCH_SPEED;
      actionName = "丢线 -> 按最后方向持续右找";
    }
    neopixelWrite(RGB_LED_PIN, 40, 15, 0);
  }
  else if (blackCount == 4) {
    err_x10 = 0;
    wasLost = false;
    targetLeft = targetRight = BASE_SPEED;
    actionName = "全黑 -> 直行过十字";
  }
  else if ((wasLost && lastErr_x10 != 0) || reacquireCnt > 0) {
    if (wasLost) {
      reacquireCnt = REACQUIRE_CYCLES;
      wasLost = false;
    }
    if (reacquireCnt > 0) reacquireCnt--;

    int sum = 0;
    if (farLeft)  sum += WEIGHT_x10[0];
    if (midLeft)  sum += WEIGHT_x10[1];
    if (midRight) sum += WEIGHT_x10[2];
    if (farRight) sum += WEIGHT_x10[3];
    err_x10 = sum / blackCount;

    lastErr_x10 = err_x10;
    if (err_x10 <= -5)      lastDir = -1;
    else if (err_x10 >= +5) lastDir = +1;

    int absErr = (err_x10 < 0) ? -err_x10 : err_x10;
    int delta = (REACQUIRE_GAIN * absErr) / 10;

    if (err_x10 < 0) {
      targetLeft  = REACQUIRE_SPEED + delta;
      targetRight = REACQUIRE_SPEED - delta;
    } else if (err_x10 > 0) {
      targetLeft  = REACQUIRE_SPEED - delta;
      targetRight = REACQUIRE_SPEED + delta;
    } else {
      targetLeft = targetRight = REACQUIRE_SPEED;
    }

    targetLeft  = constrain(targetLeft,  0, TURN_MAX);
    targetRight = constrain(targetRight, 0, TURN_MAX);
    actionName = "接线确认期 -> 低速P修正";
    neopixelWrite(RGB_LED_PIN, 0, 20, 40);
  }
  else {
    wasLost = false;

    int sum = 0;
    if (farLeft)  sum += WEIGHT_x10[0];
    if (midLeft)  sum += WEIGHT_x10[1];
    if (midRight) sum += WEIGHT_x10[2];
    if (farRight) sum += WEIGHT_x10[3];
    err_x10 = sum / blackCount;

    lastErr_x10 = err_x10;
    if (err_x10 <= -5)      lastDir = -1;
    else if (err_x10 >= +5) lastDir = +1;

    int absErr  = (err_x10 < 0) ? -err_x10 : err_x10;

    int slow = (CURVE_SLOWDOWN * absErr) / 10;
    int base = BASE_SPEED - slow;
    if (base < 20) base = 20;

    int gain = (absErr <= 10) ? GAIN_NEAR : GAIN_FAR;
    int delta = (gain * absErr) / 10;

    if (err_x10 < 0) {
      targetLeft  = base + delta;
      targetRight = base - delta;
      actionName  = (absErr <= 10) ? "线小偏左 -> 快拉" : "线大偏左 -> 稳追";
    } else if (err_x10 > 0) {
      targetLeft  = base - delta;
      targetRight = base + delta;
      actionName  = (absErr <= 10) ? "线小偏右 -> 快拉" : "线大偏右 -> 稳追";
    } else {
      targetLeft  = base;
      targetRight = base;
      actionName  = "线在正中 -> 直行";
    }

    if (targetLeft  < 0)        targetLeft  = 0;
    if (targetLeft  > TURN_MAX) targetLeft  = TURN_MAX;
    if (targetRight < 0)        targetRight = 0;
    if (targetRight > TURN_MAX) targetRight = TURN_MAX;
  }

  rampTo(targetLeft, targetRight);
  drive(curLeft, curRight);
}

/* ================================================================
 * [v15] TFT 显示模块 —— 与控制逻辑完全解耦
 * ================================================================ */

void tftPad(char *buf, uint8_t width) {
  uint8_t len = strlen(buf);
  for (; len < width && len < 63; len++) buf[len] = ' ';
  buf[len] = '\0';
}

void tftGetCarState(char *out) {
  if (stoppedAtFinalCross)            { strcpy(out, "FINISH");    return; }
  switch (obstacleState) {
    case OBS_BRAKE:                   { strcpy(out, "OBS-BRAKE"); return; }
    case OBS_MOVE_LEFT:               { strcpy(out, "MOVE-LEFT"); return; }
    case OBS_PAUSE_TO_FORWARD:        { strcpy(out, "PAUSE-FWD"); return; }
    case OBS_FORWARD:                 { strcpy(out, "PASS-OBS");  return; }
    case OBS_PAUSE_TO_RIGHT:          { strcpy(out, "PAUSE-RT");  return; }
    case OBS_MOVE_RIGHT:              { strcpy(out, "FIND-LINE"); return; }
    default: break;
  }
  if (turnState == TURN_LEFT)         { strcpy(out, "TURN-LEFT"); return; }
  if (turnState == TURN_RIGHT)        { strcpy(out, "TURN-RIGHT");return; }
  if (pendingTurn == TURN_LEFT)       { strcpy(out, "PEND-LEFT"); return; }
  if (pendingTurn == TURN_RIGHT)      { strcpy(out, "PEND-RIGHT");return; }
  if (reacquireCnt > 0)               { strcpy(out, "REACQUIRE"); return; }
  if (wasLost)                        { strcpy(out, "LOST");      return; }
  strcpy(out, "LINE");
}

void tftDrawRow(uint8_t row, const char *text, uint16_t color) {
  tft.setTextColor(color, ST77XX_BLACK);
  tft.setCursor(2, TFT_DATA_Y0 + row * TFT_LINE_H);
  tft.print(text);
}

// 画一整帧：距离 / A轮 / D轮 / B轮 / 编码器 / 状态
void updateTFT(float distance, bool distanceValid) {
  char buf[64];
  char stateStr[16];

  tftGetCarState(stateStr);

  // ---- Row 0: 超声距离 ----
  uint16_t distColor = ST77XX_GREEN;
  if (!distanceValid) {
    distColor = ST77XX_YELLOW;
    snprintf(buf, sizeof(buf), "Dist: >MAX cm");
  } else {
    int distInt = (int)distance;
    if (distInt < OBSTACLE_TRIGGER_CM)                distColor = ST77XX_RED;
    else if (distInt < OBSTACLE_TRIGGER_CM + 15)      distColor = ST77XX_YELLOW;
    snprintf(buf, sizeof(buf), "Dist: %4d cm", distInt);
  }
  tftPad(buf, TFT_LINE_PAD);
  tftDrawRow(0, buf, distColor);

  // ---- Row 1: 左前轮 A ----
  const char* aDir = (curLeft > 0) ? "FWD" : (curLeft < 0) ? "REV" : "STOP";
  snprintf(buf, sizeof(buf), "A(L): %s %4d", aDir, curLeft);
  tftPad(buf, TFT_LINE_PAD);
  tftDrawRow(1, buf, (curLeft < 0) ? ST77XX_RED : ST77XX_WHITE);

  // ---- Row 2: 右前轮 D ----
  const char* dDir = (curRight > 0) ? "FWD" : (curRight < 0) ? "REV" : "STOP";
  snprintf(buf, sizeof(buf), "D(R): %s %4d", dDir, curRight);
  tftPad(buf, TFT_LINE_PAD);
  tftDrawRow(2, buf, (curRight < 0) ? ST77XX_RED : ST77XX_WHITE);

  // ---- Row 3: 后轮 B（只有横移时转动；正=右移 负=左移） ----
  const char* bDir = (curBack > 0) ? "RGT" : (curBack < 0) ? "LFT" : "STOP";
  snprintf(buf, sizeof(buf), "B(Rr): %s %4d", bDir, curBack);
  tftPad(buf, TFT_LINE_PAD);
  tftDrawRow(3, buf, (curBack != 0) ? ST77XX_YELLOW : ST77XX_ORANGE);

  // ---- Row 4: A/D 编码器实时计数 ----
  int32_t encA, encD;
  readEncoderCounts(encA, encD);
  snprintf(buf, sizeof(buf), "Enc A:%6ld D:%6ld",
           (long)encA, (long)encD);
  tftPad(buf, TFT_LINE_PAD);
  tftDrawRow(4, buf, ST77XX_CYAN);

  // ---- Row 5: 车辆状态机译文 ----
  snprintf(buf, sizeof(buf), "St: %s", stateStr);
  tftPad(buf, TFT_LINE_PAD);
  uint16_t stColor = ST77XX_GREEN;
  if (stoppedAtFinalCross)                           stColor = ST77XX_RED;
  else if (obstacleState != OBS_FOLLOW_LINE ||
           turnState != TURN_NONE ||
           pendingTurn != TURN_NONE)                 stColor = ST77XX_MAGENTA;
  else if (wasLost || reacquireCnt > 0)              stColor = ST77XX_YELLOW;
  tftDrawRow(5, buf, stColor);
}

// 屏幕刷新任务：核心0、最低优先级，绝不干扰控制
void tftTask(void *parameter) {
  (void)parameter;

  while (true) {
    updateTFT(lastDistanceCm, lastDistanceValid);
    vTaskDelay(pdMS_TO_TICKS(TFT_REFRESH_MS));
  }
}

// 静态界面：标题栏 + 分隔线（只在 setup 画一次）
void tftInitStatic() {
  // [v25] 必须先按自定义引脚初始化SPI总线：之后库内部再调begin()时，
  // ESP32的SPIClass::begin()检测到总线已初始化会直接返回，引脚不会被覆盖
  SPI.begin(TFT_SCK, -1, TFT_MOSI);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  tft.fillRect(0, 0, 160, 18, ST77XX_BLUE);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLUE);
  tft.setCursor(4, 6);
  tft.print("LINE CAR v20+TFT");
  tft.drawLine(0, 19, 160, 19, ST77XX_ORANGE);
}

/* ================= 串口调试打印（200ms 一次） ================= */
void printStatus() {
  Serial.print("传感器[最左 中左 中右 最右] = ");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(lineBlack[i] ? 1 : 0);
    Serial.print(' ');
  }
  Serial.print("| 误差=");
  Serial.print((float)err_x10 / 10.0, 1);
  Serial.print(" | 轮速 L=");
  Serial.print(curLeft);
  Serial.print(" R=");
  Serial.print(curRight);
  Serial.print(" | 超声=");
  if (lastDistanceValid) {
    Serial.print(lastDistanceCm, 1);
    Serial.print("cm");
  } else {
    Serial.print("无回波");
  }
  if (obstacleState != OBS_FOLLOW_LINE) {
    Serial.print(" | 避障状态=");
    switch (obstacleState) {
      case OBS_BRAKE: {
        Serial.print("停顿(待左移)");
        uint32_t brakeElapsed = millis() - obstacleStateStartMs;
        uint32_t brakeRemaining =
            brakeElapsed < OBSTACLE_BRAKE_MS
                ? OBSTACLE_BRAKE_MS - brakeElapsed : 0;
        Serial.print(" | 剩余=");
        Serial.print(brakeRemaining);
        Serial.print("ms");
        break;
      }
      case OBS_MOVE_LEFT:
        Serial.print("左横移");
        Serial.print(" | 清空确认=");
        Serial.print(obstacleClearCount);
        Serial.print('/');
        Serial.print(OBSTACLE_CLEAR_COUNT);
        if (obstacleClearExtraActive) {
          uint32_t extraElapsed = millis() - obstacleClearExtraStartMs;
          uint32_t extraRemaining =
              extraElapsed < OBSTACLE_CLEAR_EXTRA_LEFT_MS
                  ? OBSTACLE_CLEAR_EXTRA_LEFT_MS - extraElapsed
                  : 0;
          Serial.print(" | 清空后续移剩余=");
          Serial.print(extraRemaining);
          Serial.print("ms");
        }
        break;
      case OBS_PAUSE_TO_FORWARD: {
        Serial.print("停顿(待直行)");
        uint32_t pauseElapsed = millis() - obstaclePauseStartMs;
        uint32_t pauseRemaining =
            pauseElapsed < OBSTACLE_STOP_PAUSE_MS
                ? OBSTACLE_STOP_PAUSE_MS - pauseElapsed : 0;
        Serial.print(" | 剩余=");
        Serial.print(pauseRemaining);
        Serial.print("ms");
        break;
      }
      case OBS_FORWARD:      Serial.print("编码器定距越障前进"); break;
      case OBS_PAUSE_TO_RIGHT: {
        Serial.print("停顿(待右移)");
        uint32_t pauseElapsed2 = millis() - obstaclePauseStartMs;
        uint32_t pauseRemaining2 =
            pauseElapsed2 < OBSTACLE_STOP_PAUSE_MS
                ? OBSTACLE_STOP_PAUSE_MS - pauseElapsed2 : 0;
        Serial.print(" | 剩余=");
        Serial.print(pauseRemaining2);
        Serial.print("ms");
        break;
      }
      case OBS_MOVE_RIGHT:
        Serial.print("右横移找线");
        Serial.print(" | 黑线确认=");
        Serial.print(obstacleLineConfirmCount);
        Serial.print('/');
        Serial.print(OBSTACLE_LINE_CONFIRM_COUNT);
        break;
      default: break;
    }
  }
  if (reacquireCnt > 0) {
    Serial.print(" | 接线期剩余=");
    Serial.print(reacquireCnt);
  }
  if (turnState != TURN_NONE) {
    Serial.print(" | 转弯锁定=");
    Serial.print((turnState == TURN_LEFT) ? "左" : "右");
  }
  if (pendingTurn != TURN_NONE) {
    Serial.print(" | 待转记忆=");
    Serial.print((pendingTurn == TURN_LEFT) ? "左" : "右");
  }
  Serial.print(" | ");
  Serial.println(actionName);

  if (obstacleState == OBS_MOVE_LEFT || obstacleState == OBS_MOVE_RIGHT) {
    printLateralEncoderLog();
  }
  if (obstacleState == OBS_FORWARD) {
    printForwardSyncLog();   // [v21] 直行期间输出同步诊断
  }
}

/* ================= setup：初始化 + 3 秒准备期 ================= */
void setup() {
  Serial.begin(115200);

  // HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // 霍尔编码器 A/B/D 三路
  pinMode(ENCODER_A_A_PIN, INPUT);
  pinMode(ENCODER_A_B_PIN, INPUT);
  pinMode(ENCODER_B_A_PIN, INPUT);
  pinMode(ENCODER_B_B_PIN, INPUT);
  pinMode(ENCODER_D_A_PIN, INPUT);
  pinMode(ENCODER_D_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_A_PIN), onEncoderAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_A_PIN), onEncoderBChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_D_A_PIN), onEncoderDChange, CHANGE);
  digitalWrite(TRIG_PIN, LOW);

  // 传感器引脚
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  // 电机引脚
  pinMode(STBY_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);  pinMode(BIN2_PIN, OUTPUT);
  pinMode(DIN1_PIN, OUTPUT);  pinMode(DIN2_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);

  ledcAttach(PWMA_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWMB_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWMD_PIN, PWM_FREQUENCY, PWM_RESOLUTION);

  drive(0, 0);
  setMotor(PWMB_PIN, BIN1_PIN, BIN2_PIN, 0, 0);

  // TFT：先初始化屏幕静态部分，任务随后独立刷新
  tftInitStatic();

  BaseType_t ultrasonicTaskResult = xTaskCreatePinnedToCore(
      ultrasonicTask,
      "hc_sr04_filter",
      4096,
      nullptr,
      1,
      nullptr,
      0);

  BaseType_t tftTaskResult = xTaskCreatePinnedToCore(
      tftTask,
      "st7735_tft",
      8192,
      nullptr,
      0,
      nullptr,
      0);

  Serial.println();
  Serial.println("=== 四路红外巡线 + 超声横移避障 v20（动作间加0.8秒停顿） ===");
  Serial.print("参数: BASE=");
  Serial.print(BASE_SPEED);
  Serial.print(" 近增益=");
  Serial.print(GAIN_NEAR);
  Serial.print(" 远增益=");
  Serial.print(GAIN_FAR);
  Serial.print(" 弯道降速=");
  Serial.println(CURVE_SLOWDOWN);
  Serial.println("3 秒准备期：把车摆上黑线（探头对准线），手离开...");
  Serial.println("超声端口: TRIG=GPIO19 ECHO=GPIO20");
  Serial.println(ultrasonicTaskResult == pdPASS
                     ? "超声后台任务: 启动成功"
                     : "超声后台任务: 启动失败，请勿运行小车");
  Serial.println(tftTaskResult == pdPASS
                     ? "TFT显示任务: 启动成功"
                     : "TFT显示任务: 启动失败（不影响小车运行，仅无显示）");
  Serial.println("[v20] 左移PWM: A=34 B=46 D=34（最低A/D PWM=33），比例220:410:220");
  Serial.println("[v20] 右移PWM: A=40 B=53 D=40");
  Serial.println("[v20] 动作间停顿: 左移->直行、直行->右移各停止800ms");
  Serial.println("[v22] 检测到障碍: 先停顿800ms，再开始左移");
  Serial.println("[v24] 直行结束: 两轮均到达才同拍停（防单轮假计数误触发）");
  Serial.println("[v20] 清空后续移: 0ms（按用户实测调整）");
  Serial.println("[v24] 越障直行: A/D各400计数结束，3秒超时兜底");
  Serial.println("[v21] 直行同步: A/D基础速度48/48(可标定) 增益1/8 上限±14 下限38");
  Serial.println("终点规则: 仅避障回线后遇到1111停车，避障前1111照常直行");

  // 3 秒准备期：红灯闪（RESET 后也会重新进入这里）
  uint32_t t0 = millis();
  while (millis() - t0 < READY_MS) {
    neopixelWrite(RGB_LED_PIN, 40, 0, 0);
    delay(200);
    neopixelWrite(RGB_LED_PIN, 0, 0, 0);
    delay(200);
  }

  digitalWrite(STBY_PIN, HIGH);
  neopixelWrite(RGB_LED_PIN, 0, 40, 0);
  Serial.println(">>> 开始巡线并监测前方障碍！<<<");
}

/* ================= loop：约 10ms 一个控制周期 ================= */
void loop() {
  bool newDistanceSample = updateUltrasonic();

  if (obstacleState == OBS_FOLLOW_LINE) {
    neopixelWrite(RGB_LED_PIN, 0, 40, 0);
    readSensors();

    if (millis() >= obstacleCooldownUntilMs &&
        newDistanceSample &&
        lastDistanceValid &&
        lastDistanceCm < OBSTACLE_TRIGGER_CM) {
      startObstacleAvoidance();
    } else {
      followLine();
      setMotor(PWMB_PIN, BIN1_PIN, BIN2_PIN, 0, 0);
      curBack = 0;
    }
  } else {
    if (obstacleState == OBS_MOVE_RIGHT) {
      readSensors();
    }
    handleObstacleAvoidance(newDistanceSample);
  }

  if (millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();
    printStatus();
  }
}
