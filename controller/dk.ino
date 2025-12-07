#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

Servo servoGan;    // gắp
Servo servoNang;   // nâng
Servo servoBan;    // bắn

const int SERVO_GAN_PIN  = 17;
const int SERVO_NANG_PIN = 4;
const int SERVO_BAN_PIN  = 16;

int angleGan  = 20;
int angleNang = 0;
int angleBan  = 95;

// Servo bắn dùng 2 góc cố định
const int BAN_FIRE_ANGLE = 180;
const int BAN_BACK_ANGLE = 95;

// --- AUTO-RETURN CONFIG FOR BAN ---
bool banAutoPending = false;            // đang chờ auto-return không
unsigned long banFireMillis = 0;        // thời điểm vừa bắn
const unsigned long BAN_AUTO_DELAY = 500; // ms chờ trước khi trả về (đổi nếu muốn)
// -------------------------------------

// Cờ cho servo gắp & nâng (giữ để xoay)
bool ganInc  = false;
bool ganDec  = false;
bool nangInc = false;
bool nangDec = false;

// Thời gian cập nhật tách riêng cho từng servo
unsigned long lastServoTimeGan = 0;
unsigned long lastServoTimeNang = 0;

// --- CHỈNH TỐC ĐỘ SERVO RIÊNG RẺ (bạn có thể thay đổi) ---
// servoGan: KHÔI PHỤC NHƯ CŨ (mượt, chậm)
const int servoStepGan = 6;                 // bước góc mỗi lần cập nhật cho gắp (nhỏ)
const unsigned long servoIntervalGan = 20;  // ms giữa các lần cập nhật cho gắp (20ms)

// servoNang: tốc độ đã yêu cầu (nhanh hơn)
const int servoStepNang = 6;                // bước góc mỗi lần cập nhật cho nâng (lớn)
const unsigned long servoIntervalNang = 10; // ms giữa các lần cập nhật cho nâng (10ms)


// WebServer
WebServer server(80);
const char* ap_ssid     = "ESP32_ROBOT";
const char* ap_password = "11111100";

// 2 chế độ: dò line / điều khiển tay
enum RobotMode {
  MODE_LINE,
  MODE_MANUAL
};
RobotMode currentMode = MODE_MANUAL;

// ================== CODE CŨ CỦA ANH (GIỮ NGUYÊN) ==================
const uint8_t SensorCount = 8; 
const uint8_t qtrPins[SensorCount] = {14, 32,33,34,35 ,22 ,23,27};
const int he_so[SensorCount] = {7, 5, 3, 1, -1, -3, -5, -7};

#define IN3 19
#define IN4 21  // Phải
#define IN1 5
#define IN2 18  // TRrái

#define ENA 26  
#define ENB 25  



const int pwmFreq     = 25000;  
const int pwmRes      = 8; 

int base_speed_line = 220;
int base_speed = 230;

int max_speed = 255;
int last_error = 0;
int error_s = 0;
float kp = 83;
float ki = 0.3;
float kd = 22;
int stop = 0;

int read()
{
  int count = 0;
  int sum = 0;
  for(int i = 0; i < SensorCount; i++)
  {
    int sen_val = digitalRead(qtrPins[i]);
    sum += sen_val*he_so[i];
    count+= sen_val;
    Serial.print(sen_val);
    Serial.print(" ");
  }
  Serial.println();
  if(count  == 6)
  {
    set_speed(0, IN1, IN2, ENA);
    set_speed(0, IN3, IN4, ENB);
    stop = 1;
    currentMode = MODE_MANUAL;

  } 
  else if(count == 0)
  {
    return last_error;
  }
  return sum;
}

void forward(int ina, int inb)
{
  digitalWrite(ina, HIGH);
  digitalWrite(inb, LOW);
}
void backward(int ina, int inb)
{
  digitalWrite(inb, HIGH);
  digitalWrite(ina, LOW);
}
void braking(int ina, int inb)
{
  digitalWrite(inb, LOW);
  digitalWrite(ina, LOW);
}
void set_speed(int speed, int ina, int inb, int en)
{
  if(speed < 0)
  {
    backward(ina, inb);
    speed = -speed;
  } else {
    forward(ina, inb);
  }
  speed %= 256;

  ledcWrite(en, speed);
}

void pid()
{
  int position = read();
  if(stop == 1)
  {
    return;
  }

  int error = 0 - position;

  error_s += error;
  if(error_s >= 200) error_s = 200;
  else if(error_s <= -200) error_s = -200; 
  int d_error = error - last_error;
  last_error = error;
  int u = (int) (kp*error + ki*error_s + kd*d_error);


  int left_speed = base_speed_line - u;
  int right_speed = base_speed_line + u;

  if(left_speed >= max_speed) left_speed = max_speed;
  else if(left_speed <= -max_speed) left_speed = -max_speed;
  if(right_speed >= max_speed) right_speed = max_speed;
  if(right_speed <= -max_speed) right_speed = -max_speed;


  set_speed(left_speed, IN1, IN2, ENB);
  set_speed(right_speed, IN3, IN4, ENA);


  Serial.print("LSP: ");
  Serial.print(left_speed);
  Serial.print(", RSP: ");
  Serial.print(right_speed);
  
  Serial.print(", PS: ");
  Serial.println(position);

  delay(10);
  
}

// ================== HTML GIAO DIỆN ==================
String htmlPage()
{
  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='utf-8'>
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>B-T-V-D ROBOT</title>

<style>
  /* ==== CẤU HÌNH CHUNG ==== */
  body {
    margin: 0;
    padding: 0;
    background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
    color: #fff;
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    user-select: none;
    min-height: 100vh; 
    overflow: auto;    
    display: flex;
    flex-direction: column;
  }

  h2 {
    text-align: center;
    margin: 15px 0;
    font-size: 24px; /* Tăng cỡ chữ lên một chút cho ngầu */
    font-weight: 900;
    text-transform: uppercase;
    letter-spacing: 2px;
    color: #00d2ff;
    text-shadow: 0 0 15px rgba(0, 210, 255, 0.7); /* Tăng độ phát sáng */
    flex-shrink: 0;
  }

  /* ==== LAYOUT CHÍNH ==== */
  .gamepad {
    display: flex;
    flex-direction: row;
    justify-content: space-between;
    width: 100%;
    padding: 10px;
    box-sizing: border-box;
    gap: 10px;
    flex-grow: 1;
  }

  /* Cột trái & phải (Di chuyển) */
  .side-left, .side-right {
    flex: 1;
    display: flex;
    flex-direction: column;
    justify-content: center;
    align-items: center;
    gap: 20px;
  }
  
  /* Cột giữa (Chức năng) */
  .center {
    flex: 1.2;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: flex-start;
    gap: 12px;
  }

  /* ==== BUTTON STYLES ==== */
  button {
    cursor: pointer;
    outline: none;
    -webkit-tap-highlight-color: transparent;
    transition: all 0.1s ease;
    box-shadow: 0 4px 6px rgba(0,0,0,0.3);
  }

  button:active {
    transform: scale(0.95);
    box-shadow: 0 2px 3px rgba(0,0,0,0.3);
  }

  /* Nút Di Chuyển (Move) */
  .move-btn {
    width: 100%;
    max-width: 140px;
    height: 80px;
    background: rgba(255, 255, 255, 0.05);
    border: 2px solid #00d2ff;
    border-radius: 15px;
    font-size: 20px;
    font-weight: 800;
    color: #00d2ff;
    backdrop-filter: blur(5px);
  }
  .move-btn:active {
    background: #00d2ff;
    color: #000;
  }

  .horizontal-row {
    display: flex;
    flex-direction: row;
    gap: 15px;
    width: 100%;
    justify-content: center;
  }

  /* ==== SYSTEM BUTTONS (Mode/Stop) ==== */
  .mode-row {
    display: flex;
    gap: 10px;
    width: 100%;
    justify-content: center;
    margin-bottom: 5px;
  }

  .btn-mode {
    flex: 1;
    height: 50px;
    background: linear-gradient(45deg, #4b6cb7, #182848);
    border: none;
    border-radius: 10px;
    color: white;
    font-size: 14px;
    font-weight: bold;
    border: 1px solid rgba(255,255,255,0.2);
  }

  .btn-stop {
    width: 80px;
    height: 50px;
    background: linear-gradient(45deg, #ff416c, #ff4b2b);
    border: none;
    border-radius: 10px;
    color: white;
    font-size: 16px;
    font-weight: 900;
    box-shadow: 0 0 15px rgba(255, 65, 108, 0.6); 
  }

  /* ==== SERVO GROUPS ==== */
  .servo-group {
    background: rgba(0, 0, 0, 0.3);
    padding: 10px;
    border-radius: 12px;
    width: 95%;
    text-align: center;
    border: 1px solid rgba(255,255,255,0.05);
  }

  .servo-title {
    font-size: 13px;
    font-weight: bold;
    margin-bottom: 5px;
    color: #aaa;
    text-transform: uppercase;
  }

  .servo-row {
    display: flex;
    justify-content: center;
    gap: 10px;
  }

  .servoBtn {
    flex: 1;
    height: 45px;
    border: none;
    border-radius: 8px;
    color: #fff;
    font-size: 15px;
    font-weight: bold;
    text-shadow: 0 1px 2px rgba(0,0,0,0.5);
  }

  /* Màu riêng cho từng nhóm Servo */
  #btnGanInc, #btnGanDec { background: linear-gradient(to bottom, #f7971e, #ffd200); color: #444; }
  #btnNangInc, #btnNangDec { background: linear-gradient(to bottom, #11998e, #38ef7d); }
  #btnBanFire, #btnBanBack { background: linear-gradient(to bottom, #cb2d3e, #ef473a); }

  /* ==== RESPONSIVE: CHẾ ĐỘ DỌC CHO ĐIỆN THOẠI ==== */
  @media (max-width: 768px) {
    .gamepad {
      flex-direction: column-reverse;
      align-items: center;
      gap: 30px;
      padding-bottom: 50px;
    }
    
    .center {
      width: 100%;
      order: 1;
    }

    .side-left, .side-right {
      width: 100%;
      flex-direction: row;
      order: 2;
      gap: 10px;
    }
    
    .move-btn {
      height: 70px;
    }
  }

</style>
</head>

<body>

<h2>B-T-V-D ROBOT</h2>

<div class="gamepad">

  <div class="side-left">
    <button class="move-btn" id="btnForward">▲ TIẾN</button>
    <button class="move-btn" id="btnBack">▼ LÙI</button>
  </div>

  <div class="center">
    <div class="mode-row">
      <button class="btn-mode" id="btnMode">MODE: MANUAL</button>
      <button class="btn-stop" id="btnStop">STOP</button>
    </div>

    <div class="servo-group">
      <div class="servo-title">Kẹp Gắp (P17)</div>
      <div class="servo-row">
        <button class="servoBtn" id="btnGanDec">Mở</button>
        <button class="servoBtn" id="btnGanInc">Kẹp</button>
      </div>
    </div>

    <div class="servo-group">
      <div class="servo-title">Cánh Tay (P4)</div>
      <div class="servo-row">
        <button class="servoBtn" id="btnNangDec">▲ Nâng</button>
        <button class="servoBtn" id="btnNangInc">▼ Hạ</button>
      </div>
    </div>

    <div class="servo-group">
      <div class="servo-title">Vũ Khí (P16)</div>
      <div class="servo-row">
        <button class="servoBtn" id="btnBanBack">Nạp</button>
        <button class="servoBtn" id="btnBanFire">🔥 BẮN</button>
      </div>
    </div>
  </div>

  <div class="side-right">
    <div class="horizontal-row">
      <button class="move-btn" id="btnLeft">◀ TRÁI</button>
      <button class="move-btn" id="btnRight">PHẢI ▶</button>
    </div>
  </div>

</div>

<script>
function send(cmd){ fetch("/"+cmd); }

// Giữ để di chuyển
function hold(btn, down, up="stop"){
  if(!btn) return;
  btn.addEventListener("touchstart", e=>{e.preventDefault();send(down);});
  btn.addEventListener("mousedown", ()=>send(down));

  btn.addEventListener("touchend", e=>{e.preventDefault();send(up);});
  btn.addEventListener("mouseup", ()=>send(up));
  btn.addEventListener("mouseleave", ()=>send(up));
}

hold(document.getElementById("btnForward"),"forward");
hold(document.getElementById("btnBack"),"back");
hold(document.getElementById("btnLeft"),"left");
hold(document.getElementById("btnRight"),"right");

// STOP
document.getElementById("btnStop").onclick = ()=>send("stop");

// MODE
const modeBtn = document.getElementById("btnMode");
if(modeBtn){
  modeBtn.onclick = ()=>{
    fetch("/mode_toggle")
      .then(r=>r.text())
      .then(t=>{
        modeBtn.innerText=t.includes("LINE")?"MODE: LINE":"MODE: MANUAL";
      });
  };
}

// Servo giữ
function holdServo(id,start,stop){
  const b=document.getElementById(id);
  if(!b) return;
  b.addEventListener("mousedown", ()=>send(start));
  b.addEventListener("mouseup", ()=>send(stop));
  b.addEventListener("mouseleave", ()=>send(stop));
  b.addEventListener("touchstart", e=>{e.preventDefault();send(start);});
  b.addEventListener("touchend", e=>{e.preventDefault();send(stop);});
}

holdServo("btnGanInc","gan_inc_start","gan_inc_stop");
holdServo("btnGanDec","gan_dec_start","gan_dec_stop");
holdServo("btnNangInc","nang_inc_start","nang_inc_stop");
holdServo("btnNangDec","nang_dec_start","nang_dec_stop");

const btnFire = document.getElementById("btnBanFire");
if(btnFire) btnFire.onclick=()=>send("ban_fire");

const btnLoad = document.getElementById("btnBanBack");
if(btnLoad) btnLoad.onclick=()=>send("ban_back");

</script>
</body>
</html>
)rawliteral";

  return page;
}

// ================== CẬP NHẬT SERVO (giữ để xoay) ==================
void updateServos() {
  unsigned long now = millis();

  // --- GẮP (servoGan) : KHÔI PHỤC HÀNH VI CŨ ---
  if (now - lastServoTimeGan >= servoIntervalGan) {
    lastServoTimeGan = now;

    if (ganInc) {
      angleGan += servoStepGan;
      if (angleGan > 180) angleGan = 180;
      servoGan.write(angleGan);
    }
    if (ganDec) {
      angleGan -= servoStepGan;
      if (angleGan < 0) angleGan = 0;
      servoGan.write(angleGan);
    }
  }

  // --- NÂNG (servoNang) : TĂNG TỐC NHƯ YÊU CẦU ---
  if (now - lastServoTimeNang >= servoIntervalNang) {
    lastServoTimeNang = now;

    if (nangInc) {
      angleNang += servoStepNang;
      if (angleNang > 180) angleNang = 180;
      servoNang.write(angleNang);
    }
    if (nangDec) {
      angleNang -= servoStepNang;
      if (angleNang < 0) angleNang = 0;
      servoNang.write(angleNang);
    }
  }
}

// ================== HANDLER ĐỘNG CƠ QUA WEB ==================
void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleForward() {
  currentMode = MODE_MANUAL;
  set_speed(base_speed, IN1, IN2, ENA);
  set_speed(base_speed, IN3, IN4, ENB);
  server.send(200, "text/plain", "FORWARD");
}

void handleBack() {
  currentMode = MODE_MANUAL;
  set_speed(-base_speed, IN1, IN2, ENA);
  set_speed(-base_speed, IN3, IN4, ENB);
  server.send(200, "text/plain", "BACK");
}

void handleLeft() {
  currentMode = MODE_MANUAL;
  set_speed(-base_speed, IN1, IN2, ENA);
  set_speed(base_speed, IN3, IN4, ENB);
  server.send(200, "text/plain", "LEFT");
}

void handleRight() {
  currentMode = MODE_MANUAL;
  set_speed(base_speed, IN1, IN2, ENA);
  set_speed(-base_speed, IN3, IN4, ENB);
  server.send(200, "text/plain", "RIGHT");
}

void stopAllServo() {
  ganInc  = false;
  ganDec  = false;
  nangInc = false;
  nangDec = false;
}

void handleStop() {
  // Dừng mọi lệnh giữ servo
  stopAllServo();

  // Hủy auto-return của servo bắn nếu đang chờ
  banAutoPending = false;

  // Dừng động cơ (brake + PWM=0)
  braking(IN1, IN2);
  braking(IN3, IN4);
  set_speed(0, IN1, IN2, ENA);
  set_speed(0, IN3, IN4, ENB);

  server.send(200, "text/plain", "STOP");
}

// ================== HANDLER MODE (CHỈ 2 CHẾ ĐỘ) ==================
void handleModeToggle() {
  if (currentMode == MODE_MANUAL) {
    // chuyển sang dò line
    braking(IN1, IN2);
    braking(IN3, IN4);
    set_speed(0, IN1, IN2, ENA);
    set_speed(0, IN3, IN4, ENB);
    currentMode = MODE_LINE;
    stop = 0;
    server.send(200, "text/plain", "LINE");
  } else {
    // chuyển sang điều khiển tay
    braking(IN1, IN2);
    braking(IN3, IN4);
    set_speed(0, IN1, IN2, ENA);
    set_speed(0, IN3, IN4, ENB);
    currentMode = MODE_MANUAL;
    stop = 1;
    server.send(200, "text/plain", "MANUAL");
  }
}

// ================== HANDLER SERVO ==================
// GẮP (giữ)
void handleGanIncStart() { ganInc = true;  server.send(200, "text/plain", "GAN_INC_START"); }
void handleGanIncStop()  { ganInc = false; server.send(200, "text/plain", "GAN_INC_STOP");  }

void handleGanDecStart() { ganDec = true;  server.send(200, "text/plain", "GAN_DEC_START"); }
void handleGanDecStop()  { ganDec = false; server.send(200, "text/plain", "GAN_DEC_STOP");  }

// NÂNG (giữ)
void handleNangIncStart() { nangInc = true;  server.send(200, "text/plain", "NANG_INC_START"); }
void handleNangIncStop()  { nangInc = false; server.send(200, "text/plain", "NANG_INC_STOP");  }

void handleNangDecStart() { nangDec = true;  server.send(200, "text/plain", "NANG_DEC_START"); }
void handleNangDecStop()  { nangDec = false; server.send(200, "text/plain", "NANG_DEC_STOP");  }

// BẮN (2 góc cố định) - bắn và bắt đầu đếm thời gian auto-return
void handleBanFire() {
  angleBan = BAN_FIRE_ANGLE;
  servoBan.write(angleBan);

  // bật auto-return
  banAutoPending = true;
  banFireMillis = millis();

  server.send(200, "text/plain", "BAN_FIRE");
}

void handleBanBack() {
  // hủy auto-return (nếu đang chờ) và trả về ngay
  banAutoPending = false;

  angleBan = BAN_BACK_ANGLE;
  servoBan.write(angleBan);
  server.send(200, "text/plain", "BAN_BACK");
}

// ================== SETUP ==================
void setup() {

  // Serial.begin(115200);


  // Serial.begin(115200);
  servoGan.setPeriodHertz(50);
  servoNang.setPeriodHertz(50);
  servoBan.setPeriodHertz(50);
  
  
  servoGan.write(angleGan);
  servoNang.write(angleNang);
  servoBan.write(angleBan);

  

  servoGan.attach(SERVO_GAN_PIN, 500, 2500);
  servoNang.attach(SERVO_NANG_PIN, 500, 2500);
  servoBan.attach(SERVO_BAN_PIN, 500, 2500);


  // Chân điều khiển L298N
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  for(int i = 0; i < SensorCount; i++)
  {
    pinMode(qtrPins[i], INPUT);
  }

  // PWM cho động cơ (theo kiểu anh đang dùng)
  ledcAttach(ENA, pwmFreq, pwmRes);
  ledcAttach(ENB, pwmFreq, pwmRes);
  braking(IN1, IN2);
  braking(IN3, IN4);
  set_speed(0, IN1, IN2, ENA);
  set_speed(0, IN3, IN4, ENB);

  // Thêm: dừng lại cho an toàn (không tự chạy sau khi nạp)
  braking(IN1, IN2);
  braking(IN3, IN4);

  // ====== SERVO ======
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // ====== WIFI AP + SERVER ======
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/",           handleRoot);
  server.on("/forward",    handleForward);
  server.on("/back",       handleBack);
  server.on("/left",       handleLeft);
  server.on("/right",      handleRight);
  server.on("/stop",       handleStop);
  server.on("/mode_toggle",handleModeToggle);

  // Servo gắp giữ
  server.on("/gan_inc_start", handleGanIncStart);
  server.on("/gan_inc_stop",  handleGanIncStop);
  server.on("/gan_dec_start", handleGanDecStart);
  server.on("/gan_dec_stop",  handleGanDecStop);

  // Servo nâng giữ
  server.on("/nang_inc_start", handleNangIncStart);
  server.on("/nang_inc_stop",  handleNangIncStop);
  server.on("/nang_dec_start", handleNangDecStart);
  server.on("/nang_dec_stop",  handleNangDecStop);

  // Servo bắn
  server.on("/ban_fire", handleBanFire);
  server.on("/ban_back", handleBanBack);

  server.begin();
  Serial.println("WebServer started.");

  currentMode = MODE_MANUAL;

}

void loop() {
  // put your main code here, to run repeatedly:
  server.handleClient();

  // cập nhật servo (gắp, nâng nếu đang giữ nút)
  updateServos();

  // nếu đang chờ auto-return và đủ thời gian -> trả về tự động
  if (banAutoPending) {
    if (millis() - banFireMillis >= BAN_AUTO_DELAY) {
      banAutoPending = false;
      angleBan = BAN_BACK_ANGLE;
      servoBan.write(angleBan);
      Serial.println("Auto ban back executed.");
    }
  }

  // chế độ dò line: chạy pid()
  if (currentMode == MODE_LINE) {
    pid();
  }
}
