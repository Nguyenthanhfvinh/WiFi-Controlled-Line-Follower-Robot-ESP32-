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

int base_speed = 220;
int max_speed = 255;
int last_error = 0;
int error_s = 0;
float kp = 84;
float ki = 0.3;
float kd = 22;
int stop = 0;




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
  ledcWrite(en, speed);
}

void pid()
{
  int position = read();
  // if(stop == 1)
  // {
  //   return;
  // }

  int error = 0 - position;

  error_s += error;
  if(error_s >= 255) error_s = 255;
  else if(error_s <= -255) error_s = -255; 
  int d_error = error - last_error;
  last_error = error;
  int u = (int) (kp*error + ki*error_s + kd*d_error);


  int left_speed = base_speed - u;
  int right_speed = base_speed + u;

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

  } 
  else if(count == 0)
  {
    stop = 0;
    // backward(IN1, IN2);
    // backward(IN3, IN4);
    // set_speed(0, IN1, IN2, ENA);
    // set_speed(0, IN3, IN4, ENB);
    return last_error;
  }
  return sum;
}
void setup()
{
  Serial.begin(115200);
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

}
void loop() {
  pid();
  // // forward(IN1, IN2);
  // // forward(IN3, IN4);
  // set_speed(170, IN1, IN2, ENA);
  // set_speed(-170, IN3, IN4, ENB);

}
