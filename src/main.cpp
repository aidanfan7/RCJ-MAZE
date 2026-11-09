//                                                                                                                     notes are tabbed to the right
//sections: //---------------------------------------------------------------------name
//comment blocks left behind are likely still useful
//delays left behind are likely necessary
//single line comments should hopefully be self-explanatory
//****** for quick ctrl+f during runs
//start with back away from wall
//X = columns, Y = rows

//--------------------------------------------------e-------------------libraries
#include <Wire.h>
#include <LittleFS.h>
#include <SingleFileDrive.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <queue>
#include <stack>
#include <map>
#include <string>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_APDS9960.h>
#include <Adafruit_NeoPixel.h>
#include <VL53L0X.h>
#include <vl53l4cx_class.h>
#include <Servo.h>
#include <PololuQik.h>

//---------------------------------------------------------------------macros
//display
#define TEXT_SIZE 1
#define TCAADDR 0x70

//bno degrees
#define WEST 270
#define EAST 90
#define SOUTH 180
#define NORTH 0

//speeds
#define TURN 75
#define FORWARD 90

//motor macros
#define WHEEL_DIAM 6
#define ENC_A 2
#define ENC_B 3

//wall distance
#define WALL 120

//mux ports for sensors
#define RB 0
#define LB 1
#define APDS_UNDER 1
#define LF 2
#define BNO 3
#define SSD 3
#define BACK 3
#define APDS 4
#define L4CX 4
#define BACKUP_BNO 4
#define FL 5
#define FR 6
#define RF 7

//led pins
#define LED 28
#define LRING 27
#define RRING 4
#define CRING 5
#define PIXELS 12
#define PIXELS2 16

//servo & stepper pins
#define STEP_PIN 18
#define DIR_PIN 19
#define EN_PIN 26
#define SERVO 7

//servo & stepper macros
#define LEFT 130
#define RIGHT 10
#define CENTER 80

#define STEPS_PER_DROP 179
#define STEP_DELAY 800

#define L -1
#define R 1

//ramp cases
#define UP 1
#define DOWN -1
#define RAMP 6

//obstacle cases
#define NONE 0
#define FAR 1
#define CLOSE 2
#define LONG_SIDEL 3
#define SIDEL 4
#define LONG_SIDER 5
#define SIDER 6

//obstacle threshholds
#define OBS_THRESH 120
#define WALL_THRESH 80
#define MAX_THRESH 330

//victim macros
#define harmed 2    //two; blink
#define stable 1    //one; blink
#define unharmed 0  //zero; blink

//return values for checkcolor
#define slver 0
#define rd 1
#define blck 2
#define blu 3

//maze specifications
#define TILE_LEN 30
#define MAZE_DIM 30

//color calibration values
struct Color {
  int r, b;
};

//******calibrate
Color white{ 3515, 7470 };
Color silver{ 4190, 7815 };
Color black{ 160, 360 };
Color blue{ 283, 1935 };
Color red{ 1295, 497 };
int color_ct = 0;

//---------------------------------------------------------------------constants
//color values
int THRESH_r = (red.r + black.r) / 2;                                     //uses r between red and black
int THRESH_b = (blue.b + black.b) / 2;                                    //uses r between blue and black
int SILVER = (white.r + silver.r) / 2;                                    //uses r between white and silver
float BLUE = (float)((float)red.b / red.r + (float)blue.b / blue.r) / 2;  //uses b/r between red and blue
float RED = (float)((float)red.r / red.b + (float)blue.r / blue.b) / 2;   //uses r/b between red and blue

//array indexes for walls
enum dirs { north = 0,
            east = 1,
            south = 2,
            west = 3 };

//misc. globals
int NSEW = NORTH;
const char* filename = "./file.txt";
char buff[100];
uint16_t r, g, b, c;

//******
bool check_back = false;
bool check_cameras = true;  //****
bool check_obstacle = false;
bool danger = true;  //go into danger zone

bool master_check = check_cameras;
int strikes = 0;  //camera fails
int current_bno = BNO;
int ramp_state = DOWN;
int kits_dropped = 0;
bool print_global = false;

//loop1 globals
int s;        //side
int kit = 0;  //count
bool turning = false;

//maze initialization
struct tile {
  uint8_t x, y;
  bool walls[4];  //wall present
  bool visited;   //visited before
};

//array to write file
struct information {
  tile maze[MAZE_DIM][MAZE_DIM];
  tile* cur;
};

//maze globals
information readinfo;
tile* starttile;
std::stack<tile*> path;
tile* current;
tile* parent[MAZE_DIM][MAZE_DIM];  //store tile came from

//sensors and co.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_APDS9960 apds;
VL53L0X tof;
VL53L4CX l4cx(&Wire, -1);

Servo servo;
PololuQik2s12v10 controller1(11, 10, 9);
PololuQik2s12v10 controller2(15, 14, 12);

Adafruit_NeoPixel pixels1 = Adafruit_NeoPixel(PIXELS, LRING, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels2 = Adafruit_NeoPixel(PIXELS, RRING, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels3 = Adafruit_NeoPixel(PIXELS2, CRING, NEO_GRB + NEO_KHZ800);

//bno globals
float off_error = 0;    //cumulative error
float start_error = 0;  //error on being started
float start_error_backup = 0;

//encoders
volatile long encoders_lf = 0;
float enc_per_cm = 84 + 9;

//---------------------------------------------------------------------forward declarations
//needed in .cpp (arduino ide generated these automatically)
float bno_z();
int checkcolor();
void write(tile* m, int rows, int cols);
void navtarget(tile* target, bool print);
void pathhome();

//---------------------------------------------------------------------motor functions
//encoder changes
//left front
void enc_update_lf() {
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);

  if (a == b) {
    encoders_lf--;
  } else {
    encoders_lf++;
  }
}

void reset_enc() {
  encoders_lf = 0;
}

//motor speeds
int cmap(int val, int olow, int ohigh, int mlow, int mhigh) {
  return constrain(map(val, olow, ohigh, mlow, mhigh), mlow, mhigh);
}

void speed(int left, int right) {
  int pwm;

  //straight
  if (left == right) {
    pwm = cmap(abs(right), 0, 100, 0, 127);
    if (right >= 0) {
      controller1.setM0Speed(pwm);
      controller1.setM1Speed(pwm);
    } else {
      controller1.setM0Speed(-pwm);
      controller1.setM1Speed(-pwm);
    }

    pwm = cmap(abs(left), 0, 100, 0, 127);
    if (left >= 0) {
      controller2.setM0Speed(-pwm);
      controller2.setM1Speed(-pwm);
    } else {
      controller2.setM0Speed(pwm);
      controller2.setM1Speed(pwm);
    }
    return;
  }

  //right
  pwm = cmap(abs(right), 0, 100, 0, 127);
  if (right >= 0) {
    controller1.setM0Speed(pwm);
    controller1.setM1Speed(pwm);
  } else {
    controller1.setM0Speed(-pwm);
    controller1.setM1Speed(-pwm);
  }

  //left
  pwm = cmap(abs(left), 0, 100, 0, 127);
  if (left >= 0) {
    controller2.setM0Speed(-pwm);
    controller2.setM1Speed(-pwm);
  } else {
    controller2.setM0Speed(pwm);
    controller2.setM1Speed(pwm);
  }
}

//---------------------------------------------------------------------misc. functions
//switch mux channels
void tcaselect(uint8_t channel) {
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  int err = Wire.endTransmission();
  delay(10);
  if (err) {
    Serial.print("mux error: ");
    Serial.println(err);
    while (1)
      ;
  }
}

//print string on oled
void printoled(int line, String s, bool single = true) {
  tcaselect(SSD);
  //don't clear for multi line displays
  if (single)
    display.clearDisplay();
  display.setCursor(0, TEXT_SIZE * line * 8);
  display.print(s);
  if (single)
    display.display();
}

//blink the led for the interval
void blink() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }

  //extra for the judges
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
}

//return if a tile is a deadend
bool isdeadend(tile* t) {
  int walls = 0;
  for (int i = 0; i < 4; i++)
    if (t->walls[i])
      walls++;
  return walls == 3;
}

//check if return value is a valid victim
bool validvic(int a) {
  if (a == harmed || a == unharmed || a == stable)
    return true;
  return false;
}

//return distance from one or two tofs
int get_dist(int tof1, int tof2 = 10) {
  tcaselect(tof1);
  delay(5);

  int a = tof.readRangeContinuousMillimeters();

  //bad reading
  if (a == 8190 || a == 8191)
    for (int i = 0; i < 5; i++) {
      tcaselect(tof1);
      delay(5);
      a = tof.readRangeContinuousMillimeters();

      if (a != 8190)
        break;
    }

  if (tof2 == 10)
    return a;

  //check second tof
  tcaselect(tof2);
  delay(5);

  int b = tof.readRangeContinuousMillimeters();

  //bad reading
  if (b == 8190 || b == 8191)
    for (int i = 0; i < 5; i++) {
      tcaselect(tof2);
      delay(5);
      b = tof.readRangeContinuousMillimeters();

      if (b != 8190)
        break;
    }
  return (a + b) / 2;  //return average if two sensors given
}

//check for and handle silver
void silvertile() {
  //turn away from wall for better reading
  // bool turn = false;
  // if (get_dist(FL, FR) < WALL)
  //   turn = true;
  if (checkcolor() == slver) {
    readinfo.cur = current;
    write(&readinfo.maze[0][0], MAZE_DIM, MAZE_DIM);  //write out current data
  }
}

void done_routine() {
  printoled(0, "all tiles visited");
  speed(0, 0);
  delay(250);

  //go back to start
  pathhome();
  navtarget(starttile, true);

  printoled(0, "done");
  while (1) {
    speed(0, 0);
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);
  }
}

//change coordinates based on direction
bool change_coords(int& nx, int& ny, int dir) {
  switch (dir) {
    case north:
      ny--;
      break;
    case east:
      nx++;
      break;
    case south:
      ny++;
      break;
    case west:
      nx--;
      break;
  }

  if (nx >= 0 && nx < MAZE_DIM && ny >= 0 && ny < MAZE_DIM)
    return true;
  return false;
}

//return offset from goal
int offset(float des, float cur) {
  int error = cur - des;
  if (error > 180)
    return error - 360;
  if (error < -180)
    return error + 360;
  return error;
}

//check for a ramp
int checkramp() {
  //ascending
  if (bno_z() > 17.0) {
    return UP;
  }
  //descending
  if (bno_z() < -4.0) {
    return DOWN;
  }
  //nothing
  return false;
}

//rescue letter victims
void letters(int val, int side) {
  switch (val) {
    case unharmed:
      blink();
      break;
    case stable:
      s = side;
      kit = 1;
      blink();
      // drop(side, 1);
      break;
    case harmed:
      s = side;
      kit = 2;
      blink();
      // drop(side, 2);  //might have to change to 1**
      break;
    default:  //no valid victim
      break;
  }
}

//rescue ring victims
void rings(int val, int side) {
  switch (val) {
    case unharmed:
      blink();
      break;
    case stable:
      s = side;
      kit = 1;
      blink();
      // drop(side, 1);
      break;
    case harmed:
      s = side;
      kit = 2;
      blink();
      // drop(side, 2);
      break;
    default:  //no valid victim
      break;
  }
}

//check if the address for the l4cx is open
bool l4cx_present() {
  tcaselect(L4CX);
  delay(10);

  Wire.beginTransmission(0x29);
  byte error = Wire.endTransmission();

  return error == 0;
}

//write to stepper
void stepmotor(int steps) {
  //turn on
  digitalWrite(EN_PIN, LOW);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY);
  }

  //turn off
  delay(200);
  digitalWrite(EN_PIN, HIGH);
}

//drop rescue kits
void drop(int side, int count) {
  if (kits_dropped >= 8)
    return;

  kits_dropped += count;

  //turn to side
  if (side == L) {
    printoled(0, "dropping left");
    servo.write(LEFT);
  } else {
    printoled(0, "dropping right");
    servo.write(RIGHT);
  }

  delay(250);

  //do for number of kits
  for (int i = 0; i < count; i++) {
    stepmotor(STEPS_PER_DROP);
    delay(200);
  }

  //reset
  delay(750);
  servo.write(CENTER);
}

//---------------------------------------------------------------------sensor and etc. functions
//return values from color sensor
void getcolor(int sensor = APDS) {
  tcaselect(sensor);
  speed(0, 0);
  delay(20);
  apds.getColorData(&r, &g, &b, &c);
  sprintf(buff, "r = %d, g = %d, b = %d, c = %d", r, g, b, c);
}

//return distance from l4cx
int readl4cx() {
  tcaselect(L4CX);

  VL53L4CX_MultiRangingData_t data;
  uint8_t newDataReady = 0;
  int status;

  //wait until data returned
  do {
    status = l4cx.VL53L4CX_GetMeasurementDataReady(&newDataReady);

    if (status != 0) {
      return -1;
    }

    delay(1);

  } while (!newDataReady);

  //error in reading
  if (status != 0)
    return -1;

  status = l4cx.VL53L4CX_GetMultiRangingData(&data);

  //reading bad
  if (status != 0 || data.NumberOfObjectsFound == 0) {
    l4cx.VL53L4CX_ClearInterruptAndStartMeasurement();
    return -1;
  }

  //pick closest object
  int mindist = 9999;
  for (int i = 0; i < data.NumberOfObjectsFound; i++) {
    int d = data.RangeData[i].RangeMilliMeter;

    if (d > 0 && d < mindist)
      mindist = d;
  }

  l4cx.VL53L4CX_ClearInterruptAndStartMeasurement();

  return mindist;
}

//return yaw
float printbno() {
  static int bad_reads = 0;

  tcaselect(current_bno);
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float heading = orientationData.orientation.x;

  //zeroed out
  if (heading == 0.00)
    bad_reads++;

  //if stuck at 0.00 too long and can switch
  if (heading == 0.00 && current_bno == BNO && bad_reads > 5) {
    printoled(0, "switching bnos");
    delay(350);

    current_bno = BACKUP_BNO;
    tcaselect(current_bno);

    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    heading = orientationData.orientation.x;

    //adjust error for new sensor
    off_error = NSEW - heading;
    bad_reads = 0;
  }

  sprintf(buff, "heading %d: %.2f", current_bno, heading);
  printoled(0, buff);

  //change for drift and start shifting
  heading = heading + off_error;
  // if (current_bno == BNO)
  //   heading -= start_error;
  // else
  //   heading -= start_error_backup;
  heading -= start_error;

  if (heading < 0)
    heading += 360;
  else if (heading >= 360)
    heading -= 360;

  return heading;
}

//return pitch
float bno_z() {
  tcaselect(current_bno);
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // Serial.println(orientationData.orientation.z);
  return -1 * orientationData.orientation.z;
}

//get victim values from cameras
void cameras(int& bull, int& greek, int& bull2, int& greek2) {
  //send pi trigger
  int bytes = Serial1.write('a');
  if (bytes < 1)
    Serial.println("serial write failed");

  //if too many blank reads or on ramp
  if (strikes > 4 || bno_z() > 20.0) {
    bull = -1;
    greek = -1;
    bull2 = -1;
    greek2 = -1;
    return;
  }

  int start = millis();
  while (Serial1.available() < 1)
    if (millis() > start + 3000) {
      strikes++;
      bull = -1;
      greek = -1;
      bull2 = -1;
      greek2 = -1;
      return;
    }

  bull = Serial1.parseInt();
  greek = Serial1.parseInt();
  bull2 = Serial1.parseInt();
  greek2 = Serial1.parseInt();
}

//return color based on threshholds
int checkcolor() {
  getcolor();

  int tries = 0;

  //can't divide by zero; check again
  while ((r == 0 || b == 0)) {
    getcolor();
    printoled(0, "zero color values");
    delay(125);
    tries++;
    if (tries > 5)
      return -1;
  }

  sprintf(buff, "%d, %d, %d, %d", r, g, b, c);
  Serial.println(buff);

  if (r > SILVER) {
    printoled(0, "silver");
    return slver;
  } else if ((float)b / r > BLUE) {
    printoled(0, "blue");
    return blu;
  } else if ((float)r / b > RED) {
    printoled(0, "red");
    if (danger)
      return rd;
    else
      return blck;
  } else if (b < THRESH_b && r < THRESH_r) {
    printoled(0, "black");
    return blck;
  }
  // autocalibrate();
  return -1;
}

void autocalibrate() {
  getcolor();
  int ct = color_ct++;
  int white_r = white.r;
  int white_b = white.b;

  white = { (white_r * ct + r) / color_ct, (white_b * ct + b) / color_ct };
  black = { white.r - 80, white.b - 80 };  //                                                                                           tune values
  red = { white.r + 80, white.b };
  blue = { white.r, white.b + 80 };
  silver = { white.r + 100, white.b + 100 };

  THRESH_r = (red.r + black.r) / 2;
  THRESH_b = (blue.b + black.b) / 2;
  SILVER = (white.r + silver.r) / 2;
  BLUE = (float)((float)red.b / red.r + (float)blue.b / blue.r) / 2;
  RED = (float)((float)red.r / red.b + (float)blue.r / blue.b) / 2;
}

//detect different types of obstacles
int obstacle_detection() {
  int left = get_dist(FL);
  int center;
  for (int i = 0; i < 3; i++)
    center = readl4cx();
  int right = get_dist(FR);

  sprintf(buff, "left: %d, center: %d, right: %d", left, center, right);
  Serial.println(buff);

  //everything too far
  if (left > MAX_THRESH && center > MAX_THRESH && right > MAX_THRESH)
    return NONE;

  //weird state
  if (center > MAX_THRESH && abs(right - left) < WALL_THRESH)
    return NONE;

  //bad l4cx reads
  if (center == -1 || center == 9999)
    return NONE;

  //everything sees; wall not obstacle
  if (abs(left - center) < WALL_THRESH && abs(right - center) < WALL_THRESH && abs(left - right) < WALL_THRESH)
    return NONE;

  //sort close obstacles
  if (center < OBS_THRESH)
    return CLOSE;
  if (left < OBS_THRESH)
    return SIDEL;
  if (right < OBS_THRESH)
    return SIDER;

  //sort far obstacles
  if (center < left && center < right)
    return FAR;
  if (left < right)
    return LONG_SIDEL;
  if (right < left)
    return LONG_SIDER;

  return NONE;
  //                                                                            add in else case later if center does not see but left and right do
}

//---------------------------------------------------------------------movement functions
//align to wall
bool corrections() {
  int correct_dist = 55;
  int dist1, dist2;
  int spd = 55;

  //distance
  if (get_dist(FL, FR) > 175)
    return false;

  // move into wall to correct heading
  while (get_dist(FL, FR) > 50)
    speed(FORWARD - 15, FORWARD - 15);
  delay(400);
  speed(0, 0);

  while (get_dist(FL, FR) < correct_dist + 10)
    speed(-spd + 15, -spd + 15);
  speed(0, 0);

  // //distance
  // if (get_dist(FL, FR) > correct_dist)
  //   while (get_dist(FL, FR) > correct_dist)
  //     speed(FORWARD - 15, FORWARD - 15);
  // else
  //   while (get_dist(FL, FR) < correct_dist)
  //     speed(-FORWARD + 20, -FORWARD + 20);

  // speed(0, 0);

  // //coarse angle
  // if (abs(get_dist(FR) - get_dist(FL)) > 10) {
  //   if (get_dist(FR) > get_dist(FL))
  //     while (get_dist(FR) > get_dist(FL))
  //       speed(-TURN + 20, TURN - 20);
  //   else
  //     while (get_dist(FL) > get_dist(FR))
  //       speed(TURN - 20, -TURN + 20);
  // }

  // speed(0, 0);
  // // delay(10);

  // //fine angle
  // if (abs(get_dist(FR) - get_dist(FL)) > 0) {
  //   if (get_dist(FR) > get_dist(FL))
  //     while (get_dist(FR) > get_dist(FL))
  //       speed(-40, 40);
  //   else
  //     while (get_dist(FL) > get_dist(FR))
  //       speed(40, -40);
  // }

  speed(0, 0);

  //change error to true direction
  off_error = NSEW - printbno();

  return true;
}

//handle values return from camera
bool victims(tile* cur, int half = 0) {
  speed(0, 0);

  int left_ring = -1, left_letter = -1, right_ring = -1, right_letter = -1;

  //don't bother if tile has already been checked
  current->visited = false;
  if (cur->visited)
    return false;
  current->visited = true;

  printoled(0, "checking victims");

  int lb = get_dist(LB), lf = get_dist(LF), rf = get_dist(RF), rb = get_dist(RB);

  // if(lb > WALL + 50 && lf > WALL + 50 && rf > WALL + 50 && rb > WALL + 50)
  //   return false;

  //check cameras three times
  for (int i = 0; i < 3; i++) {
    int a, b, c, d;
    cameras(a, b, c, d);

    //if a valid value is returned, mark it down
    if (validvic(a))
      left_ring = a;
    if (validvic(b))
      left_letter = b;
    if (validvic(c))
      right_ring = c;
    if (validvic(d))
      right_letter = d;
  }

  if (left_ring == 0 && left_letter == 0 && right_ring == 0 && right_letter == 0)
    return false;

  sprintf(buff, "%d, %d, %d, %d", left_ring, left_letter, right_ring, right_letter);
  Serial.println(buff);
  printoled(0, buff);

  //handle values only if an unchecked wall is on that side
  //left wall
  if (lf < WALL + 75 || lb < WALL + 75) {
    if (validvic(left_letter))
      letters(left_letter, L);

    if (validvic(left_ring))
      rings(left_ring, L);
  }

  //right wall
  if (rf < WALL + 75 || rb < WALL + 75) {
    if (validvic(right_letter))
      letters(right_letter, R);

    if (validvic(right_ring))
      rings(right_ring, R);
  }

  current->visited = true;

  //return if victim was found
  if (validvic(left_letter) || validvic(right_letter) || validvic(left_ring) || validvic(right_ring))
    return true;
  return false;
}

//                                                                                            need to test new 45 deg thing and different tolerance

//correct to direction
void correct(float dir, int state = 3) {
  while (true) {
    float cur = printbno();
    int err = offset(dir, cur);

    //turn 45 degrees and check cameras
    // if (state == -1) {
    //   int dir = (((int)NSEW % 360) + 360) % 360 / 90;
    //   int nx = current->x;
    //   int ny = current->y;
    //   int ct = 1;
    //   if (abs(err) < ct++ * 45) {
    //     speed(0, 0);
    //     if (check_cameras)
    //       if (change_coords(nx, ny, dir))
    //         victims(&readinfo.maze[nx][ny], true);
    //   }
    // }

    //close enough
    if (state > 0) {
      if (abs(err) <= state)
        break;
    }
    //state is -1, assume tol of 3
    else {
      if (abs(err) <= 3)
        break;
    }

    int turn_speed;

    //go slower if closer to target heading
    if (abs(err) < 12)
      turn_speed = TURN - 35;
    else
      turn_speed = TURN;

    //turn left or right depending on error
    if (err < 0)
      speed(turn_speed, -turn_speed);
    else
      speed(-turn_speed, turn_speed);
  }

  speed(0, 0);
}

//align heading to side walls
void side_adj(int s1, int s2) {
  //fine angle
  if (abs(get_dist(s1) - get_dist(s2)) > 3) {
    if (get_dist(s1) > get_dist(s2))
      while (get_dist(s1) > get_dist(s2))
        speed(-TURN + 30, TURN - 30);
    else
      while (get_dist(s2) > get_dist(s1))
        speed(TURN - 30, -TURN + 30);
  }

  speed(0, 0);
  off_error = NSEW - printbno();
}

bool obstacle_avoidance(int placement, tile* pos) {
  int dir = (((int)NSEW % 360) + 360) % 360 / 90;
  int nx = pos->x;
  int ny = pos->y;

  if (!change_coords(nx, ny, dir))
    return false;

  tile* blocked = &readinfo.maze[nx][ny];
  bool left_wall = (get_dist(LF) < WALL + 50);
  bool right_wall = (get_dist(RF) < WALL + 50);

  switch (placement) {
    //on border between current and next
    //block wall on tile ahead
    case CLOSE:
    case SIDEL:
    case SIDER:
      printoled(0, "close");
      delay(200);

      readinfo.maze[nx][ny].walls[(dir + 2) % 4] = true;
      break;
    //in middle of next tile
    //block of all walls
    case FAR:
      printoled(0, "far");
      delay(200);

      blocked->visited = true;

      for (int i = 0; i < 4; i++) {
        blocked->walls[i] = true;  //block off own walls

        //find surrounding tiles
        int nnx = nx;
        int nny = ny;

        if (change_coords(nnx, nny, i))
          readinfo.maze[nnx][nny].walls[(i + 2) % 4] = true;
      }
      break;
    // case SIDEL:  //on left of border
    //   //try to curve around it
    //     if(get_dist(LF, LB) < WALL)
    //       speed(30, 50);                                                               //fill in later; use trig to calculate stright line distance

    //     correct(NSEW);
    //   break;
    // case SIDER:  //on right of border
    //   //try to curve around it
    //     if(get_dist(RF, RB) < WALL)
    //       speed(50, 30);

    //     correct(NSEW);
    //   break;
    //on left of next tile
    //block off both walls touching the left wall and the left wall
    case LONG_SIDEL:
      printoled(0, "left");
      delay(200);

      blocked->visited = true;

      for (int i = 0; i < 4; i++) {
        if (i == (dir + 3) % 4)
          continue;
        blocked->walls[i] = true;  //block off own walls

        //find surrounding tiles
        int nnx = nx;
        int nny = ny;

        if (change_coords(nnx, nny, i))
          readinfo.maze[nnx][nny].walls[(i + 2) % 4] = true;
      }
      break;
    //on right of next tile
    //block off both walls touching the right wall and the right wall
    case LONG_SIDER:
      printoled(0, "right");
      delay(200);

      blocked->visited = true;

      for (int i = 0; i < 4; i++) {
        if (i == (dir + 1) % 4)
          continue;
        blocked->walls[i] = true;  //block off own walls

        //find surrounding tiles
        int nnx = nx;
        int nny = ny;

        if (change_coords(nnx, nny, i))
          readinfo.maze[nnx][nny].walls[(i + 2) % 4] = true;
      }
      break;
    //no obstacle detected
    case NONE:
      return false;
  }

  Serial.print("obstacle: ");
  Serial.println(placement);

  return true;
}

//travel up or down a ramp
void travelramp(int ramp) {
  int spd = 55;

  // check_walls(false);

  // if (ramp_state == DOWN && ramp == DOWN) {
  //   while (abs(bno_z()) > 2.0)
  //     speed(spd - 10, spd - 10);

  //   reset_enc();
  //   while (abs(encoders_lf) < enc_per_cm * 5)
  //     speed(spd - 10, spd - 10);

  //   NSEW = (NSEW + 180) % 360;
  //   correct(NSEW);

  //   reset_enc();
  //   while (abs(encoders_lf) < enc_per_cm * 15)
  //     speed(spd, spd);

  //   while (abs(bno_z()) > 2.0)
  //     speed(spd + 15, spd + 15);
  //   speed(0, 0);
  //   blink();
  // } else
  //   //go until level off
  //   while (abs(bno_z()) > 2.0) {
  //     if (ramp == UP)
  //       speed(spd + 15, spd + 15);
  //     else
  //       speed(spd - 10, spd - 10);
  //   }

  //keep going until level off
  reset_enc();
  float z = bno_z();

  while (z > 5 || z < -2.5) {
    z = bno_z();
    if (ramp == UP)
      speed(spd + 10, spd + 10);
    else speed(spd - 25, spd - 25);
  }

  int encs = encoders_lf;

  reset_enc();
  speed(0, 0);

  //adjust
  if (get_dist(FR) > 150) {
    reset_enc();
    if (ramp == DOWN)
      while (abs(encoders_lf) < enc_per_cm * 10)
        speed(spd - 10, spd - 10);
    else
      while (abs(encoders_lf) < enc_per_cm * 6)
        speed(spd - 10, spd - 10);
  }

  //calculate how many tiles ramp was
  // int n = encs / (enc_per_cm * 30);
  // n = 1;

  //adjust state since technically moved 2+ tiles
  int dir = (((int)NSEW % 360) + 360) % 360 / 90;  //                                              make sure this can handle different ramp lengths
  int nx = current->x;
  int ny = current->y;

  // for (int i = 0; i < n; i++) {
  //set walls on left and right
  if (change_coords(nx, ny, dir)) {
    current = &readinfo.maze[nx][ny];
    current->visited = true;
    current->walls[(dir + 1) % 4] = true;
    current->walls[(dir + 3) % 4] = true;
  }

  nx = current->x;
  ny = current->y;

  //adjust current
  if (change_coords(nx, ny, dir))
    current = &readinfo.maze[nx][ny];
  // }

  ramp_state = ramp;
}

//traverse and fill in information for a hole
void handlehole(int spd, long target) {
  printoled(0, "hole");

  //find tile going to
  int dir = (((int)NSEW % 360) + 360) % 360 / 90;
  int nx = current->x;
  int ny = current->y;

  if (change_coords(nx, ny, dir)) {
    tile* blocked = &readinfo.maze[nx][ny];
    for (int i = 0; i < 4; i++) {
      blocked->walls[i] = true;  //block off own walls

      //find surrounding tiles
      int nnx = nx;
      int nny = ny;

      if (!change_coords(nnx, nny, i))
        continue;

      //mark corresponding wall in neighbor
      int opposite = (i + 2) % 4;
      readinfo.maze[nnx][nny].walls[opposite] = true;
    }
  }

  //go back to the original tile
  reset_enc();
  while (abs(encoders_lf) < target)
    speed(-spd, -spd);
  speed(0, 0);
}

//move straight in cm
int enc_forward(int dist_cm, int spd, bool special_checks = false) {
  reset_enc();
  long target = (long)(enc_per_cm * dist_cm);
  int ramp = false;
  float z = bno_z();
  bool found = false;
  bool blueness = false;
  float dist = 7.5;
  int d = 0;

  //find tile going to
  int dir = (((int)NSEW % 360) + 360) % 360 / 90;
  int nx = current->x;
  int ny = current->y;

  int ct = 1;
  //move half a tile
  while (abs(encoders_lf) < (target + d * enc_per_cm) / 2) {
    if (ramp != DOWN)
      speed(spd - 30, spd - 30);

    //check every quarter tile for black
    if (special_checks && abs(encoders_lf) > dist * enc_per_cm * ct) {
      speed(0, 0);
      if (checkcolor() == blck) {
        handlehole(spd - 20, dist * enc_per_cm * ct);
        return blck;
      }
      ct++;
    }

    //check if ramp
    if (special_checks)
      //get out if going down ramp so don't fall
      if ((ramp = checkramp()) == DOWN)
        break;
  }

  //check if ramp was detected and handle if it was
  if (ramp)
    travelramp(ramp);

  reset_enc();
  speed(0, 0);

  //check for hole and victims
  if (special_checks && !ramp) {
    if (checkcolor() == blck) {
      handlehole(spd, target / 2);
      return blck;
    }

    //                                                                                                                            check cameras
    if (check_cameras)
      if (change_coords(nx, ny, dir))
        victims(&readinfo.maze[nx][ny], true);

    //check for blue, but probably not on enough to warrant a stop
    if (checkcolor() == blu)
      blueness = true;
  }

  //if no ramp yet, move the rest of the tile
  if (!ramp) {
    correct(NSEW, 5);
    while (abs(encoders_lf) < (target - d * enc_per_cm) / 2) {
      speed(spd, spd);

      if (special_checks)
        ramp = checkramp();
    }

    //check for ramp again
    if (ramp)
      travelramp(ramp);

    //                                                                                                                                check cameras
    if (check_cameras)
      if (special_checks && !ramp)
        if (change_coords(nx, ny, dir)) {
          victims(&readinfo.maze[nx][ny]);
        }
  }

  //check end of tile for black
  if (special_checks) {
    speed(0, 0);
    if (checkcolor() == blck) {
      handlehole(spd - 20, enc_per_cm * dist_cm);
      return blck;
    }
  }

  //check for blue again
  if (special_checks)
    if (checkcolor() == blu)
      blueness = true;

  //wait if blue at any point
  if (blueness) {
    speed(0, 0);
    delay(5500);
  }

  if (ramp)
    return RAMP;

  speed(0, 0);
  return -1;
}

//adjust if sensors on the same side are different
bool adjust(int s1, int s2) {
  int spd = 55;
  //if sensors disagree
  int one = get_dist(s1);
  int two = get_dist(s2);

  //try moving forward
  if (abs(one - two) > 170 && (one < WALL + 50 || two < WALL + 50)) {
    enc_forward(5, spd - 10);

    one = get_dist(s1);
    two = get_dist(s2);

    //check if sensors match now
    if (abs(one - two) > 170 && (one < WALL + 50 || two < WALL + 50))
      //try moving backward
      enc_forward(10, -spd + 10);
  }

  one = get_dist(s1);
  two = get_dist(s2);

  //check if sensors match now
  if (abs(one - two) > 100 && (one < WALL + 50 || two < WALL + 50)) {
    //return to original position if not
    enc_forward(9, spd + 10);
    return false;
  }
  return true;
}

//---------------------------------------------------------------------file system
//write to file
void write(tile* m, int rows, int cols) {
  File outfile = LittleFS.open(filename, "w");
  if (!outfile) {
    printoled(0, "file opening failed write");
    return;
  }

  //write in maze array
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++) {
      tile& t = m[i * cols + j];
      outfile.write(t.x);
      outfile.write(t.y);
      //write in walls & visited from struct
      for (int k = 0; k < 4; k++)
        outfile.write(t.walls[k] ? 1 : 0);
      outfile.write(t.visited ? 1 : 0);
    }

  //write in the current tile
  outfile.write((uint8_t)readinfo.cur->x);
  outfile.write((uint8_t)readinfo.cur->y);

  outfile.close();
}

//read from file
void read(tile* m, int rows, int cols) {
  File infile = LittleFS.open(filename, "r");
  if (!infile) {
    printoled(0, "file opening failed read");
    return;
  }

  //read in the maze array
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++) {
      tile& t = m[i * cols + j];
      t.x = infile.read();
      t.y = infile.read();
      //read in the walls & visited for the struct
      for (int k = 0; k < 4; k++)
        t.walls[k] = infile.read() ? true : false;
      t.visited = infile.read() ? true : false;
    }

  //read in the current tile
  uint8_t cx = infile.read();
  uint8_t cy = infile.read();
  readinfo.cur = &readinfo.maze[cx][cy];

  infile.close();
}

//initialization if there is no file or the while should be cleared
void mazeinit_empty() {
  for (int i = 0; i < MAZE_DIM; i++) {
    for (int j = 0; j < MAZE_DIM; j++) {
      readinfo.maze[i][j].x = i;
      readinfo.maze[i][j].y = j;
      readinfo.maze[i][j].visited = false;
      for (int k = 0; k < 4; k++)
        readinfo.maze[i][j].walls[k] = false;
    }
  }
  starttile = &readinfo.maze[MAZE_DIM / 2][MAZE_DIM / 2];
  readinfo.cur = starttile;
  starttile->visited = true;
}

//initalization if reading from file
void mazeinit_file() {
  read(&readinfo.maze[0][0], MAZE_DIM, MAZE_DIM);
  starttile = &readinfo.maze[MAZE_DIM / 2][MAZE_DIM / 2];  //always fixed start
  readinfo.cur = &readinfo.maze[readinfo.cur->x][readinfo.cur->y];
}

//---------------------------------------------------------------------bfs
//check in front and on left + right for walls
void check_walls(int set = true) {
  int dir_forward = (((int)NSEW % 360) + 360) % 360 / 90;
  bool works = true;

  display.clearDisplay();

  //forward
  int d = get_dist(FL, FR);
  for (int i = 0; i < 3; i++) {
    d = get_dist(FL, FR);
    if (d < WALL)
      break;
  }

  //set only if not trying to correct
  if (d < WALL)
    if (set)
      current->walls[dir_forward] = true;

  //right
  //if adjust didn't work, there might be an obstacle in that direction
  if (adjust(RF, RB)) {
    d = get_dist(RF, RB);
    int dir_right = (dir_forward + 1) % 4;
    if (d < WALL + 50) {
      //set only if not trying to adjust
      if (set)
        current->walls[dir_right] = true;
      side_adj(RB, RF);
    }
  } else
    works = false;

  //left
  //if adjust didn't work, there might be an obstacle in that direction
  if (adjust(LF, LB)) {
    d = get_dist(LF, LB);
    int dir_left = (dir_forward + 3) % 4;
    if (d < WALL + 50) {
      //set only if not trying to adjust
      if (set)
        current->walls[dir_left] = true;

      //adjust only if not already used other side
      if (!current->walls[(dir_forward + 1) % 4])
        side_adj(LF, LB);
    }
  } else
    works = false;

  //overcorrect a little to accomodate beign too close to the wall
  if (works && set) {
    int left = get_dist(LF, LB);
    int right = get_dist(RF, RB);

    //left is closer, turn right
    if (left < 70 || (right < 200 && right > 140))
      correct(NSEW + 9);

    //right is closer, turn left
    else if (right < 70 || (left < 200 && left > 140))
      correct(NSEW - 9);
  }

  off_error = NSEW - printbno();
}

//check behind upon starting for a wall
void check_back_wall() {
  int dir_back = south;

  //                                                                                                                             check cameras
  if (check_cameras)
    victims(starttile);

  NSEW = EAST;
  correct(NSEW);
  corrections();

  //                                                                                                                              check cameras
  if (check_cameras)
    victims(starttile);

  int d = get_dist(RF, RB);
  starttile->walls[dir_back] = (d < WALL);

  NSEW = NORTH;
  correct(NSEW);
  corrections();
}

//find nearest unvisited tile
tile* nearest_unvisited(tile* start) {
  //mark queued tiles
  bool seen[MAZE_DIM][MAZE_DIM] = { false };
  for (int i = 0; i < MAZE_DIM; i++)
    for (int j = 0; j < MAZE_DIM; j++)
      parent[i][j] = nullptr;

  std::queue<tile*> q;
  q.push(start);
  seen[start->x][start->y] = true;

  while (!q.empty()) {
    tile* cur = q.front();
    q.pop();

    if (cur != start && !cur->visited) {
      //if true, found the nearest unvisited tile
      while (!path.empty())
        path.pop();
      //follow the parents back to start
      for (tile* t = cur; t != nullptr; t = parent[t->x][t->y])
        //make sure tile after start is at the top
        if (t != start)
          path.push(t);
      return cur;
    }

    for (auto i : { north, west, east, south }) {
      //if already know there's a wall, skip
      if (cur->walls[i])
        continue;

      //convert direction into coordinates
      int nx = cur->x;
      int ny = cur->y;

      if (!change_coords(nx, ny, i))
        continue;

      if (!seen[nx][ny]) {
        //it's been queued
        seen[nx][ny] = true;
        parent[nx][ny] = cur;
        q.push(&readinfo.maze[nx][ny]);
      }
    }
  }
  return nullptr;
}

//follow path to target
void navtarget(tile* target, bool print) {
  while (!path.empty()) {
    tile* next = path.top();  //tile to move to
    path.pop();

    //debugging
    if (print) {
      sprintf(buff, "currently at (%d, %d)", current->x, current->y);
      Serial.println(buff);
      sprintf(buff, "travelling to (%d, %d)", next->x, next->y);
      printoled(0, buff);
      if (print_global)
        delay(350);
      Serial.println(buff);
    }

    int dx = next->x - current->x;
    int dy = next->y - current->y;

    //if can just back up
    if (((NSEW == NORTH && dx == 0 && dy == 1) || (NSEW == SOUTH && dx == 0 && dy == -1) || (NSEW == EAST && dx == -1 && dy == 0) || (NSEW == WEST && dx == 1 && dy == 0)) && (next->visited == true)) {
      if (isdeadend(current)) {
        NSEW = (NSEW + 90) % 360;
        correct(NSEW, -1);
        //                                                                                                                             check cameras
        corrections();
        if (check_cameras)
          victims(current);
      }

      NSEW = (NSEW + 90) % 360;
      correct(NSEW, -1);
      check_walls(false);

      silvertile();

      if (master_check)
        check_cameras = false;

      //back up
      int state = enc_forward(TILE_LEN, FORWARD, true);
      if (state == blck) {
        current->walls[(((int)NSEW % 360) + 360) % 360 / 90] = true;  //mark wall of current tile in direction of hole
        if (master_check)
          check_cameras = true;
        return;
      }

      if (master_check)
        check_cameras = true;

      //made it to next tile
      if (state != RAMP)
        current = next;
      else if (!path.empty())
        path.pop();

    } else {
      int beginning = NSEW;
      //create new heading angle
      //dx = 1 means east
      //dx = -1 means west
      //dy = 1 means south
      //dy = -1 means north
      if (dx == 1 && dy == 0)
        NSEW = EAST;
      else if (dx == -1 && dy == 0)
        NSEW = WEST;
      else if (dx == 0 && dy == -1)
        NSEW = NORTH;
      else if (dx == 0 && dy == 1)
        NSEW = SOUTH;

      if (beginning == NSEW)
        correct(NSEW);
      else
        correct(NSEW, -1);
      if (!corrections())
        check_walls(false);

      silvertile();

      //                                                                                                              check cameras
      if (check_cameras)
        if (beginning != NSEW)
          victims(current);

      //check for obstacle
      if (check_obstacle)
        if (obstacle_avoidance(obstacle_detection(), current)) {
          current->walls[(((int)NSEW % 360) + 360) % 360 / 90] = true;  //mark wall of current tile in direction of obstacle
          return;
        }

      //move forward
      int state = enc_forward(TILE_LEN, FORWARD, true);
      if (state == blck) {
        current->walls[(((int)NSEW % 360) + 360) % 360 / 90] = true;
        return;
      }

      //made it to next tile
      if (state != RAMP)
        current = next;
      else if (!path.empty())
        path.pop();
    }

    corrections();
  }

  correct(NSEW, 5);
}

//bfs a path back home
void pathhome() {
  bool seen[MAZE_DIM][MAZE_DIM] = { false };
  std::queue<tile*> q;
  q.push(current);
  seen[current->x][current->y] = true;

  //reset parents
  for (int i = 0; i < MAZE_DIM; i++)
    for (int j = 0; j < MAZE_DIM; j++)
      parent[i][j] = nullptr;

  //go until start found
  while (!q.empty()) {
    tile* cur = q.front();
    q.pop();

    //found path to start
    if (cur == starttile) {
      //clear old path
      while (!path.empty())
        path.pop();
      for (tile* t = cur; t != nullptr; t = parent[t->x][t->y])
        //skip current
        if (t != current)
          path.push(t);
      return;
    }

    //check neighbors
    for (int i = 0; i < 4; i++) {
      //skip if wall
      if (cur->walls[i])
        continue;

      int nx = cur->x;
      int ny = cur->y;

      if (!change_coords(nx, ny, i))
        continue;

      //add to bfs
      if (!seen[nx][ny]) {
        seen[nx][ny] = true;
        parent[nx][ny] = cur;
        q.push(&readinfo.maze[nx][ny]);
      }
    }
  }
}

//---------------------------------------------------------------------executions
void setup() {
  auto reason = rp2040.getResetReason();
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();
  Serial.begin(115200);
  Serial1.setTX(16);
  Serial1.setRX(17);
  Serial1.begin(115200);
  // while(!Serial);
  delay(2000);

  LittleFS.begin();

  controller2.init();
  Serial.print("controller2 firmware: ");
  Serial.write(controller2.getFirmwareVersion());
  Serial.println();

  controller1.init();
  Serial.print("controller1 firmware: ");
  Serial.write(controller1.getFirmwareVersion());
  Serial.println();

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), enc_update_lf, CHANGE);
  Serial.println("encoders lf is fine");

  servo.attach(SERVO);
  servo.write(CENTER);

  pixels1.begin();
  pixels2.begin();
  pixels3.begin();
  pixels1.clear();
  pixels2.clear();
  pixels3.clear();
  pixels1.show();
  pixels2.show();
  pixels3.show();

  // for (int i = 0; i < PIXELS; i++) {
  //   pixels1.setPixelColor(i, pixels2.Color(10, 10, 10));
  //   pixels2.setPixelColor(i, pixels2.Color(10, 10, 10));
  // }
  // pixels1.show();
  // pixels2.show();

  for (int i = 0; i < PIXELS2; i++)
    pixels3.setPixelColor(i, pixels3.Color(80, 80, 80));
  pixels3.show();

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);

  pinMode(20, INPUT);
  pinMode(21, INPUT);
  pinMode(LED, OUTPUT);

  tcaselect(SSD);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3d)) {
    Serial.println("ssd is not fine");
    while (1)
      ;
  } else
    Serial.println("ssd is fine");

  display.clearDisplay();
  display.setTextSize(TEXT_SIZE);
  display.setTextColor(SSD1306_WHITE);

  sprintf(buff, "reason: %d", reason);
  printoled(0, buff);

  tcaselect(APDS);
  if (!apds.begin(60, APDS9960_AGAIN_4X)) {
    Serial.println("color is not fine");
    while (1)
      ;
  } else
    Serial.println("color is fine");

  apds.enableColor(true);

  // tcaselect(APDS_UNDER);
  // if (!apds.begin(60, APDS9960_AGAIN_4X)) {
  //   Serial.println("color under is not fine");
  //   while (1)
  //     ;
  // } else
  //   Serial.println("color under is fine");

  apds.enableColor(true);

  tcaselect(RB);
  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println("tof RB is not fine");
    while (1)
      ;
  } else
    Serial.println("tof RB is fine");
  tof.startContinuous();

  tcaselect(LF);
  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println("tof LF is not fine");
    while (1)
      ;
  } else
    Serial.println("tof LF is fine");
  tof.startContinuous();

  tcaselect(LB);
  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println("tof LB is not fine");
    while (1)
      ;
  } else
    Serial.println("tof LB is fine");
  tof.startContinuous();

  tcaselect(RF);
  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println("tof RF is not fine");
    while (1)
      ;
  } else
    Serial.println("tof RF is fine");
  tof.startContinuous();

  tcaselect(FL);
  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println("tof FL is not fine");
    while (1)
      ;
  } else
    Serial.println("tof FL is fine");
  tof.startContinuous();

  tcaselect(FR);
  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println("tof FR is not fine");
    while (1)
      ;
  } else
    Serial.println("tof FR is fine");
  tof.startContinuous();

  // tcaselect(BACK);
  // tof.setTimeout(500);
  // if (!tof.init()) {
  //   Serial.println("tof BACK is not fine");
  //   while (1)
  //     ;
  // } else
  //   Serial.println("tof BACK is fine");
  // tof.startContinuous();

  tcaselect(L4CX);
  if (l4cx.begin() != VL53L4CX_ERROR_NONE || !l4cx_present()) {
    Serial.println("l4cx is not fine");
    if (!l4cx_present())
      Serial.println("0x29 is not fine");
    while (1)
      ;
  } else
    Serial.println("l4cx is fine");
  l4cx.VL53L4CX_Off();
  l4cx.InitSensor(0x29);
  l4cx.VL53L4CX_StartMeasurement();

  display.clearDisplay();
  printoled(0, "ready to start", false);
  printoled(1, "press 21 twice to clear memory", false);
  display.display();

  while (true) {
    getcolor();
    sprintf(buff, "%d, %d", r, b);
    Serial.println(buff);

    if (digitalRead(20))
      break;

    //21 twice to prevent accidental deletion
    if (digitalRead(21)) {
      delay(250);
      if (digitalRead(21)) {
        printoled(0, "clearing memory...");
        LittleFS.remove(filename);
        break;
      }
    }
  }

  //set bno to current position
  tcaselect(BNO);
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("bno is not fine");
    while (1)
      ;
  } else
    Serial.println("bno is fine");

  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  //find offset from true north
  start_error = orientationData.orientation.x;

  tcaselect(BACKUP_BNO);
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("backup bno is not fine");
    while (1)
      ;
  } else
    Serial.println("backup bno is fine");

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  //find offset from true north
  start_error_backup = orientationData.orientation.x;

  //if file exists, load it
  if (LittleFS.exists(filename)) {
    mazeinit_file();
    printoled(0, "reading from file");
    delay(200);
  } else {
    mazeinit_empty();
    printoled(0, "no file");
    delay(200);
    //check_back = true;
  }

  // getcolor();
  // white = {r, b};
  // autocalibrate();
}

void loop1() {
  // Serial.print("millis: ");
  // Serial.println(millis());
  if (kit) {
    drop(s, kit);
    kit = 0;
  }
  // if (turning) {
  //   victims(current);
  // }
}

void loop() {
  if (check_back)  //only if no file exists
    check_back_wall();
  current = readinfo.cur;

  //check for obstacle
  if (check_obstacle)
    if (obstacle_avoidance(obstacle_detection(), current))
      current->walls[(((int)NSEW % 360) + 360) % 360 / 90] = true;

  while (1) {
    check_walls();
    current->visited = true;

    tile* goal = nearest_unvisited(current);

    //all tiles visited
    if (goal == nullptr)
      done_routine();

    navtarget(goal, true);
  }
}


// void loop() {
//   blink
// }