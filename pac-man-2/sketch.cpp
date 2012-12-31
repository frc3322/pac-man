// PAC-MAN reactive execution (aka subsumption) robot
//
// The world is the state and the robot directly responds to changes
// in the world--no model or plans are used. The only state used is
// the finite state machine representing behavior transitions.

class Robot {
  private:
    static const int STBY = 10; // motor on/off standby mode

    // Right motor A
    static const int PWMA = 3; // Speed control
    static const int AIN1 = 9; // Direction
    static const int AIN2 = 8; // Direction

// If the right motor has V and GND reversed, uncomment this to
// remove the code reversing motor direction.
//#define LEFT_MOTOR_WIRED_REVERSE 1

    // Left motor B
    static const int PWMB = 5; // Speed control
    static const int BIN1 = 11; // Direction
    static const int BIN2 = 12; // Direction

    static const int LineRight_OTH = 350; // optical threshhold  Off 280  on 890
    static const int LineLeft_OTH = 350; // optical threshhold  off 175 on 870
    static const int TurnRight_OTH = 650; // optical threshhold  off 600 on  810
    static const int TurnLeft_OTH = 650; // optical threshhold off 550 on 810

    static const int MAX_SPEED = 250;
    static const int RIGHT_MAX_SPEED = MAX_SPEED * 0.92;
    static const int LEFT_MAX_SPEED = MAX_SPEED;

  public:
    int t; // current time (msec)
    float dt; // time since last cycle (msec)

    int sensorRight; // photoresistor on pin A0
    int sensorLeft; // photoresistor on pin A1
    int sensorTurnRight; // photoresistor on pin A4
    int sensorTurnLeft; // photoresistor on pin A5

    enum { NOWHERE = 0x00, ON_LEFT = 0x01, ON_RIGHT = 0x02, ON_BOTH_SIDES = 0x03 };
    int line_found; // bit mask: NOWHERE, ON_LEFT, ON_RIGHT, ON_BOTH_SIDES means centered on line
    int intersection_found; // bit mask: as above, ON_BOTH_SIDES means cross road

    float heading; // -2.0 spin left, -1.0 veer left ... 0.0 forward ... 1.0 veer right, 2.0 spin right
    float throttle; // 0.0 stop ... 1.0 full
    float turning; // same as heading, but active only when turning (also hints for search)

    int inhibit_deciding; // number of cycles to inhibit intersection checking
    int activate_turning; // number of cycles to maintain turning

    static void setup_hardware();

    void read_sensors();
    void calculate_change(const Robot &last);
    void detect_line();
    void detect_intersection();
    int direction_of_turn();
    void ignore_intersections_for(int cycles);
    void turn_for(int cycles);

    void write_actuators(const Robot &last);
    void motor_move(int motor, int speed, int direction = 1);
    void motor_start();
    void motor_stop();

    struct {
      int deciding:1;
      int searching:1;
      int following:1;
      int turning:1;
    } behavior;

    void decide_to_turn(const Robot &last);
    void search_for_line(const Robot &last);
    void follow_line(const Robot &last);
    void turn_corner(const Robot &last);
};

void Robot::setup_hardware() {
  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
}

void Robot::read_sensors() {
  t = millis();

  // analogRead shares a single AD converter between all pins and
  // high impedence signals are unreliable to read back-to-back on
  // different pins. delays or double reads can be used to let the
  // hardware settle.

  sensorRight = analogRead(A0);
  sensorRight = analogRead(A0);
  sensorLeft = analogRead(A1);
  sensorLeft = analogRead(A1);

  sensorTurnRight = analogRead(A4);
  sensorTurnRight = analogRead(A4);
  sensorTurnLeft = analogRead(A5);
  sensorTurnLeft = analogRead(A5);

  line_found = 0;
  intersection_found = 0;
}

void Robot::calculate_change(const Robot &last) {
  dt = (t - last.t) + 0.5;
}

void Robot::detect_line() {
  line_found = 0x00;
  if (sensorLeft > LineLeft_OTH) {
    line_found |= 0x01;
  }
  if (sensorRight > LineRight_OTH) {
    line_found |= 0x02;
  }
}

void Robot::detect_intersection() {
  intersection_found = 0x00;
  if (sensorTurnLeft > TurnLeft_OTH) {
    intersection_found |= 0x01;
  }
  if (sensorTurnRight > TurnRight_OTH) {
    intersection_found |= 0x02;
  }
}

int Robot::direction_of_turn() {
  // If on a cross road, 50/50 chance to turn left or right, otherwise just turn
  return (intersection_found == ON_BOTH_SIDES) ? random(ON_LEFT, ON_RIGHT) : intersection_found;
}

void Robot::ignore_intersections_for(int cycles) {
  // Inhibit intersection decisions for N cycles (useful after turning
  // or passing through an intersection)
  inhibit_deciding = cycles;
  behavior.deciding = 0;
}

void Robot::turn_for(int cycles) {
  activate_turning = cycles;
  behavior.turning = 1;
  behavior.following = 0;
}

void Robot::write_actuators(const Robot &last) {
  // Convert the outputs of behaviors into actuator signals.
  // last can be used for deadband optimization so behaviors are simpler.

  if (throttle <= 0.1) {
    heading = 0.0;
    throttle = 0.0;
    motor_stop();
  }
  else {
    float left_throttle = throttle;
    int left_direction = 1;
    float right_throttle = throttle;
    int right_direction = 1;

    if (heading < -0.01) {
      if (heading < -1.0) {
        left_direction = 0;
      }
      else {
        left_throttle *= 1.0 + heading;
      }
    }
    else if (heading > 0.01) {
      if (heading > 1.0) {
        right_direction = 0;
      }
      else {
        right_throttle *= 1.0 - heading;
      }
    }
    else {
      heading = 0.0;
    }

    motor_move(1, LEFT_MAX_SPEED * left_throttle, left_direction);
    motor_move(2, RIGHT_MAX_SPEED * right_throttle, right_direction);
    motor_start();
  }
}

void Robot::motor_move(int motor, int speed, int direction) {
  // Move specific motor at speed and direction
  // motor: 1 for B (left), 2 for A (right)
  // speed: 0 is off, and 255 is full speed
  // direction: 0 reverse, 1 forward

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

#ifndef LEFT_MOTOR_WIRED_REVERSE
  if (motor == 1) {
    direction = !direction;
  }
#endif

  if (direction) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if (motor == 2) {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }
  else {
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

void Robot::motor_start() {
  digitalWrite(STBY, HIGH);
}

void Robot::motor_stop() {
  digitalWrite(STBY, LOW);
}

// Behaviors can suppress or modify inputs, derive new inputs and inhibit
// or activate other behaviors. loop() controls the priority and sequencing of
// behaviors. Behaviors don't directly actuate devices; they only generate
// desired outputs that may be modified by other behaviors.

void Robot::decide_to_turn(const Robot &last) {
  if (intersection_found) {
    turning = (ON_LEFT == direction_of_turn()) ? -0.8 : 0.8;

    // 50/50 chance to turn at the intersection
    if (0 == random(0, 1)) {
      turn_for(20);
    }

    ignore_intersections_for(40);
  }
}

void Robot::search_for_line(const Robot &last) {
  if (line_found) {
    behavior.searching = 0;
    behavior.following = 1;
  }
  else {
    // Hard turn until line found. FIXME: If the line isn't found
    // after N cycles, drive a random direction and repeat. The intersection
    // sensors might find the best direction to drive.
    heading = (turning < 0.0) ? -2.0 : 2.0;
    throttle = 0.75;
  }
}

void Robot::follow_line(const Robot &last) {
  if (line_found || last.line_found) {
    // This will oscillate possibly losing the line. The test against the
    // last line sensor reading is a hack to get back on the line. TODO: It
    // really needs a PID control to get back to a line quicker and damp
    // the oscillation.

    if (line_found) {
      throttle = 1.0;
      heading = (ON_LEFT == line_found) ? -0.08 : (ON_RIGHT == line_found) ? 0.08 : 0.0;
    }
    else {
      throttle = 0.75;
      heading = (ON_LEFT == last.line_found) ? -0.3 : (ON_RIGHT == last.line_found) ? 0.3 : 0.0;
    }

    if (inhibit_deciding == 0) {
      behavior.deciding = 1;
    }
    else if (inhibit_deciding > 0) {
      --inhibit_deciding;
    }
  }
  else {
    behavior.searching = 1;
    behavior.following = 0;
  }
}

void Robot::turn_corner(const Robot &last) {
  // FIXME Turning needs to be callibrated to leave the robot at a nice
  // angle to the line after the turn (so searching will be fast).

  if (activate_turning > 0) {
    --activate_turning;
    heading = turning;
    throttle = 0.5;
  }
  else {
    behavior.turning = 0;
    behavior.searching = 1;
  }
}

Robot robot;
Robot history;

void setup() {
  Serial.begin(115200);
  Serial.println("Robot GO");

  Robot::setup_hardware();

  robot.read_sensors();
  robot.behavior.searching = 1;
}

void loop() {
  history = robot;
  robot.read_sensors();
  robot.calculate_change(history);

  robot.detect_line();
  robot.detect_intersection();

#if 1

  // Multiple behaviors can be active at once. Earlier (priority)
  // behaviors can inhibit later ones.

  if (robot.behavior.deciding) robot.decide_to_turn(history);
  if (robot.behavior.searching) robot.search_for_line(history);
  if (robot.behavior.following) robot.follow_line(history);
  if (robot.behavior.turning) robot.turn_corner(history);

#else

   robot.heading = 0.0;
   robot.throttle = 1.0;

#endif

  robot.write_actuators(history);
}

