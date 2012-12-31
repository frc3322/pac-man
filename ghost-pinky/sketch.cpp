// PINKY GHOST reactive execution (aka subsumption) robot
//
// The world is the state and the robot directly responds to changes
// in the world--no model or plans are used. The only state used is
// the finite state machine representing behavior transitions.

class Robot {
  private:
    static const int STBY = 7; // motor on/off standby mode

    // Left motor A
    static const int PWMA = 9; // Speed control
    static const int AIN1 = 10; // Direction
    static const int AIN2 = 8; // Direction

// If the left motor has V and GND reversed, uncomment this to
// remove the code reversing motor direction.
#define LEFT_MOTOR_WIRED_REVERSE 1

    // Right motor B
    static const int PWMB = 11; // Speed control
    static const int BIN1 = 12; // Direction
    static const int BIN2 = 13; // Direction

    static const int FB = 3;
    static const int LB = 4;
    static const int RB = 2;
    static const int BB = 6;

    static const int MAX_SPEED = 250;
    static const int RIGHT_MAX_SPEED = MAX_SPEED;
    static const int LEFT_MAX_SPEED = MAX_SPEED;

  public:
    int t; // current time (msec)
    float dt; // time since last cycle (msec)

    int bumpedFront;
    int bumpedLeft;
    int bumpedRight;
    int bumpedBack;

    float heading; // -2.0 spin left, -1.0 veer left ... 0.0 forward ... 1.0 veer right, 2.0 spin right
    float throttle; // 0.0 stop ... 1.0 full
    float turning; // same as heading, but active only when turning (also hints for search)

    int deactivate_turning; // time to stop turning

    static void setup_hardware();

    void read_sensors();
    void calculate_change(const Robot &last);
    int direction_of_turn();
    void turn_for(int cycles);

    void write_actuators(const Robot &last);
    void motor_move(int motor, int speed, int direction = 1);
    void motor_start();
    void motor_stop();

    struct {
      int driving:1;
      int reversing:1;
      int turning:1;
    } behavior;

    void drive_forward(const Robot &last);
    void drive_backward(const Robot &last);
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

  bumpedFront = digitalRead(FB);
  bumpedLeft = digitalRead(LB);
  bumpedRight = digitalRead(RB);
  bumpedBack = digitalRead(BB);
}

void Robot::calculate_change(const Robot &last) {
  dt = (t - last.t) + 0.5;
}

int Robot::direction_of_turn() {
  return 0;
}

void Robot::turn_for(int cycles) {
  deactivate_turning = t + cycles;
  behavior.turning = 1;
  behavior.driving = 0;
}

void Robot::write_actuators(const Robot &last) {
  // Convert the outputs of behaviors into actuator signals.
  // last can be used for deadband optimization so behaviors are simpler.

  int reverse = 0;
  if (throttle < 0.0) {
    throttle = -throttle;
    reverse = 1;
  }

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

    motor_move(1, LEFT_MAX_SPEED * left_throttle, (reverse) ? !left_direction : left_direction);
    motor_move(2, RIGHT_MAX_SPEED * right_throttle, (reverse) ? !right_direction : right_direction);
    motor_start();
  }
}

void Robot::motor_move(int motor, int speed, int direction) {
  // Move specific motor at speed and direction
  // motor: 1 for B (left), 2 for A (right)
  // speed: 0 is off, and 255 is full speed
  // direction: 0 reverse, 1 forward

  boolean inPin1 = HIGH;
  boolean inPin2 = LOW;

#ifndef LEFT_MOTOR_WIRED_REVERSE
  if (motor == 1) {
    direction = !direction;
  }
#endif

  if (direction) {
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if (motor == 1) {
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

void Robot::drive_forward(const Robot &last) {
  heading = 0.0;
  throttle = 1.0;
  
  if (bumpedFront) {
    if (random(0, 1000) < 250) {
      turn_for(1750);
    }
    else {
      behavior.reversing = 1;
    }
  }
  if (bumpedLeft) {
    heading = 0.5;
  }
  if (bumpedRight) {
    heading = -0.5;
  }
}

void Robot::drive_backward(const Robot &last) {
  heading = 0.0;
  throttle = -1.0;
  if (bumpedBack) {
      behavior.driving = 1;
    }
}

void Robot::turn_corner(const Robot &last) {
  if (deactivate_turning > t) {
    heading = 2.0;
    throttle = 0.5;
  }
  else {
    behavior.turning = 0;
    behavior.driving = 1;
  }
}

Robot robot;
Robot history;

void setup() {
  Serial.begin(115200);
  Serial.println("Robot GO");

  Robot::setup_hardware();

  robot.read_sensors();
  robot.behavior.driving = 1;
}

void loop() {
  history = robot;
  robot.read_sensors();
  robot.calculate_change(history);

#if 1

  // Multiple behaviors can be active at once. Earlier (priority)
  // behaviors can inhibit later ones.

  if (robot.behavior.driving) robot.drive_forward(history);
  if (robot.behavior.reversing) robot.drive_backward(history);
  if (robot.behavior.turning) robot.turn_corner(history);

#else

   robot.heading = 0.0;
   robot.throttle = 0.0;

#endif

  robot.write_actuators(history);
}

