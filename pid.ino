float Kp =35, Ki = 0, Kd = 40;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int sen1 = 0, sen2 = 0, sen3 = 0, sen4 = 0, sen5 = 0;
int initial_motor_speed = 150;
int x = 0; //backgriund
int y = 1; //line

void read_sensor_values(void);
void calculate_pid(void);

void setup()
{
  Serial.begin(9600);
  pinMode(3, OUTPUT); //PWM Pin 1
  pinMode(11, OUTPUT); //PWM Pin 2
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);

}

void loop()
{

  read_sensor_values();

  calculate_pid();
 


}

void read_sensor_values()
{
  sen1 = digitalRead(A0);
  sen2 = digitalRead(A1);
  sen3 = digitalRead(A2);
  sen4 = digitalRead(A3);
  sen5 = digitalRead(A4);

  if ((sen1 == x) && (sen2 == x) && (sen3 == x) && (sen4 == x) && (sen5 == y))
    error = 4;

  else if ((sen1 == x) && (sen2 == x) && (sen3 == x) && (sen4 == y) && (sen5 == y))
    error = 3;
  else if ((sen1 == x) && (sen2 == x) && (sen3 == x) && (sen4 == y) && (sen5 == x))
    error = 2;
  else if ((sen1 == x) && (sen2 == x) && (sen3 == y) && (sen4 == y) && (sen5 == x))
    error = 1;

  else if ((sen1 == x) && (sen2 == x) && (sen3 == y) && (sen4 == x) && (sen5 == x))
    error = 0;
 
  else if ((sen1 == x) && (sen2 == y) && (sen3 == y) && (sen4 == x) && (sen5 == x))
    error = -1;
  else if ((sen1 == x) && (sen2 == y) && (sen3 == x) && (sen4 == x) && (sen5 == x))
    error = -2;
  else if ((sen1 == y) && (sen2 == y) && (sen3 == x) && (sen4 == x) && (sen5 == x))
    error = -3;
  else if ((sen1 == y) && (sen2 == x) && (sen3 == x) && (sen4 == x) && (sen5 == x))
    error = -4;

  else if ((sen1 == x) && (sen2 == x) && (sen3 == x) && (sen4 == x) && (sen5 == x))
  {
    if (error == -4)
    {
      error = -5;
    }
    else if (error == 4)
    {
      error = 5;
    }
  }

}

void calculate_pid()
{
  P = error;
  I = error + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
  
  int left_motor_speed = initial_motor_speed + PID_value;
  int right_motor_speed = initial_motor_speed - PID_value;


  left_motor_speed = constrain(left_motor_speed, 0, 250);
  right_motor_speed = constrain(right_motor_speed, 0, 250);

  analogWrite(3, left_motor_speed ); //Left Motor Speed
  analogWrite(11, right_motor_speed); //Right Motor Speed





    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);

}
