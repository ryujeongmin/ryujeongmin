#include <Servo.h>
#include <Pixy2.h>
#include <math.h>
double find_obj();
void get_angles(double sensed);
void waiting();
void arm_move_forward(int theta1, int theta2, int theta3, int delay_time);
void arm_move_backward(int delay_time);
void gripper_move(int theta, int delay_time);
void wrist_move(int theta, int delay_time);
void base_move(int theta, int delay_time);
bool sensor();

//9V로 다 가능
//가운데: 158 / 104

Servo SG90_1;
Servo SG90_2;
Servo SG90_3;
Servo MG996R_1;
Servo MG996R_2;
Servo MG996R_3;
Pixy2 pixy;

int SG90_1_angle = 90;
int SG90_2_angle = 180;
int SG90_3_angle = 120;
int MG996R_1_angle = 130;
int MG996R_2_angle = 130;
int MG996R_3_angle = 90;


void setup() {
  pixy.init();
  Serial.begin(9600);
  SG90_1.attach(9);
  SG90_2.attach(10);
  //SG90_3.attach(9);
  MG996R_1.attach(6);
  MG996R_2.attach(5);
  MG996R_3.attach(3);

  Serial.print("Starting...\n");

  SG90_1.write(90);
  SG90_2.write(180);
  //SG90_3.write(SG90_3_angle);
  MG996R_1.write(130);
  MG996R_2.write(130);
  MG996R_3.write(90);
   
  Serial.println("Done positioning\n");
}

double theta3;
double theta2;
double theta1;

void loop()
{   sensor();
    if (sensor==1)
    {
      waiting();
      Serial.println("Start detecting");
      double sensed = find_obj(); //거리
      Serial.print("distance is ");
      Serial.println(sensed);
      delay(500);
      get_angles(sensed);
      delay(500);

      Serial.println("Start motor");
      arm_move_forward((int)theta3, (int)theta2, (int)theta1, 30);
      delay(3000);
      
      gripper_move(30, 15);
      delay(2000);

      Serial.print(SG90_1_angle);
      Serial.print(" ");
      Serial.print(SG90_2_angle);
      Serial.print(" ");
      Serial.print(MG996R_1_angle);
      Serial.print(" ");
      Serial.print(MG996R_2_angle);
      Serial.print(" ");
      Serial.println(MG996R_3_angle);
      
      Serial.println("arm move backward start");
      arm_move_backward(30);
      //SG90_1.write(150);
      Serial.println("End motor");
      delay(1000);
    }
  }


double find_obj()
{
  double pixy_distance;
  pixy.ccc.getBlocks();  
  
  while (abs(pixy.ccc.blocks[0].m_x - 165) >= 3)
  {
    pixy.ccc.getBlocks();  
    Serial.println(pixy.ccc.blocks[0].m_x);
    if(pixy.ccc.blocks[0].m_x < 165)
    {
      if (MG996R_3_angle != 180)
      {
        MG996R_3_angle++;
      }
    }
    else
    {
      if (MG996R_3_angle != 0)
      {
        MG996R_3_angle--;
      }
    }
    MG996R_3.write(MG996R_3_angle);
    delay(50);
  }
  
  Serial.println("Stop");
  double size = (double)pixy.ccc.blocks[0].m_height*pixy.ccc.blocks[0].m_width/(105*82);
  Serial.print("distance = ");
  pixy_distance = 15/sqrt(size);
  Serial.println(pixy_distance);
  Serial.print("x = ");
  Serial.println(pixy.ccc.blocks[0].m_x);
  
  return pixy_distance;
}

void get_angles(double sensed)
{
  double x = 3;
  double y = 9.58 - 8;
  double d = sensed - x - 5;
  
  double D = sqrt((d + x)*(d + x) + y*y);
  double A = atan((x+d)/y);
  double B = acos(D/24.0);
  double C = acos((12*12 + 12*12 - D*D)/(2.0*12.0*12.0));

  Serial.print("A = ");
  Serial.print(A);
  Serial.print(" B = ");
  Serial.print(B);
  Serial.print(" C = ");
  Serial.println(C);
  
  theta1 = 180/3.141592*(A + B - 3.141592/2) - 10;
  theta2 = 180/3.141592*(8.0/9.0*3.141592 - C);
  theta3 = 180/3.141592*(3.141592/2 + A - B) + 50;

  Serial.print("theta1 = ");
  Serial.print(theta1);
  Serial.print(" theta2 = ");
  Serial.print(theta2);
  Serial.print(" theta3 = ");
  Serial.println(theta3);
}

void waiting()
{
  gripper_move(100, 30); //sg90_1
  arm_move_forward(180, 130, 130, 30); //sg90_2, mg996R_1, mg996R_2
  //wrist_move(120, 50); // sg90_3
  base_move(90, 30); //mg996R_3
  Serial.println("Done waiting\n");
  delay(1000);
}

void arm_move_forward(int theta_3, int theta_2, int theta_1, int delay_time)
{
  Serial.println("-------------------------arm_move start----------------------");
  
  Serial.println("MG996R_1");
  //MG996R_1 (theta2)
  while(abs(MG996R_1_angle-theta_2) > 1)
  {
    if (MG996R_1_angle > theta_2)
      MG996R_1_angle--;
    else
      MG996R_1_angle++;

    MG996R_1.write(MG996R_1_angle);
    delay(delay_time);
  }

  Serial.println("MG996R_2");
  //MG996R_2 (theta1)
  while(abs(MG996R_2_angle-theta_1) > 1)
  {
    if (MG996R_2_angle > theta_1)
      MG996R_2_angle--;
    else
      MG996R_2_angle++;

    MG996R_2.write(MG996R_2_angle);
    delay(delay_time);
  }

  Serial.println("SG90_2");
  //SG90_2 (theta3)
  while(abs(SG90_2_angle-theta_3) > 1)
  {
    if (SG90_2_angle > theta_3)
      SG90_2_angle--;
    else
      SG90_2_angle++;
    
    SG90_2.write(SG90_2_angle);
    delay(delay_time);
  }
  
  
  Serial.println("-------------------------arm forward end-------------------------");
}

void arm_move_backward(int delay_time)
{
  Serial.println("-------------------------arm move backward start-------------------------");
  Serial.println("Step 1");
  // 각 모터들 변화량(양수)
  int dtheta2 = MG996R_1_angle - MG996R_2_angle;
  int dtheta3 = SG90_2_angle - 90;
  int loop_num;
  Serial.print("dtheta2 = ");
  Serial.print(dtheta2);
  Serial.print(" dtheta3 = ");
  Serial.println(dtheta3);
  if (dtheta2 < dtheta3)
  {
    int quotient = dtheta3/dtheta2;
    int rem = dtheta3%dtheta2;
    Serial.print(quotient);
    Serial.print(" ");
    Serial.println(rem);
    for (int i = 0; i < dtheta2 - rem; i++)
    {
      MG996R_1_angle = MG996R_1_angle - 1;
      SG90_2_angle = SG90_2_angle - quotient;
      SG90_2.write(SG90_2_angle);
      MG996R_1.write(MG996R_1_angle);
      Serial.print(MG996R_1_angle);
      Serial.print(" ");
      Serial.println(SG90_2_angle);
      delay(delay_time);
    }
    for (int i = 0; i < rem; i++)
    {
      MG996R_1_angle = MG996R_1_angle - 1;
      SG90_2_angle = SG90_2_angle - (quotient + 1);
      SG90_2.write(SG90_2_angle);
      MG996R_1.write(MG996R_1_angle);
      Serial.print(MG996R_1_angle);
      Serial.print(" ");
      Serial.println(SG90_2_angle);
      delay(delay_time);
    }
  }
  else
  {
    int quotient = dtheta2/dtheta3;
    int rem = dtheta2%dtheta3;
    for (int i = 0; i < dtheta3 - rem; i++)
    {
      MG996R_1_angle = MG996R_1_angle - quotient;
      SG90_2_angle = SG90_2_angle - 1;
      SG90_2.write(SG90_2_angle);
      MG996R_1.write(MG996R_1_angle);
      Serial.print(MG996R_1_angle);
      Serial.print(" ");
      Serial.println(SG90_2_angle);
      delay(delay_time);
    }
    for (int i = 0; i < rem; i++)
    {
      MG996R_1_angle = MG996R_1_angle - (quotient + 1);
      SG90_2_angle = SG90_2_angle - 1;
      SG90_2.write(SG90_2_angle);
      MG996R_1.write(MG996R_1_angle);
      Serial.print(MG996R_2_angle);
      Serial.print(" ");
      Serial.println(SG90_2_angle);
      delay(delay_time);
    }
  }

  Serial.println("Step 2");
  int dtheta1 = (180 - MG996R_2_angle) - MG996R_2_angle;
  dtheta2 = MG996R_1_angle + 20;
  loop_num;
  Serial.print("dtheta1 = ");
  Serial.print(dtheta1);
  Serial.print(" dtheta2 = ");
  Serial.println(dtheta2);
  if (dtheta1 < dtheta2)
  {
    int quotient = dtheta2/dtheta1;
    int rem = dtheta2%dtheta1;
    Serial.print(quotient);
    Serial.print(" ");
    Serial.println(rem);
    for (int i = 0; i < dtheta1 - rem; i++)
    {
      MG996R_2_angle = MG996R_2_angle + 1;
      MG996R_1_angle = MG996R_1_angle + quotient;
      MG996R_1.write(MG996R_1_angle);
      MG996R_2.write(MG996R_2_angle);
      Serial.print(MG996R_2_angle);
      Serial.print(" ");
      Serial.println(MG996R_1_angle);
      delay(delay_time);
    }
    for (int i = 0; i < rem; i++)
    {
      MG996R_2_angle = MG996R_2_angle + 1;
      MG996R_1_angle = MG996R_1_angle + (quotient + 1);
      MG996R_1.write(MG996R_1_angle);
      MG996R_2.write(MG996R_2_angle);
      Serial.print(MG996R_2_angle);
      Serial.print(" ");
      Serial.println(MG996R_1_angle);
      delay(delay_time);
    }
  }
  else
  {
    int quotient = dtheta1/dtheta2;
    int rem = dtheta1%dtheta2;
    for (int i = 0; i < dtheta2 - rem; i++)
    {
      MG996R_2_angle = MG996R_2_angle + quotient;
      MG996R_1_angle = MG996R_1_angle + 1;
      MG996R_1.write(MG996R_1_angle);
      MG996R_2.write(MG996R_2_angle);
      Serial.print(MG996R_2_angle);
      Serial.print(" ");
      Serial.println(MG996R_1_angle);
      delay(delay_time);
    }
    for (int i = 0; i < rem; i++)
    {
      MG996R_2_angle = MG996R_2_angle + (quotient + 1);
      MG996R_1_angle = MG996R_1_angle + 1;
      MG996R_1.write(MG996R_1_angle);
      MG996R_2.write(MG996R_2_angle);
      Serial.print(MG996R_2_angle);
      Serial.print(" ");
      Serial.println(MG996R_1_angle);
      delay(delay_time);
    }
  }
  Serial.println("End");
}

void gripper_move(int theta, int delay_time)
{
  Serial.println("gripper_move start");
  //SG90_1 
  while(abs(SG90_1_angle-theta) > 1)
  {
    if (SG90_1_angle > theta)
      SG90_1_angle--;
    else
      SG90_1_angle++;

    SG90_1.write(SG90_1_angle);
    //Serial.println(SG90_1_angle);
    delay(delay_time);
  }
  Serial.println("gripper_move end");
}

void wrist_move(int theta, int delay_time)
{
  Serial.println("wrist_move start");
  //SG90_3
  while(abs(SG90_3_angle-theta) > 1)
  {
    if (SG90_3_angle > theta)
      SG90_3_angle--;
    else
      SG90_3_angle++;

    SG90_3.write(SG90_3_angle);
    delay(delay_time);
  }
}

void base_move(int theta, int delay_time)
{
  Serial.println("base_move start");
  while(abs(MG996R_3_angle-theta) > 1)
  {
    if (MG996R_3_angle > theta)
      MG996R_3_angle--;
    else
      MG996R_3_angle++;

    MG996R_3.write(MG996R_3_angle);
    delay(delay_time);
  }
}

bool sensor()
{
  int distance=0;
  int volt=map(analogRead(11),0,1023,0,5000);
  float distacne=(27.61/(volt-0.1696))*1000;
  Serial.println(distance);
  
  if(distance<5)
  return 1;

  else
  return 0;

  
  }
