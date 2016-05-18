void chooseRoute(int buttonA = leftBtnVEX,int buttonB = centerBtnVEX,int buttonC = rightBtnVEX)
{
  while(true)
  {
    if(nLCDButtons == buttonA)
    {
    	autoRoute = 0;
      break;
      wait1Msec(1);
    }
    else if(nLCDButtons == buttonB)
    {
    	autoRoute = 1;
      break;
      wait1Msec(1);
    }
    else if(nLCDButtons == buttonC)
    {
    	autoRoute = 2;
    	break;
    	wait1Msec(1);
    }
    else if(SensorValue[AutoButton] == 1)
    {
    	autoRoute = 3;
    	break;
    	wait1Msec(1);
    }
    else if(SensorValue[AutoButton2] == 1)
    {
    	autoRoute = 4;
    	break;
    	wait1Msec(1);
    }
    wait1Msec(1);
  }
}
void clearDriveEncoders()
{
	SensorValue[LeftEncoder] = 0;
	SensorValue[RightEncoder] = 0;
	LDrivePower = 0;
	RDrivePower = 0;
}


//int convertI2C (int I2CValue)
//{
//		float QuadEncoderValue;
//		QuadEncoderValue = I2CValue * 627.2 / 360;
//		return (int) QuadEncoderValue;
//}

void Intake(int power)
{
	motor[RightIntake] = motor[LeftIntake] = power;
}

////int mySensorValue(tSensors sensorName)
////{
////	if(sensorName == EncoderFrontLeft || sensorName == EncoderFrontRight)
////	{
////		float QuadEncoderValue;
////		QuadEncoderValue = SensorValue[sensorName] * 627.2 / 360;
////		return (int) QuadEncoderValue;

////	}
////	else
////	{
////		return SensorValue[sensorName];
////	}

////}
////// Ensure motor power is within -127 and 127 range
int motorCap(int motorPowervalue)
{

	if (motorPowervalue >= 127)
	{
		return (127);
	}
	else if (motorPowervalue <= -127)
	{
		return (-127);
	}
	return (motorPowervalue);
}
void drive(int forwardTicks, int TargetPower = 50, int wheelBrake = -10 ,int xtime = 15000)
{
	clearTimer(T1);
	SensorValue[LeftEncoder] = SensorValue[RightEncoder] = 0;
	while(((abs(SensorValue[RightEncoder]) + abs(SensorValue[LeftEncoder])) / 2) < forwardTicks && time1[T1] < xtime)
	{
		int Adjustment = ((SensorValue[RightEncoder]) - (SensorValue[LeftEncoder])) * 1.5;
		RDrivePower = TargetPower - Adjustment;
		LDrivePower = TargetPower + Adjustment;
	}
	RDrivePower = LDrivePower = wheelBrake;

}


void turnDrive(int forwardTicks, int speed = 50, int turndirection = 8, int extraPower = 20, int wheelBrake = -10 ,int xtime = 15000)
{
	clearTimer(T1);
	SensorValue[LeftEncoder] = SensorValue[RightEncoder] = 0;
	while(((abs(SensorValue[RightEncoder]) + abs(SensorValue[LeftEncoder])) / 2) < forwardTicks && time1[T1] < xtime)
	{
		if(turndirection == 8)
		{
			RDrivePower = speed ;
			LDrivePower = (speed + extraPower);

		}
		else if (turndirection == 9)
		{
			RDrivePower = (speed + extraPower);
			LDrivePower = speed;
		}
	}
	RDrivePower = LDrivePower = wheelBrake;

}


void driveADP(int direction = 1 ,int desired, int speed = 50, int wheelBrake = -10 ,int xtime = 15000, float Kd = 0.30, float Ka = 0.45)
{
	clearTimer(T1);
	float pSpeed = 0;
	SensorValue[LeftEncoder] = SensorValue[RightEncoder] = 0;
	while(((abs(SensorValue[RightEncoder]) + abs(SensorValue[LeftEncoder])) / 2) < desired && time1[T1] < xtime)
	{
		if(((abs(SensorValue[RightEncoder]) + abs(SensorValue[LeftEncoder])) / 2) < (desired*0.5))
		{
			pSpeed = ((((abs(SensorValue[RightEncoder]) + abs(SensorValue[LeftEncoder])) / 2))*Ka);
			PIDvalue = pSpeed;
			int Adjustment = ((SensorValue[RightEncoder]) - (SensorValue[LeftEncoder])) * 1.0;
			if (abs(pSpeed+minMovePower)>speed)
			{

			RDrivePower =((direction > 0)? speed - Adjustment: -speed + Adjustment);
			LDrivePower =((direction > 0)? speed + Adjustment: -speed - Adjustment);
			}
			else
			{
				RDrivePower = (direction*minMovePower) + (direction*pSpeed) - Adjustment;
				LDrivePower = (direction*minMovePower) + (direction*pSpeed) + Adjustment;
			}

		}
		else
		{
			pSpeed = ((desired - ((abs(SensorValue[RightEncoder]) + abs(SensorValue[LeftEncoder])) / 2))*Kd);
			PIDvalue = pSpeed;
			int Adjustment = ((SensorValue[RightEncoder]) - (SensorValue[LeftEncoder])) * 1.0;
	        if (abs(pSpeed+minMovePower)>speed)
			{

			RDrivePower =((direction > 0)? speed - Adjustment: -speed + Adjustment);
			LDrivePower =((direction > 0)? speed + Adjustment: -speed - Adjustment);
			}
			else
			{
				RDrivePower = (direction*minMovePower) + (direction*pSpeed) - Adjustment;
				LDrivePower = (direction*minMovePower) + (direction*pSpeed) + Adjustment;
			}

		}
	}
	RDrivePower = LDrivePower = direction*wheelBrake;
}

void driveAP(int direction = 1, int desired, int speed = 50, int wheelBrake = -10 ,int xtime = 15000, float Ka = 0.45)
{
	clearTimer(T1);
	float pSpeed = 0;
	SensorValue[LeftEncoder] = SensorValue[RightEncoder] = 0;
	while(((abs(SensorValue[RightEncoder]) + abs(SensorValue[LeftEncoder])) / 2) < desired && time1[T1] < xtime)
	{
		pSpeed = ((((abs(SensorValue[RightEncoder]) + abs(SensorValue[LeftEncoder])) / 2))*Ka);
		PIDvalue = pSpeed;
		int Adjustment = (SensorValue[RightEncoder] - SensorValue[LeftEncoder]) * 1.0;
	if (abs(pSpeed+minMovePower)>speed)
			{

	   	RDrivePower =((direction > 0)? speed - Adjustment: -speed + Adjustment);
			LDrivePower =((direction > 0)? speed + Adjustment: -speed - Adjustment);
			}
			else
			{
				RDrivePower = (direction*minMovePower) + (direction*pSpeed) - Adjustment;
				LDrivePower = (direction*minMovePower) + (direction*pSpeed) + Adjustment;
			}
	}
	RDrivePower = LDrivePower = direction*wheelBrake;
}

void driveDP(int direction = 1, int desired, int speed = 50, int wheelBrake = -10 ,int xtime = 15000 , float K = 0.25)
{
	clearTimer(T1);
	float pSpeed = 0;
	SensorValue[LeftEncoder] = SensorValue[RightEncoder] = 0;
	while(((abs(SensorValue[RightEncoder]) + abs(SensorValue[LeftEncoder])) / 2) < desired && time1[T1] < xtime)
	{
		pSpeed = ((desired - ((abs(SensorValue[RightEncoder]) + abs(SensorValue[LeftEncoder])) / 2))*K);
		PIDvalue = pSpeed;
		int Adjustment = ((SensorValue[RightEncoder]) - (SensorValue[LeftEncoder])) * 1.0;
	if (abs(pSpeed+minMovePower)>speed)
			{

			 RDrivePower =((direction > 0)? speed - Adjustment: -speed + Adjustment);
			 LDrivePower =((direction > 0)? speed + Adjustment: -speed - Adjustment);
			}
			else
			{
				RDrivePower = (direction*minMovePower) + (direction*pSpeed) - Adjustment;
				LDrivePower = (direction*minMovePower) + (direction*pSpeed) + Adjustment;
			}

	}
	RDrivePower = LDrivePower = wheelBrake*direction;

}


//void driveUntilLineFront(int TargetPower = 50, int wheelBrake = -10,int xtime = 15000 )
//{
//	clearTimer(T1);
//	SensorValue[LeftEncoder] = SensorValue[RightEncoder] = 0;
//	while( SensorValue[LineFrontLeft] > threshold && SensorValue[LineFrontRight] > threshold && time1[T1] < xtime)
//	{
//		int Adjustment = ((SensorValue[RightEncoder]) - (SensorValue[LeftEncoder])) * 1.0;
//		RDrivePower = TargetPower - Adjustment;
//		LDrivePower = TargetPower + Adjustment;
//	}
//	RDrivePower = LDrivePower = wheelBrake;
//}

//void driveUntilLineBack(int TargetPower = 50, int wheelBrake = -10,int xtime = 15000 )
//{
//	clearTimer(T1);
//	SensorValue[LeftEncoder] = SensorValue[RightEncoder] = 0;
//	while( SensorValue[LineBackLeft] > threshold && SensorValue[LineBackRight] > threshold && time1[T1] < xtime)
//	{
//		int Adjustment = ((SensorValue[RightEncoder]) - (SensorValue[LeftEncoder])) * 1.25;
//		RDrivePower = TargetPower - Adjustment;
//		LDrivePower = TargetPower + Adjustment;
//	}
//	RDrivePower = LDrivePower = wheelBrake;
//}

/*void driveLine(int forwardTicks, int wheelBrake = 0.1, int TargetPower = 50, int xtime = 15000)
{
clearTimer(T1);
SensorValue[LeftEncoder] = SensorValue[RightEncoder] = 0;
while(((abs(SensorValue[RightEncoder]) + abs(SensorValue[LeftEncoder])) / 2) < forwardTicks < threshold && time1[T1] < xtime)
{
//follow line	straight
while((SensorValue[LineMiddle] < threshold && SensorValue[LineLeft] & SensorValue[LineRight] > threshold)||(SensorValue[LineMiddle] & SensorValue[LineLeft] < threshold && SensorValue[LineRight] < threshold))
{
int Adjustment = ((SensorValue[RightEncoder]) - (SensorValue[LeftEncoder])) * 1.25;
RDrivePower = TargetPower;
LDrivePower = TargetPower;
}
//adjust to right
while(SensorValue[LineMiddle] < threshold && SensorValue[LineLeft] > threshold && SensorValue[LineRight] < threshold)
{
int Adjustment = ((SensorValue[RightEncoder]) - (SensorValue[LeftEncoder])) * 1.25;
RDrivePower = TargetPower - 10;
LDrivePower = TargetPower + 10;
}
//adjust to left
while(SensorValue[LineMiddle] < threshold && SensorValue[LineLeft] < threshold && SensorValue[LineRight] > threshold)
{
int Adjustment = ((SensorValue[RightEncoder]) - (SensorValue[LeftEncoder])) * 1.25;
RDrivePower = TargetPower + 10;
LDrivePower = TargetPower - 10;
}
//adjust to left
while(SensorValue[LineMiddle] > threshold && SensorValue[LineLeft] > threshold && SensorValue[LineRight] < threshold)
{
int Adjustment = ((SensorValue[RightEncoder]) - (SensorValue[LeftEncoder])) * 1.25;
RDrivePower = TargetPower - 20;
LDrivePower = TargetPower + 20;
}
//adjust to right
while(SensorValue[LineMiddle] > threshold && SensorValue[LineLeft] < threshold && SensorValue[LineRight] > threshold)
{
int Adjustment = ((SensorValue[RightEncoder]) - (SensorValue[LeftEncoder])) * 1.25;
RDrivePower = TargetPower + 20;
LDrivePower = TargetPower - 20;
}


}
RDrivePower = LDrivePower = wheelBrake*TargetPower;
}
*/

//void timedMove(int speed, int time) { // Move Forward by time
//  wheelLeftPower = speed;
//  wheelRightPower = speed;
//  wait10Msec(time);
//  wheelLeftPower = 0;
//  wheelRightPower = 0;
//}


void turnDP(int turndirection , int encoderticks, int speed = 50, float K = 0.35)
{
	SensorValue[RightEncoder] = 0;
	SensorValue[LeftEncoder] = 0;
	float pSpeed = 0;
	if(turndirection == 8) //RIGHT
	{
		while(abs(SensorValue[LeftEncoder]) < encoderticks)
		{
			pSpeed = ((encoderticks - (SensorValue[LeftEncoder]))*K);
			if (abs(pSpeed+minMovePower)>speed)
			{
			RDrivePower =  -speed;
			LDrivePower =  +speed;
			}
			else
			{
				RDrivePower = -minMovePower - pSpeed;
				LDrivePower = minMovePower + pSpeed;
			}
		}
		RDrivePower = 10;
		LDrivePower = -10;
	}
	else if (turndirection == 9) //LEFT
	{
		while(abs(SensorValue[RightEncoder]) < encoderticks)
		{
			pSpeed = ((encoderticks - (SensorValue[RightEncoder]))*K);
			if (abs(pSpeed+minMovePower)> speed)
			{
			LDrivePower = -speed;
			RDrivePower = speed;
			}
			else
			{
				LDrivePower = -minMovePower - pSpeed;
				RDrivePower = minMovePower + pSpeed;
			}
	   }
		LDrivePower = -10;
		RDrivePower = 10;
	}
}












void turn(int turndirection , int encoderticks, int speed, int slowspeed  = -10)
{
	SensorValue[RightEncoder] = 0;
	SensorValue[LeftEncoder] = 0;

	if(turndirection == 8)
	{
		while(abs(SensorValue[LeftEncoder]) < encoderticks)
		{
			RDrivePower = -speed;
			LDrivePower = speed;
		}
		RDrivePower = 10;
		LDrivePower = -10;


	}
	else if (turndirection == 9)
	{
		while(abs(SensorValue[RightEncoder]) < encoderticks)
		{
			RDrivePower = speed;
			LDrivePower = -speed;
		}
		RDrivePower = -10;
		LDrivePower = 10;
	}
	else if (turndirection == 10) //swingLEFT
	{
		while(abs(SensorValue[LeftEncoder]) < encoderticks)
		{
			RDrivePower = slowspeed;
			LDrivePower = speed;
		}
		RDrivePower = -0.1*slowspeed;
		LDrivePower = -0.1*speed;
	}
	else if (turndirection == 11) //swingRIGHT
	{
		while(abs(SensorValue[RightEncoder]) < encoderticks)
		{
			RDrivePower = speed;
			LDrivePower = slowspeed;
		}
	  LDrivePower = -0.1*slowspeed;
		RDrivePower = -0.1*speed;
	}

}











//void correctWheels (int time) { // If robot is tilted to one side, this functions straightens the robot
//  int timer = 0;
//  if (SensorValue[LeftEncoder] > SensorValue[RightEncoder]) {
//	  while (SensorValue[LeftEncoder] > SensorValue[RightEncoder] && timer < time) {
//	  LDrivePower = -17;
//	  RDrivePower = 17;
//	  wait1Msec(10);
//	  timer += 1;
//	  }
//	}
//	else if (SensorValue[LeftEncoder] < SensorValue[RightEncoder]) {
//	  while (SensorValue[LeftEncoder] < SensorValue[RightEncoder] && timer < time) {
//	    LDrivePower = 30;
//	    RDrivePower =  -30;
//	    wait1Msec(10);
//	    timer += 1;
//	  }
//  }
//  motor[BackDriveLOne] = motor[BackDriveLTwo] = motor[FrontDriveL] = 0;
//  motor[BackDriveROne] = motor[BackDriveRTwo] = motor[FrontDriveR] = 0;
//}









void wiggle(int times)
{ // JIGGLE!!! Moves robot forward and back, forward and back
	int timesCounter =0 ;
	int wiggledir = -1;
	while(timesCounter < times)
	{
		if (wiggledir > 0) { // Moves forward for 130ms
			RDrivePower = 45;
			LDrivePower = 45;
			wait1Msec(170);
			timesCounter ++;
			wiggledir *= -1; // Switch direction
		}
		else if (wiggledir < 0) { // Moves backward for 130ms
			RDrivePower = -45;
			LDrivePower= -45;
			wait1Msec(170);
			timesCounter ++;
			wiggledir *= -1; // Switch direction
		}
	}
	LDrivePower =0;
	RDrivePower = 0;
}
void shootFunction()
{
	TrayState = 1;
	wait1Msec(200);
	TrayState = 0;
}
void autoInit()
{
  clearDriveEncoders();
	armstate=0;
	Intake(-127);
	wait1Msec(400);
	Intake(0);
}
