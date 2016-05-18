mode = 0;
L_sensor = 0;
R_sensor = 0;

void loop()
{
	if(btn == LOW)
	{
		selectMode(); // choose mode
	}
	else
	{
		switch(mode)
		{
		case(1):
			while(btn = HIGH)
			{
				lineFollow();
				updateButton();
			}
			break;
			
		case(2):
			while(btn = HIGH)
			{
				int distance = getUltrasonic();
				motorSpeed = getMotorSpeed(distance);
				driveStraight(motorSpeed, L_sensor, R_sensor);
				if(distance < distanceThreshold)
				{
					int dir = scanSurroundings();
					turn(dir, 500);
				}
				updateButton();
			}
			break;
				
		case(3):
			while(btn = HIGH)
			{
				checkSensors();
				clarencesShit();
			}
			break;
			
		case(4):
			while(btn = HIGH)
			{
				IRSensor();
				updateButton();
			}
			break;
			
		}
	}
}