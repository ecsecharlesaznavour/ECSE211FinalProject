package classes2;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class MasterBrick {
	
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private EV3UltrasonicSensor frontSensor, rightSensor;
	private SampleProvider Usp1, Usp2;
	private Odometer odo;
	private DualOdometryCorrection odoCor;
	private float[] data;
	
	public MasterBrick(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			EV3UltrasonicSensor frontSensor, EV3UltrasonicSensor rightSensor,
			Odometer odo, DualOdometryCorrection odoCor)
	{
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.frontSensor = frontSensor;
		this.rightSensor = rightSensor;
		this.odo = odo;
		this.odoCor = odoCor;
		this.Usp1 = frontSensor.getDistanceMode();
		this.Usp2 = rightSensor.getDistanceMode();
		this.data = new float[Usp1.sampleSize()];
		this.frontSensor.enable();
		this.rightSensor.disable();
	}
	
	public void doLocalization()
	{
		USDisable(rightSensor);
		USEnable(frontSensor);
		
		double angleA, angleB;
		
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		
		leftMotor.forward();
		rightMotor.backward();
		
		float dist = getUSFilteredData(Usp1,data, 0);
		while((dist = getUSFilteredData(Usp1, data,dist)) < 41);
		while((dist = getUSFilteredData(Usp1, data,dist)) > 40);
		
		angleA = odo.getAng();
		
		leftMotor.backward();
		rightMotor.forward();
		
		while((dist = getUSFilteredData(Usp1, data,dist)) < 41);
		while((dist = getUSFilteredData(Usp1, data,dist)) > 40);
		angleB = odo.getAng();
		
		double delta = 0;
		if(angleB > 180)
			angleB = angleB-360;
		if(angleA > 180)
			angleA = angleA-360;
			
		if(angleA<angleB)
			delta= ((angleA + angleB)/2)-45;
		else
			delta= ((angleA + angleB)/2)-225;
			
		if(delta < 0)
			delta = delta+360;
			
		turnTo((delta)%360);
		
		odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
	}
	
	public void turnTo(double angle)
	{
		double error = angle - this.odo.getAng();
		
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		
		if (error < -180.0) {
			leftMotor.backward();
			rightMotor.forward();
		} else if (error < 0.0) {
			leftMotor.forward();
			rightMotor.backward();
		} else if (error > 180.0) {
			leftMotor.forward();
			rightMotor.backward();
		} else {
			leftMotor.backward();
			rightMotor.forward();
		}
		while(Math.abs(odo.getAng()-angle) > 0.5);
		
		leftMotor.stop(true);
		rightMotor.stop(false);
	}
	
	public void travelTo(double x, double y)
	{
		USEnable(frontSensor);
		USEnable(rightSensor);
		odoCor.Enable();
		
		double X = odo.getX();
		double Y = odo.getY();
		double ang = 0;
		
		//odoCor.setAllow(true);
		
		if(x<X && y>Y)
			ang = 90;
		else if(x>X && y>Y)
			ang = 0;
		else if(x<X && y<Y)
			ang = 180;
		else
			ang = 270;
		
		turnTo(ang);
		
		double dist = getUSFilteredData(Usp1, data, 0);
		
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		
		leftMotor.forward();
		rightMotor.forward();
		
		
		while(Math.abs(x-odo.getX()) > 1 && Math.abs(y - odo.getY()) > 1)
		{
			if(dist<30)
				Avoid();
			
			dist = getUSFilteredData(Usp1, data, dist);
		}
		
		ang += 90;
		if(ang>360)
			ang-=360;
		
		turnTo(ang);
		
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		
		leftMotor.forward();
		rightMotor.forward();
		
		while(!(Math.abs(x-odo.getX()) < 1 && Math.abs(y - odo.getY()) < 1))
		{
			dist = getUSFilteredData(Usp1, data, dist);
			
			if(dist<30)
				Avoid();
		}
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		if(Math.pow(x - odo.getX(), 2) + Math.pow(y - odo.getY(), 2) > 2)
			travelTo(x,y);
	}
	
	public void Avoid()
	{
		double ang = odo.getAng()+90;
		
		if(ang>359)
			ang-=360;
		turnTo(ang);
		
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.forward();
		rightMotor.forward();
		
		double dist = getUSFilteredData(Usp2, data, 0);
		
		while((dist = getUSFilteredData(Usp2, data, dist))>60);
		while((dist = getUSFilteredData(Usp2, data, dist))<60);
		
		try{Thread.sleep(2000);} catch(Exception e){}
		
		ang-= 90;
		if(ang<0)
			ang+=360;
		turnTo(ang);
		
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.forward();
		rightMotor.forward();
	}
	
	private float getUSFilteredData(SampleProvider sp, float[] data, double last)
	{
		sp.fetchSample(data, 0);
		float newDist = data[0]*100;
		int Filter = 0;
		
		if(last == 0)
			return newDist;
		else
		{
			if(Math.abs(last-newDist) > 15)
			{
				while(Filter < 15 && Math.abs(last-newDist) > 15)
				{
					Filter++;
					sp.fetchSample(data, 0);
					newDist = data[0]*100;
					try{Thread.sleep(25);} catch(Exception e) {}
				}
			}
		}
		
		return newDist;
	}
	
	public void USEnable(EV3UltrasonicSensor sensor)
	{
		sensor.enable();
	}
	
	public void USDisable(EV3UltrasonicSensor sensor)
	{
		sensor.disable();
	}

}
