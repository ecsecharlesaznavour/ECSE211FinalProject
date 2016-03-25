package classes2;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class MasterBrick {
	
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private EV3UltrasonicSensor frontSensor, rightSensor;
	private EV3ColorSensor midSensor, sideSensor;
	private SampleProvider Usp1, Usp2, Csp1, Csp2;
	private Odometer odo;
	private float[] data;
	
	public MasterBrick(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			EV3UltrasonicSensor frontSensor, EV3UltrasonicSensor rightSensor,
			EV3ColorSensor midSensor, EV3ColorSensor sideSensor,
			Odometer odo)
	{
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.frontSensor = frontSensor;
		this.rightSensor = rightSensor;
		this.midSensor = midSensor;
		this.sideSensor = sideSensor;
		this.Usp1 = frontSensor.getDistanceMode();
		this.Usp2 = rightSensor.getDistanceMode();
		this.Csp1 = midSensor.getRedMode();
		this.Csp2 = sideSensor.getRedMode();
		this.data = new float[Usp1.sampleSize()];
		this.frontSensor.enable();
		this.rightSensor.disable();
		this.midSensor.setFloodlight(false);
		this.sideSensor.setFloodlight(false);
	}
	
	public void doLocalization()
	{
		USDisable(rightSensor);
		CSDisable(midSensor);
		CSDisable(sideSensor);
		USEnable(frontSensor);
		
		double angleA, angleB;
		
		leftMotor.forward();
		rightMotor.backward();
		
		float dist = getFilteredData(Usp1,data, 0, 1);
		while((dist = getFilteredData(Usp1, data,dist, 1)) < 41);
		while((dist = getFilteredData(Usp1, data,dist, 1)) > 40);
		
		angleA = odo.getAng();
		leftMotor.backward();
		rightMotor.forward();
		
		try{ Thread.sleep(2000);} catch (Exception e) {}
		
		while((dist = getFilteredData(Usp1, data,dist, 1)) < 41);
		while((dist = getFilteredData(Usp1, data,dist, 1)) > 40);
		angleB = odo.getAng();
		leftMotor.stop(true);
		rightMotor.stop(false);
		
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
		
		leftMotor.setSpeed(70);
		rightMotor.setSpeed(70);
		
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
		
		leftMotor.stop();
		rightMotor.stop();
	}
	
	public void travelTo(double x, double y)
	{
		
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
		
		while(getFilteredData(Usp2, data, 0, 1)<60);
		
		ang-= 90;
		if(ang<0)
			ang+=360;
		turnTo(ang);
		
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.forward();
		rightMotor.forward();
	}
	
	private float getFilteredData(SampleProvider sp, float[] data, double last, double div)
	{
		sp.fetchSample(data, 0);
		float newDist = data[0]*100;
		int Filter = 0;
		
		if(last == 0)
			return newDist;
		else
		{
			if(Math.abs(last-newDist) > 15/div)
			{
				while(Filter < 15/div && Math.abs(last-newDist) > 15/div)
				{
					Filter++;
					sp.fetchSample(data, 0);
					newDist = data[0]*100;
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
	
	public void CSEnable(EV3ColorSensor sensor)
	{
		sensor.setFloodlight(true);
	}
	
	public void CSDisable(EV3ColorSensor sensor)
	{
		sensor.setFloodlight(false);
	}

}
