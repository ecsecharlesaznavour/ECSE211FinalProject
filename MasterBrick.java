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
	
	/**
	 * Command to do localization.
	 */
	public void doLocalization()
	{
		//Disables the right sensor and enables the left sensor.
		USDisable(rightSensor);
		USEnable(frontSensor);
		
		double angleA, angleB;
		
		//sets rotating speeds and start turning right
		leftMotor.setSpeed(125);
		rightMotor.setSpeed(125);
		
		leftMotor.forward();
		rightMotor.backward();
		
		//turn until first falling edge
		float dist = getUSFilteredData(Usp1,data, 0);
		while((dist = getUSFilteredData(Usp1, data,dist)) < 41);
		while((dist = getUSFilteredData(Usp1, data,dist)) > 40);
		//gets angle
		angleA = odo.getAng();
		//turn the other way around
		leftMotor.backward();
		rightMotor.forward();
		//looks for second edge
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
		
		//turns to 0 degrees and sets angle
		turnTo((delta)%360);
		odo.setPosition(new double [] {-15.0, -15.0, 0.0}, new boolean [] {true, true, true});
	}
	
	public void turnTo(double angle)
	{
		if(angle<0)
			angle+=360;
		angle = angle%360;
		double error = angle - this.odo.getAng();
		
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		
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
	
	/**
	 * command responsible for the travelTo method
	 * @param x x-coordinate of destination
	 * @param y y-coordinate of destination
	 */
	public void travelTo(double x, double y)
	{
		//Enables the needed sensors
		USEnable(frontSensor);
		USEnable(rightSensor);
		
		boolean avoided = false;
		
		double ang = getFirstAng(odo.getX(), odo.getY(), x, y);
		
		turnTo(ang);
		
		this.odoCor.relocalize(ang);
		this.odoCor.doCorrection();
		
		double dist = getUSFilteredData(Usp1, data, 0);
		
		while(Math.abs(odo.getX()-x)>1 && Math.abs(odo.getY()-y)>1)
		{
			if((dist = getUSFilteredData(Usp1, data, dist))<15)
			{
				this.odoCor.stopCorrection();
				Avoid(ang, x, y);
			}
		}
		
		this.odoCor.stopCorrection();
		
		if(!avoided)
		{
			ang = (ang+90)%360;
			
			turnTo(ang);
			
			this.odoCor.relocalize(ang);
			this.odoCor.doCorrection();
			
			while(Math.abs(odo.getX()-x)>2 || Math.abs(odo.getY()-y)>2)
			{
				if((dist = getUSFilteredData(Usp1, data, dist))<15)
				{
					this.odoCor.stopCorrection();
					Avoid(ang, x, y);
				}
			}
			
			leftMotor.stop(true);
			rightMotor.stop(false);
			
			this.odoCor.stopCorrection();
		}
	}
	
	private double getFirstAng(double x, double y, double destx, double desty)
	{
		if(destx<x && desty<y)
			return 180;
		else if(destx<x && desty>y)
			return 90;
		else if(destx>x && desty<y)
			return 270;
		else return 0;
	}
	
	public double distToDeg(double dist)
	{
		return(dist/(2.1*Math.PI)*360);
	}
	
	public void Avoid(double ang, double destx, double desty)
	{
		double dist = getUSFilteredData(Usp1, data, 0);
		while((dist = getUSFilteredData(Usp1, data, dist))<60)
		{
			turnTo((ang = (ang+90)%360));
		}
		
		leftMotor.setSpeed(300);
		rightMotor.setSpeed(300);
		leftMotor.forward();
		rightMotor.forward();
		
		double dist2 = getUSFilteredData(Usp2, data, 0);
		
		while((dist2 = getUSFilteredData(Usp2, data, dist2))>60);
		while((dist2 = getUSFilteredData(Usp2, data, dist2))<60);
		
		forward(18);
		
		ang -=90;
		if(ang < 0)
			ang+=360;
		
		turnTo(ang);
		
		leftMotor.setSpeed(300);
		rightMotor.setSpeed(300);
		leftMotor.forward();
		rightMotor.forward();
		
		while((dist2 = getUSFilteredData(Usp2, data, dist2))>60);
		while((dist2 = getUSFilteredData(Usp2, data, dist2))<60);
		
		forward(18);
		
		leftMotor.stop(true);
		rightMotor.stop(false);
		
		travelTo(destx, desty);
	}
	
	public void forward(double length)
	{
		int tacho = (int) (length/(2.1*Math.PI)*360);
		int TC = leftMotor.getTachoCount();
		while(leftMotor.getTachoCount()<(TC+tacho));
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
					try{Thread.sleep(5);} catch(Exception e) {}
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
