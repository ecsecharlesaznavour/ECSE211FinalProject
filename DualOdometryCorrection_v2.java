package finalClasses;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class DualOdometryCorrection_v2 extends Thread{

	private EV3ColorSensor rightSensor, leftSensor;
	private SampleProvider Csp1, Csp2;
	private float[] data;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private Odometer odometer;
	private boolean correct = false;
	private int x,y = -1;
	
	public DualOdometryCorrection_v2(EV3ColorSensor leftSensor, EV3ColorSensor rightSensor,
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			Odometer odometer)
	{
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.rightSensor = rightSensor;
		this.leftSensor = leftSensor;
		this.Csp1 = this.rightSensor.getRedMode();
		this.Csp2 = this.leftSensor.getRedMode();
		this.data = new float[Csp1.sampleSize()];
	}
	public void run()
	{
		int col = -1;
		while(true)
		{
			if(correct)
			{
				int col2;
				if(col==-1)
					col = getCSFilteredData(Csp1, data, 0);
				
				if(col - (col2 = getCSFilteredData(Csp1, data, col)) > 15)
				{
					Sound.beep();
					Adjust();
					col = col2;
					try{ Thread.sleep(1000);} catch(Exception e){}
				}
			}
			else
			{
				col = -1;
			}
		}
	}
	
	private void Adjust()
	{
		if(Math.abs(odometer.getAng()-90) < 10)
			odometer.setY((y++)*30.0 + 17);
		else if(Math.abs(odometer.getAng() - 270) < 10)
			odometer.setY((y--)*30.0 + 13);
		else if(Math.abs(odometer.getAng() - 180) < 10)
			odometer.setX((x--)*30.0 + 13);
		else
			odometer.setX((x++)*30.0 + 17);
	}
	
	public void doCorrection()
	{
		correct = true;
	}
	
	public void stopCorrection()
	{
		correct = false;
	}
	
	public void relocalize()
	{
		leftMotor.setSpeed(75);
		rightMotor.setSpeed(75);
		
		leftMotor.forward();
		rightMotor.forward();
		
		int col1 = getCSFilteredData(Csp1, data, 0);
		int col2 = getCSFilteredData(Csp2, data, 0);
		
		int col1b;
		int col2b;
		while(true)
		{
			if(col1-(col1b = getCSFilteredData(Csp1, data, col1))> 5)
			{
				if(col2-(col2b = getCSFilteredData(Csp2, data, col2)) < 5)
				{
					rightMotor.stop();
					col1 = col1b;
					while(col2-(col2b = getCSFilteredData(Csp2, data, col2))<5)
						col2 = col2b;
					leftMotor.stop();
				}
				break;
			} else if(col2-(col2b = getCSFilteredData(Csp2, data, col2))> 5)
			{
				if(col1-(col1b = getCSFilteredData(Csp1, data, col1))< 5)
				{
					leftMotor.stop();
					col2 = col2b;
					while(col1-(col1b = getCSFilteredData(Csp1, data, col1))<5)
						col1 = col1b;
					rightMotor.stop();
				}
				break;
			}
			col1 = col1b;
			col2 = col2b;
		}
		Adjust();
	}
	
	public void setFactors(int x, int y)
	{
		this.x = x;
		this.y = y;
	}
	
	/**
	 * Filters the data returned by the desired Sensor.
	 * @param sp SampleProvider of the Sensor.
	 * @param datas Data array of the Sensor.
	 * @param lData Last Data of the Sensor.
	 * @return Filtered Data detected by the Sensor.
	 */
	private int getCSFilteredData(SampleProvider sp, float[] data, int last)
	{
		sp.fetchSample(data, 0);
		int newDist = (int) (data[0]*100);
		int Filter = 0;
		
		if(last == 0)
			return newDist;
		else
		{
			if(Math.abs(last-newDist) > 10)
			{
				while(Filter < 15 && Math.abs(last-newDist) > 10)
				{
					Filter++;
					sp.fetchSample(data, 0);
					newDist = (int) (data[0]*100);
				}
			}
		}
		
		return newDist;
	}

}