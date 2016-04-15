package classes2;

/*
 * File: Odometer.java
 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * Ported to EV3 by: Francois Ouellet Delorme
 * Fall 2015
 * 
 * Class which controls the odometer for the robot
 * 
 * Odometer defines cooridinate system as such...
 * 
 * 					90Deg:pos y-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 180Deg:neg x-axis------------------0Deg:pos x-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 					270Deg:neg y-axis
 * 
 * The odometer is initalized to 90 degrees, assuming the robot is facing up the positive y-axis
 * 
 */

import lejos.utility.Timer;
import lejos.utility.TimerListener;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer implements TimerListener {

	private Timer timer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private final int DEFAULT_TIMEOUT_PERIOD = 20;
	private double leftRadius, rightRadius, width;
	private double x, y, theta;
	private double[] oldDH, dDH;
	private final static TextLCD t = LocalEV3.get().getTextLCD();
	
	/**
	 * Constructor for the odometer
	 * @param leftMotor left motor of the robot
	 * @param rightMotor right motor of the robot
	 * @param INTERVAL update interval in ms
	 * @param autostart autostart for the odometer
	 */
	public Odometer (EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int INTERVAL, boolean autostart) {
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		this.rightRadius = 2.1;
		this.leftRadius = 2.1;
		this.width = 15.8;
		
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 90.0;
		this.oldDH = new double[2];
		this.dDH = new double[2];

		if (autostart) {
			// if the timeout interval is given as <= 0, default to 20ms timeout 
			this.timer = new Timer((INTERVAL <= 0) ? INTERVAL : DEFAULT_TIMEOUT_PERIOD, this);
			this.timer.start();
		} else
			this.timer = null;
	}
	
	/**
	 * stop the timerlistener
	 */
	public void stop() {
		if (this.timer != null)
			this.timer.stop();
	}
	
	/**
	 * start the timerlistener
	 */
	public void start() {
		if (this.timer != null)
			this.timer.start();
	}
	
	/**
	 * Calculates displacement and heading as title suggests
	 * @param data array for storing data
	 */
	private void getDisplacementAndHeading(double[] data) {
		int leftTacho, rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();

		data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) * Math.PI / 360.0;
		data[1] = (rightTacho * rightRadius - leftTacho * leftRadius) / width;
	}
	
	
	/**
	 * Recompute the odometer values using the displacement and heading changes
	 */
	public void timedOut() {
		this.getDisplacementAndHeading(dDH);
		dDH[0] -= oldDH[0];
		dDH[1] -= oldDH[1];

		synchronized (this) {
			theta += dDH[1];
			theta = fixDegAngle(theta);

			x += dDH[0] * Math.cos(Math.toRadians(theta));
			y += dDH[0] * Math.sin(Math.toRadians(theta));
			t.drawInt((int) x, 0, 0);
			t.drawInt((int) y, 0, 1);
		}

		oldDH[0] += dDH[0];
		oldDH[1] += dDH[1];
	}

	/**
	 * return x value
	 * @return x valus of the odometer
	 */
	public double getX() {
		synchronized (this) {
			return x;
		}
	}

	/**
	 * return y value
	 * @return y value of the odometer
	 */
	public double getY() {
		synchronized (this) {
			return y;
		}
	}

	/**
	 * return angle
	 * @return angle value of the odometer
	 */
	public double getAng() {
		synchronized (this) {
			return theta;
		}
	}

	/**
	 * Sets x,y, and angle values of the odometer
	 * @param position position values
	 * @param update positions to update
	 */
	public void setPosition(double[] position, boolean[] update) {
		synchronized (this) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	/**
	 * return x, y, angle
	 * @param position
	 */
	public void getPosition(double[] position) {
		synchronized (this) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	/**
	 * returns position of the robot
	 * @return position of the robot
	 */
	public double[] getPosition() {
		synchronized (this) {
			return new double[] { x, y, theta };
		}
	}
	
	/**
	 * gets motors used by odometer
	 * @return motors used by odometer
	 */
	public EV3LargeRegulatedMotor [] getMotors() {
		return new EV3LargeRegulatedMotor[] {this.leftMotor, this.rightMotor};
	}
	
	/**
	 * gets left motor used by odometer
	 * @return left motor used by the odometer
	 */
	public EV3LargeRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}
	
	/**
	 * gets right motor used by odometer
	 * @return right motor used by the odometer
	 */
	public EV3LargeRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}

	/**
	 * sets angle between 0 and 360
	 * @param angle current angle
	 * @return angle between 0 and 360
	 */
	public static double fixDegAngle(double angle) {
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);

		return angle % 360.0;
	}

	/**
	 * gets minimum angle to destination
	 * @param a first angle
	 * @param b second angle
	 * @return minimum angle between a and b
	 */
	public static double minimumAngleFromTo(double a, double b) {
		double d = fixDegAngle(b - a);

		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}
	
	/**
	 * sets x-value of the odometer
	 * @param x new x-value
	 */
	public void setX(double x) {
		synchronized (this) {
			this.x = x;
		}
	}

	/**
	 * sets y-value of the odometer
	 * @param y new y-vslue
	 */
	public void setY(double y) {
		synchronized (this) {
			this.y = y;
		}
	}

	/**
	 * sets angle of the odometer
	 * @param theta new angler
	 */
	public void setAng(double theta) {
		synchronized (this) {
			this.theta = theta;
		}
	}
}