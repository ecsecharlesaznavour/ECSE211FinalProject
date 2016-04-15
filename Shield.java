package classes2;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Shield {
	
	private EV3LargeRegulatedMotor motor;
	
	/**
	 * Contructor for the Shield class
	 * @param motor motor used by the shield
	 */
	public Shield(EV3LargeRegulatedMotor motor){
		this.motor = motor;
		motor.setAcceleration(20);
		motor.setSpeed(50);
	}
	
	/**
	 * Deploy the Shield
	 */
	public void deploy(){
		motor.rotate(-100, true);
	}
	
	/**
	 * Retract the Shield
	 */
	public void retract (){
		motor.rotate(100, true);
	}
}
