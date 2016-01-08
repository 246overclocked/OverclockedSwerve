package org.usfirst.frc.team246.robot.OverclockedLibraries;

public class PIDConstants {
	
	public double kP;
	public double kI;
	public double kD;
	public double kF;
	public double period;
	
	/*
	 * Creates a new {@link PIDConstants} object.
	 * 
	 * @param p
	 * 			The P constant for a PIDController.
	 * @param i
	 * 			The I constant for a PIDController.
	 * @param d
	 * 			The D constant for a PIDController.
	 * @param f
	 * 			The F constant for a PIDController.
	 * @param loopTime
	 * 			The loop time for a PIDController (in milliseconds).
	 */
	public PIDConstants(double kP, double kI, double kD, double kF, double period)
	{
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.period = period;
	}

}
