package org.usfirst.frc.team246.robot.OverclockedLibraries;

import edu.wpi.first.wpilibj.CANTalon;

public class CANTalonPotentiometer extends CANTalon{
	int offset;
	double range;
	
	public CANTalonPotentiometer(int deviceNumber, int controlPeriodMs,int offset, double range) {
		super(deviceNumber, controlPeriodMs);
		this.offset=offset;
		this.range=range;
	}
	
	public double getScaledAnalogInPosition() {
		return getAnalogInPosition()/1024. * range - offset; //needs to add stuff for top 14 bits
	}
	
	public double getScaledAnalogInRaw(){
		return getAnalogInRaw()/1024. * range - offset;
	}
	
	public double getScaledPosition(){
		return getPosition()/1024. * range - offset;
	}
	
	public void scaledSet(double outputValue){
		set((outputValue+offset)/range*1024.);
	}
	
	public double getScaledSetpoint(){
		return getSetpoint()/1024.0 * range - offset;
	}
}
