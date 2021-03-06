package org.usfirst.frc.team246.robot.commands;

import org.usfirst.frc.team246.robot.Robot;
import org.usfirst.frc.team246.robot.OverclockedLibraries.Vector2D;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */

public class AutoDrive extends Command {

    private Vector2D targetLocation;
    private boolean zero;
    private double targetHeading;
	
    //Target location is in feet
	public AutoDrive(Vector2D targetLocation, double targetHeading, boolean zeroOdometry) {
    	this.targetLocation = targetLocation;
    	zero = zeroOdometry;
    	this.targetHeading = targetHeading;
    }

    protected void initialize() {
    	if (zero) Robot.drivetrain.odometry.resetLinearDiplacement();
		Robot.drivetrain.setAccelerationRamping(Robot.AUTONOMOUS_SPEED_RAMP_RATE);;
		Robot.drivetrain.enableCrab(true);
		Robot.drivetrain.enableTwist(true);
		Robot.drivetrain.crabPID.setSetpoint(targetLocation);
		Robot.drivetrain.twistPID.setSetpoint(targetHeading);
    }

    protected void execute() {
    	Robot.drivetrain.crabPID.setOutputRange(-Robot.drivetrain.maxCrabSpeed, Robot.drivetrain.maxCrabSpeed);
    	Robot.drivetrain.twistPID.setOutputRange(-Robot.drivetrain.maxSpinSpeed, Robot.drivetrain.maxSpinSpeed);
    }

    protected boolean isFinished() {
    	return (Robot.drivetrain.crabPID.onTarget() && Robot.drivetrain.twistPID.onTarget());
    }

    protected void end() {
    	Robot.drivetrain.setAccelerationRamping(Robot.TELEOP_SPEED_RAMP_RATE);
		Robot.drivetrain.enableCrab(false);
		Robot.drivetrain.enableTwist(false);
    }

    protected void interrupted() {
    	end();
    }

}
