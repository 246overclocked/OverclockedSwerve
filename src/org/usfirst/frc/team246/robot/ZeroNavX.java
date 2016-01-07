package org.usfirst.frc.team246.robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ZeroNavX extends Command {

	double offset;
	
    public ZeroNavX(double offset) {
    	this.offset = offset;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.navX.zeroYaw(offset);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
