package org.usfirst.frc.team246.robot.commands;

import org.usfirst.frc.team246.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Unwind extends Command {

    public Unwind() {
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drivetrain.unwind();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.drivetrain.isUnwound();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
