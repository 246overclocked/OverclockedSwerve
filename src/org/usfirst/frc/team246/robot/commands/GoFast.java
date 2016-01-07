package org.usfirst.frc.team246.robot.commands;

import org.usfirst.frc.team246.robot.Robot;
import org.usfirst.frc.team246.robot.OverclockedLibraries.AlertMessage;
import org.usfirst.frc.team246.robot.OverclockedLibraries.UdpAlertService;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GoFast extends Command {

    public GoFast() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.setMaxSpeed(Robot.FAST_CRAB_SPEED, Robot.FAST_SPIN_SPEED);
    	UdpAlertService.sendAlert(new AlertMessage("Going fast"));
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.setMaxSpeed(Robot.SLOW_CRAB_SPEED, Robot.SLOW_SPIN_SPEED);
    	UdpAlertService.sendAlert(new AlertMessage("Slowing Down"));
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
