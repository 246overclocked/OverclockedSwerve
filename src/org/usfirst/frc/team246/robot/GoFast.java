package org.usfirst.frc.team246.robot;

import org.usfirst.frc.team246.robot.Robot;
import org.usfirst.frc.team246.robot.RobotMap;
import org.usfirst.frc.team246.robot.overclockedLibraries.AlertMessage;
import org.usfirst.frc.team246.robot.overclockedLibraries.UdpAlertService;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GoFast extends Command {

    public GoFast() {
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.setMaxSpeed(RobotMap.FAST_MAX_CRAB_SPEED, RobotMap.FAST_MAX_SPIN_SPEED);
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
    	Robot.drivetrain.setMaxSpeed(RobotMap.SLOW_MAX_CRAB_SPEED, RobotMap.SLOW_MAX_SPIN_SPEED);
    	UdpAlertService.sendAlert(new AlertMessage("Slowing Down"));
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
