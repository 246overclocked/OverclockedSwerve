package org.usfirst.frc.team246.robot.commands;

import org.usfirst.frc.team246.robot.Robot;
import org.usfirst.frc.team246.robot.OverclockedLibraries.AlertMessage;
import org.usfirst.frc.team246.robot.OverclockedLibraries.UdpAlertService;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SteeringWatchdog extends Command {

    public SteeringWatchdog() {
    	setRunWhenDisabled(true);
    	setInterruptible(false);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	SmartDashboard.putBoolean("motorKilled", false);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//This is a safety to prevent any of the modules from rotating too far and overtwisting the wires. 
        //If any module angle surpasses RobotMap.UNSAFE_MODULE_ANGLE, the motor controlling it will be automatically shut off
    	for(int i=0; i < Robot.drivetrain.swerves.length; i++)
    	{
    		if(Math.abs(Robot.drivetrain.swerves[i].getModuleAngle()) > Robot.drivetrain.swerves[i].maxAngle + 180)
            {
                if(!SmartDashboard.getBoolean("motorKilled")) UdpAlertService.sendAlert(new AlertMessage("Steering Motor Killed").playSound("malfunction.wav"));
                (Robot.drivetrain.swerves[i].moduleMotor).disableControl();
                SmartDashboard.putBoolean("motorKilled", true);
            }
    	}

        //allows the operator to manually return control of all modules to their respective PIDcontrollers
        if(!SmartDashboard.getBoolean("motorKilled", true))
        {
            for(int i=0; i<Robot.drivetrain.swerves.length; i++)
            {
            	(Robot.drivetrain.swerves[i].moduleMotor).enableControl();
            }
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
