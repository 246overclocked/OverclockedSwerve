package org.usfirst.frc.team246.robot.commands;

import org.usfirst.frc.team246.robot.OverclockedLibraries.AlertMessage;
import org.usfirst.frc.team246.robot.OverclockedLibraries.UdpAlertService;
import org.usfirst.frc.team246.robot.OverclockedLibraries.Vector2D;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team246.robot.Robot;

/**
 *
 * @author michaelsilver
 */
public class RobotCentricCrabWithTwist extends DrivingCommand{

//    driverLeftJoystick is controlling "twist"
//    driverRightJoystick is controlling "crab"
    
    protected Vector2D getCrabVector() {
    	Vector2D v = new Vector2D(true, Robot.getCrabX(), -Robot.getCrabY());
    	if(v.getMagnitude() > 0)
    	{
    		if(v.getMagnitude() < Robot.CRAB_ZERO_ZONE) v.setMagnitude(0.0001);
    		else v.setMagnitude((v.getMagnitude() - Robot.CRAB_ZERO_ZONE)*(1/(1 - Robot.CRAB_ZERO_ZONE)));
    	}
    	v.setMagnitude(Math.pow(v.getMagnitude(), 3) + .01);
        return v;
    }

    protected double getSpinRate() {
        
        return Math.pow(Robot.getSpin(), 3);
    }

    // return center of rotation
    protected Vector2D getCOR() {
        return new Vector2D(true,0,0);
    }

    protected void initialize() {
        UdpAlertService.sendAlert(new AlertMessage("Entering Robot-Centric Mode").playSound("r2d2.wav"));
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        
    }

    protected void interrupted() {
        end();
    }

	@Override
	protected double updateHeading() {
		return 0;
	}
    
}
