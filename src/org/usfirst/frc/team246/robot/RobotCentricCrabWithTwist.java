package org.usfirst.frc.team246.robot;

import org.usfirst.frc.team246.robot.overclockedLibraries.AlertMessage;
import org.usfirst.frc.team246.robot.overclockedLibraries.UdpAlertService;
import org.usfirst.frc.team246.robot.overclockedLibraries.Vector2D;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team246.robot.RobotMap;
import org.usfirst.frc.team246.robot.Robot;

/**
 *
 * @author michaelsilver
 */
public class RobotCentricCrabWithTwist extends DrivingCommand{

//    driverLeftJoystick is controlling "twist"
//    driverRightJoystick is controlling "crab"
    
    protected Vector2D getCrabVector() {
    	Vector2D v = new Vector2D(true, Robot.oi.driver.getLeftXAxis(), -Robot.oi.driver.getLeftYAxis());
    	if(v.getMagnitude() > 0)
    	{
    		if(v.getMagnitude() < RobotMap.crabZeroZone) v.setMagnitude(0.0001);
    		else v.setMagnitude((v.getMagnitude() - RobotMap.crabZeroZone)*(1/(1 - RobotMap.crabZeroZone)));
    	}
    	v.setMagnitude(Math.pow(v.getMagnitude(), 3) + .01);
        return v;
    }

    protected double getSpinRate() {
        if(Robot.test2)
        {
            return SmartDashboard.getNumber("spinRate", 0) / RobotMap.WHEEL_TOP_ABSOLUTE_SPEED;
        } 
        return Math.pow(Robot.oi.driver.getRightXAxis(), 3);
    }

    // return center of rotation
    protected Vector2D getCOR() {
        return RobotMap.centerOfRobot;
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
