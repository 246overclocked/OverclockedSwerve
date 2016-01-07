/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team246.robot;

import org.usfirst.frc.team246.robot.Robot;

/**
 *
 * @author Paul
 */
public abstract class FieldCentricDrivingCommand extends DrivingCommand{
    
    public double updateHeading()
    {
        if(Robot.gyroDisabled)
        {
            return 0;
        }
        else
        {
            //updates the FOV so that it is always pointing true north (the same direction that the driver is facing)
            return Robot.drivetrain.getFieldCentricHeading();
    
        }
    }
}
