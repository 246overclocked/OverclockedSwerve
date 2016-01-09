/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team246.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team246.robot.Robot;
import org.usfirst.frc.team246.robot.OverclockedLibraries.Vector2D;

/**
 * An abstract subclass of CommandBase used as a basis for all other commands 
 * pertaining to the control scheme of the drivetrain.
 * 
 * @author Paul
 */
public abstract class DrivingCommand extends Command {
    
    public DrivingCommand() {
        requires(Robot.drivetrain);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.drivetrain.setFOV(updateHeading());
        
        Vector2D crabVector = getCrabVector();
        crabVector.setAngle(crabVector.getAngle() - Robot.drivetrain.getFOV());
        crabVector.setMagnitude(crabVector.getMagnitude());
        Vector2D COR = getCOR();
        
        Robot.drivetrain.teleopDrive(crabVector.getMagnitude(), crabVector.getAngle(), getSpinRate(), COR.getX(), COR.getY());
    }
    
    // use these to set the parameters of drivetrain.drive(). The results will be automatically adjusted to be relative to the FOV
    protected abstract Vector2D getCrabVector(); //This method should return a vector between -1 and 1 which controls the crab aspect of our drive. It will be scaled to the set max speed later.
    protected abstract double getSpinRate(); //This method should return a number between -1 and 1 to signify how fast the robot should rotate around the COR for the snake aspect of our drive. Positive values cause the robot to move clockwise. It will be scaled to the set max speed later.
    protected abstract Vector2D getCOR(); //This method should return a vector which represents the center of rotation for the snake aspect of our drive.
    
    protected abstract double updateHeading(); //This method should return what the code FOV of the robot is in degrees from physical north. The crabVector and CORVector will be rotated by this angle.
}