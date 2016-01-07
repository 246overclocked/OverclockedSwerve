package org.usfirst.frc.team246.robot;

import org.usfirst.frc.team246.robot.Robot;
import org.usfirst.frc.team246.robot.OverclockedLibraries.Diagnostics;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.ControlMode;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Paul
 */
public class SwerveModule 
{
    double x; //the horizontal distance between this wheel and the center of the robot
    double y; //the vertical distance between this wheel and the center of the robot
    
    public String name;

    public CANTalon wheelMotor; //the motor controlling wheel speed

    public CANTalonPotentiometer moduleMotor; //the motor controlling module angle
    
    public boolean invertSpeed = false; // true when the wheel is pointing backwards
    
    public boolean unwinding = false; //if true, then the wheels will return to pointing forwards with the wires completely untwisted
    
    public double maxSpeed;
    public double maxAngle;
    
    public double kDelta;
    public double kTwist;
    public double kReverse;
    
    public boolean gasMode = false;
    
    public SwerveModule(CANTalon wheelMotor, CANTalonPotentiometer moduleMotor, double maxSpeed, double maxAngle, double x, double y, PIDConstants speedPIDConstants, PIDConstants anglePIDConstants, double kDelta, double kTwist, double kReverse, String name)
    {
        // set globals
        
        this.x = x;
        this.y = y;
        
        this.name = name;
        
        this.wheelMotor = wheelMotor;
        wheelMotor.changeControlMode(ControlMode.Speed);
		wheelMotor.set(0);
		LiveWindow.addActuator("Drivetrain", name + "WheelMotor", (LiveWindowSendable) wheelMotor);
		Diagnostics.addSRXEncoder(wheelMotor, name + "Encoder");
        
		this.moduleMotor = moduleMotor;
		moduleMotor.changeControlMode(ControlMode.Position);
        LiveWindow.addActuator("Drivetrain", name + "ModuleMotor", (LiveWindowSendable) moduleMotor);
		Diagnostics.addSRXPotentiometer(moduleMotor, name + "Potentiometer");
        
        this.maxSpeed = maxSpeed;
        this.maxAngle = maxAngle;
        
        this.kDelta = kDelta;
        this.kTwist = kTwist;
        this.kReverse = kReverse;
        
        wheelMotor.setPID(speedPIDConstants.kP, speedPIDConstants.kI, speedPIDConstants.kD, speedPIDConstants.kF, 0, 12, 0);  //THESE CONSTANTS NEED TO BE DISCUSSED AND SET
        moduleMotor.setPID(anglePIDConstants.kP, anglePIDConstants.kI, anglePIDConstants.kD, anglePIDConstants.kF, 0, 12, 0);
    }
    
//    coordinates
    public double getX(){
        return x;
    }
    
    public double getY(){
        return y;
    }
    
    // PID Methods
    
    //whenever possible, call setAngle before setSpeed
    
    public void setAngle(double angle){

        if(!unwinding)
        {
            //The following is uses a weighted rating system to decide which direction we rotate the module
            
            //converts the inputed angle into its reference angle
            angle = angle % 360;

            double setPointForward = angle; // angle setpoint if we want the wheel to move forward
            double setPointBackward = angle + 180; // angle setpoint if we want the wheel to move backwards

            //The following code ensures that our 2 potential setpoints are the ones closest to our current angle
            double moduleAngle=moduleMotor.getScaledAnalogInRaw();
            while(Math.abs(setPointForward - moduleAngle) > 180)
            {
                if(setPointForward - moduleAngle < 0 && setPointForward < maxAngle - 360) setPointForward += 360; //if we need to add 360 to get closer to moduleEncoder, do so
                else if (setPointForward - moduleAngle > 0 && setPointForward > -maxAngle + 360) setPointForward -= 360; //else subtract 360
                else break;
            }

            while(Math.abs(setPointBackward - moduleAngle) > 180)
            {
                if(setPointBackward - moduleAngle < 0 && setPointBackward < maxAngle - 360) setPointBackward += 360; //if we need to add 360 to get closer to moduleEncoder, do so
                else if (setPointBackward - moduleAngle > 0 && setPointBackward > -maxAngle + 360) setPointBackward -= 360; //else subtract 360
                else break;
            }

            //rating for how desirable each setpoint is. Higher numbers are better
            double forwardsRating = 0;
            double backwardsRating = 0;

            //Rating for the distance between where the module is currently pointing and each of the setpoints
            forwardsRating -= kDelta*Math.abs(setPointForward - moduleAngle);
            backwardsRating -= kDelta*Math.abs(setPointBackward - moduleAngle);

            //Rating boost if this setpoint is closer to the 0 (where the wire is completely untwisted) that the current module angle
            if(setPointForward > 0){
                forwardsRating += (moduleAngle - setPointForward)*kTwist; // positive => we are unwinding (moving closer to zero)
            } else {
                forwardsRating += (setPointForward - moduleAngle)*kTwist; // negative => we are winding up (moving farther from zero)
            }

            if(setPointBackward > 0){
                backwardsRating += (moduleAngle - setPointBackward)*kTwist; // positive => we are unwinding (moving closer to zero)
            } else {
                backwardsRating += (setPointBackward - moduleAngle)*kTwist; // negative => we are winding up (moving farther from zero)
            }

            //Rating for if the how much the velocity will need to change in order the make the wheel go further. Forwards rating gets a positive boost if wheel is already moving forwards, if the wheel is currently moving backwards it gets a deduction.
            forwardsRating += kReverse * wheelMotor.getEncVelocity();

            //Decision making time
            if(forwardsRating > backwardsRating)
            {
                moduleMotor.scaledSet(setPointForward);
                invertSpeed = false;
            }
            else
            {
                moduleMotor.scaledSet(setPointBackward);
                invertSpeed = true;
            }
        }
    }
    
    public void setWheelSpeed(double speed){
        if(invertSpeed) speed = -speed;
        if(!gasMode)
        {
        	wheelMotor.changeControlMode(ControlMode.Speed);
            wheelMotor.set(speed*maxSpeed);
        }
        else
        {
        	wheelMotor.changeControlMode(ControlMode.PercentVbus);
            wheelMotor.set(speed);
            
        }
    }
    
    //Makes the wheels point forwards with the wires being completely untwisted. 
    //Once this method is called, setAngle(double angle) will be disabled until stopUnwinding is called()
    public void unwind()
    {
        unwinding = true;
        if(!moduleMotor.isControlEnabled())moduleMotor.enableControl();
        moduleMotor.scaledSet(0);
    }
    
    //Stops the wheels from trying to point forwards and restores control to setAngle(double angle)
    public void stopUnwinding()
    {
        unwinding = false;
    }
    
    public void setMaxSpeed(double max)
    {
    	maxSpeed = max;
    }
    
    public void anglePIDOn(boolean on){
        if (on) moduleMotor.disableControl();
        else moduleMotor.enableControl();
    }
    
    public void speedPIDOn(boolean on){
        if (on) wheelMotor.enableControl();
        else wheelMotor.disableControl();
    }
    
    public void setGasMode(boolean on)
    {
    	gasMode = on;
    }
    
    public void setSpeedRampRate(double rampRate)
    {
    	wheelMotor.setCloseLoopRampRate(rampRate);
    }
    
    public double getAngleSetpoint() {
        return moduleMotor.getScaledSetpoint();
    }
    
    public double getSpeedSetpoint() {
        return wheelMotor.getSetpoint();//This may need to be adjusted depending on format
    }
    
    public double getAngle() {
        return moduleMotor.getScaledAnalogInRaw();
    }
    
    public double getSpeed() {
        return wheelMotor.get();
    }

    // Wheel Encoder Methods
    
    public double getWheelSpeed() {
        return wheelMotor.getSpeed();
    }
    
    public double getWheelDistance() {
        return wheelMotor.getEncPosition();
    }
    
    public void resetWheelEncoder(){
        wheelMotor.setPosition(0);
    }
    
    // Module Potentiometer Methods
    
    public double getModuleAngle() {
        return moduleMotor.getScaledAnalogInRaw();
    }
}
