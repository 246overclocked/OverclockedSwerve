
package org.usfirst.frc.team246.robot;

import org.usfirst.frc.team246.nav6.IMUAdvanced;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	public Drivetrain drivetrain;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	PIDConstants speedPIDConstants = new PIDConstants(1, 0, 0, .5, 0);
    	PIDConstants anglePIDConstants = new PIDConstants(1, 0, 0, 0, 0);
    	SwerveModule[] swerves = {
        	new SwerveModule(new CANTalon(0, 1), new CANTalonPotentiometer(0, 5, 360, 720), 14, 360, 0, 12, speedPIDConstants, anglePIDConstants, 1, 0, 0, "frontModule"),
        	new SwerveModule(new CANTalon(0, 2), new CANTalonPotentiometer(0, 6, 360, 720), 14, 360, 0, -12, speedPIDConstants, anglePIDConstants, 1, 0, 0, "rearModule"),
        	new SwerveModule(new CANTalon(0, 3), new CANTalonPotentiometer(0, 7, 360, 720), 14, 360, 12, 0, speedPIDConstants, anglePIDConstants, 1, 0, 0, "rightModule"),
        	new SwerveModule(new CANTalon(0, 4), new CANTalonPotentiometer(0, 8, 360, 720), 14, 360, -12, 0, speedPIDConstants, anglePIDConstants, 1, 0, 0, "leftModule"),
        };
    	
    	//We were having occasional errors with the creation of the nav6 object, so we make 5 attempts before allowing the error to go through and being forced to redeploy.
        IMUAdvanced navX = new IMUAdvanced(new SerialPort(57600,SerialPort.Port.kMXP), (byte)50);;
    	int count = 1;
        int maxTries = 5;
        while(navX == null) {
            try {
                navX = new IMUAdvanced(new SerialPort(57600,SerialPort.Port.kMXP), (byte)50);
            } catch (Exception e) {
                if (++count == maxTries)
                {
                    e.printStackTrace();
                    break;
                }
                Timer.delay(.01);
            }
        }
        
        PIDConstants crabPIDConstants = new PIDConstants(1, 0, 0, 0, 20);
    	PIDConstants twistPIDConstants = new PIDConstants(1, 0, 0, 0, 20);
        
        drivetrain = new Drivetrain(swerves, navX, crabPIDConstants, twistPIDConstants);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
