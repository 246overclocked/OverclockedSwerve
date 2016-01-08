
package org.usfirst.frc.team246.robot;

import org.usfirst.frc.team246.nav6.IMUAdvanced;
import org.usfirst.frc.team246.robot.OverclockedLibraries.UdpAlertService;
import org.usfirst.frc.team246.robot.OverclockedLibraries.CANTalonPotentiometer;
import org.usfirst.frc.team246.robot.OverclockedLibraries.PIDConstants;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
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
	
	public static Drivetrain drivetrain;
	
	public boolean alertServiceWasConnected = false;
	
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
        
        //Instantiate communication for our notifier service
        if(DriverStation.getInstance().isDSAttached() && !alertServiceWasConnected)
		{
			System.out.print("Connected to driver station");
			UdpAlertService.initialize("MSilver-PC.local", 5801);
			alertServiceWasConnected = true;
		}
    }
    
    //A bunch of constants and methods to make the sample commands work
    public static final double FAST_CRAB_SPEED = 12;
    public static final double FAST_SPIN_SPEED = 14;
    public static final double SLOW_CRAB_SPEED = 6;
    public static final double SLOW_SPIN_SPEED = 8;
    
    public static final double CRAB_ZERO_ZONE = .1;
    
    public static final double AUTONOMOUS_SPEED_RAMP_RATE = 3;
    public static final double TELEOP_SPEED_RAMP_RATE = 12;
    
    public static double getCrabX()
    {
    	return 0;
    }
    public static double getCrabY()
    {
    	return 0;
    }
    public static double getSpin()
    {
    	return 0;
    }
    
}
