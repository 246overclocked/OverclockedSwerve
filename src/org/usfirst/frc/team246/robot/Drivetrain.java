package org.usfirst.frc.team246.robot;

import org.usfirst.frc.team246.nav6.IMUAdvanced;
import org.usfirst.frc.team246.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * This is the heart of the swerve code. All vector calculations are done here.
 *
 * @author michaelsilver
 */
public class Drivetrain extends Subsystem {
    
	//every SwerveModule object is referenced twice: once in the array and once in its own variable
    public SwerveModule[] swerves;
    public IMUAdvanced navX;
    
    public Odometry odometry;
    
    public double FOV = 0; //the front of the vehicle in degrees. May be used in different ways by different control schemes.
    
    public double maxCrabSpeed;
    public double maxSpinSpeed;
    
    public Drivetrain(SwerveModule[] swerves, IMUAdvanced navX, PIDConstants crabPIDConstants, PIDConstants twistPIDConstants)
    {
    	this.swerves = swerves;
    	this.navX = navX;
    	LiveWindow.addSensor("Drivetrain", "Gyro", navX);
    	
    	maxCrabSpeed = swerves[0].maxSpeed;
    	maxSpinSpeed = swerves[0].maxSpeed;
    	
    	odometry = new Odometry();
    	twistPID = new PIDController(twistPIDConstants.kP, twistPIDConstants.kI, twistPIDConstants.kD, navX, twistPIDOutput, twistPIDConstants.period);
        twistPID.setInputRange(-180, 180);
        twistPID.setOutputRange(-1, 1);
        twistPID.setContinuous();
        twistPID.setAbsoluteTolerance(1);
        
        crabPID = new VectorPIDController(crabPIDConstants.kP, crabPIDConstants.kI, crabPIDConstants.kD, crabPIDConstants.kF, odometry, crabPIDOutput, crabPIDConstants.period);
        crabPID.setOutputRange(-1, 1);
        crabPID.setAbsoluteTolerance(.2);
        
        (new Thread(odometry)).start();
    }

    @Override
	public void initDefaultCommand() {
        setDefaultCommand(new CrabWithTwist());
        
    }
    
//    CRABDRIVE METHODS:
    
//    turns all modules in the same direction
    public Vector2D[] crab(double angle, double speed){
        Vector2D[] moduleVectors = new Vector2D[swerves.length];
        for(int i=0; i<moduleVectors.length; i++){
            moduleVectors[i] = new Vector2D(false, speed, angle);
        }
        return moduleVectors;
    }
    
//    turns modules tangential to arc
//    where (x-cor, y-cor) is the location of the center of the circle
//    we are turning about if we have a cartesian coordinate system with cor
//    being the robot's center of rotation = "origin"
    public Vector2D[] snake(double rate, double corXDist, double corYDist){
        Vector2D cor = new Vector2D(true, corXDist, corYDist);
        Vector2D[] moduleLocations = new Vector2D[swerves.length];
        for(int i=0; i<moduleLocations.length; i++)
        {
        	moduleLocations[i] = new Vector2D(true, swerves[i].getX(), swerves[i].getY());
        }
        
//        makes the module locations vectors have an origin at the center of rotation
        double[] moduleDists = new double[swerves.length]; //array of module distances (the magnitudes of the distance vectors)
        for(int i=0; i<moduleLocations.length; i++){
            moduleLocations[i] = Vector2D.subtractVectors(moduleLocations[i], cor);
            moduleDists[i] = moduleLocations[i].getMagnitude();
        }
        
//        find the farthest module from the center of rotation
        int farthestModule = 0;
        for(int i=0; i<moduleDists.length; i++){
            if(moduleDists[i] > moduleDists[farthestModule]){
                farthestModule = i;
            }
        }
        
//        rotate the moduleLocations vectors -90 degrees.
        Vector2D[] moduleSetpoints = new Vector2D[swerves.length];
        for(int i=0; i<moduleSetpoints.length; i++){
            moduleSetpoints[i] = new Vector2D(true, moduleLocations[i].getY(), -moduleLocations[i].getX());
            moduleSetpoints[i].setMagnitude(rate*moduleDists[i]/moduleDists[farthestModule]); //The furthest module should move at the same speed as the rate, and all of the other ones should scale directly porportionally to it based on the ratio of their distances to the center of rotation.
        }
        return moduleSetpoints;
    }
    
    //The primary driving method. Adds the crab and snake vectors together, allowing the robot to drive in any direction while rotating at the same time.
    public void drive(double speed, double direction, double spinRate, double corX, double corY)
    {
        Vector2D[] moduleSetpoints = new Vector2D[swerves.length];
        Vector2D[] crab = crab(direction, speed);
        Vector2D[] snake = snake(spinRate, corX, corY);
        
        //Scale the crab and snake vectors according to the max speeds
        for(int i = 0; i < moduleSetpoints.length; i++)
        {
        	crab[i].setMagnitude(crab[i].getMagnitude() * (maxCrabSpeed/swerves[i].maxSpeed));
        	snake[i].setMagnitude(snake[i].getMagnitude() * (maxSpinSpeed/swerves[i].maxSpeed));
        }
        
        //Add together the crab and snake vectors. Also find which wheel will be spinning the fastest.
        double largestVector = 0;
        for(int i=0; i<moduleSetpoints.length; i++){
            moduleSetpoints[i] = Vector2D.addVectors(crab[i], snake[i]);
            if(moduleSetpoints[i].getMagnitude() > largestVector) largestVector = moduleSetpoints[i].getMagnitude();
        }
        
        //normalize the vectors so that none of them have a magnitude greater than 1
        if(largestVector > 1)
        {
            for(int i = 0; i < moduleSetpoints.length; i++)
            {
                moduleSetpoints[i].setMagnitude(moduleSetpoints[i].getMagnitude() / largestVector);
            }
        }
        
        for(int i=0; i < swerves.length; i++)
        {
        	if(moduleSetpoints[i].getMagnitude() != 0)
        	{
        		swerves[i].setAngle(moduleSetpoints[i].getAngle());
        	}
        	swerves[i].setWheelSpeed(moduleSetpoints[i].getMagnitude());
        }
    }
    
    public void setMaxSpeed(double maxCrabSpeed, double maxSpinSpeed)
    {
    	this.maxCrabSpeed = maxCrabSpeed;
    	this.maxSpinSpeed = maxSpinSpeed;
    }
    
    public void setFOV(double fov)
    {
        FOV = fov;
    }
    
    public double getFOV()
    {
        return FOV;
    }
    
    public double getFieldCentricHeading() //returns the direction of the robot relative to the direction the driver is facing.
    {
        return navX.getYaw();
    }
    
    public void setAccelerationRamping(double rampRate)
    {
    	for(int i = 0; i < swerves.length; i++)
    	{
    		swerves[i].setSpeedRampRate(rampRate);;
    	}
    }
    
    public void PIDOn(boolean on)
    {
        if(on)
        {
        	for(int i=0; i<swerves.length; i++)
        	{
            	swerves[i].speedPIDOn(true);
            	swerves[i].anglePIDOn(true);
            }
        }
        else
        {
            for(int i=0; i<swerves.length; i++)
        	{
            	swerves[i].speedPIDOn(false);
            	swerves[i].anglePIDOn(false);
            }
        }
    }
    
    public void unwind()
    {
    	for(int i=0; i<swerves.length; i++)
        {
            swerves[i].unwind();
        }
    }
    
    public void stopUnwinding()
    {
        for(int i=0; i<swerves.length; i++)
        {
            swerves[i].stopUnwinding();
        }
    }
     
    public boolean isOverRotated()
    {
    	for(int i=0; i<swerves.length; i++)
        {
            if(swerves[i].getModuleAngle() > swerves[i].maxAngle) return true;
        }
        return false;
    }
    
    // DRIVETRAIN AUTONOMOUS PID
    
    public DrivetrainPID drivetrainPID = new DrivetrainPID();
    
    private TwistPIDOutput twistPIDOutput = new TwistPIDOutput();
    public PIDController twistPID;
    
    /**
     *@author Paul Terrasi
     */
     
    private class TwistPIDOutput implements PIDOutput
    {   
        public void pidWrite(double output) {
        	drivetrainPID.setTwist(-output); // TODO deal with scaling later
        }
    }

    public void enableTwist(boolean on) {
        if(on) twistPID.enable();
        else twistPID.disable();
    }

    private CrabPIDOutput crabPIDOutput = new CrabPIDOutput();
    public VectorPIDController crabPID;
     
    private class CrabPIDOutput implements VectorPIDOutput
    {   
        public void pidWrite(Vector2D output) {
        	drivetrainPID.setCrab(output);
        }
    }
    
    public void enableCrab(boolean on) {
        if(on) crabPID.enable();
        else crabPID.disable();
    }
    
    public class DrivetrainPID
    {
    	private double speed;
    	private double direction;
    	private Vector2D COR = new Vector2D(true, 0, 0);
    	private double spinRate = 0;
    	
    	public void setCrab(Vector2D velocity) {
    		this.speed = velocity.getMagnitude();
    		this.direction = velocity.getAngle();
    		deploy();
    	}
    	
    	public void setTwist(double spinRate){
    		this.spinRate = spinRate;
    		deploy();
    	}
    	
    	public void setCOR(Vector2D COR){
    		this.COR = COR;
    		deploy();
    	}
    	
    	private void deploy(){
    		drive(speed, direction, spinRate, COR.getX(), COR.getY());
    	}
    }
    
    public class Odometry implements Runnable, VectorPIDSource
    {
    	private Vector2D linearDisplacement = new Vector2D(true, 0, 0);
    	
    	private Vector2D[] swervesDisplacementVectors = new Vector2D[swerves.length];
    	
    	@Override
		public void run() {
			while(true){
				setSwerveDisplacementVectors();
				calculateLinearDisplacement();
				Timer.delay(.05); // in seconds	
			}
		}
    	
//    	GETTERS:
    	public Vector2D getLinearDisplacement(){
    		return linearDisplacement;
    	}
    	
    	public double getAngularDisplacement(){
			return navX.getYaw();
		}
    	
//    	RESET ODOMETRY:
    	public void resetLinearDiplacement() {
    		linearDisplacement = new Vector2D(true, 0, 0);
		}
		
//    	SET DISPLACEMENT VECTORS (needed for both calculation methods below)
    	private void setSwerveDisplacementVectors() {
    		for(int i=0; i<swerves.length; i++){
    			double dist = swerves[i].wheelMotor.getEncPosition();
    			swervesDisplacementVectors[i] = new Vector2D(false, dist, swerves[i].getAngle()+ navX.getYaw());
    			swerves[i].resetWheelEncoder();
    		}
		}
    	
//    	CALCULATE NETS for Odometry:
    	private void calculateLinearDisplacement(){
    		linearDisplacement = Vector2D.addVectors(linearDisplacement, averageOfVectors(swervesDisplacementVectors));    		
    	}
    	
//    	MATH UTILITIES:
    	private Vector2D sumOfVectors(Vector2D[] vectorArray){
    		Vector2D sum = new Vector2D(true, 0, 0);
    		for(int i=0; i<vectorArray.length; i++){
    			sum = Vector2D.addVectors(sum, vectorArray[i]);
    		}
    		return sum;
    	}
    	
    	private Vector2D averageOfVectors(Vector2D[] vectorArray){
    		Vector2D sum = sumOfVectors(vectorArray);
    		sum.setMagnitude(sum.getMagnitude()/vectorArray.length);
    		return sum;
    	}

		@Override
		public Vector2D pidGet() {
			return getLinearDisplacement();
		}
    }
}
