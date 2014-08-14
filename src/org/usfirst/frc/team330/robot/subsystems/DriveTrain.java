package org.usfirst.frc.team330.robot.subsystems;

import org.usfirst.frc.team330.robot.Robot;
import org.usfirst.frc.team330.robot.commands.TankDrive;
import org.usfirst.frc330.wpilibj.DummyPIDOutput;
import org.usfirst.frc330.wpilibj.MultiPrefSendablePIDController;
import org.usfirst.frc330.wpilibj.PrefSendablePIDController;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The DriveTrain subsystem controls the robot's drivetrain and reads in
 * information about it's speed and position.
 */
public class DriveTrain extends Subsystem implements PIDSource {
	// Subsystem devices
	private SpeedController frontLeftCIM, frontRightCIM;
	private SpeedController backLeftCIM, backRightCIM;
	private RobotDrive drive;
	private Encoder rightEncoder, leftEncoder;

	private Gyro gyro;
	//TODO change to PID Controller until simulator handles preferences
//    public PrefSendablePIDController leftDrivePID, rightDrivePID;
//    public MultiPrefSendablePIDController gyroPID;
	public PIDController leftDrivePID, rightDrivePID, gyroPID;
    private DummyPIDOutput gyroOutput, leftDriveOutput, rightDriveOutput;
    
    public static final String DRIVE = "Drive";
    public final static String TURN = "Turn";
    
    double left, right;

	public DriveTrain() {
		// Configure drive motors
		frontLeftCIM = new Victor(1);
		frontRightCIM = new Victor(2);
		backLeftCIM = new Victor(3);
		backRightCIM = new Victor(4);
		LiveWindow.addActuator("DriveTrain", "Front Left CIM", (Victor) frontLeftCIM);
		LiveWindow.addActuator("DriveTrain", "Front Right CIM", (Victor) frontRightCIM);
		LiveWindow.addActuator("DriveTrain", "Back Left CIM", (Victor) backLeftCIM);
		LiveWindow.addActuator("DriveTrain", "Back Right CIM", (Victor) backRightCIM);

		// Configure the RobotDrive to reflect the fact that all our motors are
		// wired backwards and our drivers sensitivity preferences.
		drive = new RobotDrive(frontLeftCIM, backLeftCIM, frontRightCIM, backRightCIM);
		drive.setSafetyEnabled(true);
		drive.setExpiration(0.1);
		drive.setMaxOutput(1.0);
		drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
		drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
		drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
		drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);

		// Configure encoders
		rightEncoder = new Encoder(1, 2, false, EncodingType.k4X);
		leftEncoder = new Encoder(3, 4, false, EncodingType.k4X);
		rightEncoder.setPIDSourceParameter(PIDSourceParameter.kDistance);
		leftEncoder.setPIDSourceParameter(PIDSourceParameter.kDistance);

		if (Robot.isReal()) { // Converts to feet
			rightEncoder.setDistancePerPulse(0.0785398);
			leftEncoder.setDistancePerPulse(0.0785398);
		} else { // Convert to feet 4in diameter wheels with 360 tick simulated encoders
			rightEncoder.setDistancePerPulse((-4.0/*in*/*Math.PI)/(360.0/**12.0/*in/ft*/));
			leftEncoder.setDistancePerPulse((-4.0/*in*/*Math.PI)/(360.0/**12.0/*in/ft*/));
		}

		rightEncoder.start();
		leftEncoder.start();		
		LiveWindow.addSensor("DriveTrain", "Right Encoder", rightEncoder);
		LiveWindow.addSensor("DriveTrain", "Left Encoder", leftEncoder);

		// Configure gyro
		gyro = new Gyro(2);

		LiveWindow.addSensor("DriveTrain", "Gyro", gyro);
		
        gyroOutput = new DummyPIDOutput();
        leftDriveOutput = new DummyPIDOutput();
        rightDriveOutput = new DummyPIDOutput();
        
        //TODO preferences
//        gyroPID = new MultiPrefSendablePIDController(0.11,0,0,this,gyroOutput,"gyro");
//        leftDrivePID = new PrefSendablePIDController(0.2,0,0,leftEncoder,leftDriveOutput,"leftDrive");
//        rightDrivePID = new PrefSendablePIDController(0.2,0,0,rightEncoder,rightDriveOutput,"rightDrive");
        gyroPID = new PIDController(0.11,0,0,this,gyroOutput);
        leftDrivePID = new PIDController(0.01,0,0,leftEncoder,leftDriveOutput);
        rightDrivePID = new PIDController(0.01,0,0,rightEncoder,rightDriveOutput);
        
        leftDrivePID.setOutputRange(-0.8, 0.8);
        rightDrivePID.setOutputRange(-0.8, 0.8);
        
        SmartDashboard.putData("gyroPID", gyroPID);
        SmartDashboard.putData("leftDrivePID", leftDrivePID);
        SmartDashboard.putData("rightDrivePID", rightDrivePID);
	}

	/**
	 * When other commands aren't using the drivetrain, allow tank drive with
	 * the joystick.
	 */
	public void initDefaultCommand() {
		setDefaultCommand(new TankDrive());
	}

    public double getDriveRampStep() {
        if (!Preferences.getInstance().containsKey("DriveRampMaxOutpuStep"))
        {
            Preferences.getInstance().putDouble("DriveRampMaxOutpuStep", 
                                                0.02);
            Preferences.getInstance().save();
        }
        return Preferences.getInstance().getDouble("DriveRampMaxOutpuStep", 
                                                   0.02);
    }
    
    
     public void tankDrive(Joystick leftJoystick, Joystick rightJoystick)
    {
        left = -leftJoystick.getY();
        right = -rightJoystick.getY();
    }
    
    public void tankDrive(double left, double right)
    {
        this.left = left;
        this.right = right;
    }
    
    private void drive(double left, double right) {
        drive.setLeftRightMotorOutputs(-left, -right);
//        System.out.println("Left: " + left + " Right: " + right);
    }
    
    
    public double getRightDistance() {
        return rightEncoder.getDistance();
    }
    
    public double getLeftDistance() {
        return leftEncoder.getDistance();
    }
    
    public double getAngle() {
        return gyro.getAngle() + gyroComp;
    }
    
    double maxGyroRate = 0;
    
    public void pidDrive()
    {
        double left, right;
        if (DriverStation.getInstance().isDisabled())
        {
            stopDrive();
        }
        else
        {
            left = this.left+leftDriveOutput.getOutput() - gyroOutput.getOutput();
            right = this.right+rightDriveOutput.getOutput() + gyroOutput.getOutput();
            drive(left, right);
            this.left = 0;
            this.right = 0;
        }
    
    }
    
    private double gyroComp = 0;
    public void setGyroOffset(double gyroComp) {
        this.gyroComp = gyroComp;
        SmartDashboard.putNumber("gyroComp", gyroComp);
    }
    
    private double x=0, y=0;
    private double prevLeftEncoderDistance=0, prevRightEncoderDistance=0;
    
    public void setXY(double x, double y)
    {
        this.x = x;
        this.y = y;
    }
    
    int counter = 0;
    public void calcXY()
    {
        double distance, leftEncoderDistance, rightEncoderDistance, gyroAngle;
        
        leftEncoderDistance = leftEncoder.getDistance();
        rightEncoderDistance = rightEncoder.getDistance();
        gyroAngle = getAngle();
        distance =  ((leftEncoderDistance - prevLeftEncoderDistance) + (rightEncoderDistance - prevRightEncoderDistance))/2;
        x = x + distance * Math.sin(Math.toRadians(gyroAngle));
        y = y + distance * Math.cos(Math.toRadians(gyroAngle));
        prevLeftEncoderDistance = leftEncoderDistance;
        prevRightEncoderDistance = rightEncoderDistance;
    }
    
    public void calcPeriodic()
    {
        calcXY();
        pidDrive();
        counter++;
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    
    public void resetPosition()
    {
        leftEncoder.reset();
        rightEncoder.reset();
        gyro.reset();
        setXY(0,0);
        this.prevLeftEncoderDistance = 0;
        this.prevRightEncoderDistance = 0;
    }
    
    public void stopDrive()
    {
        if (gyroPID.isEnable())
            gyroPID.reset();
        if (leftDrivePID.isEnable())
            leftDrivePID.reset();
        if (rightDrivePID.isEnable())
            rightDrivePID.reset();        
        tankDrive(0, 0);  
    }
    public double pidGet() {
        return getAngle();
    }
    
    double gain = .5;
    public void cheesyDrive(Joystick leftJoystick, Joystick rightJoystick)     {
        double turn = rightJoystick.getAxis(Joystick.AxisType.kX);
        double throttle = -leftJoystick.getAxis(Joystick.AxisType.kY);
        double left, right;
        
        if (Math.abs(throttle) > 0.2)
            turn = Math.abs(throttle) * turn * gain;
        
        left = throttle  + turn;
        right = throttle - turn;
        
        tankDrive(left + skim(right),right + skim(left));
//        right = right + skim(left);
    }
    
    private double skim(double v) {
        // gain determines how much to skim off the top
        if (v > 1.0)
          return -((v - 1.0) * gain);
        else if (v < -1.0)
          return -((v + 1.0) * gain);
        return 0;
    }
    
	public Encoder getRightEncoder() {
		return rightEncoder;
	}

	public Encoder getLeftEncoder() {
		return leftEncoder;
	}
}