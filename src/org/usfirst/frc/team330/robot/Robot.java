package org.usfirst.frc.team330.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.AutoSpreadsheet;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team330.robot.commands.DriveDistance;
import org.usfirst.frc.team330.robot.commands.DriveTime;
import org.usfirst.frc.team330.robot.commands.TurnGyroAbs;
import org.usfirst.frc.team330.robot.subsystems.Collector;
import org.usfirst.frc.team330.robot.subsystems.DriveTrain;
import org.usfirst.frc.team330.robot.subsystems.Pivot;
import org.usfirst.frc.team330.robot.subsystems.Shooter;
import org.usfirst.frc.team330.robot.subsystems.SmartDashboardSender;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is the main class for running the PacGoat code.
 * 
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Command autonomousCommand;
	public static OI oi;

	public static DriveTrain drivetrain;
	public static Collector collector;
	public static Shooter shooter;
	public static Pivot pivot;
	public static SmartDashboardSender smartDashboardSender;

//	public SendableChooser autoChooser;
//	public SendableChooser autonomousDirectionChooser;
	public static AutoSpreadsheet auto;

	// This function is run when the robot is first started up and should be
	// used for any initialization code.
	public void robotInit() {
		// Initialize the subsystems
		drivetrain = new DriveTrain();
		collector = new Collector();
		shooter = new Shooter();
		pivot = new Pivot();
		smartDashboardSender = new SmartDashboardSender();
		
		SmartDashboard.putData(drivetrain);
		SmartDashboard.putData(collector);
		SmartDashboard.putData(shooter);
		SmartDashboard.putData(pivot);
		SmartDashboard.putData(smartDashboardSender);

		// This MUST be here. If the OI creates Commands (which it very likely
		// will), constructing it during the construction of CommandBase (from
		// which commands extend), subsystems are not guaranteed to be
		// yet. Thus, their requires() statements may grab null pointers. Bad
		// news. Don't move it.
		oi = new OI();
		
		auto = new AutoSpreadsheet();
        auto.readScripts();
        
        auto.addCommand(new DriveTime());
        auto.addCommand(new DriveDistance());
        auto.addCommand(new TurnGyroAbs(0));
	}

	public void autonomousInit() {
		drivetrain.resetPosition();
		autonomousCommand = auto.getSelected();
		if (autonomousCommand != null) autonomousCommand.start();
	}

	// This function is called periodically during autonomous
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		log();
		drivetrain.calcPeriodic();
	}

	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	// This function is called periodically during operator control
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		log();
		drivetrain.calcPeriodic();
	}

	// This function called periodically during test mode
	public void testPeriodic() {
		LiveWindow.run();
	}

	public void disabledInit() {
		drivetrain.stopDrive();
        if (autonomousCommand != null) autonomousCommand.cancel();
        auto.readScripts();
	}

	// This function is called periodically while disabled
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		log();
		drivetrain.calcPeriodic();
	}
	
	/**
	 * Log interesting values to the SmartDashboard.
	 */
	private void log() {
		SmartDashboard.putNumber("Pivot Pot Value", Robot.pivot.getAngle());
	}
}
