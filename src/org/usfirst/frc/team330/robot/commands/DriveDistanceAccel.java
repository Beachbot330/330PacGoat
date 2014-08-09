// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
package org.usfirst.frc.team330.robot.commands;
import org.usfirst.frc.team330.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
/**
 *
 */

public class  DriveDistanceAccel extends DriveDistance{
    double maxoutput = 0;
    double maxoutputStep = Robot.drivetrain.getDriveRampStep();
    
    public DriveDistanceAccel(double distance) {
        this(distance, 0, 0, true);
    }
    
    public DriveDistanceAccel(double distance, double tolerance, double timeout, boolean stopAtEnd) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(drivetrain);
	
        super(distance, tolerance, timeout, stopAtEnd);
    }
    // Called just before this Command runs the first time
    protected void initialize() {
        maxoutput = maxoutputStep;
        
        super.initialize();
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

        maxoutput = maxoutput + maxoutputStep;
        if (maxoutput > .8)
        {
            maxoutput = .8;
        }
        Robot.drivetrain.leftDrivePID.setOutputRange(-maxoutput, maxoutput);
        Robot.drivetrain.rightDrivePID.setOutputRange(-maxoutput, maxoutput);
    }
    
    public Command copy() {
        return new DriveDistanceAccel(0);
    }
}
