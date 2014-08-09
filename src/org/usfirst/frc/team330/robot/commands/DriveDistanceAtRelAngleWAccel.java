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
public class DriveDistanceAtRelAngleWAccel extends DriveDistanceAtAngleWAccel {
    double origDistance = 0;
    public DriveDistanceAtRelAngleWAccel(double distance, double tolerance, double angle, double timeout, boolean stopAtEnd)
    {
        super(distance, tolerance, angle, timeout, stopAtEnd);
        origDistance = distance;
    }
    
    public DriveDistanceAtRelAngleWAccel()
    {
        super(0,0,0,0,true);
    }

    protected void initialize() {
        leftDistance = leftDistance + Robot.drivetrain.getLeftDistance();
        rightDistance = rightDistance + Robot.drivetrain.getRightDistance();
        super.initialize();
    }

    protected void end() {
        super.end(); //To change body of generated methods, choose Tools | Templates.
        leftDistance = origDistance;
        rightDistance = origDistance;
    }

    public Command copy() {
        return new DriveDistanceAtRelAngleWAccel();
    }
}
