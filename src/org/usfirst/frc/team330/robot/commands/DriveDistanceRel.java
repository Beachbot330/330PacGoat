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

public class DriveDistanceRel extends DriveDistance{
    double origDistance = 0;
    public DriveDistanceRel(double distance, double tolerance, double timeout, boolean stopAtEnd)
    {
        super(distance, tolerance, timeout, stopAtEnd);
        origDistance = distance;
    }
    
    public DriveDistanceRel()
    {
        super(0,0,0,true);
    }

    protected void initialize() {
        double leftEncoder, rightEncoder;
        leftEncoder = Robot.drivetrain.getLeftDistance();
        rightEncoder = Robot.drivetrain.getRightDistance();
//        System.out.println("leftDistance: " + leftDistance + " leftEncoder: " + leftEncoder + " rightDistance " + rightDistance + " rightEncoder " + rightEncoder);
        leftDistance = leftDistance + leftEncoder;
        rightDistance = rightDistance + rightEncoder;
        super.initialize();
    }

    protected void end() {
        super.end(); 
        leftDistance = origDistance;
        rightDistance = origDistance;
    } 
}
