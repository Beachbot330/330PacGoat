/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team330.robot.commands;

import org.usfirst.frc.team330.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWaypointBackward extends DriveWaypoint {

    public DriveWaypointBackward(double x, double y, double tolerance, double timeout, boolean stopAtEnd) {
        super(x, y, tolerance, timeout, stopAtEnd);
    }

    protected void calcXY(double x, double y) {
        double gyroAngle;
        
        super.calcXY(x, y);

        leftDistance = -leftDistance;
        rightDistance = -rightDistance;
        
        gyroAngle = Robot.drivetrain.getAngle();
        if (gyroAngle < angle)
            angle = angle-180;
        else
            angle = angle+180;
//        System.out.println("Backward Angle: " + angle);
    }

    public Command copy() {
        return new DriveWaypointBackward(0,0,0,0,true);
    }
    
}
