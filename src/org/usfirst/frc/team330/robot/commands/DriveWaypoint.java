/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team330.robot.commands;

import org.usfirst.frc.team330.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
/*
 * This will drive the robot forwards to a waypoint on the field based on its 
 * original starting position.
 */
public class DriveWaypoint extends DriveDistanceAtRelAngleWAccel {
    double x,y;
    public DriveWaypoint(double x, double y, double tolerance, double timeout, boolean stopAtEnd) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(drivetrain);
        super();
        this.x = x;
        this.y = y;
        calcXY(x, y);
        super.setParam2(tolerance);
        this.setStopAtEnd(stopAtEnd);
        this.setTimeout(timeout);
    }
    
    public DriveWaypoint() {
    	this(0,0,0,0,false);
    }

    protected void initialize() {
        calcXY(x,y);
        super.initialize();
    }
    
    
    
     /**
     * The first parameter in the AutoSpreadsheet, X
     * @param x The X component of the waypoint in inches
     */
    public void setParam1(double x) {
        this.x = x;
    }
    /**
     * The second parameter in the AutoSpreadsheet, Y
     * @param y The Y component of the waypoint in inches
     */
    public void setParam2(double y) {
        this.y = y;
    }
    
     /**
     * The third parameter in the AutoSpreadsheet, tolerance. 
     * The tolerance in inches for how close to be to the distance before stopping.
     * 3 inches is a reasonable tolerance for normal movements. If a smaller
     * tolerance is used, the robot may not ever reach the tolerance, and the 
     * {@link #setTimeout(double) timeout} may be reached.
     * @param tolerance in inches
     * @see edu.wpi.first.wpilibj.PIDController#setAbsoluteTolerance
     */
    public void setParam3(double tolerance) {
        super.setParam2(tolerance);
    }

    protected void calcXY(double x, double y) {
        double curX, curY, deltaX, deltaY, calcAngle, calcDistance, robotAngle;
        
        curX = Robot.drivetrain.getX();
        curY = Robot.drivetrain.getY();
        
        deltaX = x - curX;
        deltaY = y - curY;
        
        calcDistance = Math.sqrt(deltaX*deltaX+deltaY*deltaY);
        calcAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));
        
        if (Double.isNaN(calcAngle) || Double.isInfinite(calcAngle))
        {
            System.err.println("Infinite calcAngle in DriveWaypoint");
            calcAngle = 0;
        }
        
        robotAngle = Robot.drivetrain.getAngle();
        
        if (Double.isNaN(robotAngle) || Double.isInfinite(robotAngle))
        {
            System.err.println("Infinite robotAngle in DriveWaypoint");
            robotAngle = 0;
        }
        if (Math.abs(robotAngle-calcAngle)<180)
        {
            //do nothing
        }
        else if (robotAngle > calcAngle)
        {
            while (robotAngle > calcAngle)
                calcAngle += 360;
        }
        else 
        {
            while (robotAngle < calcAngle)
                calcAngle -= 360;
        }
        System.out.println("distance: " + calcDistance);
        System.out.println("angle: " + calcAngle);
        
        super.setParam1(calcDistance);
        super.setParam3(calcAngle);
    }
}
