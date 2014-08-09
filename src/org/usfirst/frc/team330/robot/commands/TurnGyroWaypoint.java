/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team330.robot.commands;

import org.usfirst.frc.team330.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
/*
 * $Log: TurnGyroWaypoint.java,v $
 * Revision 1.6  2013-04-03 05:17:14  jross
 * trap an infinite calcAngle
 *
 * Revision 1.5  2013-03-15 02:51:17  echan
 * added cvs log comments
 *
 */
 
/**
 * Turn in place towards a waypoint using the gyro.
 */
public class TurnGyroWaypoint extends TurnGyroAbs {
    double x, y;
    public TurnGyroWaypoint()
    {
        super(0,0,0,true,true);
    }

    protected void initialize() {
        calcAngle(x, y);
        super.initialize();
    }

    public Command copy() {
        return new TurnGyroWaypoint();
    }
    
     /**
     * The first parameter in the AutoSpreadsheet
     * @param x The X component of the waypoint to turn towards
     */
    public void setParam1(double x) {
        this.x = x;
    }
    /**
     * The second parameter in the AutoSpreadsheet
     * @param y The Y component of the waypoint to turn towards
     */
    public void setParam2(double y) {
        this.y = y;
    }
    /**
     * The third parameter in the AutoSpreadsheet
     * @param tolerance the angle tolerance in degrees. When the robot rotates to inside the tolerance, the command stops.
     */
    public void setParam3(double tolerance)
    {
        super.setParam2(tolerance);
    }
    
    
    protected void calcAngle(double x, double y) {
        double curX, curY, deltaX, deltaY, calcAngle, robotAngle;
        
        curX = Robot.drivetrain.getX();
        curY = Robot.drivetrain.getY();
        
        deltaX = x - curX;
        deltaY = y - curY;
        
        calcAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));
        
        if (Double.isNaN(calcAngle) || Double.isInfinite(calcAngle))
        {
            System.err.println("Infinite calcAngle in TurnGyroWaypoint");
            calcAngle = 0;
        }
        
        robotAngle = Robot.drivetrain.getAngle();
        
        if (Double.isNaN(robotAngle) || Double.isInfinite(robotAngle))
        {
            System.err.println("Infinite robotAngle in TurnWaypoint");
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
//        System.out.println("angle: " + calcAngle);
        
        super.setParam1(calcAngle);
    }
}
