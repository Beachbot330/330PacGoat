package org.usfirst.frc.team330.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The operator interface of the robot, it has been simplified from the real
 * robot to allow control with a single PS3 joystick. As a result, not all
 * functionality from the real robot is available.
 */
public class OI {
	private Joystick leftJoystick, rightJoystick;

	public OI() {
		leftJoystick = new Joystick(1);
		rightJoystick = new Joystick(2);
	}
    public Joystick getLeftJoystick() {
        return leftJoystick;
    }
    public Joystick getRightJoystick() {
        return rightJoystick;
    }
}
