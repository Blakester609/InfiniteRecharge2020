/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2017 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * A {@link Button} that gets its state from an {@link XboxController}.
 */
public class XboxButton extends Button {
	/**
	 * Represents a digital button on an XboxController.
	 */
	public enum Button {
		BumperLeft(5),
		BumperRight(6),
		StickLeft(9),
		StickRight(10),
		A(1),
		B(2),
		X(3),
		Y(4),
		Back(7),
		Start(8);
		@SuppressWarnings("MemberName")
		private int value;
		Button(int value) {
			this.value = value;
		}
	}
	private final XboxController m_joystick;
	private final int m_buttonNumber;
	/**
	 * Create a joystick button for triggering commands.
	 *
	 * @param joystick The XboxController object that has the button
	 * @param button The button to use (see {@link XboxButton.Button}
	 */
	public XboxButton(XboxController joystick, XboxButton.Button button) {
		m_joystick = joystick;
		m_buttonNumber = button.value;
	}
	/**
	 * Gets the value of the joystick button.
	 * @return The value of the joystick button
	 */
	public boolean get() {
		return m_joystick.getRawButton(m_buttonNumber);
	}
}