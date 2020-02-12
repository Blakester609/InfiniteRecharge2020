/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import  frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DMC60;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
public class DriveTrain extends SubsystemBase {
  private Victor motor1;
  private Victor motor2;
  private Victor motor3;
  private DMC60 motor4;
  private SpeedControllerGroup leftDrive;
  private SpeedControllerGroup rightDrive;
  private DifferentialDrive diffDrive;
  public DriveTrain() {
    motor1 = new Victor(Constants.Drive.motor1victor);
    motor2 = new Victor(Constants.Drive.motor2victor);
    motor3 = new Victor(Constants.Drive.motor3victor);
    motor4 = new DMC60(Constants.Drive.motor4dmc60);
    leftDrive = new SpeedControllerGroup(motor1, motor3);
    rightDrive = new SpeedControllerGroup(motor2, motor4);
    diffDrive = new DifferentialDrive(motor1, motor2);
  }
  public void setArcadeDrive(double joyForward, double joyTurn){
       diffDrive.arcadeDrive(-joyForward, joyTurn);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
