/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX motor1;
  private WPI_TalonFX motor2;
  private WPI_TalonFX motor3;
  private WPI_TalonFX motor4;
  private DifferentialDrive diffDrive;
  private Solenoid frontBallGate;
  private Solenoid backBallGate;

  private boolean frontBallGateOn = false;
  private boolean backBallGateOn = false;
  public DriveTrain() {
    motor1 = new WPI_TalonFX(Constants.Drive.motor1);
    motor2 = new WPI_TalonFX(Constants.Drive.motor2);
    motor3 = new WPI_TalonFX(Constants.Drive.motor3);
    motor4 = new WPI_TalonFX(Constants.Drive.motor4);
    motor3.follow(motor1);
    motor4.follow(motor2);
    diffDrive = new DifferentialDrive(motor1, motor2);
    motor1.configOpenloopRamp(3, 20);
    motor2.configOpenloopRamp(3, 20);
    frontBallGate = new Solenoid(Constants.pcmChannel,Constants.Lifty.frontBallGate);
    backBallGate = new Solenoid(Constants.pcmChannel,Constants.Lifty.backBallGate);
    frontBallGate.set(false);
    backBallGate.set(false);
   }
   
  public void frontGateSolenoidOn() {
    frontBallGateOn = !frontBallGateOn;
    frontBallGate.set(frontBallGateOn);
    
  }

  public void backGateSolenoidOn() {
    backBallGateOn = !backBallGateOn;
    backBallGate.set(backBallGateOn);
  }
  
  public void setArcadeDrive(double joyForward, double joyTurn){
       diffDrive.arcadeDrive(-joyForward, joyTurn);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
