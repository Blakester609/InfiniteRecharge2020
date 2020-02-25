/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj.Solenoid;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  Supplier<Double> gyroAngleRadians;

  NetworkTableEntry autoSpeedEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry =
    NetworkTableInstance.getDefault().getEntry("/robot/rotate");
  AHRS navx = new AHRS(SerialPort.Port.kMXP);
  double priorAutospeed = 0;
  Number[] numberArray = new Number[10];
  
  public DriveTrain() {
    motor1 = new WPI_TalonFX(Constants.Drive.motor1);
    motor1.setInverted(false);
    motor1.setSensorPhase(false);
    motor1.setNeutralMode(NeutralMode.Brake);
    motor2 = new WPI_TalonFX(Constants.Drive.motor2);
    motor2.setInverted(false);
    motor2.setSensorPhase(false);
    motor2.setNeutralMode(NeutralMode.Brake);
    motor3 = new WPI_TalonFX(Constants.Drive.motor3);
    motor3.setInverted(false);
    motor3.setSensorPhase(false);
    motor3.setNeutralMode(NeutralMode.Brake);
    motor4 = new WPI_TalonFX(Constants.Drive.motor4);
    motor4.setInverted(false);
    motor4.setSensorPhase(false);
    motor4.setNeutralMode(NeutralMode.Brake);
    motor3.follow(motor1);
    motor4.follow(motor2);
    diffDrive = new DifferentialDrive(motor1, motor2);
    motor1.configOpenloopRamp(2, 20);
    motor2.configOpenloopRamp(2, 20);
    frontBallGate = new Solenoid(Constants.pcmChannel,Constants.Lifty.frontBallGate);
    backBallGate = new Solenoid(Constants.pcmChannel,Constants.Lifty.backBallGate);
    frontBallGate.set(false);
    backBallGate.set(false);

    diffDrive.setDeadband(0);
    gyroAngleRadians = () -> -1 * Math.toRadians(navx.getAngle());
    double encoderConstant =
        (1 / Constants.Drive.ENCODER_EDGES_PER_REV) * Constants.Drive.WHEEL_DIAMETER * Math.PI;

    motor1.configSelectedFeedbackSensor(
        FeedbackDevice.IntegratedSensor,
        Constants.Drive.PIDIDX, 10
    );
    leftEncoderPosition = ()
        -> motor1.getSelectedSensorPosition(Constants.Drive.PIDIDX) * encoderConstant;
    leftEncoderRate = ()
        -> motor1.getSelectedSensorVelocity(Constants.Drive.PIDIDX) * encoderConstant *
               10;

    motor2.configSelectedFeedbackSensor(
        FeedbackDevice.IntegratedSensor,
        Constants.Drive.PIDIDX, 10
    );
    rightEncoderPosition = ()
        -> motor2.getSelectedSensorPosition(Constants.Drive.PIDIDX) * encoderConstant;
    rightEncoderRate = ()
        -> motor2.getSelectedSensorVelocity(Constants.Drive.PIDIDX) * encoderConstant *
               10;
    motor1.setSelectedSensorPosition(0);
    motor2.setSelectedSensorPosition(0);
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
   }
   
  public void frontGateSolenoidOn() {
    frontBallGateOn = !frontBallGateOn;
    frontBallGate.set(frontBallGateOn);
    
  }

  public void backGateSolenoidOn() {
    backBallGateOn = !backBallGateOn;
    backBallGate.set(backBallGateOn);
  }
  
  
   public void setArcadeDrive(double joyForward, double joyTurn, boolean isQuickTurnOn){
       diffDrive.curvatureDrive(-joyForward, joyTurn, isQuickTurnOn);
  }
  public void autonomousConfig(){
    double now = Timer.getFPGATimestamp();
    double leftPosition = leftEncoderPosition.get();
    double leftRate = leftEncoderRate.get();
    double rightPosition = rightEncoderPosition.get();
    double rightRate = rightEncoderRate.get();
    double battery = RobotController.getBatteryVoltage();
    double leftMotorVolts = motor1.getMotorOutputVoltage();
    double rightMotorVolts = motor2.getMotorOutputVoltage();
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;
    diffDrive.tankDrive(
      (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed,
      false
    );
    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = gyroAngleRadians.get();
    telemetryEntry.setNumberArray(numberArray);
  }  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
