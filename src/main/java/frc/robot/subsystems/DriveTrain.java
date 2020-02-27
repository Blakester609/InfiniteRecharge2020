/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.Drive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj.Solenoid;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX motor1;
  private WPI_TalonFX motor2;
  private WPI_TalonFX motor3;
  private WPI_TalonFX motor4;


  private Solenoid frontBallGate;
  private Solenoid backBallGate;

  private boolean frontBallGateOn = false;
  private boolean backBallGateOn = false;

  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  Supplier<Double> gyroAngleRadians;
  private UsbCamera usbCamera;

  NetworkTableEntry autoSpeedEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry =
    NetworkTableInstance.getDefault().getEntry("/robot/rotate");
  AHRS navx = new AHRS(SerialPort.Port.kMXP);
  double priorAutospeed = 0;
  Number[] numberArray = new Number[10];
  
  private NetworkTable table;
  private NetworkTableEntry xOffset;
  private NetworkTableEntry yOffset;
  private NetworkTableEntry area; 
  private NetworkTableEntry validTarget;
  private NetworkTableEntry skew;
  private NetworkTableEntry tl;

  public DriveTrain() {
    motor1 = new WPI_TalonFX(Constants.Drive.motor1);
    motor1.setInverted(false);
    motor1.setSensorPhase(false);
    motor1.setNeutralMode(NeutralMode.Brake);
    motor1.configNeutralDeadband(0.05);
    motor1.clearStickyFaults();
    motor2 = new WPI_TalonFX(Constants.Drive.motor2);
    motor2.setInverted(false);
    motor2.setSensorPhase(false);
    motor2.setNeutralMode(NeutralMode.Brake);
    motor2.configNeutralDeadband(0.05);
    motor2.clearStickyFaults();
    motor3 = new WPI_TalonFX(Constants.Drive.motor3);
    motor3.setInverted(false);
    motor3.setSensorPhase(false);
    motor3.setNeutralMode(NeutralMode.Brake);
    motor3.configNeutralDeadband(0.05);
    motor3.clearStickyFaults();
    motor4 = new WPI_TalonFX(Constants.Drive.motor4);
    motor4.setInverted(false);
    motor4.setSensorPhase(false);
    motor4.setNeutralMode(NeutralMode.Brake);
    motor4.configNeutralDeadband(0.05);
    motor3.follow(motor1);
    motor4.follow(motor2);

    motor4.clearStickyFaults();
    motor1.configOpenloopRamp(1.5, 20);
    motor2.configOpenloopRamp(1.5, 20);
    frontBallGate = new Solenoid(Constants.pcmChannel,Constants.Lifty.frontBallGate);
    backBallGate = new Solenoid(Constants.pcmChannel,Constants.Lifty.backBallGate);
    frontBallGate.set(false);
    backBallGate.set(false);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    xOffset = table.getEntry("tx");
    yOffset = table.getEntry("ty");
    area = table.getEntry("area");
    validTarget = table.getEntry("tv"); //Whether the limelight has a valid target (either 0 or 1)
    skew = table.getEntry("ts"); //Skew or rotation of target (-90.0 to 0.0 deg.)
    tl = table.getEntry("tl");

    usbCamera = CameraServer.getInstance().startAutomaticCapture(0);
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
  
  
  //  public void setArcadeDrive(double joyForward, double joyTurn, boolean isQuickTurnOn){
  //      diffDrive.curvatureDrive(-joyForward, joyTurn, isQuickTurnOn);
  // }

  public void setArcadeDrive(double joyForward, double joyTurn) {
    double rightForward;
    double leftForward;
    // if(Math.abs(joyTurn) > 0.4) {
    //     joyTurn = 0.4 * joyTurn;
    // }

    rightForward = +joyForward;
    leftForward = -joyForward;

    // if(Math.abs(joyForward) < 0.1 && joyTurn < 0.1) {
    //     rightForward = -0.0;
    //     leftForward = +joyForward;
    // } else if(Math.abs(joyForward) < 0.1 && joyTurn > 0.1) {
    //     leftForward = 0.0;
    //     rightForward = -joyForward;
    // } else {
    //     rightForward = -joyForward;
    //     leftForward = +joyForward;
    // }
    motor1.set(ControlMode.PercentOutput, leftForward, DemandType.ArbitraryFeedForward, joyTurn);
    motor2.set(ControlMode.PercentOutput, rightForward, DemandType.ArbitraryFeedForward, joyTurn);
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
    motor1.set(ControlMode.PercentOutput, (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, DemandType.ArbitraryFeedForward,  autospeed);
    motor2.set(ControlMode.PercentOutput, (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, DemandType.ArbitraryFeedForward,  autospeed);
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
  
    

   public static class LLData {
    public final double xOffset, yOffset, area, targetExists, skew;

    public LLData(double xOffset, double yOffset, double area, double targetExists, double skew) {
        this.xOffset = xOffset;    
        this.yOffset = yOffset;
        this.area = area;
        this.targetExists = targetExists;     
        this.skew = skew;
      }
  }

  public double getHeadingError() {
    var limelightData = this.getData(); //Java 10 'var' automatically creates new LLData object.
 
    double minDrive = Constants.Limelight.minTurnPower; //speed the motor will move the robot regardless of how miniscule the error is
    double kP = Constants.Limelight.kpAim; //constant for turn power
    double xOffset = limelightData.xOffset;
    System.out.println(xOffset);
    double heading = 0.0; //should be opposite of offset (in signs)

    if (xOffset < 1.0) {
        heading = ((kP * xOffset) + minDrive);
        System.out.println("GOING LEFT");
    } else { //xOffset less than or equal to 1.0
        heading = ((kP * xOffset) - minDrive);
        System.out.println("GOING RIGHT");
    }

    return heading;
}

public void aimTowardsTarget(double speed) {
  setArcadeDrive(speed, getHeadingError());
}

  public LLData getData() {
    double x = this.xOffset.getDouble(0.0);
    double y = this.yOffset.getDouble(0.0);
    double area = this.area.getDouble(0.0);
    double skew = this.skew.getDouble(0.0);
    double v = this.validTarget.getDouble(0.0);
    return new DriveTrain.LLData(x, y, area, v, skew);
  }
  public double estimatingDistance(){
    double distance;
    distance = (Constants.Limelight.targetHeight - Constants.Limelight.cameraHeight)/(Math.tan(Constants.Limelight.cameraAngle+this.getData().yOffset));
    return distance;
  }

  public void aimingInRange(double driveAdjust){
    driveAdjust = Constants.Limelight.kpDistance * driveAdjust;
    System.out.println("Heading error: " + getHeadingError());
    System.out.println("driveAdjustment: " + driveAdjust);
    setArcadeDrive(-driveAdjust, getHeadingError());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
