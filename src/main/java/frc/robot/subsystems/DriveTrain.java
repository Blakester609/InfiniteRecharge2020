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
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
public class DriveTrain extends SubsystemBase {
  private final WPI_TalonFX motor1;
  private final WPI_TalonFX motor2;
  private final WPI_TalonFX motor3;
  private final WPI_TalonFX motor4;


  // private final Solenoid frontBallGate;
  // private final Solenoid backBallGate;

  // private boolean frontBallGateOn = false;
  // private boolean backBallGateOn = false;

  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  Supplier<Double> gyroAngleRadians;
  private final UsbCamera usbCamera;
  private final DifferentialDriveOdometry odometry;
  NetworkTableEntry autoSpeedEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry =
    NetworkTableInstance.getDefault().getEntry("/robot/rotate");
  AHRS navx = new AHRS(SerialPort.Port.kMXP);
  double priorAutospeed = 0;
  Number[] numberArray = new Number[10];
  
  private final NetworkTable table;
  private final NetworkTableEntry xOffset;
  private final NetworkTableEntry yOffset;
  private final NetworkTableEntry area; 
  private final NetworkTableEntry validTarget;
  private final NetworkTableEntry skew;
  private final NetworkTableEntry tl;
  private final double heading;

  public DriveTrain() {
    motor1 = new WPI_TalonFX(Constants.Drive.motor1);
    // motor1.setInverted(false);
    // motor1.setSensorPhase(false);
    motor1.configFactoryDefault();
    
    motor1.setNeutralMode(NeutralMode.Brake);
    motor1.configNeutralDeadband(0.05);
    // motor1.configVoltageCompSaturation(11);
    // motor1.enableVoltageCompensation(true);
    // motor1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, -10, -15, 1.0));
    // motor1.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, -7, -12, 1.0));
    
    motor1.clearStickyFaults();
    motor2 = new WPI_TalonFX(Constants.Drive.motor2);
    // motor2.setInverted(false);
    // motor2.setSensorPhase(false);
    motor2.configFactoryDefault();
    
    motor2.setNeutralMode(NeutralMode.Brake);
    // motor2.configVoltageCompSaturation(11);
    // motor2.enableVoltageCompensation(true);
    motor2.configNeutralDeadband(0.05);
    // motor2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, -10, -15, 1.0));
    // motor2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, -7, -12, 1.0));
    
    motor2.clearStickyFaults();
    motor3 = new WPI_TalonFX(Constants.Drive.motor3);
    // motor3.setInverted(false);
    // motor3.setSensorPhase(false);
    motor3.configFactoryDefault();
    
    motor3.setNeutralMode(NeutralMode.Brake);
    // motor3.configVoltageCompSaturation(11);
    // motor3.enableVoltageCompensation(true);
    motor3.configNeutralDeadband(0.05);
    // motor3.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, -10, -15, 1.0));
    // motor3.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, -7, -12, 1.0));
    
    motor3.clearStickyFaults();
    
    motor4 = new WPI_TalonFX(Constants.Drive.motor4);
    // motor4.setInverted(false);
    // motor4.setSensorPhase(false);
    motor4.configFactoryDefault();
    
    motor4.setNeutralMode(NeutralMode.Brake);
    // motor4.configVoltageCompSaturation(11);
    // motor4.enableVoltageCompensation(true);
    motor4.configNeutralDeadband(0.05);
    // motor4.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, -10, -15, 1.0));
    // motor4.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, -7, -12, 1.0));
    
    motor3.follow(motor1);
    motor4.follow(motor2);
    motor4.clearStickyFaults();
    motor1.configOpenloopRamp(1.5, 20);
    motor2.configOpenloopRamp(1.5, 20);
    // frontBallGate = new Solenoid(Constants.pcmChannel,Constants.Lifty.frontBallGate);
    // backBallGate = new Solenoid(Constants.pcmChannel,Constants.Lifty.backBallGate);
    // frontBallGate.set(false);
    // backBallGate.set(false);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    xOffset = table.getEntry("tx");
    yOffset = table.getEntry("ty");
    area = table.getEntry("area");
    validTarget = table.getEntry("tv"); //Whether the limelight has a valid target (either 0 or 1)
    skew = table.getEntry("ts"); //Skew or rotation of target (-90.0 to 0.0 deg.)
    tl = table.getEntry("tl");
    heading = 0.0;
    usbCamera = CameraServer.getInstance().startAutomaticCapture(0);
    gyroAngleRadians = () -> -1 * Math.toRadians(navx.getAngle());
    final double gyroAngle = navx.getAngle();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    final double encoderConstant =
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
   
   
  // public void frontGateSolenoidOn() {
  //   frontBallGateOn = !frontBallGateOn;
  //   frontBallGate.set(frontBallGateOn);
    
  // }

  // public void backGateSolenoidOn() {
  //   backBallGateOn = !backBallGateOn;
  //   backBallGate.set(backBallGateOn);
  // }
  
  
  //  public void setArcadeDrive(double joyForward, double joyTurn, boolean isQuickTurnOn){
  //      diffDrive.curvatureDrive(-joyForward, joyTurn, isQuickTurnOn);
  // }

  public void setArcadeDrive(final double joyForward, final double joyTurn) {
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
    // System.out.println("Motor 1 output: " + motor1.get());
    // System.out.println("Motor 2 output: " + motor2.get());
}

public void setArcadeDriveTwo(final double joyForward, final double joyTurn) {
  motor1.set(ControlMode.PercentOutput, joyForward, DemandType.ArbitraryFeedForward, joyTurn);
  motor2.set(ControlMode.PercentOutput, -joyForward, DemandType.ArbitraryFeedForward, joyTurn); 
}
  public void autonomousConfig(){
    final double now = Timer.getFPGATimestamp();
    final double leftPosition = leftEncoderPosition.get();
    final double leftRate = leftEncoderRate.get();
    final double rightPosition = rightEncoderPosition.get();
    final double rightRate = rightEncoderRate.get();
    final double battery = RobotController.getBatteryVoltage();
    final double leftMotorVolts = motor1.getMotorOutputVoltage();
    final double rightMotorVolts = motor2.getMotorOutputVoltage();
    final double autospeed = autoSpeedEntry.getDouble(0);
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

    public LLData(final double xOffset, final double yOffset, final double area, final double targetExists, final double skew) {
        this.xOffset = xOffset;    
        this.yOffset = yOffset;
        this.area = area;
        this.targetExists = targetExists;     
        this.skew = skew;
      }
  }



  public double getHeadingError(double heading) {
    final var limelightData = this.getData(); //Java 10 'var' automatically creates new LLData object.
 
    final double minDrive = Constants.Limelight.minTurnPower; //speed the motor will move the robot regardless of how miniscule the error is
    final double kP = Constants.Limelight.kpAim; //constant for turn power
    final double xOffset = limelightData.xOffset;
    System.out.println(xOffset);
     //should be opposite of offset (in signs)

    if (xOffset < 1.0) {
        heading = ((kP * xOffset) + minDrive);
        System.out.println("GOING LEFT");
    } else { //xOffset less than or equal to 1.0
        heading = ((kP * xOffset) - minDrive);
        System.out.println("GOING RIGHT");
    }

    return heading;
}

public void aimTowardsTarget(final double speed) {
  setArcadeDrive(speed, getHeadingError(heading));
}

  public LLData getData() {
    final double x = this.xOffset.getDouble(0.0);
    final double y = this.yOffset.getDouble(0.0);
    final double area = this.area.getDouble(0.0);
    final double skew = this.skew.getDouble(0.0);
    final double v = this.validTarget.getDouble(0.0);
    return new DriveTrain.LLData(x, y, area, v, skew);
  }
  public double estimatingDistance(){
    double distance;
    distance = this.getData().yOffset;
    return distance;
  }

  public void aimingInRange(double driveAdjust, final double heading){
    driveAdjust = Constants.Limelight.kpDistance * driveAdjust;
    System.out.println("Heading error: " + getHeadingError(heading));
    System.out.println("driveAdjustment: " + driveAdjust);
    setArcadeDrive(-driveAdjust, getHeadingError(heading));
  }
  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoderPosition.get(), rightEncoderPosition.get());
  }
  public double getHeading(){
    return Math.IEEEremainder(navx.getAngle(), 360) * (Constants.Drive.kGyroReversed ? -1.0:1.0);
  }
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoderPosition.get(), rightEncoderRate.get());
  }
  public void resetOdometry(final Pose2d pose){
    motor1.setSelectedSensorPosition(0);
    motor2.setSelectedSensorPosition(0);
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }
  public void tankDriveVolts(final double leftVolts, final double rightVolts){
    motor1.setVoltage(leftVolts);
    motor2.setVoltage(-rightVolts);
    motor1.feed();
    motor2.feed();
  }
  public void resetEncoders(){
    motor1.setSelectedSensorPosition(0);
    motor2.setSelectedSensorPosition(0);
  }
  public double getAverageEncoderDistance(){
    return (leftEncoderPosition.get() + rightEncoderPosition.get())/2.0;
  }
  public void setMaxOutput(double maxOutput){
    motor1.configClosedLoopPeakOutput(Constants.Drive.PIDIDX, maxOutput, 20);
    motor2.configClosedLoopPeakOutput(Constants.Drive.PIDIDX, maxOutput, 20);
  }
  public void zeroHeading(){
    navx.reset();
  }



}
