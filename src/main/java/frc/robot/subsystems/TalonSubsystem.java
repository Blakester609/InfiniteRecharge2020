/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Talon stuffs
public class TalonSubsystem extends SubsystemBase {

  public static class TalonPIDConfig {
    private double feedForwardGain = 0;
    private double proportionalGain = 0;
    private double integralGain = 0;
    private double derivativeGain = 0;
    private int integralZone;
    private double peakOutputClosedLoop;

    public TalonPIDConfig(double feedForwardGain, double proportionalGain, double integralGain, double derivativeGain) {
      this.feedForwardGain = feedForwardGain;
      this.proportionalGain = proportionalGain;
      this.integralGain = integralGain;
      this.derivativeGain = derivativeGain;
    }

    public TalonPIDConfig(double feedForwardGain, double proportionalGain, double integralGain, double derivativeGain, int integralZone, double peakOutputClosedLoop)  {
      this(feedForwardGain, proportionalGain, integralGain, derivativeGain);
      this.integralZone = integralZone;
      this.peakOutputClosedLoop = peakOutputClosedLoop;
    }

    /**
     * @return the derivativeGain
     */
    public double getDerivativeGain() {
      return derivativeGain;
    }

    /**
     * @return the integralGain
     */
    public double getIntegralGain() {
      return integralGain;
    }

    /**
     * @return the proportionalGain
     */
    public double getProportionalGain() {
      return proportionalGain;
    }

    /**
     * @return the feedForwardGain
     */
    public double getFeedForwardGain() {
      return feedForwardGain;
    }

    public int getIntegralZone() {
      return integralZone;
    }

    public double getPeakOutputClosedLoop() {
      return peakOutputClosedLoop;
    }

  }
  
  public static class TalonConfiguration {

    
    private TalonPIDConfig closedLoopGains;
    private TalonPIDConfig auxClosedLoopGains;

    /**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
    private int primaryPidIndex = 0;
    
    private int auxPidIndex = 1;

    private int remoteOrdinal_0 = 0;
    private int remoteOrdinal_1 = 1;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
    private int pidSlot_0 = 0;
    private int pidSlot_1 = 1;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    private int kTimeoutMs = 30;

    public TalonConfiguration(TalonPIDConfig closedLoopGains) {
      this.closedLoopGains = closedLoopGains;
    }

    public TalonConfiguration(TalonPIDConfig closedLoopGains, TalonPIDConfig auxClosedLoopGains) {
      this(closedLoopGains);
      this.auxClosedLoopGains = auxClosedLoopGains;
    }

    public TalonConfiguration(TalonPIDConfig closedLoopGains, int kSlotIdx, int kPIDLoopIdx, int kTimeoutMs) {
      this(closedLoopGains);
      this.primaryPidIndex = kSlotIdx;
      this.pidSlot_0 = kPIDLoopIdx;
      this.kTimeoutMs = kTimeoutMs;
    }

    public TalonConfiguration(TalonPIDConfig closedLoopGains, TalonPIDConfig auxClosedLoopGains, int kSlotIdx, int kPIDLoopIdx, int auxPIDid, int kTimeoutMs) {
      this(closedLoopGains, kSlotIdx, kPIDLoopIdx, kTimeoutMs);
      this.auxClosedLoopGains = auxClosedLoopGains;
      this.auxPidIndex = auxPIDid;
    }

    public TalonPIDConfig getClosedLoopGains() {
      return this.closedLoopGains;
    }

    public TalonPIDConfig getAuxClosedLoopGains() {
      return this.auxClosedLoopGains;
    }

    public int getKTimeoutMs() {
      return this.kTimeoutMs;
    }

    public int getPrimaryPidIndex() {
      return this.primaryPidIndex;
    }

    public int getauxPidIndex() {
      return this.auxPidIndex;
    }

    public int getPidSlot_0() {
      return this.pidSlot_0;
    }

    public int getPidSlot_1() {
      return this.pidSlot_1;
    }

    public int getRemoteOrdinal_0() {
      return this.remoteOrdinal_0;
    }

    public int getRemoteOrdinal_1() {
      return this.remoteOrdinal_1;
    }

  }

  public static void configureTalon(WPI_TalonSRX talon, TalonConfiguration config, FeedbackDevice feedbackDevice) {
    talon.configFactoryDefault();
    talon.configSelectedFeedbackSensor(feedbackDevice, config.getPidSlot_0(), config.getKTimeoutMs());    
    talon.selectProfileSlot(config.getPrimaryPidIndex(), config.getPidSlot_0());
		configureTalonGains(talon, config);
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, config.getKTimeoutMs());
    talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, config.getKTimeoutMs());
  }

  private static void configureTalonGains(WPI_TalonSRX talon, TalonConfiguration config) {
    talon.config_kF(config.getPrimaryPidIndex(), config.getClosedLoopGains().getFeedForwardGain(), config.getKTimeoutMs());
		talon.config_kP(config.getPrimaryPidIndex(), config.getClosedLoopGains().getProportionalGain(), config.getKTimeoutMs());
		talon.config_kI(config.getPrimaryPidIndex(), config.getClosedLoopGains().getIntegralGain(), config.getKTimeoutMs());
    talon.config_kD(config.getPrimaryPidIndex(), config.getClosedLoopGains().getDerivativeGain(), config.getKTimeoutMs());
  }

  private static void configurePrimaryTalonGains(WPI_TalonSRX talon, TalonConfiguration config) {
    configureTalonGains(talon, config);
    // talon.config_IntegralZone(config.getPrimaryPidIndex(), izone, timeoutMs)
  }


  private static void configureAuxTalonGains(WPI_TalonSRX talon, TalonConfiguration config) {
    talon.config_kF(config.getauxPidIndex(), config.getAuxClosedLoopGains().getFeedForwardGain(), config.getKTimeoutMs());
		talon.config_kP(config.getauxPidIndex(), config.getAuxClosedLoopGains().getProportionalGain(), config.getKTimeoutMs());
		talon.config_kI(config.getauxPidIndex(), config.getAuxClosedLoopGains().getIntegralGain(), config.getKTimeoutMs());
    talon.config_kD(config.getauxPidIndex(), config.getAuxClosedLoopGains().getDerivativeGain(), config.getKTimeoutMs());
  }

  private static void configPrimaryIZoneAndClosedLoopSetting(WPI_TalonSRX talon, TalonConfiguration config) {
    talon.config_IntegralZone(config.getPrimaryPidIndex(), config.getClosedLoopGains().getIntegralZone(), config.getKTimeoutMs());
    talon.configClosedLoopPeakOutput(config.getPrimaryPidIndex(), config.getClosedLoopGains().getPeakOutputClosedLoop(), config.getKTimeoutMs());
    talon.configAllowableClosedloopError(config.getPrimaryPidIndex(), 0, config.getKTimeoutMs());
  }

  private static void configAuxIZoneAndClosedLoopSetting(WPI_TalonSRX talon, TalonConfiguration config) {
    talon.config_IntegralZone(config.getauxPidIndex(), config.getAuxClosedLoopGains().getIntegralZone(), config.getKTimeoutMs());
    talon.configClosedLoopPeakOutput(config.getauxPidIndex(), config.getAuxClosedLoopGains().getPeakOutputClosedLoop(), config.getKTimeoutMs());
    talon.configAllowableClosedloopError(config.getauxPidIndex(), 0, config.getKTimeoutMs());
  }


  public static void configureDriveTrainTalons(WPI_TalonSRX leftTalon, WPI_TalonSRX rightTalon, TalonConfiguration config, FeedbackDevice primaryDevice) {
    leftTalon.configFactoryDefault();
    rightTalon.configFactoryDefault();
    leftTalon.configSelectedFeedbackSensor(primaryDevice, config.getPrimaryPidIndex(), config.getKTimeoutMs());
    rightTalon.configRemoteFeedbackFilter(leftTalon.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, config.getRemoteOrdinal_0(), config.getKTimeoutMs());
    rightTalon.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, config.getKTimeoutMs());				// Feedback Device of Remote Talon
    rightTalon.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, config.getKTimeoutMs()); 
    rightTalon.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, config.getKTimeoutMs());
    rightTalon.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, config.getKTimeoutMs());
    
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, config.getPrimaryPidIndex(), config.getKTimeoutMs());
    
    rightTalon.configSelectedFeedbackCoefficient(	0.5, config.getPrimaryPidIndex(), config.getKTimeoutMs());
    
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, config.getauxPidIndex(), config.getKTimeoutMs());
    
    rightTalon.configSelectedFeedbackCoefficient(1, config.getauxPidIndex(), config.getKTimeoutMs());
    rightTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, config.getKTimeoutMs());
		rightTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, config.getKTimeoutMs());
		rightTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, config.getKTimeoutMs());
    rightTalon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, config.getKTimeoutMs());
    leftTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, config.getKTimeoutMs());

    rightTalon.configNeutralDeadband(0.001, config.getKTimeoutMs());
    leftTalon.configNeutralDeadband(0.001, config.getKTimeoutMs());
    configurePrimaryTalonGains(rightTalon, config);
    configPrimaryIZoneAndClosedLoopSetting(rightTalon, config);
    configureAuxTalonGains(rightTalon, config);
    configAuxIZoneAndClosedLoopSetting(rightTalon, config);

    int closedLoopTimeMs = 1;
    rightTalon.configClosedLoopPeriod(0, closedLoopTimeMs, config.getKTimeoutMs());
    rightTalon.configClosedLoopPeriod(1, closedLoopTimeMs, config.getKTimeoutMs());
    rightTalon.configAuxPIDPolarity(false, config.getKTimeoutMs());

  }

  public static void configureMotionMagicValues(WPI_TalonSRX talon, TalonConfiguration config, int velocityUnits, int accelerationUnits) {   
    talon.configMotionCruiseVelocity(velocityUnits, config.getKTimeoutMs());
    talon.configMotionAcceleration(accelerationUnits, config.getKTimeoutMs());
    
  }

  public static void configureNominalAndPeakOutputs(WPI_TalonSRX talon, TalonConfiguration config, double nomForward, double nomReverse, double peakForward, double peakReverse) {
    talon.configNominalOutputForward(nomForward, config.getKTimeoutMs());
		talon.configNominalOutputReverse(nomReverse, config.getKTimeoutMs());
		talon.configPeakOutputForward(peakForward, config.getKTimeoutMs());
    talon.configPeakOutputReverse(peakReverse, config.getKTimeoutMs());
    
  }

  public static void zeroSensor(WPI_TalonSRX talon, TalonConfiguration config) {
    talon.setSelectedSensorPosition(0, config.getPidSlot_0(), config.getKTimeoutMs());
  }
  
  public static void printTalonOutputs(WPI_TalonSRX talon) {
        System.out.println("Sensor Vel:" + talon.getSelectedSensorVelocity());
        System.out.println("Sensor Pos:" + talon.getSelectedSensorPosition());
        System.out.println("Out %" + talon.getMotorOutputPercent());
  }

  public static void setTalonMotionMagic(WPI_TalonSRX talon, double setpoint) {
    talon.set(ControlMode.MotionMagic, setpoint);
  }

  public static void putTalonOutputsSmartDash(WPI_TalonSRX talon) {
    int selSenPos = talon.getSelectedSensorPosition(0);
    int pulseWidthWithoutOverflows = talon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    double talonOutput = talon.getMotorOutputPercent();
    SmartDashboard.putNumber("pulseWidthPosition", pulseWidthWithoutOverflows);
    SmartDashboard.putNumber("selSenPos", selSenPos);
    SmartDashboard.putNumber("Talon output value: ", talonOutput);
}

public static void setProfileSlots(WPI_TalonSRX talon, int pidSlot, int primaryOrAuxIndex) {
  talon.selectProfileSlot(pidSlot, primaryOrAuxIndex);
}



  @Override
  public void periodic() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
