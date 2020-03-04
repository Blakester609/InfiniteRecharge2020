/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int pcmChannel = 8;
    public static final class Drive {
        public static final int motor1 = 0;
        public static final int motor2 = 2;
        public static final int motor3 = 1;
        public static final int motor4 = 3;
        public static final double WHEEL_DIAMETER = 6.0;
        public static final double ENCODER_EDGES_PER_REV = 2048;
        public static final int PIDIDX = 0;
		public static final boolean kGyroReversed = true;
    }
    public static final class OI {
        public static final int XPort = 0;
        public static final int climbingJoystick = 1;
        public static final int xAxis = 0;
        public static final int yAxis = 1;
    }
    public static final class Lifty {
        public static final int motor1 = 6;
        public static final int motor2 = 7;

        public static final int frontBallGate = 2;
        public static final int backBallGate = 4;
        public static final int clawOne = 0;
        public static final int clawTwo = 3;
        public static final int liftStopPiston = 1;
    }
    public static final class PIDConstantsDrivetrain {
        public static final double ksVolts = 0.228;
        public static final double ksVoltSecondsPerMeter = 0.00554;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0;

        public static final double kPDriveVel = 0.00000841;
        public static final double kTrackWidthMeters = 0.5715;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
        public static final double kMaxSpeedFeetPerSecond = 2;
        public static final double kMaxAccelerationFeetPerSecond = 2;
        public static final double kRamseteB = 6.56168;
        public static final double kRamseteZeta = 2.29659;
    }
    
    public static final class Shooty {
        public static final int shootyMotor = 4;
        public static final int suckyMotor = 5;
        public static final int spinnyMotor = 10;
        public static final int topShooterSensorPort = 0;
    }
    public static final class PIDConstantsShooter {
        public static final int pidSlot0 = 0;
        public static final double kFShooter = (1.0 * 1023) / 19833;
        public static final double kPShooter = 0.004;
        public static final double kIShooter = 0.0000000001;
        public static final double kDShooter = 0.0009;
        public static final int kTimeoutMS = 20;
    }
    public static final class Limelight{
        public static final double cameraAngle = 14.804;
        public static final double mediumRange = 175;
        //Measurements in centimeters
        public static final double targetHeight = 83.25;
        public static final double cameraHeight = 37;
        // -0.0086;
        public static final double kpAim = -0.012;
        public static final double kpDistance = 0.08;
        public static final double minTurnPower = -0.01;
    }
}
