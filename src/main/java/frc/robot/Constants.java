/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drive {
        public static final int motor1 = 0;
        public static final int motor2 = 2;
        public static final int motor3 = 1;
        public static final int motor4 = 3;
    }
    public static final class OI {
        public static final int joyPort = 0;
        public static final int xAxis = 0;
        public static final int yAxis = 1;
    }
    public static final class Lifty {
        public static final int motor1 = 0;
        public static final int motor2 = 0;
    
    }
    public static final class Shooty {
        public static final int shootyMotor = 4;
        public static final int suckyMotor = 5;
        public static final int spinnyMotor = 0;
        public static final int topShooterSensorPort = 0;
    }
    public static final class Limelight{
        public static final double cameraAngle = 10;
        //Measurements in centimeters
        public static final double targetHeight = 211.455;
        public static final double cameraHeight = 93.98;
        public static final double kpAim = 0.1;
        public static final double kpDistance = 0.1;
    }
}
