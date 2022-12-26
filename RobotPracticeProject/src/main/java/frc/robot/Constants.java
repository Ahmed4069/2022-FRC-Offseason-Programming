// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Drive Train
    public final static byte RIGHT_MASTER_ID = 5;
    public final static byte RIGHT_SLAVE_ID = 6;
    public final static byte LEFT_MASTER_ID = 1;
    public final static byte LEFT_SLAVE_ID = 2;
    //Encoder
    public final static byte RIGHT_A_ID = 4;
    public final static byte LEFT_A_ID = 6;
    public final static byte RIGHT_B_ID = 5;
    public final static byte LEFT_B_ID = 7;
    //Game Controllers
    public final static byte con1Port = 0;
    //public final static byte con2Port = 1;
    //Intake
    public final static byte ARTICULATE_ID = 41;
    public final static byte DRIVE_ID = 13;
    //gyro
    public final static byte TALON_SRX_ID_1 = 9;
    public final static byte TALON_SRX_ID_2 = 32;
    //Tower
    public final static byte TOWER_ID = 12;
    //Climber
    public final static byte LEFT_CL_ID = 30;
    public final static byte RIGHT_CL_ID = 31;
    //shooter
    public final static byte LEFT_SHOOTER_ID = 11;
    public final static byte RIGHT_SHOOTER_ID = 10;
    //Colour Wheel
    public final static byte COLOUR_WHEEL_ID = 45;
    //Characterization 
    public final static double kS = 0.18795;
    public final static double kV = 0.36856;
    public final static double kA = 0.22758;

    public static final double kTrackWidthMeters = 0.70;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final double MaxSpeedMetersPerSec = 0.3;
    public static final double MaxAccelerationMetersPerSecSquared = 0.3;

    public static final double kRamseteB = 2;
    public static final double kRamSeteZeta = 0.7;

    public static final double maxVoltageConstraint = 4;
}
