// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    //characterization values
    public static final double ksVolts = 1.25;
    public static final double kvVoltSecondsPerMeter = 1.38;
    public static final double kaVoltSecondsSquaredPerMeter = 0.00454;
    public static final double kPDriveVel = 0.00291;

    
    public static final double kTrackwidthMeters = .1397;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);


    public static final double kMaxSpeedMetersPerSecond = .1;
    public static final double kMaxAccelerationMetersPerSecondSquared = .1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
