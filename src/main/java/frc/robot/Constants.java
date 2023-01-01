// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double CAMERA_HEIGHT = Units.inchesToMeters(4.5);
    public static final double[] TARGET_HEIGHT = {Units.inchesToMeters(5),Units.inchesToMeters(32+(13/16)),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5)
    ,Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),
    Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),
    Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),
    Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5),Units.inchesToMeters(5)};
    public static final double CAMERA_PITCH = 0.0;
    public static final double KP = 6.0;
    public static final double KI = 0.0;
    public static final double KD = 0.2;
    public static final double rotKP = 0.005;
    public static final double rotKI = 0.0;
    public static final double rotKD = 0.0;
    public static final double MAX_ROT_SPEED = 2.0;
    public static final double MAX_ROT_ACCEL = 5.0;
    public static final double MAX_SPEED = .5;
    public static final double MAX_ACCEL = 1.0;
}
