// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

// Motors
public static final int intakeTopID = 13;
public static final int intakeBottomID = 14;
public static final int shooterTopID = 17;
public static final int shooterBottomID = 18;
public static final int feedID = 19;
public static final int elevatorID = 20;
public static final int AimID = 22;

// Encoder
public static final int AimEncoder = 25;
public static final int revEncoderDIOPort = 1;

// CANdle
public static final int candleID = 26;

// Speeds
public static final double autoSpeed = 0.5;
}
