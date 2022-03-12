// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
//Drivetrain
    public static int leftM = 0;
    public static int leftS = 1;
    public static int rightM = 2;
    public static int rightS = 3;

    public static int[] driveSol = {1,5}; //Solenoid

    public static final double kWidthMeters = 0.5;

//Conveyor
    public static int conveyor = 4;
    public static int sensor = 6;

//Shooter 
    public static int top = 8;
    public static int bottom = 7;

//Intake
    public static int intake = 9;
    public static int[] solenoid = {0,4};

//Climber
    public static int[] winch = {2,6};
    public static int[] pivot = {3,7};

    public static int master = 6;
    public static int follower = 5;

}
