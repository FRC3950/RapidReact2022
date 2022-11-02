// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import frc.robot.misc.DashboardSettings;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;




public class DrivetrainSubsystem extends SubsystemBase {

  //Gear Ratio
 public static final double kGearRatio = 15; //Gear ratio needed - must switch robot to correct gear
 
  //Left Motors
  protected static final WPI_TalonFX leftM = new WPI_TalonFX(Constants.leftM);
  protected static final WPI_TalonFX leftS = new WPI_TalonFX(Constants.leftS);

  //Right Motors
  protected static final  WPI_TalonFX rightM = new WPI_TalonFX(Constants.rightM);
  protected static final WPI_TalonFX rightS = new WPI_TalonFX(Constants.rightS);

  //Robot's Drive
  private final DifferentialDrive m_drive;

  //Gyro
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  // Odometry class for tracking robot pose 
  private final DifferentialDriveOdometry m_odometry; 
    
  //Gear Shifter
  private final DoubleSolenoid shifter = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, 1, 5);
  
  //Subsystem's Timer
  private final Timer time = new Timer();

  //SlewRate for Driving
  private double k_FilterTwist = 2.2;
  private double k_FilterX = 4;
  private final SlewRateLimiter filterTwist = new SlewRateLimiter(k_FilterTwist);
  private final SlewRateLimiter filterX = new SlewRateLimiter(k_FilterX);
  
  private double s, count; //Time (s) and encoder count
  private double angle;
  private int direction = 1;



  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {


    //Motors Coast/Break
    leftM.setNeutralMode(NeutralMode.Coast); 
    rightM.setNeutralMode(NeutralMode.Coast); 
    leftS.setNeutralMode(NeutralMode.Coast); 
    rightS.setNeutralMode(NeutralMode.Coast); 

    //Motors Invert
    rightM.setInverted(true);
    rightS.setInverted(true);

    m_drive = new DifferentialDrive(leftM, rightM);

    //Slave follows Master
    leftS.follow(leftM); 
    rightS.follow(rightM); 

    //Motors default sensor - Integrated
    leftM.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 
    rightM.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 
    // leftM.setSensorPhase(true);
    // rightM.setSensorPhase(false);

    //Zero two front drive encoders
    setEncoderCount(0); 

    //Calibrate gyro for auto
    gyro.setYawAxis(IMUAxis.kY);
    gyro.calibrate(); 
    gyro.reset();
    

    //Calibrates Position of robot for Auto
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getAngle())); //That was a nightmare to figure out :/ 

    ///////////////////////////////////////////////////////////////////////////////////////
    //Sets the gear to - low/high?????

    //high is forward
    shifter.set(Value.kForward); 

    resetHeadingEncoder();



  }

  @Override
  public void periodic() {
    
    //toggle true/false to get rid of smartDashboard INFO
    //DashboardSettings.isInInfoMode(this)
    if(true){
    
      SmartDashboard.putNumber("Encoder Left: ", getLeftEncoderCount());
      SmartDashboard.putNumber("Encoder Right", getRightEncoderCount());
      SmartDashboard.putNumber("Average Encoder: ", Odometry.getAverageEncoderCount());
      SmartDashboard.putNumber("Distance Traveled (m)", Odometry.nativeUnitsToDistanceMeters(Odometry.getAverageEncoderCount()));

      SmartDashboard.putNumber("Left encoder (m)", Odometry.nativeUnitsToDistanceMeters(leftM.getSelectedSensorPosition()));
      SmartDashboard.putNumber("right encoder (m)", Odometry.nativeUnitsToDistanceMeters(rightM.getSelectedSensorPosition()));

      SmartDashboard.putNumber("Heading: ", getAngle());

      SmartDashboard.putNumber("Left Velocity", Odometry.nativeUnitsToVelocity(leftM.getSelectedSensorVelocity()));
      SmartDashboard.putNumber("Right Velocity", Odometry.nativeUnitsToVelocity(rightM.getSelectedSensorVelocity()));


      // System.out.println("Left Veloicty: " + Odometry.nativeUnitsToVelocity(leftM.getSelectedSensorVelocity()) + 
      // " Right Velocity: " + Odometry.nativeUnitsToVelocity(rightM.getSelectedSensorVelocity()) + "  Gyro" + m_odometry.toString());
      //Research how to put field and rotation pose on Dashboard!
      ///////////////////////////////////////////////////////////
    }


    // Update the odometry in the periodic block 
    m_odometry.update( 
      Rotation2d.fromDegrees(gyro.getAngle()), 
      Odometry.nativeUnitsToDistanceMeters(getLeftEncoderCount()), 
      Odometry.nativeUnitsToDistanceMeters(getRightEncoderCount())
    );  //encoder count needs to be in meters 



  }

  //Teleop driving: 
  public void toggleDriveGear(){
    if(shifter.get() == Value.kReverse){
      shifter.set(Value.kForward);
    }
    else if(shifter.get() != Value.kReverse){
      shifter.set(Value.kReverse);
    }
  }

  public void shiftBack(){
    shifter.set(Value.kReverse);
  }

  public void setLowGear(){
    shifter.set(Value.kReverse);
  }

  public Value getGear(){
    return shifter.get();
  }

  public void teleDrive(double x, double y){
 

      m_drive.arcadeDrive(filterX.calculate(x * direction), y * direction);
    
  }

 
  

  public void linearDrive(double speed){
    m_drive.arcadeDrive(speed, 0);
  }

  public void turn(double speed){
    m_drive.arcadeDrive(0, speed);
  }





  //Rotation2d.fromDegrees(gyro.getAngle()) This took forever to figure out!!!!
  /**
   * Returns the currently-estimated pose of the robot.
   * 
   * @return The pose.
   */
  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void resetHeadingEncoder(){
    leftM.setSelectedSensorPosition(0);
    rightM.setSelectedSensorPosition(0);
    gyro.reset();
  }



  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
    Odometry.nativeUnitsToVelocity(leftM.getSelectedSensorVelocity()), 
    Odometry.nativeUnitsToVelocity(rightM.getSelectedSensorVelocity()));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(gyro.getAngle()));
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftM.setVoltage(leftVolts);
    rightM.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }


////////////Encoder//////////////////
///////////Gyro/////////////////////

  public void setEncoderCount(double count){ 
    leftM.getSensorCollection().setIntegratedSensorPosition(count, 0); 
    rightM.getSensorCollection().setIntegratedSensorPosition(count, 0); 
  } 

  public void resetEncoders(){
    setEncoderCount(0);
  }

  public static double getLeftEncoderCount(){ 
    return leftM.getSelectedSensorPosition(); 
  } 

  public static double getRightEncoderCount(){ 
    return rightM.getSelectedSensorPosition(); 
  } 

  //Gyro - getters and resseters 
  public double getAngle(){   return gyro.getAngle(); } 
  public void resetAngle(){  gyro.reset(); } 

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }

   /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRate();
  }

  //Timer Getters & Restarts
  public double getTime(){ return time.get(); } 
  public void restartTime(){    time.reset();    time.start(); } 





  //Josh was here
}
