// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;


public class DrivetrainSubsystem extends SubsystemBase {

  //Gear Ratio
 private final double kGearRatio = 15; //Gear ratio needed - must switch robot to correct gear
 
  //Left Motors
  private final WPI_TalonFX leftM = new WPI_TalonFX(Constants.leftM);
  private final WPI_TalonFX leftS = new WPI_TalonFX(Constants.leftS);

  //Right Motors
  private final WPI_TalonFX rightM = new WPI_TalonFX(Constants.rightM);
  private final WPI_TalonFX rightS = new WPI_TalonFX(Constants.rightS);

  //Robot's Drive
  private final DifferentialDrive m_drive;

  //Gyro
  private static final ADIS16470_IMU gyro = new ADIS16470_IMU();

  // Odometry class for tracking robot pose 
  private final DifferentialDriveOdometry m_odometry; 
    
  //Gear Shifter
  private final DoubleSolenoid solenoid = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, 1, 5);
  
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

  private boolean speedIsHalved = false;
  private boolean driveIsInverted = false;

  private boolean allowTurnInPlace = true;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {



    //Motors Coast/Break
    leftM.setNeutralMode(NeutralMode.Coast); 
    rightM.setNeutralMode(NeutralMode.Coast); 

   
    //Invert
    rightM.setInverted(true);
    rightS.setInverted(true);
    //Slave follows Master
    leftS.follow(leftM); 
    rightS.follow(rightM); 

    

     m_drive = new DifferentialDrive(leftM, rightM);


    //Motors default sensor - Integrated
    leftM.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 
    rightM.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 

    //Zero two front drive encoders
    setEncoderCount(0); 

    //Calibrate gyro for auto
    gyro.setYawAxis(IMUAxis.kY);
    gyro.calibrate();

    //Calibrates Position of robot for Auto
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getAngle())); //That was a nightmare to figure out :/ 

 
    ///////////////////////////////////////////////////////////////////////////////////////
    //Sets the gear to - low/high?????
    solenoid.set(Value.kReverse); 

  }

  @Override
  public void periodic() {
    
    s = getTime(); 

    //toggle true/false to get rid of smartDashboard INFO
    if(true){
      SmartDashboard.putNumber("Encoder Left: ", getLeftEncoderCount());
      SmartDashboard.putNumber("Encoder Right", -1 * getRightEncoderCount());
      SmartDashboard.putNumber("Average Encoder: ", getAverageEncoderCount());
     //Converted 
      SmartDashboard.putNumber("Left encoder(m)", nativeUnitsToDistanceMeters(leftM.getSelectedSensorPosition()));
      SmartDashboard.putNumber("Right encoder(m)", -1*nativeUnitsToDistanceMeters(rightM.getSelectedSensorPosition()));
      SmartDashboard.putNumber("Average Distance(m)", getAverageEncoderDistance());

      SmartDashboard.putNumber("Heading: ", getAngle());


      //Research how to put field and rotation pose on Dashboard!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }

    



    // Update the odometry in the periodic block 
    m_odometry.update( 
      Rotation2d.fromDegrees(gyro.getAngle()), 
      nativeUnitsToDistanceMeters(getLeftEncoderCount()), 
      nativeUnitsToDistanceMeters(getRightEncoderCount())
    );  //encoder count needs to be in meters 

    SmartDashboard.putBoolean("DRIVE INVERTED (GREEN = TRUE)", driveIsInverted);
    SmartDashboard.putBoolean("DRIVE SPEED HALVED (GREEN = TRUE)", speedIsHalved);

    k_FilterX = SmartDashboard.getNumber("X-axis rate limit", 2.2);
    k_FilterTwist = SmartDashboard.getNumber("Twist rate limit", 4);

  }


//Rotation2d.fromDegrees(gyro.getAngle()) This too forever to figure out!!!!

/**
 * Returns the currently-estimated pose of the robot.
 * 
 * @return The pose.
 */
public Pose2d getPose(){
  return m_odometry.getPoseMeters();
}

/**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
    nativeUnitsToVelocity(leftM.getSelectedSensorVelocity()), 
    nativeUnitsToVelocity(rightM.getSelectedSensorVelocity()));
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

  public void toggleDriveGear(){
    if(solenoid.get() == Value.kReverse){
      solenoid.set(Value.kForward);
    }
    else if(solenoid.get() != Value.kReverse){
      solenoid.set(Value.kReverse);
    }
  }

  public void shiftBack(){
    solenoid.set(Value.kReverse);
  }


  public void teleDrive(double x, double y){
    if(speedIsHalved){
      m_drive.arcadeDrive(-1*x * direction * .5, y * direction * .5);
    }
    else {
      m_drive.arcadeDrive(-1*filterX.calculate(x * direction), filterTwist.calculate(y * direction));
    } 
  }

  public void linearDrive(double speed){
    m_drive.arcadeDrive(0, speed);
  }

  public void turn(double speed){
    m_drive.arcadeDrive(speed, 0);
  }

  public void toggleInvertDrive(){

    direction *= -1;
    driveIsInverted = !driveIsInverted;

  }

  public void toggleHalvedSpeed(){
    speedIsHalved = !speedIsHalved;
  }

////////////Encoder//////////////////
///////////Gyro/////////////////////
//////////Conversions///////////////

 
  public void setEncoderCount(double count){ 
    leftM.getSensorCollection().setIntegratedSensorPosition(count, 0); 
    rightM.getSensorCollection().setIntegratedSensorPosition(count, 0); 
  } 

  public void resetEncoders(){
    setEncoderCount(0);
  }

  public double getAverageEncoderCount(){  
    return (leftM.getSelectedSensorPosition() - rightM.getSelectedSensorPosition() ) / 2.0; 
  } 

  /**
   * Gets the average distance of the two encoders in METERS.
   *
   * @return the average of the two encoder readings in METERS
   */
  public double getAverageEncoderDistance() {
    return nativeUnitsToDistanceMeters(getAverageEncoderCount());
  }

  public double getLeftEncoderCount(){ 
    return leftM.getSelectedSensorPosition(); 
  } 

  public double getRightEncoderCount(){ 
    return -rightM.getSelectedSensorPosition(); 
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




  private int distanceToNativeUnits(double positionMeters){ 

    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(2)); 

    double motorRotations = wheelRotations * kGearRatio; 

    int sensorCounts = (int)(motorRotations * 2048); 

    return sensorCounts; 

  } 




  private int velocityToNativeUnits(double velocityMetersPerSecond){ 

    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(2)); 

    double motorRotationsPerSecond = wheelRotationsPerSecond * kGearRatio; 

    double motorRotationsPer100ms = motorRotationsPerSecond / 10; 

    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * 2048); 

    return sensorCountsPer100ms; 

  } 

  private double nativeUnitsToVelocity(double ticksPer100ms){ 

    double rotationPer100ms = ticksPer100ms / 2048;
    double rotationPerSec = rotationPer100ms * 10;
    double wheelRotPerSec = rotationPerSec / kGearRatio;
    double wheelVelocity = wheelRotPerSec * (2 * Math.PI * Units.inchesToMeters(2));

    return wheelVelocity; // In Meters per second


  } 

  private double nativeUnitsToDistanceMeters(double sensorCounts){ 

    double motorRotations = (double)sensorCounts / 2048; 

    double wheelRotations = motorRotations / kGearRatio; 

    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(2)); 

    return positionMeters; 

  } 
  //Josh was here
}
