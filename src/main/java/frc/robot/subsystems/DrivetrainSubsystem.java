// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.misc.Constants;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;


public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private final WPI_TalonFX leftM = new WPI_TalonFX(Constants.leftM);
  private final WPI_TalonFX leftS = new WPI_TalonFX(Constants.leftS);

  private final WPI_TalonFX rightM = new WPI_TalonFX(Constants.rightM);
  private final WPI_TalonFX rightS = new WPI_TalonFX(Constants.rightS);

  private final DifferentialDrive m_drive;

  private static final ADIS16470_IMU gyro = new ADIS16470_IMU();


  //private final DoubleSolenoid sol = new DoubleSolenoid(PneumaticsModuleType.REVPH, forwardChannel, reverseChannel)
  private final DoubleSolenoid solenoid = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, 1, 5);
  private final Timer time = new Timer();

  private double s, count; //Time (s) and encoder count
  private double angle;
  private int direction = 1;

  private boolean speedIsHalved = false;
  private boolean driveIsInverted = false;


  public DrivetrainSubsystem() {

    leftM.setNeutralMode(NeutralMode.Coast);
    rightM.setNeutralMode(NeutralMode.Coast);

    leftS.follow(leftM);
    rightS.follow(rightM);

    m_drive = new DifferentialDrive(leftM, rightM);

    leftM.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightM.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    setEncoderCount(0);

    gyro.calibrate();

    solenoid.set(Value.kForward);

  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("Shift:", getShift());
    s = getTime();
    count = getEncoderCount();
    SmartDashboard.putNumber("Encoder Front Left: ", count);
    SmartDashboard.putNumber("Yaw angle", getAngle());

    SmartDashboard.putBoolean("DRIVE INVERTED (GREEN = TRUE)", driveIsInverted);
    SmartDashboard.putBoolean("DRIVE SPEED HALVED (GREEN = TRUE)", speedIsHalved);
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
  // public boolean getShift(){
  //   boolean shift = (solenoid.get() == Value.kForward);
  //   return shift;
  // }

  public void teleDrive(double x, double y){
    if(speedIsHalved){
      m_drive.arcadeDrive(x * direction * .5, -y * direction * .5);
    }
    else {
      m_drive.arcadeDrive(x * direction, -y * direction);
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

  public void setEncoderCount(double count){
    leftM.getSensorCollection().setIntegratedSensorPosition(count, 0);
  }
  public double getEncoderCount(){ 
    return leftM.getSelectedSensorPosition();
  }
  
  //Gyro and timer stuff: 
  public double getAngle(){
    return gyro.getAngle();
  }
  public void resetAngle(){
    gyro.reset();
  }

  public double getTime(){ 
    return time.get(); 
  }
  public void restartTime(){ 
    time.reset(); 
    time.start();
  }

  //Josh was here
}
