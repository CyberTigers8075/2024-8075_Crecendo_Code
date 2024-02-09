// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase extends SubsystemBase {
  //Swerve Module's declarations
  private final SwerveModule frontLeft = new SwerveModule(
    DriveConstants.FRONT_LEFT_SPEED, 
    DriveConstants.FRONT_LEFT_ANGULAR, 
    DriveConstants.F_L_S_REVERSED, 
    DriveConstants.F_L_A_REVERSED);
    
  private final SwerveModule frontRight = new SwerveModule(
    DriveConstants.FRONT_RIGHT_SPEED, 
    DriveConstants.FRONT_RIGHT_ANGULAR, 
    DriveConstants.F_R_S_REVERSED, 
    DriveConstants.F_R_A_REVERSED);
  
  private final SwerveModule backLeft = new SwerveModule(
    DriveConstants.BACK_LEFT_SPEED, 
    DriveConstants.BACK_LEFT_ANGULAR, 
    DriveConstants.B_L_S_REVERSED, 
    DriveConstants.B_L_A_REVERSED);
    
  private final SwerveModule backRight = new SwerveModule(
    DriveConstants.BACK_RIGHT_SPEED, 
    DriveConstants.BACK_RIGHT_ANGULAR, 
    DriveConstants.B_R_S_REVERSED, 
    DriveConstants.B_R_A_REVERSED);
  
  //private AHRS gyro = new AHRS(SerialPort.Port.kUSB);

   
  /** Creates a new DriveBase. */
  public DriveBase() {
    new Thread(() -> {
      try { 
        Thread.sleep(1000);
        //zeroHeading();
      } catch (Exception e) {
      }
      }).start(); 
    
  }
  /*public void zeroHeading(){
    gyro.reset();
  }
  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360); 
  }
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Robot Heading", getHeading());
  }
  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();  
    backLeft.stop();
    backRight.stop();
  }
   public void setModuleStates (SwerveModuleState [] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
    }
  
}
