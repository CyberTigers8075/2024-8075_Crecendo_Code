// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveBase;

public class SwerveJoystick extends Command {
  /** Creates a new SwerveJoystick. */
  private DriveBase driveBase;
  private Supplier<Double> xSpdFunction, ySpdFunction, angularSpdFunction;
  private Supplier<Boolean> fieldOrientedFunction;
  //private final SlewRateLimiter xLimiter, yLimiter, angularLimiter;
  
  public SwerveJoystick( DriveBase driveBase,
      Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> angularSpdFunction,
      Supplier<Boolean> fieldOrientedFunction) {
    this.driveBase = driveBase;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.angularSpdFunction = angularSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    
    //this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    //this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    //this.angularLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    
    addRequirements(driveBase);
    } 
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 1. Get real-time joystick inputsputs
    double xSpeed = 0;//xSpdFunction.get();
    double ySpeed = 0.06;//ySpdFunction.get();
    double angularSpeed = 0;//angularSpdFunction.get();    

    System.out.println("X Speed: " + xSpeed);
    System.out.println("Y Speed: " + ySpeed);
    System.out.println("AngularSpeed: " + angularSpeed);
    
    //2. Apply Deadband                                                                                       
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    angularSpeed = Math.abs(angularSpeed) > OIConstants.kDeadband ? angularSpeed : 0.0;
    
    // 3. Make the driving smoother
    xSpeed = xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
   ySpeed = ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
   angularSpeed = angularSpeed * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;   
    
    System.out.println("Limited X Speed: " + xSpeed);
    System.out.println("Limited Y Speed: " + ySpeed);
    System.out.println("Limited AngularSpeed: " + angularSpeed);
    System.out.println(fieldOrientedFunction.get());
    System.out.println("");

    // 4. Construct desired chasis speeds
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds (xSpeed, ySpeed, angularSpeed);
    /*if (fieldOrientedFunction.get()) {
      //Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, angularSpeed, driveBase.getRotation2d());
    } else {
      //relative to robot
      chassisSpeeds = new ChassisSpeeds (xSpeed, ySpeed, angularSpeed);
    
  }
  */
    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKindematics.toSwerveModuleStates(chassisSpeeds);
   System.out.println(Arrays.toString(moduleStates));
    //6. Output each module states to wheels
   driveBase.setModuleStates(moduleStates);
  }

      

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.stopModules();
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
