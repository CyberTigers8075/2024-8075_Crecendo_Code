// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveModule extends SubsystemBase {

  //Declarations of swerve module.
  private WPI_TalonFX speedMotor, angularMotor;
  private PIDController angularPidController, speedPidController;

  /*Creates a new SwerveModule. */
  public SwerveModule(int speedMotorId, int angularMotorId, boolean speedMotorReversed, boolean angularMotorReversed) {

    speedMotor = new WPI_TalonFX(speedMotorId);
    angularMotor = new WPI_TalonFX(angularMotorId);

    speedMotor.setInverted(speedMotorReversed);
    angularMotor.setInverted(angularMotorReversed);
    
    angularPidController = new PIDController(ModuleConstants.kPAngular, ModuleConstants.kIAngular, ModuleConstants.kDAngular);
    //angularPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //returns the speed motor's position in meters.
  public double getSpeedPosition(){
    return speedMotor.getSelectedSensorPosition() * ModuleConstants.kSpeedEncoderRot2Meter;
  }

  //Returns the angular motor's position in radians
  public double getAngularPosition(){
    return (angularMotor.getSelectedSensorPosition() * ModuleConstants.kAngularEncoderRot2Rad) % (Math.PI);
  }

  //gets speed motors velocity in meters per second
  public double getSpeedVelocity(){
    return speedMotor.getSelectedSensorVelocity() * ModuleConstants.kSpeedEncoderRPM2MeterPerSec;
  }
  
  //gets angular motor's velocity in radians
  public double getAngularVelocity(){
    return angularMotor.getSelectedSensorVelocity() * ModuleConstants.kAngularEncoderRot2Rad;
  }

  //Sets speed motor position to 0 and angular motor's position to a number between -2048 and 2048 relative to its position.
  public void resetEncoders(){
     speedMotor.getSelectedSensorPosition(0);
     double pos = angularMotor.getSelectedSensorPosition() % 2048;
     angularMotor.setSelectedSensorPosition(pos);
     
  }
  
  //Converts the information on the two motors to a swerve module state.
  public SwerveModuleState getState() { 
    return new SwerveModuleState(getSpeedVelocity(), new Rotation2d(getAngularPosition()));
  }
  
  public void setDesiredState(SwerveModuleState state){
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    
    state = SwerveModuleState.optimize(state, getState().angle);
    System.out.println("Angular Position: " + getAngularPosition() + " " + state.angle.getRadians());
    System.out.println("Angular PID: " + angularPidController.calculate(getAngularPosition(), state.angle.getRadians()));
    speedMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    double angularSpeed = (angularPidController.calculate(getAngularPosition(), state.angle.getRadians()));
    /*if(angularSpeed > 0 && angularSpeed < 0.06){
      angularSpeed = 0.06;
    }
    else if(angularSpeed < 0 && angularSpeed > -0.06){
      angularSpeed = -0.06;
    }*/
    angularMotor.set(angularSpeed);
  } 
  public void stop(){
    speedMotor.set(0);
    angularMotor.set(0);
  }
}
