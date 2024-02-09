// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

      //Constants for swerve modules
      public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(2.0);
        public static final double kSpeedMotorGearRatio = 1 / 6.75;
        public static final double kAngularMotorGearRatio = 1 / 12.8;
        public static final double kSpeedEncoderRot2Meter = kSpeedMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kAngularEncoderRot2Rad = kAngularMotorGearRatio * 2 * Math.PI;
        public static final double kSpeedEncoderRPM2MeterPerSec = kSpeedEncoderRot2Meter / 60;
        public static final double kAngularEncoderRPM2RadPerSec = kAngularEncoderRot2Rad / 60;
        public static final double kPAngular = 0.5;//0.00015;
        public static final double kIAngular = 0.0;//0.00015;
        public static final double kDAngular = 0.05;//0.00015;
        public static final double kPSpeed = 0.00015;


        private static final int kEncoderPositionToMeters = 0;
        public static final int kEncoderVelocitytoMetersPerSecond = kEncoderPositionToMeters * 10;
    }

     public static final class DriveConstants {
        //Chassis Numbers
        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(22.5);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(25.5);
        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380 * ModuleConstants.kSpeedEncoderRPM2MeterPerSec;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond=3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        public static final SwerveDriveKinematics kDriveKindematics = new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2), 
          new Translation2d(kWheelBase /2, kTrackWidth / 2), 
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),                
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
        );
        // Start of fourth section uncommented build successful 12/4/23
        public static final int FRONT_LEFT_SPEED = 4;
        public static final int FRONT_LEFT_ANGULAR = 3;
        public static final boolean F_L_S_REVERSED = true;
        public static final boolean F_L_A_REVERSED = false;
        
        public static final int FRONT_RIGHT_SPEED = 5;
        public static final int FRONT_RIGHT_ANGULAR = 6;
        public static final boolean F_R_S_REVERSED = true;
        public static final boolean F_R_A_REVERSED = false; 
        
        public static final int BACK_LEFT_SPEED = 1;
        public static final int BACK_LEFT_ANGULAR = 2;
        public static final boolean B_L_S_REVERSED = true;
        public static final boolean B_L_A_REVERSED = false;
        
        public static final int BACK_RIGHT_SPEED = 7;
        public static final int BACK_RIGHT_ANGULAR = 8; 
        public static final boolean B_R_S_REVERSED = false;
        public static final boolean B_R_A_REVERSED = false;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 4;
      }
    
     public static final class OIConstants{
      public static final double kDeadband = 0.05;
      public static final int kDriverControllerPort = 0;
      public static final int kDriverYAxis = 1;
      public static final int kDriverXAxis = 0;
      public static final int kDriverRotAxis = 4;
      public static final int kDriverFieldOrientedButtonIdx = 1;
      
     }
      // End of fourth section uncommented
    public static final int DRIVER_JOYSTICK = 0;
    //Joystick #'s'
    public static final int LEFT_JOYSTICKV = 1;
    public static final int LEFT_JOYSTICKH = 0;
    public static final int RIGHT_JOYSTICKH = 4;
    public static final int Right_JOYSTICKV = 5;
    public static final int LT = 2;
    public static final int RT = 3;
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3; 
    public static final int Y = 4; 
    public static final int LB = 5; 
    public static final int RB = 6;
    public static final int BACK = 7; 
    public static final int START = 8;
  
  // Field Values 
    public static final double AMP_APRILTAG_HEIGHT = 53.38;
    public static final double STAGE_APRILTAG_HEIGHT = 52.06;
    public static final double SPEAKER_APRILTAG_HEIGHT = 57.13;
    public static final double SOURCE_APRILTAG_HEIGHT = 51.125;
    
  // Robot Values
    public static final double LIMELIGHT_HEIGHT = 31.750;
    public static final double LIMELIGHT_ANGLE = 60.0;
    
  
  public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
    }
    public final static double ROBOT_LENGTH = 1;
    public final static double ROBOT_WIDTH = 1;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 0;
  

}
