// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  
  public static class shooterConstants {
          public static final double kVelocity = 0.5;
          public static final double kVelocityStop = 0;
          public static final double kAngle = 0.5;
          public static final double kAngleStop = 0;
          public static final double kVelocityIntake = -5.0;
          public static final double kVelocityIntakeStop = 0;
          public static final double kAMax = 100;
          public static final double kAMin = 0;
          public static final double kVMax = 100;
                    //PID values for the ShooterMotor.
          public static final double kVP = 1;
          public static final double kVI = 0;
          public static final double kVD = 0;
                    //PID Values for the ShooterAngleMotor.
          public static final double kAP = 1;
          public static final double kAI = 0;
          public static final double kAD = 0;
  }

  public static class OperatorConstants {
         public static final int kDriverControllerPort = 0;
         public static final int kDemoControllerPort = 2;
         public static final double kDeadband = 0.01;
         public static final double kCubic = 0.95;
         public static final double kLinear = 0.05;
  }

  public static final class ModuleConstants {
    public static final class Drive {
         public static final double kGearRatio = (36.0 / 14.0) * (16.0 / 24.0) * (45.0 / 15.0);
         public static final double kWheelDiameter = 0.0985;
         public static final double kToMeters = (1.0 / kGearRatio) * kWheelDiameter * Math.PI;
         public static final double kToRots = 1 / kToMeters;
         public static final double kMaxNEORots = 96.0*11.0/12.0;
         public static final double kp = 0.0001;
         public static final double kf = 1.0/(kMaxNEORots*kToMeters);
         public static final double kMaxTheoSpeed = kMaxNEORots*kToMeters;   
    }

    public static final class Aziumth {
         public static final double kGearRatio = (50.0 / 12.0) * (72.0 / 12.0);
         public static final double kPositionFactor = 2 * Math.PI;
         public static final double kVelocityFactor = kPositionFactor / 60.0;
         public static final double kp = 0.35;
         public static final double rioKp = 0.8;
         public static final double rioKi = 0.0;
         public static final double rioKd = 0.0;
    }
}

public static final class DriveConstants {

        public static final double kWheelBaseWidth = 23.0/39.37;
        public static final double kWheelBaseLength = 18.0/39.37;
        public static final double kWheelBaseRadius = 0.5
           * Math.sqrt(Math.pow(kWheelBaseLength, 2) + Math.pow(kWheelBaseWidth, 2));
  

  public static final class FrontLeft {
    public static final int kDrive = 2;
    public static final int kAzimuth = 1;
    public static final int kEncoder = 0;
    public static final double kOffset = -1.94;
    public static final Translation2d kLocation = new Translation2d(0.5*kWheelBaseLength,0.5*kWheelBaseWidth );
  }

  public static final class FrontRight {
    public static final int kDrive = 4;
    public static final int kAzimuth = 3;
    public static final int kEncoder = 1;
    public static final double kOffset = -1.54;
    public static final Translation2d kLocation = new Translation2d(0.5*kWheelBaseLength, 0.5*kWheelBaseWidth);
  }

  public static final class RearLeft {
    public static final int kDrive = 8;
    public static final int kAzimuth = 7;
    public static final int kEncoder = 3;
    public static final double kOffset = -5.84;
    public static final Translation2d kLocation = new Translation2d(0.5*kWheelBaseLength, 0.5*kWheelBaseWidth);
  }

  public static final class RearRight {
    public static final int kDrive = 6;
    public static final int kAzimuth = 5;
    public static final int kEncoder = 2;
    public static final double kOffset = -3.22;
    public static final Translation2d kLocation = new Translation2d(0.5*kWheelBaseLength, 0.5*kWheelBaseWidth);
  }

  public static final double kTransSlewRate = 12.0;
  public static final double kRotSlewRate = 30.0;

  public static final double kMaxSpeedMetersPerSecond = 4.0;
  public static final double kMaxAngularSpeed = 2.0 * Math.PI;
  public static final double kMaxAngularAccel = 1.5 * Math.PI;

  

  public static final class KeepAngle {
    public static final double kp = 0.50;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
  }

  public static final  class Gryo {
    public static final int kModuleID = 0;
  }

  public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(FrontLeft.kLocation,
      FrontRight.kLocation, RearLeft.kLocation, RearRight.kLocation);

  public static final double kRotTransFactor = 0.085;
/* 
  public static final class Auto {

    public static final HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig,
                                                                                                  // this should
                                                                                                  // likely live in
                                                                                                  // your
        // Constants class
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(3.5, 0.0, 0.0), // Rotation PID constants
        4.8, // Max module speed, in m/s
        DriveConstants.kWheelBaseRadius, // D\[]rive base radius in meters. Distance from robot center to furthest
                                         // module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
    );
  } */
}
public static final class CurrentLimit {
  public static final int kIntake = 60;
  public static final int kFeeder = 60;
  public static final int kPivot = 30;
  public static final int kDrive = 50;
  public static final int kAzimuth = 20;
}
public static final class GlobalConstants {
  public static final double kVoltageCompensation = 11.0;
  public static double kLoopTime = 0.020;
}

}
