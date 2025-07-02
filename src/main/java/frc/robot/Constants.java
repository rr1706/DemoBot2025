// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import java.awt.geom.Point2D;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final class ModuleConstants {
    public static final class Drive {
      public static final double kGearRatio = (36.0 / 14.0) * (16.0 / 24.0) * (45.0 / 15.0);
      public static final double kWheelDiameter = 0.0985;
      public static final double kToMeters = (1.0 / kGearRatio) * kWheelDiameter * Math.PI;
      public static final double kToRots = 1 / kToMeters;
      public static final double kMaxNEORots = 96.0;
      public static final double kp = 0.0001;
      public static final double kf = 1.0/(kMaxNEORots*kToMeters);
   
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
    public static final int kModuleID = 1;
    public static final double kOffset = Math.PI-0.15266667306423187;
    public static final Translation2d kLocation = new Translation2d(9.000/39.37, 11.500/39.37);
  }

  public static final class FrontRight {
    public static final int kModuleID = 2;
    public static final double kOffset = 3*Math.PI-3.568241596221924;
    public static final Translation2d kLocation = new Translation2d(9.000/39.37, -11.500/39.37);
  }

  public static final class RearLeft {
    public static final int kModuleID = 4;
    public static final double kOffset = 2*Math.PI-2.6641688346862793;
    public static final Translation2d kLocation = new Translation2d(-9.000/39.37, 11.500/39.37);
  }

  public static final class RearRight {
    public static final int kModuleID = 3;
    public static final double kOffset = 2*Math.PI-6.256606578826904;
    public static final Translation2d kLocation = new Translation2d(-9.000/39.37, -11.500/39.37);
  }

  public static final double kTransSlewRate = 16.0;
  public static final double kRotSlewRate = 30.0;

  public static final double kMaxSpeedMetersPerSecond = 5.6;
  public static final double kMaxAngularSpeed = 2.0 * Math.PI;
  public static final double kMaxAngularAccel = 1.5 * Math.PI;

  

  public static final class KeepAngle {
    public static final double kp = 0.50;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
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
  public static final double kVoltageCompensation = 12.6;
  public static double kLoopTime = 0.020;
}

}
