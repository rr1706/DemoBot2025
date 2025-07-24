// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeConstants {
    public static final double kOut = -60;
    // Out angle for the rotation motor.

    public static final double kVelocity = 1;
    // Speed at which the rollers will spin.

    public static final double kIn = 0;
    // In angle for the rotation Motor.

    public static final double kVelocityIn = 0;
    // Speed for disabling the rollers.
  }

  public static class ShooterConstants {
    public static final double kVelocity = 50;
    public static final double kVelocityStop = 0;
  }
}
