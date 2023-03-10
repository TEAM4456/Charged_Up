// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.configs.SwerveModuleConstants;


public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(20.75);
    public static final double wheelBase = Units.inchesToMeters(27.5);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (8.14 / 1.0); // 6.75:1
    public static final double angleGearRatio = (150/7 / 1.0); //150/7:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int driveContinuousCurrentLimit = 40;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 1.0;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.467;
    public static final double driveKV = 1.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 3.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Back Right Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(54.074);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(260.297);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(280.787);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Left Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(45.8);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    public int getGyroRoll() {
      return 0;
    }

    public void autoBalance() {
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = .01;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();
  }
  public static final class armConstants {
    //Elevator Constants
    
    public static final double elevatorHighCone = -75.35;
    public static final double elevatorHighCube = -67;
    public static final double elevatorLowCone = -26.10;
    public static final double elevatorLowCube = 0;
    public static final double elevatorHybrid = -14.5;
    public static final double elevatorDrive = -3;
    public static final double elevatorPickup = -3;

    //Rotate Constants
    public static final double rotateHighCone = 181.7;
    public static final double rotateHighCube = 208.79;
    public static final double rotateLowCone = 212.37;
    public static final double rotateLowCube = 255;
    public static final double rotateHybrid = 355.3;
    public static final double rotateDrive = 10;
    public static final double rotatePickup = 195;

    //Right Clamp Constants
    public static final double clampRightPickupCone =12.85 ;
    public static final double clampRightPickupCube = 11.5;
    public static final double clampRightDrop = 10.05;
    
    //Left Clamp Constants
    public static final double clampLeftPickupCone = 12.85;
    public static final double clampLeftPickupCube =11.5 ;
    public static final double clampLeftDrop = 10.05;

    //While Driving
    public static final double elevatorStowed = 0;
    public static final double rotateStowed = 31.96;

  }
}