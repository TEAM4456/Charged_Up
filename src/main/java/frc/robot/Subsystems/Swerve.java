// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final AHRS m_gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  private PIDController m_balancePID = new PIDController(2, 0, 0);

  public Field2d field;
  private final Arm arm;
  public Swerve(Arm arm) {
    this.arm = arm;
    m_gyro = new AHRS(SPI.Port.kMXP);
    m_gyro.setAngleAdjustment(180);
    //.configFactoryDefault();
    zeroHeading();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    Timer.delay(1.0);
    resetModulesToAbsolute();

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getRotation2d(), getModulePositions());

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }
  
  public void drive(
      Translation2d translation, double rotation, /*boolean fieldRelative,*/ boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getRotation2d()));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }
/*  Drive with field relative boolean
  public void drive(
    Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
  SwerveModuleState[] swerveModuleStates =
      Constants.Swerve.swerveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  translation.getX(), translation.getY(), rotation, getYaw())
              : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
  SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

  for (SwerveModule mod : mSwerveMods) {
    mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
  }
}
*/

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroHeading() {
    m_gyro.reset();
  }
  public void zeroHeadingAdjust() {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);
  }
  public void setHeading(){
    m_gyro.setAngleAdjustment(180);
  }

  public double getHeading() {
    return Math.IEEEremainder(-m_gyro.getAngle(), 360);
}
  public double getGyroRoll(){
    return m_gyro.getRoll();
  }

  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }
/*
public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
}
*/

 /* 
  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }
  */

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}

  public void resetModulesToAbsolute() {
      for(SwerveModule mod : mSwerveMods) {
        mod.resetToAbsolute();
        System.out.println("Modules Reset to Absolute");
      }
  }

  public CommandBase followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
   return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommand(
            traj, 
            this::getPose, // Pose supplier
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDController(Constants.AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            this::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
    }
    public void autoBalance() {
      m_balancePID.setTolerance(.1);
      double pidOutput;
      pidOutput = MathUtil.clamp(m_balancePID.calculate(m_gyro.getRoll(), 0), -0.4, 0.4);
      drive(new Translation2d(-pidOutput, 0), 0.0, false);
      SmartDashboard.putNumber("gyro PID output", pidOutput);
      System.out.println("ran");
    }

    public CommandBase autoBalanceContinuous() {
      return run(() -> autoBalance()).until(() -> Math.abs(m_gyro.getRoll()) < 0);
    }

    public void driveTo(double locationX, double locationY){
      Pose2d locationList = field.getRobotPose();
      double currentX= locationList.getX();
      double currentY = locationList.getY();
      double targetX = currentX - locationX;
      double targetY = currentY - locationY;
      if(currentX < 0.99 && currentX > 1.0){
        arm.setHybridPositionRotate();
      }
      if(targetX > 2){
        targetX = 2;
      }else if(targetX > 1){
        targetX = 1;
      }
      else if(targetX < -2){
        targetX = -2;
      }
      else if(targetX < -1){
        targetX = -1;
      }

      if(targetY > 1){
        targetY = 1;
      }else if(targetY > .5){
        targetY = .5;
      }
      else if(targetY < -.1){
        targetY = -1;
      }
      else if(targetY < -.5){
        targetY = -.5;
      }
      if(Math.abs(targetX) < .025){
        targetX = 0;
      }
      if(Math.abs(targetY) < .025){
        targetY = 0;
      }
      SmartDashboard.putNumber("targetX", targetX);
      SmartDashboard.putNumber("targetY",targetY);
      drive(new Translation2d(targetX*2,targetY), 0, false);



    }
    public Command driveToCommand(double locX, double locY){
      return run(() -> driveTo(locX,locY)).until(() -> (Math.abs(field.getRobotPose().getX() - locX) < .025 && Math.abs(field.getRobotPose().getY() - locY) < .025));
    }
    public Command driveToCommandGeneral(double locX, double locY){
      return run(() -> driveTo(locX,locY)).until(() -> (Math.abs(field.getRobotPose().getX() + locX) < 1.5 && Math.abs(field.getRobotPose().getY() - locY) < .25));
    }
  @Override
  public void periodic() {
    swerveOdometry.update(getRotation2d(), getModulePositions());
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
    SmartDashboard.putNumber("poseX", field.getRobotPose().getX());
    SmartDashboard.putNumber("poseY", field.getRobotPose().getY());
    SmartDashboard.putNumber("NAVX Heading", this.getHeading());
  }
}
