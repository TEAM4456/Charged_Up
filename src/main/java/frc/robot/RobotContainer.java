// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Commands.autoToPose;
import frc.robot.Commands.drivePosition;
import frc.robot.Commands.limeLightSwerve;
import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.Swerve;
import frc.robot.Autos.AutoDropLowCone;
import frc.robot.Autos.AutoDropStart;
import frc.robot.Autos.AutoPickUpCone;
import frc.robot.Commands.ClampIn;
import frc.robot.Commands.ClampInLeft;
import frc.robot.Commands.ClampInRight;
import frc.robot.Commands.ClampOut;
import frc.robot.Commands.ClampOutLeft;
import frc.robot.Commands.ClampOutRight;
import frc.robot.Commands.ClampPositionCone;
import frc.robot.Commands.ClampPositionCube;
import frc.robot.Commands.ClampPositionDrop;
import frc.robot.Commands.pivotDownSpeed;
import frc.robot.Commands.pivotUpSpeed;
import frc.robot.Commands.reset;
import frc.robot.Commands.rotateDown;
import frc.robot.Commands.rotateUp;
import frc.robot.Commands.toggleSpeed;
import frc.robot.Subsystems.Arm;
import frc.robot.Commands.ClampInRight;
import frc.robot.Commands.ClampOutRight;

import frc.robot.Commands.ElevatorIn;
import frc.robot.Commands.ElevatorOut;
import frc.robot.Commands.HighCone;
import frc.robot.Commands.HighCube;
import frc.robot.Commands.Hybrid;
import frc.robot.Commands.LowCone;
import frc.robot.Commands.LowCube;
import frc.robot.Commands.RotateToPosition;
import frc.robot.Commands.rotateDown;
import frc.robot.Commands.rotateUp;
import frc.robot.Subsystems.Arm;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, Commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController second = new CommandXboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value ;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
 // private final JoystickButton zeroGyro =
 //     new JoystickButton(driver, XboxController.Button.kY.value);
 //private final JoystickButton robotCentric =
 //     new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Arm arm = new Arm();

  private final SendableChooser<Command> m_Chooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and Commands. */
  public Command moveOutCommand;
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)));

    // Configure the button bindings
    configureButtonBindings();
/*     TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        //Trajectory's
        
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1), new Translation2d(3,0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 0, new Rotation2d(0)),
                config);
          Trajectory moveOut =
            TrajectoryGenerator.generateTrajectory(
              new Pose2d(0, 0, new Rotation2d(0)),
              List.of(new Translation2d(-2, 0)),
              new Pose2d(-4, 0, new Rotation2d(Math.PI)),
              config);
          Trajectory moveBackClose = 
            TrajectoryGenerator.generateTrajectory(
              new Pose2d(-4, 0, new Rotation2d(Math.PI)),
              List.of(),
              new Pose2d(-1, 0, new Rotation2d(0)),
              config);
          Trajectory moveBackFinal = 
            TrajectoryGenerator.generateTrajectory(
              new Pose2d(-1, 0, new Rotation2d(Math.PI)),
              List.of(),
              new Pose2d(0, 0, new Rotation2d(0)),
              config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

        new InstantCommand(() -> s_Swerve.resetOdometry(moveOut.getInitialPose()));
        moveOutCommand = new SwerveControllerCommand(
                moveOut,
                s_Swerve::getPose,Constants.Swerve.swerveKinematics,new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),thetaController,s_Swerve::setModuleStates,s_Swerve);
        
        m_Chooser.addOption("Nothing", new InstantCommand());
        //m_Chooser.addOption("Starting Drop", new AutoDropStart(arm));
        //m_Chooser.addOption("Low Drop", new AutoDropLowCone(arm));
        m_Chooser.addOption("Move Out",moveOutCommand);
        //m_Chooser.addOption("Pick Up Cone", new AutoPickUpCone(arm));

        SmartDashboard.putData("Auto Chooser", m_Chooser);
        
  */     
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    //driver.x().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    //driver.start().whileTrue(new limeLightSwerve(s_Swerve));
 //    driver.a().whileTrue(new moveUpCommand(arm));
//    driver.y().whileTrue(new moveDownCommand(arm));
//


    driver.back().whileTrue(s_Swerve.autoBalanceContinuous());
    driver.back().toggleOnTrue(
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)));

    driver.y().whileTrue(new pivotUpSpeed(arm));
    driver.a().whileTrue(new pivotDownSpeed(arm));

    driver.x().whileTrue(new ElevatorIn(arm));
    driver.b().whileTrue(new ElevatorOut(arm));

    driver.leftTrigger().whileTrue(new ClampPositionCone(arm));
    driver.rightBumper().whileTrue(new ClampPositionDrop(arm));
    driver.rightTrigger().whileTrue(new ClampPositionCube(arm));

    driver.leftBumper().whileTrue(new Hybrid(arm));

    second.leftTrigger().whileTrue(new ClampOut(arm));
    second.rightTrigger().whileTrue(new ClampIn(arm));

    second.leftBumper().whileTrue(new drivePosition(arm));
    second.rightBumper().whileTrue(new reset(arm));


    //second.start().whileTrue(new limeLightSwerve(s_Swerve));
    second.y().whileTrue(new HighCube(arm));
    second.x().whileTrue(new LowCone(arm));
    second.b().whileTrue(new HighCone(arm));
    second.a().whileTrue(new LowCube(arm));
    


/* 
    driver.leftBumper().whileTrue(new ClampInRight(arm));
    driver.rightBumper().whileTrue(new ClampOutRight(arm));
//
    driver.rightTrigger().whileTrue(new ElevatorOut(arm));
    driver.leftTrigger().whileTrue(new ElevatorIn(arm));

    driver.b().whileTrue(new pivotUpSpeed(arm));
    driver.x().whileTrue(new pivotDownSpeed(arm));
   

   
    driver.start().toggleOnTrue(new RotateToPosition(arm));
    */

    

  
  }
  public Swerve getSwerve(){
    return s_Swerve;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  

  public Command getAutonomousCommand() {
    s_Swerve.zeroHeading();
    s_Swerve.resetModulesToAbsolute();
    //PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(3, 2));
    PathPlannerTrajectory traj = PathPlanner.generatePath(
      new PathConstraints(3, 3),
      new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation
      new PathPoint(new Translation2d(-5.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
     // new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-90)) // position, heading(direction of travel), holonomic rotation
      );
    // This will load the file "Example Path.path" and generate it with a max
    // velocity of 3 m/s and a max acceleration of 2 m/s^2

    //s_Swerve.field.getObject("traj").setTrajectory(examplePath);

    return new SequentialCommandGroup(
     
      new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            s_Swerve.resetOdometry(traj.getInitialPose());

        }, s_Swerve),

        new PPSwerveControllerCommand(
          traj,
          s_Swerve::getPose, // Pose supplier
          Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
          new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(1, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          s_Swerve::setModuleStates, // Module states consumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          s_Swerve // Requires this drive subsystem
        ));
    // An ExampleCommand will run in autonomous
       // return m_Chooser.getSelected();
    /*     TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
    
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1), new Translation2d(3,0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(0, 0, new Rotation2d(0)),
            config);
     
        Trajectory moveOut =
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(-2, 0)),
          new Pose2d(-4, 0, new Rotation2d(Math.PI)),
          config);
      Trajectory moveBackClose = 
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(-4, 0, new Rotation2d(Math.PI)),
          List.of(),
          new Pose2d(-1, 0, new Rotation2d(0)),
          config);
      Trajectory moveBackFinal = 
        TrajectoryGenerator.generateTrajectory(
          new Pose2d(-1, 0, new Rotation2d(Math.PI)),
          List.of(),
          new Pose2d(0, 0, new Rotation2d(0)),
          config);
          

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

    new InstantCommand(() -> s_Swerve.resetOdometry(moveOut.getInitialPose()));
        return new SwerveControllerCommand(
              moveOut,
              s_Swerve::getPose,Constants.Swerve.swerveKinematics,new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),thetaController,s_Swerve::setModuleStates,s_Swerve);
        
       */
  
              

    }
      // This will load the file "Example Path.path" and generate it with a max
      // velocity of 3 m/s and a max acceleration of 2 m/s^2

      //s_Swerve.field.getObject("traj").setTrajectory(examplePath);

         
          
  
}
