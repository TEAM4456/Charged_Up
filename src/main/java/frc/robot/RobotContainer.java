// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Commands.autoToPose;
import frc.robot.Commands.limeLightSwerve;
import frc.robot.Subsystems.LimeLightSubsystem;
import frc.robot.Subsystems.Swerve;

import frc.robot.Commands.ClampInLeft;
import frc.robot.Commands.ClampInRight;
import frc.robot.Commands.ClampOutLeft;
import frc.robot.Commands.ClampOutRight;
import frc.robot.Commands.moveDownCommand;
import frc.robot.Commands.moveUpCommand;
import frc.robot.Commands.pivotDownSpeed;
import frc.robot.Commands.pivotUpSpeed;
import frc.robot.Commands.rotateDown;
import frc.robot.Commands.rotateUp;
import frc.robot.Commands.toggleSpeed;
import frc.robot.Subsystems.Arm;
import frc.robot.Commands.ClampInRight;
import frc.robot.Commands.ClampOutRight;
import frc.robot.Commands.ClampPosition;
import frc.robot.Commands.ElevatorIn;
import frc.robot.Commands.ElevatorOut;
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
  private final int translationAxis = (XboxController.Axis.kLeftY.value * .8);
  private final int strafeAxis = (XboxController.Axis.kLeftX.value * .8);
  private final int rotationAxis = (XboxController.Axis.kRightX.value * .8);

  /* Driver Buttons */
 // private final JoystickButton zeroGyro =
 //     new JoystickButton(driver, XboxController.Button.kY.value);
 //private final JoystickButton robotCentric =
 //     new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Arm arm = new Arm();



  /** The container for the robot. Contains subsystems, OI devices, and Commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)));

    // Configure the button bindings
    configureButtonBindings();
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
    driver.leftTrigger().whileTrue(new ClampOutLeft(arm));
    driver.leftBumper().whileTrue(new ClampInLeft(arm));

    driver.rightTrigerr().whileTrue(new ClampOutRight(arm));
    driver.rightBumpber().whileTrue(new clampInLeft(arm));

    driver.back().toggleOnTrue(new toggleSpeed(s_Swerve));
    driver.start().whileTrue(new limeLightSwerve(s_Swerve));

    driver.y().whileTrue(new pivotUpSpeed(arm));
    driver.a().whileTrue(new pivotDownSpeed(arm));

    driver.x().whileTrue(new ElevatorIn(arm));
    driver.b().whileTrue(new ElevatorIn(arm));

    second.leftTrigger().whileTrue(new ClampPositionCone(arm));



    driver.leftBumper().whileTrue(new ClampInRight(arm));
    driver.rightBumper().whileTrue(new ClampOutRight(arm));
//
    driver.rightTrigger().whileTrue(new ElevatorOut(arm));
    driver.leftTrigger().whileTrue(new ElevatorIn(arm));

    driver.b().whileTrue(new pivotUpSpeed(arm));
    driver.x().whileTrue(new pivotDownSpeed(arm));
   
    driver.a().toggleOnTrue(new ClampPosition(arm));
   
    driver.start().toggleOnTrue(new RotateToPosition(arm));

    driver.back().toggleOnTrue(
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)));

  
  }
  public Swerve getSwerve(){
    return s_Swerve;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public getAutonomousCommand() {
    private static final String kDefaultAuto = "Default";
    private Static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_Chooser = new SendableChooser<>();

    m_Chooser.setDefaultOption("Default Auto",kDefaultAuto);
    m_Choooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putDate("Auto Choiser", m_Chooser);
     /* 
    // An ExampleCommand will run in autonomous
    return null;
 
      s_Swerve.zeroHeading();
      s_Swerve.resetModulesToAbsolute();
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1), new Translation2d(3,0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

        new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose()));
        return new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

    }
      // This will load the file "Example Path.path" and generate it with a max
      // velocity of 3 m/s and a max acceleration of 2 m/s^2

      //s_Swerve.field.getObject("traj").setTrajectory(examplePath);
*/
         
          
  
}
