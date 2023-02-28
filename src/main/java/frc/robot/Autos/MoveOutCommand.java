// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;



public class MoveOutCommand extends CommandBase {
  public final Swerve s_Swerve;
  public MoveOutCommand(Swerve s){
    s_Swerve = s;

    PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(3, 2));
    s_Swerve.followTrajectoryCommand(examplePath,true);
    
  }

}

