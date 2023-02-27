// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class pivotUpSpeed extends CommandBase {
  /** Creates a new pivotUpSpeed. */

  public final Arm arm;

  public pivotUpSpeed(Arm arm) {
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  
  @Override
  public void execute() {
    if(arm.rotateEncoder.getPosition()<1){
      arm.rotateSpeedHold();
    }
    else{
    arm.rotateSpeedUp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.motor17.set(0);
    Timer.delay(.5);
    arm.rotateSpeedHold();
  }
}
