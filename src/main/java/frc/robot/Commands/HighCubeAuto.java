package frc.robot.Commands;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class HighCubeAuto extends CommandBase{
    public final Arm arm;
    public HighCubeAuto(Arm arm) {
        this.arm = arm;
    }
    public void execute() {
        arm.rotatePosition(Constants.armConstants.rotateHighCube);
        arm.elevatorPosition(Constants.armConstants.elevatorHighCube);
    }
    public boolean isFinished() {
        return 
        (Math.abs(arm.elevatorEncoderRight.getPosition() - Constants.armConstants.elevatorHighCube) < 1)
        &&
        (Math.abs(arm.rotateEncoder.getPosition() - Constants.armConstants.rotateHighCube) < 1);
      }


}
