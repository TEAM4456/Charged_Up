package frc.robot.Commands;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class HighCube extends CommandBase{
    public final Arm arm;
    public HighCube(Arm arm) {
        this.arm = arm;
    }
    public void execute() {
        arm.rotatePosition(Constants.armConstants.rotateHighCube);
        arm.elevatorPosition(Constants.armConstants.elevatorHighCube);
    }

}
