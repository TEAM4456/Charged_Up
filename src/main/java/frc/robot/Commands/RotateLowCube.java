package frc.robot.Commands;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class RotateLowCube extends CommandBase{
    public final Arm arm;
    public RotateLowCube(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.rotatePosition(Constants.rotateLowCube);
    }

}
