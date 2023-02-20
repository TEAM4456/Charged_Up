package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class moveUpCommand extends CommandBase{
    public final Arm arm;
    public moveUpCommand(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.armUp();
    }
}

