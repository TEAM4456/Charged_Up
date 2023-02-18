package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class moveDownCommand extends CommandBase{
    public final Arm arm;
    public moveDownCommand(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.armDown();
    }
    public void end(boolean interrupted) {
        arm.armStop();

    }
}
