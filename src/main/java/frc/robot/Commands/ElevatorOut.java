
package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ElevatorOut extends CommandBase{
    public final Arm arm;
    public ElevatorOut(Arm arm){
        this.arm = arm;
    }
    public void execute() {
        arm.elevatorOut();
    }
}