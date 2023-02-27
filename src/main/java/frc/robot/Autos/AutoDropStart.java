package frc.robot.Autos;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class AutoDropStart extends CommandBase{
    public final Arm arm;
    public AutoDropStart(Arm arm) {
        this.arm = arm;
        arm.rotatePosition(Constants.armConstants.rotateHighCone);
        arm.elevatorPosition(Constants.armConstants.elevatorHighCone);
        Timer.delay(1);
        arm.clampOutPosition();
        Timer.delay(.5);




    }
}
