package frc.robot.Autos;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class AutoDropLowCone extends CommandBase{
    public final Arm arm;
    public AutoDropLowCone(Arm arm) {
        this.arm = arm;
    }
    public void execute(){
        arm.rotatePosition(Constants.armConstants.rotateLowCone);
        arm.elevatorPosition(Constants.armConstants.elevatorLowCone);
        Timer.delay(1);
        arm.clampOutPosition();
        Timer.delay(.5);
    }
}

