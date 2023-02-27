package frc.robot.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;

public class AutoPickUpCone extends CommandBase{
    public final Arm arm;
    public AutoPickUpCone(Arm arm) {
        this.arm = arm;
        arm.rotatePosition(Constants.armConstants.rotateHybrid);
        arm.elevatorPosition(Constants.armConstants.elevatorHybrid);
        Timer.delay(1);
        arm.clampInPositionCone();
        Timer.delay(.5);
        arm.rotatePosition(Constants.armConstants.rotateDrive);
        arm.elevatorPosition(Constants.armConstants.elevatorDrive);




    }
}
