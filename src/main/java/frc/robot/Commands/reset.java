package frc.robot.Commands;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class reset extends CommandBase{
    public final Arm arm;
    public reset(Arm arm) {
        this.arm = arm;
    }
    public void execute() {
        arm.rotatePosition(0);
        arm.elevatorPosition(0);
        arm.clampReset();
    }

}