package frc.robot.Autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve;

public class AutoBalanceAuto extends CommandBase{
    public PIDController m_balancePID;
    public Swerve s_Swerve;
    public AutoBalanceAuto(Swerve s_Swerve){
        this.s_Swerve = s_Swerve;
    }
    public void execute(){
        s_Swerve.autoBalance();
    }
    
}
