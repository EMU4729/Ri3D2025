package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class resetAll extends Command{
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        Subsystems.algaeArm.reset();
        Subsystems.coralArm.reset();
        Subsystems.elevator.reset();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
