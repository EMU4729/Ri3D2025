package frc.robot.commands;

import javax.xml.stream.events.EndDocument;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AlgaeArmConstants;
import frc.robot.constants.CoralArmConstants;
import frc.robot.constants.ElevatorConstants;

public class ActivateStow extends Command{

    public ActivateStow(){
        addRequirements(Subsystems.coralArm, Subsystems.algaeArm, Subsystems.elevator);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.coralArm.setTargetAngle(CoralArmConstants.ANGLES.STOW);
        Subsystems.algaeArm.setTargetAngle(AlgaeArmConstants.ANGLES.STOW);
        Subsystems.elevator.setTargetHeight(ElevatorConstants.HEIGHTS.STOW);
        super.end(interrupted);
    }
}