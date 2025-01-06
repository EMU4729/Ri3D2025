package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.constants.AlgaeArmConstants;
import frc.robot.constants.CoralArmConstants;
import frc.robot.constants.ElevatorConstants;

public class ActivateAlgaeL3 extends ActivateBase{
    public ActivateAlgaeL3(){this(false);}
    public ActivateAlgaeL3(boolean StayUnstowed){
        super(StayUnstowed);
    }

    @Override
    public void initialize() {
        super.initialize();
        Subsystems.elevator.setTargetHeight(ElevatorConstants.HEIGHTS.ALGAE_L3);
        Subsystems.algaeArm.setTargetAngle(AlgaeArmConstants.ANGLES.LOAD_START);
        //start wheels
    }

    @Override
    public void end(boolean interrupted) {
        if(stayUnstowed){return;}
        Subsystems.algaeArm.setTargetAngle(AlgaeArmConstants.ANGLES.STOW);
        SequentialCommandGroup delayedRuns = new SequentialCommandGroup();
        delayedRuns.addCommands(
            new WaitCommand(2),
            new InstantCommand(()->Subsystems.elevator.setTargetHeight(ElevatorConstants.HEIGHTS.STOW))
        );
        delayedRuns.schedule();
    }
}
