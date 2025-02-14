package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Utilities;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveByController extends Command {
    private final Drivetrain m_drivetrain;
    private final CommandXboxController m_controller;

public DriveByController(Drivetrain drive, CommandXboxController controller){
    m_drivetrain = drive;
    m_controller = controller;
    addRequirements(m_drivetrain);
}

@Override
public void execute() {
    var alliance = DriverStation.getAlliance();

    double xInput = -m_controller.getLeftY();
    double yInput = -m_controller.getLeftX();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
        xInput = -xInput;
        yInput = -yInput;
    }

    double desiredTrans[] = Utilities.inputTransform(xInput, yInput);
    desiredTrans[0] *= DriveConstants.kMaxSpeed;
    desiredTrans[1] *= DriveConstants.kMaxSpeed;

    double desiredRot = -Utilities.inputTransform(m_controller.getRightX()) * DriveConstants.kMaxAngularSpeed;

    m_drivetrain.drive(desiredTrans[0], desiredTrans[1], desiredRot, true);
    }
}
