package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.gripper.GripperSubsystem;

public class ReleaseGamepiece extends Command {

    private final GripperSubsystem m_gripperSubsystem;
    private boolean m_gotPiece = true;
    private int m_releasePieceCountdown = 0;

    private boolean m_finished = false;
    private boolean m_OKToMove = false;
    private final ArmLevel m_ArmLevel;

    public ReleaseGamepiece(GripperSubsystem gripperSubsystem, ArmLevel armLevel) {

        m_gripperSubsystem = gripperSubsystem;
        addRequirements(m_gripperSubsystem);
        m_ArmLevel = armLevel;
    }

    @Override
    public void initialize() {
        m_gotPiece = true;
        m_releasePieceCountdown = 15;
        m_finished = false;
        m_OKToMove = false;

    }

    @Override
    public void execute() {

        if (m_OKToMove) {
            m_gotPiece = m_gripperSubsystem.lightSensorTripped();

            if (m_gotPiece) // wait for piece in intake position position
            {
                if (m_ArmLevel == ArmLevel.Four || m_ArmLevel == ArmLevel.Two) {
                    m_gripperSubsystem.setIntakeSpeed(-GripperConstants.GRIPPER_RELEASE_SPEED);
                } else {
                    m_gripperSubsystem.setIntakeSpeed(GripperConstants.GRIPPER_RELEASE_SPEED);
                }
            } else if (m_releasePieceCountdown > 0) {
                m_releasePieceCountdown--;

            } else // got piece and its far enough in
            {
                m_finished = true;
            }
        } else {
            {
                m_OKToMove = true;
            }

        }

    }

    @Override
    public void end(boolean interrupted) {
        m_gripperSubsystem.setIntakeSpeed(0.0);

    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
