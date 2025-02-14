package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.ModluleConstant.Drive;
import frc.robot.Constants.ModluleConstant.Turn;

public class SwerveModule {
    private final SparkMax m_drive;
    private final SparkMax m_turn;
    private final SparkMaxConfig m_driveconfig = new SparkMaxConfig();
    private final SparkMaxConfig m_turnconfig = new SparkMaxConfig();
    private final PIDController m_pid = new PIDController(Turn.kTurnP, 0, 0);
    private final AnalogEncoder m_turnencoder;
    private final double m_turnID;
    private final double m_offset;

    public SwerveModule(int driveID, int turnID, int encoderID, double encoderOffSet){
        m_drive = new SparkMax(driveID, MotorType.kBrushless);
        m_turn = new SparkMax(turnID, MotorType.kBrushless);
        m_turnID = turnID;
        m_turnencoder = new AnalogEncoder(encoderID);
        m_offset = encoderOffSet;

        m_driveconfig.inverted(true);
        m_driveconfig.idleMode(IdleMode.kBrake);
        //m_driveconfig.voltageCompensation(Constants.kVoltComp);
        m_driveconfig.smartCurrentLimit(CurrentLimit.kDrive);
        m_driveconfig.encoder.positionConversionFactor(Drive.kToMeters);
        m_driveconfig.encoder.velocityConversionFactor(Drive.kToMeters / 60.0);
        m_driveconfig.closedLoop.velocityFF(Drive.kFF);

        m_drive.configure(m_driveconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_turnconfig.inverted(false);
        m_turnconfig.idleMode(IdleMode.kBrake);
        //m_turnconfig.voltageCompensation(Constants.kVoltComp);
        m_turnconfig.smartCurrentLimit(CurrentLimit.kTurn);

        m_turn.configure(m_turnconfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_pid.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putBoolean("Driving", false);

    }
    
    public void setModuleState(SwerveModuleState state){
        state.optimize(getModuleAngle());
        m_drive.getClosedLoopController().setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        //m_drive.set(state.speedMetersPerSecond*Drive.kFF);
        double turnOutPut = m_pid.calculate(getModuleAngle().getRadians(), state.angle.getRadians());
        SmartDashboard.putNumber("Output"+m_turnID, turnOutPut);
        m_turn.set(turnOutPut);
        SmartDashboard.putBoolean("Driving"+m_turnID, true);
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(m_drive.getEncoder().getPosition(), getModuleAngle());
    }

    public Rotation2d getModuleAngle(){
        return new Rotation2d(Utilities.toUnitCircAngle(m_turnencoder.get() * 2 * Math.PI - m_offset));
    }
}
