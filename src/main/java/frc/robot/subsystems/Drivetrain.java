package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.FrontLeft;
import frc.robot.Constants.DriveConstants.FrontRight;
import frc.robot.Constants.DriveConstants.RearLeft;
import frc.robot.Constants.DriveConstants.RearRight;
import frc.robot.Constants.ModluleConstant.Drive;

public class Drivetrain extends SubsystemBase {
    private final SwerveModule m_FL = new SwerveModule(FrontLeft.kDriveID, FrontLeft.kTurnID, FrontLeft.kEncoderID, FrontLeft.m_offset);
    private final SwerveModule m_FR = new SwerveModule(FrontRight.kDriveID, FrontRight.kTurnID, FrontRight.kEncoderID, FrontRight.m_offset);
    private final SwerveModule m_RL = new SwerveModule(RearLeft.kDriveID, RearLeft.kTurnID, RearLeft.kEncoderID, RearLeft.m_offset);
    private final SwerveModule m_RR = new SwerveModule(RearRight.kDriveID, RearRight.kTurnID, RearRight.kEncoderID, RearRight.m_offset);
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    private final SlewRateLimiter m_slewX = new SlewRateLimiter(9.0);
    private final SlewRateLimiter m_slewY = new SlewRateLimiter(9.0);
    private final SlewRateLimiter m_rotationSlew = new SlewRateLimiter(18.0);

    private final SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(DriveConstants.kKinematics, m_gyro.getRotation2d(), getModulePosition());

    public Drivetrain(){
       
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
        xSpeed = m_slewX.calculate(xSpeed);
        ySpeed = m_slewY.calculate(ySpeed);
        rot = m_rotationSlew.calculate(rot);

        if (Math.abs(rot) < 0.02){
            rot = 0.0;
        }
        if (Math.abs(xSpeed) < 0.02){
            xSpeed = 0.0;
        }
        if (Math.abs(ySpeed) < 0.02){
            ySpeed = 0.0;
        }

        ChassisSpeeds m_output = new ChassisSpeeds(xSpeed, ySpeed, rot);

        if (fieldRelative){
            m_output = ChassisSpeeds.fromFieldRelativeSpeeds(m_output, m_gyro.getRotation2d());
        }

        setModuleStates(m_output);



    }
    public void setModuleStates(ChassisSpeeds desiredChassisSpeed){
        SwerveModuleState[] desiredStates = DriveConstants.kKinematics.toSwerveModuleStates(desiredChassisSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Drive.kMaxVel);
        m_FL.setModuleState(desiredStates[0]);
        m_FR.setModuleState(desiredStates[1]);
        m_RL.setModuleState(desiredStates[2]);
        m_RR.setModuleState(desiredStates[3]);
    }

    public Command resetGyro(Rotation2d angle){
        return runOnce(()->{
            m_gyro.reset();
            m_gyro.setAngleAdjustment(angle.getDegrees());
        });
    }

    public Rotation2d getGyro(){
        return m_gyro.getRotation2d();
    }

@Override
public void periodic() {
    SmartDashboard.putNumber("Front Left Encoder", m_FL.getModuleAngle().getRadians());
    SmartDashboard.putNumber("Front Right Encoder", m_FR.getModuleAngle().getRadians());
    SmartDashboard.putNumber("Rear Left Encoder", m_RL.getModuleAngle().getRadians());
    SmartDashboard.putNumber("Rear Right Encoder", m_RR.getModuleAngle().getRadians());
    updateOdometry();
    super.periodic();
}

    public void updateOdometry(){
        m_Odometry.update(m_gyro.getRotation2d(), getModulePosition());
    }

    public SwerveModulePosition[] getModulePosition(){
        return new SwerveModulePosition[]{
            m_FL.getModulePosition(), m_FR.getModulePosition(), m_RL.getModulePosition(), m_RR.getModulePosition()
        };
    }
}

