// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax m_intake;
  private final double SPEED = -0.4; 

  /** Creates a new Intake. */
  public Intake() {
     m_intake = new SparkMax(IntakeConstants.intakeID, MotorType.kBrushed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command intakeIn() {
    return Commands.startEnd(() -> m_intake.set(SPEED), () -> m_intake.set(0));
  }

  public Command intakeOut() {
    return Commands.startEnd(() -> m_intake.set(-SPEED), () -> m_intake.set(0));
  }
}
