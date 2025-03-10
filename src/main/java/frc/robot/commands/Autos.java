// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command centerAuto(Drivetrain m_drive, Coral m_coral) {
    return Commands.run(() -> m_drive.drive(0, -.4, 0, true))
      .withTimeout(4)
      .andThen(() -> m_drive.drive(0, 0, 0, false))
      .andThen(m_coral.pinRelease())
      .andThen(Commands.waitSeconds(1))
      .andThen(m_coral.pinHold())
      .andThen(() -> m_drive.resetGyro(new Rotation2d(Units.degreesToRadians(-90))));
  }

  public static Command moveAuto(Drivetrain m_drive, Coral m_coral) {
    return Commands.run(() -> m_drive.drive(0, -.4, 0, true))
      .withTimeout(4)
      .andThen(() -> m_drive.drive(0, 0, 0, false));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
