// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
 private final DoubleSolenoid m_coralArm = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
 private final DoubleSolenoid m_coralPin = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
 
private int armState;
private int pinState;

  public Coral() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command liftArm() {
    armState = 1;
    return runOnce(
        () -> {
          m_coralArm.set(DoubleSolenoid.Value.kForward);
        });
  }
  public Command lowerArm(){
    armState = -1;
    return runOnce(
      () -> {
        m_coralArm.set(DoubleSolenoid.Value.kReverse);
      }
    );
  }
  public Command pinHold(){
    pinState = 1;
    return runOnce(
      () -> {
        m_coralPin.set(DoubleSolenoid.Value.kForward);
      }
    );
  }
  public Command pinRelease(){
    pinState = -1;
    return runOnce(
      () -> {
        m_coralPin.set(DoubleSolenoid.Value.kReverse);
      }
    );
  }

  

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean isArmUp() {
    return armState == 1;
  }
  public boolean isPinUp(){
    return pinState == 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
