package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.team7419.PaddedXbox;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSub;

public class EncoderMove extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final DriveBaseSub driveBase;

  private final PaddedXbox joystick;

  private int _pov = -1;
  private final StringBuilder left = new StringBuilder();
  private final StringBuilder right = new StringBuilder();
  private int _smoothing = 0;

  private final double kF = 0.05;
  private final double kP = 0.5;
  private final double kI = 0.0;
  private final double kD = 0.001;
  public static int kPIDLoopIdx = 0;

  // driveBase.leftMast() and right are your talons

  public EncoderMove(final DriveBaseSub dB, final PaddedXbox joystick) {
    this.joystick = joystick;
    this.driveBase = dB;
    addRequirements(driveBase);

    driveBase.rightMast.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
    driveBase.rightMast.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

    // driveBase.leftMast.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,
    // 10);
    // driveBase.leftMast.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
    // 10);

    driveBase.rightMast.configNeutralDeadband(0.001);
    driveBase.rightMast.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // driveBase.leftMast.configNeutralDeadband(0.001);
    // driveBase.leftMast.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // driveBase.leftMast.configNeutralDeadband(0.001);
    // driveBase.leftMast.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    driveBase.rightMast.selectProfileSlot(0, kPIDLoopIdx);
    driveBase.rightMast.config_kF(0, kF);
    driveBase.rightMast.config_kP(0, kP);
    driveBase.rightMast.config_kI(0, kI);
    driveBase.rightMast.config_kD(0, kD);

    // driveBase.leftMast.selectProfileSlot(0, kPIDLoopIdx);
    // driveBase.leftMast.config_kF(0,kF);
    // driveBase.leftMast.config_kP(0, kP);
    // driveBase.leftMast.config_kI(0, kI);
    // driveBase.leftMast.config_kD(0, kD);

    // driveBase.leftMast.configMotionCruiseVelocity(15000);
    // driveBase.leftMast.configMotionAcceleration(6000);

    driveBase.rightMast.configMotionCruiseVelocity(15000);
    driveBase.rightMast.configMotionAcceleration(6000);

    // driveBase.leftMast.setSelectedSensorPosition(0, kPIDLoopIdx, 30);
    driveBase.rightMast.setSelectedSensorPosition(0, kPIDLoopIdx, 30);

  }

  @Override
  public void initialize() {
    driveBase.rightMast.configFactoryDefault();
    // driveBase.leftMast.configFactoryDefault();

  }

  @Override
  public void execute() {
    double leftY = -1.0 * joystick.getLeftY();
    double rightY = -1.0 * joystick.getRawAxis(5);

    if (Math.abs(leftY) < 0.1)
      leftY = 0;

    if (Math.abs(rightY) < 0.1)
      rightY = 0;

    final double rtMotorOut = driveBase.rightMast.getMotorOutputPercent();
    final double lfMotorOut = driveBase.leftMast.getMotorOutputPercent();

    // left talon
    left.append("\tOut%:");
    left.append(lfMotorOut);
    left.append("\tVel:");
    left.append(driveBase.leftMast.getSelectedSensorVelocity(kPIDLoopIdx));

    // right Talon
    right.append("\tOut%:");
    right.append(rtMotorOut);
    right.append("\tVel:");
    right.append(driveBase.rightMast.getSelectedSensorVelocity(kPIDLoopIdx));

    if (joystick.getRawButton(1)) {

      // 4096 ticks/rev * 10 Rotations in either direction
      final double targetPos = (rightY) * 4096 * 10;

      driveBase.rightMast.set(ControlMode.MotionMagic, targetPos);
      driveBase.leftMast.set(ControlMode.MotionMagic, targetPos);

      // //left talon
      // left.append("\tterr%:");
      // left.append(driveBase.leftMast.getClosedLoopError(kPIDLoopIdx));
      // left.append("\ttrg:");
      // left.append(targetPos);

      // right Talon
      right.append("\tterr%:");
      right.append(driveBase.rightMast.getClosedLoopError(kPIDLoopIdx));
      right.append("\ttrg:");
      right.append(targetPos);
    }

    else {
      driveBase.rightMast.set(ControlMode.MotionMagic, leftY);
      // driveBase.leftMast.set(ControlMode.MotionMagic, leftY);

    }

    if (joystick.getRawButton(2)) {
      /* Zero sensor positions */
      // driveBase.leftMast.setSelectedSensorPosition(0);
      driveBase.rightMast.setSelectedSensorPosition(0);
    }

    final int pov = joystick.getPOV();

    if (_pov == pov) {
      /* no change */
    } else if (_pov == 180) { // D-Pad down
      /* Decrease smoothing */
      _smoothing--;

      if (_smoothing < 0)
        _smoothing = 0;

      driveBase.rightMast.configMotionSCurveStrength(_smoothing);
      // driveBase.leftMast.configMotionSCurveStrength(_smoothing);

      System.out.println("Smoothing is set to: " + _smoothing);
    } else if (_pov == 0) { // D-Pad up
      /* Increase smoothing */
      _smoothing++;
      if (_smoothing > 8)
        _smoothing = 8;
      driveBase.rightMast.configMotionSCurveStrength(_smoothing);
      // driveBase.leftMast.configMotionSCurveStrength(_smoothing);

      System.out.println("Smoothing is set to: " + _smoothing);
    }
    _pov = pov; /* save the pov value for next time */

    /* Instrumentation */
    // Instrum.Process(driveBase.leftMast, left);
    Instrum.Process(driveBase.rightMast, right);

  }

  @Override
  public void end(final boolean interrupted) {
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}
