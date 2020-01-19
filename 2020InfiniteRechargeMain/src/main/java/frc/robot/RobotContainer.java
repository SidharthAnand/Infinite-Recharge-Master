package frc.robot;

import com.team7419.PaddedXbox;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.drive.*;

public class RobotContainer {

  private final DriveBaseSub driveBase = new DriveBaseSub();
  private final Dashboard dashboard = new Dashboard();
  private final PaddedXbox joystick = new PaddedXbox();

  private final EncoderPrint encoderPrint = new EncoderPrint(joystick, driveBase, 0.4, 0.4);
  private final ArcadeDrive arcade = new ArcadeDrive(joystick, driveBase, .4, .4);
  private final EncoderMove straightFunc = new EncoderMove(driveBase, joystick);

  public RobotContainer() {
    //arcade.schedule();
    configureButtonBindings();
  }

  private void configureButtonBindings() {
  }

  public Command getDefaultTeleOpCommand(){
    return encoderPrint;
  }

  // public Command getAutonomousCommand() {
  //   return print;
  // }
}
