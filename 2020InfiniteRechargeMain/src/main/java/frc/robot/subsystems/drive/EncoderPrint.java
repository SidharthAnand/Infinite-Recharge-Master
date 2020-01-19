package frc.robot.subsystems.drive;

import com.team7419.PaddedXbox;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.*;
import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * Reusable arcade command
 */
public class EncoderPrint extends CommandBase {

    private DriveBaseSub driveBase;
    private double kStraight;
    private double kTurn;
    private PaddedXbox joystick;

    public EncoderPrint(PaddedXbox joystick, DriveBaseSub driveBase, double kStraight, double kTurn){
        this.joystick = joystick;
        this.driveBase = driveBase;
        this.kStraight = kStraight;
        this.kTurn = kTurn;
        addRequirements(driveBase);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double encoderLeftVal = driveBase.leftMast.getSelectedSensorPosition(0);
        double encoderRightVal = driveBase.rightMast.getSelectedSensorPosition(0);

        SmartDashboard.putNumber("Left Encoder", encoderLeftVal);
        SmartDashboard.putNumber("Right Encoder", encoderRightVal);

        SmartDashboard.putString("command status", "exec arcade");
    
        double leftPower = kTurn * joystick.getRightX() - kStraight * joystick.getLeftY();
        double rightPower = -kTurn * joystick.getRightX() - kStraight * joystick.getLeftY();
    
        driveBase.leftSide.setPower(leftPower);
        driveBase.rightSide.setPower(rightPower);
    
        if(joystick.getRightShoulder()){
          driveBase.getLeftMast().getSensorCollection().setQuadraturePosition(0, 10);
          driveBase.getRightMast().getSensorCollection().setQuadraturePosition(0, 10);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //encoder.reset();
    }

}
