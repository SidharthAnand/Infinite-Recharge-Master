package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Instrum {
	/* Tracking variables for instrumentation */
	private static int _loops = 0;
	private static int _timesInMotionMagic = 0;

	public static void Process(TalonSRX tal, StringBuilder sb) {
		/* Smart dash plots */
		SmartDashboard.putNumber("SensorVel", tal.getSelectedSensorVelocity(EncoderMove.kPIDLoopIdx));
		SmartDashboard.putNumber("SensorPos", tal.getSelectedSensorPosition(EncoderMove.kPIDLoopIdx));
		SmartDashboard.putNumber("MotorOutputPercent", tal.getMotorOutputPercent());
		SmartDashboard.putNumber("ClosedLoopError", tal.getClosedLoopError(EncoderMove.kPIDLoopIdx));
		
		/* Check if Talon SRX is performing Motion Magic */
		if (tal.getControlMode() == ControlMode.MotionMagic) {
			++_timesInMotionMagic;
		} else {
			_timesInMotionMagic = 0;
		}

		if (_timesInMotionMagic > 10) {
			/* Print the Active Trajectory Point Motion Magic is servoing towards */
			SmartDashboard.putNumber("ClosedLoopTarget", tal.getClosedLoopTarget(EncoderMove.kPIDLoopIdx));
    		SmartDashboard.putNumber("ActTrajVelocity", tal.getActiveTrajectoryVelocity());
    		SmartDashboard.putNumber("ActTrajPosition", tal.getActiveTrajectoryPosition());
		}

		/* Periodically print to console */
		if (++_loops >= 20) {
			_loops = 0;
			System.out.println(sb.toString());
		}

		/* Reset created string for next loop */
		sb.setLength(0);
	}
}