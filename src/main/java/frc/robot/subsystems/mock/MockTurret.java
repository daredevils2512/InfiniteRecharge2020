package frc.robot.subsystems.mock;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.subsystems.interfaces.ITurret;
import frc.robot.utils.DareMathUtil;

public class MockTurret implements ITurret {

    private final WPI_TalonSRX motor;
    private final double m_maxTurnDegrees;
    private final double m_tolerance;
    private final int m_encoderResolution;
    private final double m_gearRatio;

    public MockTurret(WPI_TalonSRX motor, double maxTurnDegrees, double tolerance, int encoderResolution, double gearRatio) {
        this.motor = motor;
        resetEncoder();
        m_tolerance = tolerance;
        m_maxTurnDegrees = maxTurnDegrees;
        m_encoderResolution = encoderResolution;
        m_gearRatio = gearRatio;
    }

    @Override
    public Map<String, Object> getValues() {
        return null;
    }

    @Override
    public double getAngle() {
        return toDegrees(motor.getSelectedSensorPosition());
    }

    public double getEncoderPosition() {
        return motor.getSelectedSensorPosition();
    }

    @Override
    public void resetEncoder() {
        motor.setSelectedSensorPosition(0);
    }

    public void setEncoderPosition(double degrees) {
        motor.setSelectedSensorPosition(toEncoderPulses(degrees));
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    public double getSpeed() {
        return motor.getMotorOutputPercent();
    }

    @Override
    public void runPosition(double degrees) {
        motor.set(ControlMode.MotionMagic, degrees);
    }

    @Override
    public double wrapDegrees(double degrees) {
        return ((degrees + Math.signum(degrees) * m_maxTurnDegrees) % 360) - Math.signum(degrees) * m_maxTurnDegrees;
    }

    @Override
    public void setTargetAngle(double angle) {
    }

    @Override
    public boolean isAtSetpoint() {
        return DareMathUtil.isWithinXOf(motor.getClosedLoopError(), 0.0, m_tolerance);
    }

    public double toDegrees(int encoderPulses) {
        return (double) (encoderPulses / m_encoderResolution) * 360 * m_gearRatio;
    }

    // returns a fused heading problaby
    public int toEncoderPulses(double angle) {
        int x = (int) ((angle / 360) / m_gearRatio * m_encoderResolution);
        return x;
    }
    

    
}