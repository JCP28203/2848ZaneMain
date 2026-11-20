package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.kTurretCANcoderID;
import static frc.robot.Constants.ShooterConstants.kTurretMotorID;
import static frc.robot.Constants.ShooterConstants.redHubPose;
import static frc.robot.RangerHelpers.retryConfigApply;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;





public class TestTurret extends SubsystemBase{

    private final TalonFX m_turret = new TalonFX(kTurretMotorID);
    private final CANcoder m_encoder = new CANcoder(kTurretCANcoderID);
    
    private Field2d field = new Field2d();
    
    private Translation2d VectorToHub = new Translation2d();
    private double currentPosition=m_encoder.getAbsolutePosition().getValueAsDouble() * Math.PI;

    private Translation2d vectorOfRobot = new Translation2d();
    private double currentAngleOfRobot;
    private double kP;
    private double motorOutput;
    private double angleError;

//Soft Limit so Turret cant go past certain degree values

    public TestTurret(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.Slot0.kP = 30;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kA = 0;
        config.Slot0.kV = .12;
        config.Slot0.kS = 0;
        this.kP = 30;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        CANcoderConfiguration Canconfig = new CANcoderConfiguration();
        Canconfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        Canconfig.MagnetSensor.MagnetOffset = 0;
        retryConfigApply(() -> m_encoder.getConfigurator().apply(Canconfig));
        m_turret.getConfigurator().apply(config);
        
    }
    
    public double getAngleToHub(){
    return VectorToHub.getAngle().getRadians();

    }

   @Override
 public void periodic(){
    vectorOfRobot = RobotContainer.getDrivetrain().getState().Pose.getTranslation();
    VectorToHub = redHubPose.minus(vectorOfRobot);
    currentAngleOfRobot = RobotContainer.getDrivetrain().getState().Pose.getRotation().getRadians();
        //m_encoder.getAbsolutePosition().getValueAsDouble() * Math.PI;
        //MathUtil.angleModulus(robotToHub.getAngle().getRadians());
        //MathUtil.angleModulus(x-y);
        //1.Find a vector position of the robot in relation to the target, correct robot position to always be at the target
        //redHubPose.get


        //offset = -.37
        
         angleError = MathUtil.angleModulus(getAngleToHub()) - MathUtil.angleModulus(currentAngleOfRobot);
        
        motorOutput = this.kP * angleError;
        if(motorOutput > 100)
         motorOutput = 100;
        if (motorOutput < -100)
        motorOutput = -100;

        m_turret.set(motorOutput);
          
        SmartDashboard.putData(field);
        SmartDashboard.putData(this);


    }
    @Override
    public void initSendable(SendableBuilder builder){
        super.initSendable(builder);

        builder.addDoubleProperty("AngleToHub",() -> VectorToHub.getAngle().getRadians(),null);
        builder.addDoubleProperty("Angle ERROR", () -> angleError, null);
        builder.addDoubleProperty("MotorOutput", () -> motorOutput, null);
        builder.addDoubleProperty("Current Position of Turret",()->m_encoder.getAbsolutePosition().getValueAsDouble() * Math.PI, null);
        builder.addDoubleProperty("Current Angle of Turret", ()->currentAngleOfRobot,null);

    }

    

    
}