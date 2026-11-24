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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;





public class TestTurret extends SubsystemBase{

    private final TalonFX m_turret = new TalonFX(kTurretMotorID);
    private final CANcoder m_encoder = new CANcoder(kTurretCANcoderID);
    
    private Field2d field = new Field2d();
    
    private Translation2d VectorToHub = new Translation2d();
    private double currentPosition=m_encoder.getAbsolutePosition().getValueAsDouble() * Math.PI;

    private Translation2d vectorOfRobot = new Translation2d();
    private double currentAngleOfRobot;
    private PositionVoltage positionVoltage = new PositionVoltage(0);
    private double alpha;
    private double theta;
    private double phi;
    private  TalonFXConfiguration config = new TalonFXConfiguration();


    

//Soft Limit so Turret cant go past certain degree values

    public TestTurret(){
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;//enabled them, now swt them
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.25;//Next set the zeroPowerBehavior
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        config.Slot0.kP = 30;


        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kA = 0;
        config.Slot0.kV = .12;
        config.Slot0.kS = 0;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;//Setting Feedback we did in Phenoix CTR Tuner
        config.Feedback.FeedbackRemoteSensorID = kTurretCANcoderID;//giving iD
              CANcoderConfiguration Canconfig = new CANcoderConfiguration();//initialize ts

        Canconfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        Canconfig.MagnetSensor.MagnetOffset = -.0327637;//we tuned ts
        Canconfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;//Cant pass, only turns 90 and -90 in opp directions, as seen by soft limits
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
    phi = RobotContainer.getDrivetrain().getState().Pose.getRotation().getRotations();//Phi is current angle fo robot
    alpha = VectorToHub.getAngle().getRotations();//Alpha is desired angle of VectorToHub -> Subtract to get what we want
    theta = alpha = phi;

    positionVoltage.Position = theta;//setting position to theta
    m_turret.setControl(positionVoltage);
    SmartDashboard.putData(this);

    }
    @Override
    public void initSendable(SendableBuilder builder){
        super.initSendable(builder);

       builder.addDoubleProperty("Angle Theta",() -> theta,null);
       builder.addDoubleProperty("Motor Output", () -> m_turret.get(),null);
        

    }

    

    
}