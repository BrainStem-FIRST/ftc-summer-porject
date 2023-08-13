package org.firstinspires.ftc.teamcode.drive.swerveBasicTests;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.util.math.Pose;
import org.openftc.easyopencv.OpenCvCamera;


import javax.annotation.concurrent.GuardedBy;

public class TestingBrainSTEMRobot {


    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;


    public CRServo frontLeftServo;
    public CRServo frontRightServo;
    public CRServo backLeftServo;
    public CRServo backRightServo;

    public AnalogInput frontLeftEncoder;
    public AnalogInput frontRightEncoder;
    public AnalogInput backLeftEncoder;
    public AnalogInput backRightEncoder;

    public Motor.Encoder parallelPod;
    public Motor.Encoder perpindicularPod;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    private double voltage = 0.0;
    private ElapsedTime voltageTimer;

    public OpenCvCamera backCamera;


    private static TestingBrainSTEMRobot instance = null;

    public boolean enabled;

    private HardwareMap hardwareMap;

    public static TestingBrainSTEMRobot getInstance() {
        if (instance == null) {
            instance = new TestingBrainSTEMRobot();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        if (Constants.USING_IMU) {
            synchronized (imuLock) {
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
                imu.initialize(parameters);
            }
        }

        voltageTimer = new ElapsedTime();


        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FLDandFOdo");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FRdrive");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "BLdrive");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "BRDandBOdo");



        frontLeftServo = hardwareMap.get(CRServo.class, "FLturn");
        frontRightServo = hardwareMap.get(CRServo.class, "FRturn");
        backLeftServo = hardwareMap.get(CRServo.class, "BLturn");
        backRightServo = hardwareMap.get(CRServo.class, "BRturn");

        frontLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "FLE");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "FRE");
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "BLE");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "BRE");



        parallelPod = new MotorEx(hardwareMap, "BLDandLOdo").encoder;
        parallelPod.setDirection(Motor.Direction.REVERSE);
        perpindicularPod = new MotorEx(hardwareMap, "FRDandROdo").encoder;
        perpindicularPod.setDirection(Motor.Direction.REVERSE);


        if (Constants.AUTO) {

        }
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void loop(Pose drive, SampleSwerveDrive drivetrain) {
        try {
            if (drive != null) {
                drivetrain.set(drive);
            }
            drivetrain.updateModules();
        } catch (Exception ignored) {
        }


        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }
    }

    public void read(SampleSwerveDrive drivetrain) {

        try {
            drivetrain.read();

        } catch (Exception ignored) {
        }
    }

    public void write(SampleSwerveDrive drivetrain) {

        try {
            drivetrain.write();

        } catch (Exception ignored) {
        }
    }

    public void reset() {
        try {
            parallelPod.reset();
            perpindicularPod.reset();
        } catch (Exception e) {
        }
        imuOffset = imu.getAngularOrientation().firstAngle;
    }

    public void clearBulkCache() {
        PhotonCore.CONTROL_HUB.clearBulkCache();
    }

    public double getAngle() {
        return imuAngle - imuOffset;
    }

    public void startIMUThread(LinearOpMode opMode) {
        if (Constants.USING_IMU) {
            imuThread = new Thread(() -> {
                while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                    synchronized (imuLock) {
                        imuAngle = imu.getAngularOrientation().firstAngle;
                    }
                }
            });
            imuThread.start();
        }
    }

    public void stopCameraStream() {
        backCamera.closeCameraDeviceAsync(() -> System.out.println("Stopped Back Camera"));
    }

    public double getVoltage() {
        return voltage;
    }

    public void zero() {

    }
}
