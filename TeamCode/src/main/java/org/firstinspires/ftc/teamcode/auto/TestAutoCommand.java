package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.locolization.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.util.math.Pose;

import java.util.function.DoubleSupplier;

@Autonomous(name = "Test Auto")
@Config
public class TestAutoCommand extends LinearOpMode {

    private BrainSTEMRobot robot = BrainSTEMRobot.getInstance();
    private SwerveDrivetrain drivetrain;
    //    private IntakeSubsystem intake;
//    private LiftSubsystem lift;
    private TwoWheelLocalizer localizer;
    private ElapsedTime timer;

    private double loopTime;
    private double endtime;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CommandScheduler.getInstance().reset();
        Constants.AUTO = false;
        Constants.USING_IMU = true;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        localizer = new TwoWheelLocalizer(robot);

        robot.enabled = true;

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        while (!isStarted()) {
            robot.read(drivetrain);
            drivetrain.frontLeftModule.setTargetRotation(0);
            drivetrain.frontRightModule.setTargetRotation(0);
            drivetrain.backLeftModule.setTargetRotation(0);
            drivetrain.backRightModule.setTargetRotation(0);
            drivetrain.updateModules();

            telemetry.addLine("INIT ACTIVE");
            telemetry.addData("x", localizer.getPos().x);
            telemetry.addData("y", localizer.getPos().y);
            telemetry.update();

            robot.clearBulkCache();
            robot.write(drivetrain);
        }


        robot.startIMUThread(this);
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));
        robot.reset();
        timer = new ElapsedTime();

        DoubleSupplier time_left = () -> 30 - timer.seconds();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PositionCommand(drivetrain, localizer, new Pose(10, 0, 0), 500, 2500, robot.getVoltage()),
//                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.PI), 0, 2000, robot.getVoltage()),
                        new InstantCommand(() -> endtime = timer.seconds())
                )
        );


        while (opModeIsActive()) {
            robot.read(drivetrain);

            CommandScheduler.getInstance().run();
            robot.loop(null, drivetrain);
            localizer.periodic();

            double loop = System.nanoTime();
            telemetry.addData("loop time ", 1000000000 / (loop - loopTime));
            telemetry.addData("endtime", endtime);
            telemetry.addData("x", localizer.getPos().x);
            telemetry.addData("y", localizer.getPos().y);
            telemetry.addData("h", localizer.getPos().heading);
            loopTime = loop;
            telemetry.update();
            robot.write(drivetrain);
            robot.clearBulkCache();
        }
    }
}