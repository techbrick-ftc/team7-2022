package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.StarterAuto;

@Config
@Autonomous
public class AutoParkingRight extends StarterAuto {
    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        initialize();
        initAprilTags();

        packet.put("angle", imu.getAngularOrientation().firstAngle);
        dashboard.sendTelemetryPacket(packet);

        double startAngle = imu.getAngularOrientation().firstAngle;
        imuAngle();

        waitForStart();
        packet.addLine("id after");
        dashboard.sendTelemetryPacket(packet);
        int tag = getAprilTag(5);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // We want to start the bot at x: -36, y: -60, heading: 180 degrees
        Pose2d startPose = new Pose2d(-36, -61.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        double slowerVelocity = 65;

        Trajectory traj1 = drive.trajectoryBuilder(startPose, false)

                .strafeTo(new Vector2d(-36, -19),SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory endingStraight = drive.trajectoryBuilder(traj1.end(), false)
                .strafeTo(new Vector2d(-36, -15))
                .build();

        Trajectory endingLeft = drive.trajectoryBuilder(endingStraight.end(), false)
                .strafeTo(new Vector2d(-12, -15))
                .build();
        Trajectory endingRight = drive.trajectoryBuilder(endingStraight.end(), false)
                .strafeTo(new Vector2d(-60, -15))
                .build();

        //change for right values^^

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);

        drive.followTrajectory(endingStraight);

        if (tag == 1) {
            drive.followTrajectory(endingLeft);
            packet.put("tag", tag);
            dashboard.sendTelemetryPacket(packet);
            motorsStop();
            sleep(1000);
        } else if (tag == 2) {
            packet.put("tag", tag);
            dashboard.sendTelemetryPacket(packet);

            motorsStop();
            sleep(1000);
        } else if (tag == 3) {
            drive.followTrajectory(endingRight);
            packet.put("tag", tag);
            dashboard.sendTelemetryPacket(packet);
            motorsStop();
            sleep(1000);
        }
    }
}
