package org.firstinspires.ftc.teamcode.autonomytest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilderKt;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr_quickstart_examples.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "Autonomie0Discuri", group = "autonomy")
public class Autonomie0Discuri extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

//        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Pose2d(30, 30, 0))
//                .build();

        Trajectory trajPowershot1 = new TrajectoryBuilder(new Pose2d(), drive.constraints)
                .splineTo(new Vector2d(63, 12), 0)
                .build();

        Trajectory trajPowershot2 = new TrajectoryBuilder(trajPowershot1.end(), drive.constraints)
                .strafeTo(new Vector2d(63,8))
                .build();

        Trajectory trajPowershot3 = new TrajectoryBuilder(trajPowershot2.end(), drive.constraints)
                .strafeTo(new Vector2d(63,4))
                .build();

        Trajectory putAwayWobble1 = new TrajectoryBuilder(trajPowershot3.end(), drive.constraints)
                .splineTo(new Vector2d(69,-18),Math.toRadians(-90))
                .build();

        Trajectory littleBack = new TrajectoryBuilder(putAwayWobble1.end(), drive.constraints)
                .strafeTo(new Vector2d(69,-8))
                .build();

        Trajectory returnToBase = new TrajectoryBuilder(littleBack.end(), drive.constraints)
                .splineTo(new Vector2d(45,-11),Math.toRadians(180))
                .build();

        Trajectory littleFront = new TrajectoryBuilder(returnToBase.end(), drive.constraints)
                .strafeTo(new Vector2d(30,-11))
                .build();

        Trajectory putAwayWobble2 = new TrajectoryBuilder(littleFront.end().plus(new Pose2d(0,0,Math.toRadians(-180))), drive.constraints)
                .splineTo(new Vector2d(63,-22.5),0)
                .build();

        Trajectory parkRobot = new TrajectoryBuilder(putAwayWobble2.end(), drive.constraints)
                .back(3)
                .splineToConstantHeading(new Vector2d(65,0),0)
                .build();


        drive.followTrajectory(trajPowershot1);
        sleep(500);
        drive.followTrajectory(trajPowershot2);
        sleep(500);
        drive.followTrajectory(trajPowershot3);
        sleep(500);
        drive.followTrajectory(putAwayWobble1);
        sleep(500);
        drive.followTrajectory(littleBack);
        drive.followTrajectory(returnToBase);
        drive.followTrajectory(littleFront);
        sleep(500);
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(putAwayWobble2);
        sleep(500);
        drive.followTrajectory(parkRobot);

//        drive.followTrajectory(
//                drive.trajectoryBuilder(new Pose2d(30, 30, Math.toRadians(0)))
//                        .splineTo(new Pose2d(0, 0, Math.toRadians(180)))
//                        .build()
//        );
    }
}
