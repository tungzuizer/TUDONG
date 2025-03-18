package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;



@Autonomous(name = "TUDONG")
public class TUDONG extends LinearOpMode {
    public class Lift {
        private DcMotorEx linerLeft;
        private DcMotorEx linerRight;

        public Lift(HardwareMap hardwareMap) {
            linerLeft = hardwareMap.get(DcMotorEx.class, "motor");
            linerRight = hardwareMap.get(DcMotorEx.class, "motor");
            linerLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linerRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linerLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            linerRight.setDirection(DcMotorSimple.Direction.REVERSE);

        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linerRight.setPower(0.8);
                    linerLeft.setPower(0.8);
                    initialized = true;
                }

                double pos1 = linerLeft.getCurrentPosition();
                double pos2 = linerRight.getCurrentPosition();

                if (pos1 < 1800 && pos2 < 1800) {
                    return true;
                } else {
                    linerLeft.setPower(0);
                    linerRight.setPower(0);
                    return false;
                }
            }
        }

        // Hành động hạ xuống (về vị trí 0)
        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linerRight.setPower(-0.8);
                    linerLeft.setPower(-0.8);
                    initialized = true;
                }

                double pos1 = linerLeft.getCurrentPosition();
                double pos2 = linerRight.getCurrentPosition();

                if (pos1 > 0 && pos2 > 0) {
                    return true;
                } else {
                    linerLeft.setPower(0);
                    linerRight.setPower(0);
                    return false;
                }
            }
        }

        // Trả về hành động nâng lên
        public Action liftUp() {
            return new LiftUp();
        }

        // Trả về hành động hạ xuống
        public Action liftDown() {
            return new LiftDown();
        }

    }

    public class Claw {
        private Servo Servo0, Servo1, Servo2,Servo3,Servo4,Servo5;

        public Claw(HardwareMap hardwareMap) {
            Servo4 = hardwareMap.get(Servo.class, "Servo4");
            Servo5 = hardwareMap.get(Servo.class, "Servo5");
            Servo0 = hardwareMap.get(Servo.class, "Servo0");

            Servo3 = hardwareMap.get(Servo.class, "Servo3");
            Servo2 = hardwareMap.get(Servo.class, "Servo2");
            Servo1 = hardwareMap.get(Servo.class, "Servo1");        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Servo3.setPosition(0.0);
                Servo4.setPosition(0.0);
                Servo2.setPosition(0.2);
                sleep(300);
                Servo0.setPosition(0.38);
                Servo1.setPosition(1);
                Servo2.setPosition(0.6);
                sleep(500);
                Servo3.setPosition(0.3);
                Servo4.setPosition(0.3);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Servo2.setPosition(0.9);


                sleep(200);
                Servo0.setPosition(0.65);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }
    private PinpointDrive drivetrain;
    private Lift lift;
    private Claw claw;

    @Override
    public void runOpMode() {
        drivetrain = new PinpointDrive(hardwareMap, new Pose2d(24, 62, Math.toRadians(270)));
        lift = new Lift(hardwareMap);
        claw = new Claw(hardwareMap);

        waitForStart();

        drivetrain.updatePoseEstimate();
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose)
                //  .splineTo(new Vector2d(-52,56),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(56,56),Math.toRadians(225))
                .waitSeconds(0.5)
                .build());

        Actions.runBlocking(lift.liftUp());
        Actions.runBlocking(claw.closeClaw());
        drivetrain.updatePoseEstimate();
        Actions.runBlocking(drivetrain.actionBuilder(drivetrain.pose)
                .splineTo(new Vector2d(48,38),Math.toRadians(270))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(56,56),Math.toRadians(225))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(58,38),Math.toRadians(270))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(56,56),Math.toRadians(225))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(56,25),Math.toRadians(0))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(56,56),Math.toRadians(225))
                .build());


    }
}
