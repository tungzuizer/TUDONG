

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous(name="Tudongkhuvangreal")
public class Tudongkhuvang2 extends LinearOpMode {

    private Servo Servo0, Servo1, Servo2,Servo5;


    private DcMotor         rightLift   = null;
    private DcMotor         Motor3  = null;
    private DcMotor         Motor1  = null;
    private DcMotor         motor  = null;



    private DcMotor         leftWheel   = null;
    private DcMotor         rightWheel  = null;
    private IMU             imu         = null;      // Control/Expansion Hub IMU
    private ElapsedTime     runtime = new ElapsedTime();

    private double          headingError  = 0;

    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.6;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    
    static final double     P_TURN_GAIN            = 0.009;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.02;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftWheel  = hardwareMap.get(DcMotor.class, "leftWheel");
        rightWheel = hardwareMap.get(DcMotor.class, "rightWheel");
        Motor3  = hardwareMap.get(DcMotor.class, "Motor3");
        Motor1  = hardwareMap.get(DcMotor.class, "Motor1");
        motor  = hardwareMap.get(DcMotor.class, "motor");


        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        Servo5 = hardwareMap.get(Servo.class, "Servo5");
        Servo2 = hardwareMap.get(Servo.class, "Servo2");
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo0 = hardwareMap.get(Servo.class, "Servo0");
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setDirection(DcMotor.Direction.FORWARD);
  rightLift.setDirection(DcMotor.Direction.FORWARD);
        Motor3.setDirection(DcMotor.Direction.REVERSE);
        Motor1.setDirection(DcMotor.Direction.FORWARD);
        motor.setDirection(DcMotor.Direction.REVERSE);



        Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        
        Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
 
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        
        Servo1.setDirection(Servo.Direction.REVERSE);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }


        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();


        //-----------------------------------------
        driveStraight(0.3,-13, 0.0);
                                                                                                    
              Servo5.setPosition(0.45);
            Servo0.setPosition(0.55);
             turnToHeading( 0.7 ,-75.0); 
             encoderDrive(0.9,5 ,5 ,4);
        Servo1.setPosition(0.45);
         Servo2.setPosition(0.45);
        encoderkethop(0.6,11,11,-18,7);
        
            Servo0.setPosition(0.82);
        holdHeading( 0.8, -75, 1);

            

//-------------------------------------------------------
             turnToHeading( 1 ,-40.0); 

          encoderkeocang(0.5,-1,3);


          Servo1.setPosition(1);
              Servo2.setPosition(1);
              Servo0.setPosition(0.82);

                encoderDrive(0.2,-14.8,-14.8,4);

              
              
              Servo0.setPosition(0.6);   //chỉnh góp gắp
              Servo0.setPosition(0.6);
              sleep(800);
              Servo2.setPosition(0.35);
              Servo1.setPosition(0.35);
              Servo5.setPosition(0.45);
             encoderDrive(0.9,14 ,14,4);

                turnToHeading( 1,-70.0); 
        //    driveStraight(0.3,33.0, 0.0);  

              Servo0.setPosition(0.82);
                turnToHeading( 0.7,-60.0); 
//-----------------------------------------------------------------
             
        driveStraight(0.2,-2.0, 0.0);  

          Servo1.setPosition(1);
              Servo2.setPosition(1);
              Servo0.setPosition(0.82);
                 holdHeading( 0.8, -65, 0.5);   
              //   encoderkeocang(0.8,-2,3);

          encoderDrive(0.2,-10 ,-10 ,4);
          encoderDrive(0.1,-4 ,-4 ,4);


            Servo0.setPosition(0.6);   //chỉnh góp gắp

              Servo0.setPosition(0.6);   //chỉnh góp gắp
              sleep(700);
              Servo2.setPosition(0.35);
              Servo1.setPosition(0.35);
              Servo5.setPosition(0.4);

            encoderDrive(0.6,14 ,14 ,4);
                turnToHeading( 0.9,-75.0); 

              Servo0.setPosition(0.82);
              
          sleep(300);
//----------------------------------------------------------------------
                turnToHeading( 0.8 ,-22.0);

                Servo1.setPosition(1);
              Servo2.setPosition(1);
              Servo0.setPosition(0.82);
             encoderDrive(0.2,-10 ,-10 ,4);
             encoderDrive(0.1,-4.5 ,-4.5 ,4);

             Servo0.setPosition(0.6);
              sleep(500);
              Servo2.setPosition(0.38);
              Servo1.setPosition(0.38);
              Servo5.setPosition(0.45);   
             // encoderkeocang(0.8,3,3);
             encoderDrive(0.3,14.5 ,14.5 ,4);
            turnToHeading( 0.5 ,-70.0);

              Servo0.setPosition(0.82);
   //----------------------------------------------------------------       
          //  driveStraight(0.8,-33.0, -50.0); 
          //  Servo2.setPosition(0);
          //    Servo1.setPosition(0);
        //    turnToHeading( 0.7 ,-125.0); 
        //    driveStraight(0.8,-25.0, -125.0);  
        //     encoderDrive(1,-10 ,-10 ,4);


       //  encoderkeocang(1,14,4);
       // encoderkeocang(1,-14,4);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = leftWheel.getCurrentPosition() + moveCounts;
            rightTarget = rightWheel.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftWheel.setTargetPosition(leftTarget);
            rightWheel.setTargetPosition(rightTarget);

            leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftWheel.isBusy() && rightWheel.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

   
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

   
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

   
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftWheel.setPower(leftSpeed);
        rightWheel.setPower(rightSpeed);
    }

    
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftWheel.getCurrentPosition(),
                    rightWheel.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
     public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = Motor3.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightLift.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            Motor3.setTargetPosition(newLeftTarget);
            rightLift.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            Motor3.setPower(Math.abs(speed));
            rightLift.setPower(Math.abs(speed));

          
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (Motor3.isBusy() && rightLift.isBusy())) {

                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            Motor3.getCurrentPosition(), rightLift.getCurrentPosition());
                telemetry.update();
            }

            Motor3.setPower(0);
            rightLift.setPower(0);

            Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    
    
    public void encoderDingang(double speed1,
                             double leftInches1, 
                             double timeoutS1) {
        int newLeftTarget1;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget1 = Motor1.getCurrentPosition() + (int)(leftInches1 * COUNTS_PER_INCH);
            Motor1.setTargetPosition(newLeftTarget1);

            // Turn On RUN_TO_POSITION
            Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            Motor1.setPower(Math.abs(speed1));

          
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS1) &&
                   (Motor1.isBusy())) {

               
            }

            Motor1.setPower(0);

            Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    
    
    public void encoderkeocang(double speed2,
                             double leftInches2, 
                             double timeoutS2) {
        int newLeftTarget2;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget2 = motor.getCurrentPosition() + (int)(leftInches2 * COUNTS_PER_INCH);
            motor.setTargetPosition(newLeftTarget2);

            // Turn On RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            motor.setPower(Math.abs(speed2));

          
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS2) &&
                   (motor.isBusy())) {

               
            }

            motor.setPower(0);

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
       public void encoderkethop(double speed,
                             double leftInches3, double rightInches3,double keoInches3,
                             double timeoutS) {
        int newLeftTarget3;
        int newRightTarget3;
        int newRightTarget4;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget3 = Motor3.getCurrentPosition() + (int)(leftInches3 * COUNTS_PER_INCH);
            newRightTarget3 = rightLift.getCurrentPosition() + (int)(rightInches3 * COUNTS_PER_INCH);
            newRightTarget4 = motor.getCurrentPosition() + (int)(keoInches3 * COUNTS_PER_INCH);

            Motor3.setTargetPosition(newLeftTarget3);
            rightLift.setTargetPosition(newRightTarget3);
            motor.setTargetPosition(newRightTarget4);


            // Turn On RUN_TO_POSITION
            Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            Motor3.setPower(Math.abs(speed));
            rightLift.setPower(Math.abs(speed));
            motor.setPower(Math.abs(speed));

          
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (Motor3.isBusy() && rightLift.isBusy() && motor.isBusy())) {

                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget3,  newRightTarget3 , newRightTarget4);
                telemetry.addData("Currently at",  " at %7d :%7d", Motor3.getCurrentPosition(), rightLift.getCurrentPosition(),motor.getCurrentPosition());
                telemetry.update();
            }

            Motor3.setPower(0);
            rightLift.setPower(0);
            motor.setPower(0);

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    
    
    
    
}
