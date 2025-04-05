package org.firstinspires.ftc.Autonomous_13Nov24;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class OverallControl_22Nov2024 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Find a motor in the hardware map named "Arm Motor"
        DcMotor slidermotor = hardwareMap.dcMotor.get("slidermotor");
        DcMotor ArmMotor = hardwareMap.dcMotor.get("motor2");
        int once_occured=0;
        int ArmMotorPosition = 0, SliderMotorSpeed=0;
        int Temp=0;
        int SliderMotorPositionIncrement = 250;
        int ArmMotorPositionIncrement = 150;
        int Reduced_ArmMotorPositionIncrement = 75;
        double ArmMotorPower = 0.7;
        int AvoidGripperCrashingPosition = 400;
        double ReducedArmPowerAvoidGripperCrashing = 0.4;
        
        //Pre-configured Positions
        
        int PreConfig_ArmMotorPosition = 300;
        int PreConfig_SliderMotorPosition = -1900;
        int StartPreconfigMotion_Collect = 0;
        int StartPreconfigMotion_Move = 0;
        int PreConfigMotionCounter = 0;
        int StartPreconfigMotion_Drop = 0;
        int RunPIDLessOften = 0;
        
        /*

        * Proportional Integral Derivative Controller
        PID Control Variable declaration START
        
        */
        
        //Working Well
        /*double Kp = 0.001;
        double Ki = 0.0001;
        double Kd = 0;*/
        
        //Working Well -
        /*double Kp = 0.05;
        double Ki = 0.03;
        double Kd = 0.5;*/
        
        //Working Well +
        /*double Kp = 0.001;
        double Ki = 0.0007;
        double Kd = 0.00;*/
        
        double Kp = 0.0009;
        double Ki = 0.0008;
        double Kd = 0.;
        
        int reference;
        
        double integralSum = 0;
        
        double lastError = 0;
        
        int encoderPosition = 0;
        double error = 0;
        double derivative = 0;
        double out = 0;
                    
        
        // Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        /*PID Control Variable declaration END*/
        
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LeftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LeftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RightRear");
        
         /* Define and initialize servos.*/
        CRServo intake = hardwareMap.get(CRServo.class, "intake");
        Servo wrist  = hardwareMap.get(Servo.class, "wrist");

        
        /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
        final double INTAKE_COLLECT    = -1.0;
        final double INTAKE_OFF        =  0.0;
        final double INTAKE_DEPOSIT    =  0.5;
        
        /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
        final double WRIST_FOLDED_IN   = 0.9;
        final double WRIST_FOLDED_OUT  = 0.6;


        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);
        

        // Reset the motor encoder so that it reads zero ticks
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setMode(DcMotor.RunMode.);

        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        slidermotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        slidermotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        slidermotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        int SliderMotor_ActivateBrakeDelay = 0;
        //Avoiding slider from sliding down after init is pressed.
        //Start with slider all the way in
        int SliderMotorPosition = slidermotor.getCurrentPosition();
        
        //slidermotor.setTargetPosition(SliderMotorPosition);
        //slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //slidermotor.setPower(0.5);


        waitForStart();

        while (opModeIsActive()) {
            // Get the current position of the motor
            //int position = slidermotor.getCurrentPosition();
            int ArmMotor_CurrentPosition = ArmMotor.getCurrentPosition();

            
            //SLIDER MOTION - START
            if (gamepad1 .left_stick_y==1)
            {
                SliderMotorPosition = slidermotor.getCurrentPosition();
                SliderMotorPosition = SliderMotorPosition + SliderMotorPositionIncrement;

                
                slidermotor.setTargetPosition(SliderMotorPosition);
                slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidermotor.setPower(0.8);
                SliderMotor_ActivateBrakeDelay = 0;
                SliderMotorSpeed = 80;
            }
            
            //Move Slider only if the arm is not at its lowest position
            //this is to avoid slider and gripper scratching the floor
            else if ((gamepad1 .left_stick_y==-1) /*&& (ArmMotor_CurrentPosition>10)*/)
            {
                SliderMotorPosition = slidermotor.getCurrentPosition();
                SliderMotorPosition = SliderMotorPosition - SliderMotorPositionIncrement;

                
                slidermotor.setTargetPosition(SliderMotorPosition);
                slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidermotor.setPower(0.8);
                SliderMotor_ActivateBrakeDelay = 0;
                SliderMotorSpeed = 80;
            }

            else if((StartPreconfigMotion_Collect == 0) &&
                    (StartPreconfigMotion_Move ==0) && 
                    (StartPreconfigMotion_Drop == 0))
            {
                
                SliderMotor_ActivateBrakeDelay++;
                
                if(SliderMotor_ActivateBrakeDelay > 600)
                {   
                    
                    /*PID Control Variable declaration START*/
                    RunPIDLessOften++;
                    //if(RunPIDLessOften>4)
                    {
                        RunPIDLessOften = 0;
                    
                        if (slidermotor.getCurrentPosition() != SliderMotorPosition) 
                        {
    
                            // obtain the encoder position
                            encoderPosition = slidermotor.getCurrentPosition();
                            // calculate the error
                            error = SliderMotorPosition - encoderPosition;
                        
                            // rate of change of the error
                            derivative = (error - lastError) / timer.seconds();
                        
                            // sum of all error over time
                            integralSum = integralSum + (error * timer.seconds());
                        
                            out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
                            
                            out = out/100;//trying if zero power brakes the slider
                            //out = 0.00035;
                        
                            //Apply PID control only if arm position is high
                            //else use motor braking torque
                            if(ArmMotor.getCurrentPosition() < 1500)
                            {
                                slidermotor.setPower(0);
                            }
                            else
                            {
                                //PID Control of slider motor may not be required
                                slidermotor.setTargetPosition(SliderMotorPosition);
                                slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                slidermotor.setPower(out);
                            }
                        
                            lastError = error;
                        
                            // reset the timer for next time
                            timer.reset();
                    
                        }
                    }
                    //else
                    {
                        //slidermotor.setPower(0);
                    }
                    
                    
                    
                    /*PID Control Variable declaration END*/
                    
                    
                    
                    
                    /*int sliderpos_diff = Math.abs(slidermotor.getCurrentPosition()) - Math.abs(SliderMotorPosition);
                    if(Math.abs(sliderpos_diff) > 150)
                    {
                        slidermotor.setPower(SliderMotorPosition);
                        slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slidermotor.setPower(0.001);
                        
                        SliderMotorSpeed = 1;
                    }*/
                }
                //slidermotor.setPower(0);
            }
            //SLIDER MOTION - END
            
            //WRIST MOTION - START
            if (gamepad1 .left_stick_x==1)
            {
                wrist.setPosition(WRIST_FOLDED_IN);
            }
            else if (gamepad1 .left_stick_x==-1)
            {
                wrist.setPosition(WRIST_FOLDED_OUT);
            }
            //WRIST MOTION - END
            

            //ARM MOTOR MOTION - START
            if (gamepad1.right_stick_y==-1)
            {
                ArmMotorPosition = ArmMotor.getCurrentPosition();
                ArmMotorPosition = ArmMotorPosition + ArmMotorPositionIncrement;

                
                ArmMotor.setTargetPosition(ArmMotorPosition);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(ArmMotorPower);
                //while (ArmMotorPosition != ArmMotor.getCurrentPosition()) 
                {
                                        }
                //ArmMotorPosition = ArmMotor.getCurrentPosition();
            }
            else if (gamepad1 .right_stick_y==1)
            {
                ArmMotorPosition = ArmMotor.getCurrentPosition();
                if(ArmMotorPosition>0)
                {
                    //Reduce Arm Motor speed as arm comes near ground
                    if(ArmMotorPosition < AvoidGripperCrashingPosition)
                    {
                       ArmMotorPosition = ArmMotorPosition - Reduced_ArmMotorPositionIncrement; 
                    }
                    else
                    {
                        //Arm is not near ground, go at normal speed
                        ArmMotorPosition = ArmMotorPosition - ArmMotorPositionIncrement;
                    }
                }
                

                
                ArmMotor.setTargetPosition((ArmMotorPosition));
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(ArmMotorPosition < AvoidGripperCrashingPosition)
                {
                    ArmMotor.setPower(ReducedArmPowerAvoidGripperCrashing);
                }
                else
                {
                    ArmMotor.setPower(ArmMotorPower);
                }

            }
            
            //ARM MOTOR MOTION - END
            
            
            /*if(gamepad1.right_stick_y==1)
            {
                ArmMotor.setTargetPosition(ArmMotorPosition);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(0.5);
            }*/
            
            
            
            
            if (gamepad1 .a) {
                intake.setPower(INTAKE_COLLECT);
                

            }
            
            //Preconfigured Pickup
            else if(gamepad1.left_trigger==1.0)
            {

                StartPreconfigMotion_Collect = 1;
                StartPreconfigMotion_Move = 0;
                PreConfigMotionCounter = 0;
                StartPreconfigMotion_Drop = 0;
            }
            //Preconfigure get ready to move
            else if(gamepad1.left_bumper)
            {
                
                StartPreconfigMotion_Collect = 0;
                StartPreconfigMotion_Move = 1;
                PreConfigMotionCounter = 0;
                StartPreconfigMotion_Drop = 0;
                
            }
            //Preconfigure drop
            else if(gamepad1.right_trigger==1.0)
            {

                StartPreconfigMotion_Drop = 1;
                StartPreconfigMotion_Collect = 0;
                StartPreconfigMotion_Move = 0;
                PreConfigMotionCounter = 0;
            }
            //else if(gamepad1.left_stick_button)
            else if (gamepad1 .x) {
                intake.setPower(INTAKE_OFF);
                
            }
            else if (gamepad2 .b) {
                intake.setPower(INTAKE_DEPOSIT);
                
            }
            else if (gamepad1 .y) {
                slidermotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                
            }
            
            
            
            if(StartPreconfigMotion_Collect ==1)
            {
                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_COLLECT);
                
                if(PreConfigMotionCounter < 600)
                {
                   PreConfigMotionCounter++; 
                }
                else
                {
                    PreConfigMotionCounter = 0;
                    StartPreconfigMotion_Collect = 0;
                    
                }
                

                ArmMotor.setTargetPosition(PreConfig_ArmMotorPosition);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(0.7); 
                    
                if(PreConfigMotionCounter > 200)
                {
                    SliderMotorPosition = PreConfig_SliderMotorPosition;
                    slidermotor.setTargetPosition(PreConfig_SliderMotorPosition);
                    slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidermotor.setPower(0.7);
                }
            }
            
            else if(StartPreconfigMotion_Move ==1)
            {
                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_OFF);
                
                if(PreConfigMotionCounter < 600)
                {
                   PreConfigMotionCounter++; 
                }
                else
                {
                    PreConfigMotionCounter = 0;
                    StartPreconfigMotion_Move = 0;
                    
                }
                
                SliderMotorPosition = -100;
                slidermotor.setTargetPosition(-100);
                slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidermotor.setPower(0.7);
                
                if(PreConfigMotionCounter > 200)
                {
                    ArmMotor.setTargetPosition(100);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmMotor.setPower(0.7); 
                }
            }
            
            if(StartPreconfigMotion_Drop ==1)
            {
                wrist.setPosition(WRIST_FOLDED_OUT);
                
                
                if(PreConfigMotionCounter < 1000)
                {
                   PreConfigMotionCounter++; 
                }
                else
                {
                    PreConfigMotionCounter = 0;
                    StartPreconfigMotion_Drop = 0;
                    
                }
                

                ArmMotor.setTargetPosition(1900);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(0.7); 
                    
                if(PreConfigMotionCounter > 300)
                {
                    SliderMotorPosition = -2500;
                    
                    slidermotor.setTargetPosition(-2500);
                    slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidermotor.setPower(0.7);
                    
                    /*if(PreConfigMotionCounter > 700)
                    {
                        intake.setPower(INTAKE_DEPOSIT);
                    }*/
                }
            }
        
            
            /* MECANNUM WHEEL DRIVE CODE START*/
            double y = gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad2.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx)*0.6) / denominator;
            double backLeftPower = ((y - x + rx)* 0.6) / denominator;
            double frontRightPower = ((y - x - rx)*0.6) / denominator;
            double backRightPower = ((y + x - rx)*0.6) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            
            /* MECANNUM WHEEL DRIVE CODE END*/
            
            /*Telemetry Code*/
            
            telemetry.addData("frontLeftPower = ", frontLeftPower);
            telemetry.addLine();
            telemetry.addData("backLeftPower = ", backLeftPower);
            telemetry.addLine();
            
            telemetry.addData("frontRightPower = ", frontRightPower);
            telemetry.addLine();
            
            
            telemetry.addData("backRightPower = ", backRightPower);
            telemetry.addLine();
            
            telemetry.addData("SliderPosition = ", slidermotor.getCurrentPosition());
            telemetry.addLine();
            
            telemetry.addData("PreConfigMotionCounter = ", PreConfigMotionCounter);
            telemetry.addLine();
            
            telemetry.addData("SliderMotor_ActivateBrakeDelay = ", SliderMotor_ActivateBrakeDelay);
            telemetry.addLine();
            
            telemetry.addData("ArmMotorPosition = ", ArmMotor.getCurrentPosition());
            telemetry.addLine();
            
            
            telemetry.addData("PID OUTPUT = ", out);
            //telemetry.addLine();
            
            //telemetry.addData("getZeroPowerBehavior = ", slidermotor.getZeroPowerBehavior());
            
            telemetry.update();
            }

        }
    }

