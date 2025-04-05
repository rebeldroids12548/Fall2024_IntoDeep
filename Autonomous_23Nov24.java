package org.firstinspires.ftc.Autonomous_13Nov24;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class Autonomous_23Nov24 extends LinearOpMode{
    // Find a motor in the hardware map named "Arm Motor"
   
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
    double PreConfigMotionCounter = 0;
    int StartPreconfigMotion_Drop = 0;

    double Autonomous_Counter = 0;
    
    
    
    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    double INTAKE_COLLECT    = -0.7;
    double INTAKE_OFF =  0.0;
    double INTAKE_DEPOSIT    =  0.8;
    
    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    double WRIST_FOLDED_IN   = 0.9;
    double WRIST_FOLDED_OUT  = 0.6;
    
    
    
    int SliderMotor_ActivateBrakeDelay = 0;
    int StartPickingFromFloor = 0;
    //Avoiding slider from sliding down after init is pressed.
    //Start with slider all the way in
    
    


    
  @Override
  public void runOpMode() {
    
    DcMotor slidermotor = hardwareMap.dcMotor.get("slidermotor");
    DcMotor ArmMotor = hardwareMap.dcMotor.get("motor2");  
    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LeftFront");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("LeftRear");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("RightFront");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("RightRear");
    
     /* Define and initialize servos.*/
    CRServo intake = hardwareMap.get(CRServo.class, "intake");
    Servo wrist  = hardwareMap.get(Servo.class, "wrist");
    int SliderMotorPosition = slidermotor.getCurrentPosition();

      
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
    
    // Reverse the right side motors. This may be wrong for your setup.
    // If your robot moves backwards when commanded to go forwards,
    // reverse the left side instead.
    // See the note about this earlier on this page.
    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      
    slidermotor.setTargetPosition(SliderMotorPosition);
    slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slidermotor.setPower(0.5);  
    
    waitForStart();
    
    while(opModeIsActive())
    {
        
        int run_only_once = 0;
        
            int ArmMotor_CurrentPosition = ArmMotor.getCurrentPosition();

            
            //SLIDER MOTION - START
            
            /* MECANNUM WHEEL DRIVE CODE START*/
            
            
            double frontLeftPower = -0.5;
            double backLeftPower = -0.5;
            double frontRightPower = -0.5;
            double backRightPower = -0.5;
            
             
            
            telemetry.addData("getTargetPosition = ", frontLeftMotor.getTargetPosition());
            telemetry.addLine();
            
            telemetry.addData("frontLeftPower = ", frontLeftPower);
            telemetry.addLine();
            telemetry.addData("backLeftPower = ", backLeftPower);
            telemetry.addLine();
            
            telemetry.addData("SliderMotorSpeed = ", slidermotor.getPower());
            telemetry.addLine();
            
            telemetry.addData("frontRightPower = ", frontRightPower);
            telemetry.addLine();
            
            telemetry.addData("backRightPower ", backRightPower);
            
            telemetry.addData("Autonomous_Counter ", Autonomous_Counter);
            
            telemetry.update();
    
    if(run_only_once ==0)   
    {
        run_only_once = 1;
        
        while(Autonomous_Counter<3000)
        {
            frontLeftMotor.setPower(-0.5);   //MAke -0.5 to turn right
            backLeftMotor.setPower(-0.5);        //MAke -0.5 to turn right
            frontRightMotor.setPower(0.5);     //MAke 0.5 to turn right
            backRightMotor.setPower(0.5);      //MAke 0.5 to turn right
            Autonomous_Counter++;
        }
            
            Autonomous_Counter = 0; 
    
            while(Autonomous_Counter<42800)
            {
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
                Autonomous_Counter++;
            }
            Autonomous_Counter = 0;
            
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(00);
            backRightMotor.setPower(0);
            
            
            
            while(Autonomous_Counter<500)
            {
                telemetry.addData("Stopping now ", Autonomous_Counter);
                        telemetry.update();
            
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(00);
            backRightMotor.setPower(0);
                Autonomous_Counter++;
            }
            
            telemetry.addData("Turn Left now ", Autonomous_Counter);
            
            telemetry.update();
                        Autonomous_Counter = 0;
            
            while(Autonomous_Counter<9000)
            {
                //Turn LEFT
                frontLeftMotor.setPower(0.5);   //MAke -0.5 to turn right
                backLeftMotor.setPower(0.5);        //MAke -0.5 to turn right
                frontRightMotor.setPower(-0.5);     //MAke 0.5 to turn right
                backRightMotor.setPower(-0.5);      //MAke 0.5 to turn right
                Autonomous_Counter++;
            }
            
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(00);
            backRightMotor.setPower(0);
            
            PreConfigMotionCounter = 0;
            
            while(PreConfigMotionCounter < 100000)
            {
                telemetry.addData("PreConfigMotionCounter = ", PreConfigMotionCounter);
            
                telemetry.addLine();
                
                //Get wrist ready to drop
                wrist.setPosition(WRIST_FOLDED_OUT);
                
                telemetry.addData("Drop Sample ", Autonomous_Counter);
            
                telemetry.update();
                if(PreConfigMotionCounter < 5000)
                {
                   PreConfigMotionCounter++; 
                }
                else
                {
                    PreConfigMotionCounter = 0;
                    //StartPreconfigMotion_Drop = 0;
                    
                }
                
                //Get arm to move to drop position
                if((PreConfigMotionCounter > 0) && (PreConfigMotionCounter < 1450))
                {
                    ArmMotor.setTargetPosition(1900);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmMotor.setPower(0.7); 
                }
                
                //Wait for some time before starting to move the slider to drop position    
                if((PreConfigMotionCounter > 300) && (PreConfigMotionCounter < 1450))
                {
                    slidermotor.setTargetPosition(-2500);
                    slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidermotor.setPower(0.7);
                    
                    //Wait for Slider to fully extend
                    if(PreConfigMotionCounter > 1000)
                    {
                        intake.setPower(INTAKE_DEPOSIT);
                    }
                }
                    
                    //Before ending collapse the slider and move the arm down
                if(PreConfigMotionCounter > 1500)
                {
                    intake.setPower(INTAKE_OFF);
                    
                    slidermotor.setTargetPosition(-100);
                    slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidermotor.setPower(0.7);
                    
                    if(PreConfigMotionCounter > 2000)
                    {
                        ArmMotor.setTargetPosition(100);
                        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ArmMotor.setPower(0.7); 
                        
                        if(PreConfigMotionCounter > 2500)
                        {
                            //Add Code for PARKING
                            //break;
                            //requestOpModeStop();
                            StartPickingFromFloor = 1;
                            telemetry.addData("PreConfigMotionCounter = ", PreConfigMotionCounter);
            
                            telemetry.update();
                            
                        }
                        
                        
                    }
                    
                    
                }
                
            if(StartPickingFromFloor ==1)
            {
                
             frontLeftPower = -0.6;
             backLeftPower = 0.6;
             frontRightPower = 0.6;
             backRightPower = -0.6;
                
            Autonomous_Counter = 0;
            
            //Move Back
            while(Autonomous_Counter<28000)
            {
                frontLeftMotor.setPower(0.5);
                backLeftMotor.setPower(0.5);
                frontRightMotor.setPower(0.5);
                backRightMotor.setPower(0.5);
                Autonomous_Counter++;
            }
            Autonomous_Counter = 0;
            
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(00);
            backRightMotor.setPower(0);
            
            
            //Strafe RIGHT
            //while(Autonomous_Counter<32500)
                     frontLeftPower = -0.6;
             backLeftPower = 0.6;
             frontRightPower = 0.6;
             backRightPower = -0.6;
            while(Autonomous_Counter<10000)
            {
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
                Autonomous_Counter++;
            }
            Autonomous_Counter = 0; 
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            
            while(Autonomous_Counter<15000)
            {
                //Turn right
                frontLeftMotor.setPower(-0.5);   //MAke -0.5 to turn right
                backLeftMotor.setPower(-0.5);        //MAke -0.5 to turn right
                frontRightMotor.setPower(0.5);     //MAke 0.5 to turn right
                backRightMotor.setPower(0.5);      //MAke 0.5 to turn right
                Autonomous_Counter++;
            }
            
            Autonomous_Counter = 0; 
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
                
            //Get wrist ready to drop
            wrist.setPosition(WRIST_FOLDED_OUT);
            intake.setPower(INTAKE_COLLECT);
            
            PreConfigMotionCounter = 0;
            
            //Pickup sample
            while(PreConfigMotionCounter<1500)
            {
                
                
                PreConfigMotionCounter++;
               
               //Get ready to pickup sample
                ArmMotor.setTargetPosition(PreConfig_ArmMotorPosition);
                ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmMotor.setPower(0.7); 
               
                if(PreConfigMotionCounter > 200)
                {
                    //SliderMotorPosition = PreConfig_SliderMotorPosition;
                    slidermotor.setTargetPosition(PreConfig_SliderMotorPosition);
                    slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidermotor.setPower(0.7);
                }
            
            //Move forward a bit to sweep sample from floor    
                if((PreConfigMotionCounter > 700) && (PreConfigMotionCounter < 750))
                {
                    frontLeftMotor.setPower(-0.5);
                    backLeftMotor.setPower(-0.5);
                    frontRightMotor.setPower(-0.5);
                    backRightMotor.setPower(-0.5);
                    
                }
                else{
                        
                        frontLeftMotor.setPower(0);
                        backLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        backRightMotor.setPower(0);
                    }
                    
                    if(PreConfigMotionCounter > 1000)
                    {
                        intake.setPower(INTAKE_OFF);
                    }

            
                telemetry.addData("Start Pickup from Floor PreConfigMotionCounter =", PreConfigMotionCounter);
            
                telemetry.update();
               
            }
            
            telemetry.addData("Out of While PreConfigMotionCounter =", PreConfigMotionCounter);
            
            telemetry.update();
            
            PreConfigMotionCounter = 0;
            
            //Retract slider
            while(PreConfigMotionCounter < 500)
            {
                //SliderMotorPosition = PreConfig_SliderMotorPosition;
                slidermotor.setTargetPosition(-100);
                slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidermotor.setPower(0.7);
                
                PreConfigMotionCounter++;
                
                
            }
            

            Autonomous_Counter = 0;
            PreConfigMotionCounter = 0;
            
            //Move and drop sample
            while(Autonomous_Counter<9000)
            {
                telemetry.addData("Move LEFT", Autonomous_Counter);
            
                telemetry.update();
                frontLeftPower = 0.6;
                 backLeftPower = -0.6;
                 frontRightPower = -0.6;
                 backRightPower = 0.6;
                
                //Strafe LEFT

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
                Autonomous_Counter++;
            }  
                Autonomous_Counter = 0;  
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                PreConfigMotionCounter = 0;
                
                
                
                
            //Turn left to orient towards basket
            
            Autonomous_Counter = 0;
            
            while(Autonomous_Counter<15000)
            {
                frontLeftMotor.setPower(0.5);   //MAke -0.5 to turn right
                backLeftMotor.setPower(0.5);        //MAke -0.5 to turn right
                frontRightMotor.setPower(-0.5);     //MAke 0.5 to turn right
                backRightMotor.setPower(-0.5);      //MAke 0.5 to turn right
                Autonomous_Counter++;
            }
            
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(00);
            backRightMotor.setPower(0);
            Autonomous_Counter = 0;
            
            //Move Foward a bit towards basket
            while(Autonomous_Counter<8000)
            {
                frontLeftMotor.setPower(-0.5);
                backLeftMotor.setPower(-0.5);
                frontRightMotor.setPower(-0.5);
                backRightMotor.setPower(-0.5);
                Autonomous_Counter++;
            }
            Autonomous_Counter = 0;
            
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(00);
            backRightMotor.setPower(0);
            
                
            while(PreConfigMotionCounter <2000)
            {
                PreConfigMotionCounter++;
                telemetry.addData("Drop floor sample 1 ", Autonomous_Counter);
            
                telemetry.update();
                //Get arm to move to drop position
                if((PreConfigMotionCounter > 0) && (PreConfigMotionCounter < 1450))
                {
                    ArmMotor.setTargetPosition(1900);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmMotor.setPower(0.7); 
                }
                
                //Wait for some time before starting to move the slider to drop position    
                if((PreConfigMotionCounter > 300) && (PreConfigMotionCounter < 1450))
                {
                    slidermotor.setTargetPosition(-2500);
                    slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidermotor.setPower(0.7);
                    
                    //Wait for Slider to fully extend
                    if(PreConfigMotionCounter > 1000)
                    {
                        intake.setPower(INTAKE_DEPOSIT);
                    }
                }
            }
            PreConfigMotionCounter = 0;
            
            //Code to Raise arm and drop sample
            while(PreConfigMotionCounter < 1500)
            {
                telemetry.addData("Dropping sample 1 PreConfigMotionCounter = ", PreConfigMotionCounter);
            
                telemetry.addLine();
                
                PreConfigMotionCounter++;
                
                //Get wrist ready to drop
                wrist.setPosition(WRIST_FOLDED_OUT);
                
                telemetry.addData("Drop Sample ", Autonomous_Counter);
            
                telemetry.update();
                
                
                //Get arm to move to drop position
                if((PreConfigMotionCounter > 0) && (PreConfigMotionCounter < 550))
                {
                    ArmMotor.setTargetPosition(1900);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmMotor.setPower(0.7); 
                }
                
                //Wait for some time before starting to move the slider to drop position    
                if((PreConfigMotionCounter > 300) && (PreConfigMotionCounter < 550))
                {
                    slidermotor.setTargetPosition(-2500);
                    slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidermotor.setPower(0.7);
                    
                    //Wait for Slider to fully extend
                    if(PreConfigMotionCounter > 600)
                    {
                        intake.setPower(INTAKE_DEPOSIT);
                    }
                }
                    
                    //Before ending collapse the slider and move the arm down
                if(PreConfigMotionCounter > 650)
                {
                    intake.setPower(INTAKE_OFF);
                    
                    slidermotor.setTargetPosition(-100);
                    slidermotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slidermotor.setPower(0.7);
                    
                    if(PreConfigMotionCounter > 1100)
                    {
                        ArmMotor.setTargetPosition(100);
                        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ArmMotor.setPower(0.7); 
                        
                        if(PreConfigMotionCounter > 900)
                        {
                            //Add Code for PARKING
                            //break;
                            //requestOpModeStop();
                            StartPickingFromFloor = 1;
                            telemetry.addData("PreConfigMotionCounter = ", PreConfigMotionCounter);
            
                            telemetry.update();
                            
                        }
                        
                        
                    }
                    
                    
                }
            }
            
            
            //PARKING CODE START
            //Strafe Right
            Autonomous_Counter = 0; 
            frontLeftPower = -0.9;
             backLeftPower = 0.9;
             frontRightPower = 0.9;
             backRightPower = -0.9;
            while(Autonomous_Counter<8000)
            {
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
                Autonomous_Counter++;
            }
            Autonomous_Counter = 0; 
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            
            //Move Back
            
            while(Autonomous_Counter<3000)
            {
                frontLeftMotor.setPower(0.9);
                backLeftMotor.setPower(0.9);
                frontRightMotor.setPower(0.9);
                backRightMotor.setPower(0.9);
                Autonomous_Counter++;
            }
            Autonomous_Counter = 0;
            
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(00);
            backRightMotor.setPower(0);
            
            //End code to raise arm and drop sample
            requestOpModeStop();
            
            

            
                

                
            }
                    
            }
                
            
                
    }      
        
        
        
    }
      
      
  }
}
