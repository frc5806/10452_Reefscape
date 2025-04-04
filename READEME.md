# The Code for 10452's REEFSCAPE Game

## Done about 1% by Sebastian Baxter, 3% by Ethan Rosenfeld, 6% by Sophie Willer-Buchardi, and 90% by people we don't know
This is the Readme for team 10452's code

Core Componants:
- Java WPILib command-based Framework
- 299 Files
- 14 Subsystems
- 7 Vender Dependencies
- Pathplanner Autonomous
- AprilTag recognition with Limelight that autoaligns robot



Programming functionalities (in approximate order of completion)
Field or Robot centric Swerve Drive programmed with NavX Gyro, CTRE CANcoders, and SparkMax NEO Motors.

Elevator programmed with an encoder for the height of the elevator, and PID to ensure smooth height change.

Camera functionality for Logitech MX Brio Webcam to allow for live feed and recording of matches to improve driver performance. We had issues with sending a high resolution video at a meaningfully useful frame rate. To rectify this we decrease the resolution and frame of view to improve frame rate to provide valuable data to the driver and to help line up for climb and ensure the winch rolls up successfully. The camera also allows for post match analysis

Implemented a pathplanner autonomous and created named commands to accomplish all robot functionalities during autonomous routines. Autonomous Routines prepared for different game strategies. Our autonomous routines did not work perfectly and align successfully during practice, so we implemented limelight alignment functionality during autonomous. This required knowing when our limelight was aligned, which meant that we had to create acceptable buffers for translation and rotation relative to the apriltag than once within our alignment command.

We wanted to intake coral while autonomous, though we didn't want to just wait 5 seconds, which could damage the motor if it continues spinning after the coral enters, or have us complete the remainder of our autonomous routine without a coral piece. To rectify this we measured the difference between the coral motor’s theoretical and actual speeds (if coral or algae is in the mechanism actual speed will be lower than theoretical speed), to identify when the algae was actually placed.

Implemented limelight functionality with automatic robot alignment with April tags upon April Tage recognition when a button on the controller. However, there were major issues that the robot would rotate into the proper orientation before it had approached the 
Implemented a custom function (derivative of arctangent) to adjust rotation speed depending on horizontal distance to the april tag.
We also identified that we would need another limelight to align with the coral, though we were unsure how to use two limelights in the code. We eventually found that by giving limelights different references in the limelight web interface we could identify two limelights.

Programmed WS2812B compatible LED strip lights using a CTRE CANdle. The CANdle is not supported in phoenix 6, so we needed to import phoenix 5 in order to support the CANdle. Allows for animations and different colors depending on game states.

Implemented automatic changing of LED color with recognition of an April tag to help drivers identify when an april tag is actually recognized.
This was actually a major roadblock for a while because we continued to attempt to get whether or not there was a valid target based on a boolean, when the network table actually outputted a long, which as soon as we figured out allowed our code to work!

Commented code base and implemented an informative README file.



We use a command based framework. Main.java begins robot.java, which has a start method that goes to robotcontainer initializes all of our controls, autonomous routines, and other command bindings (like switching LEDs when an April tag is identified). In robot.java there is a periodic method (which runs again and again);which runs our command scheduler instance (the Singleton that runs all of the commands in a command based framework). This allows all of our commands to be executed and our robot to perform it's tasks.

File system
Main.java
- First file that runs, runs robot.java

Robot.java
- runs robotcontainer key bindings
- period method runs an instance of the command scheduler constantly.

RobotContainer.java
- Initializes all controller bindings, autonomous routines, and other command bindings (like switching LEDs when an April tag is identified)

Constants.java
- A place to store all constants (motorIDs, etc) for our code

Subsystems
- Algae.java
    - Runs all algae mechanisms
- Climb.java    
    - Runs all climb mechanisms 
- Coral.java 
    - Runs all mechaniisms related to coral 
- Elevator.java 
    - Runs all elevator mechanisms of the robot 
- Led.java 
    - self explanatory
- Limelight.java 
    - Imported stuff for Limelight 
- LinearServo.java 
    - Code to run the servo beneath the coral 
- Vision.java
    - Opperates our webcam






Notes:
- if you put in smartdashboard what alliance then it will reset your odomotry for you


TODO:
- Tigthenscrews
- Wrap LED end
- Get locknuts for