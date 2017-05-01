/*
 *  UWRF 2017 Drag System Controller
 *  Written by Adam Hendel
 *  
 */

//  Include Libraries
    #include <math.h>
    #include <SparkFunMPL3115A2.h>
    MPL3115A2 altimeter;
    #include <AFMotor.h>

AF_DCMotor motor(1); // create motor #1
       
//  Drag system setup
    long    prevAlt     = 0,            //  Set last loop variables
            prevAcc     = 0,
            prevVel     = 0,
            delayStart  = 99999999,
            openDragTime  = 0,
            prevTime    = 0;
      
    boolean runDrag     = true,         //  Run drag system this run
            hasFired    = false,        //  If engine has been fired
            openv2      = false,
            burnout     = false;        //  If engine has burned out

//all time units in milliseconds
    int     tdragOpen    = 2000,   // time for drag to fully open
            tdragClose   = 2000,   // time for drag to fully close
            tstayOpen    = 5000,   // time to stay open, once fully open
            burnDelay    = 3000,   // time for engine to finish burn
                                   // drag starts immediately after burnout           
            burnAlt      = 50,     // change in alt. that triggers ignition timer
                                   // lots of noise in sensor, so this is set to 30 meters
                        
            activeTest  = 0,            //  Currently running test
            sweep       = 0,
            sweeps      = 1,            //  Amount of intitial sweeps
            target      = 1160;         //  Target altitude (2nd launch)

    double  ground      = 0,
            A           = 0.20,         //constants for curve fit A-D
            B           = 8.35,
            C           = 1.67;
    
void setup(){
    //setup DC motor
    motor.setSpeed(255);     // set the speed to 255 max
    
    //  Altimeter Setup
    altimeter.begin();
    altimeter.setModeAltimeter();       //  Measures in meters
    altimeter.setOversampleRate(0);     //  Set Oversample to 0
    altimeter.enableEventFlags();       //  Enable all three pressure and temp event flags
    prevAlt = altimeter.readAltitude(); //  Required for setup
    delay(100);
    ground = altimeter.readAltitude();

    Serial.begin(9600);

    //  Initialization sweeps
  for(sweeps;sweep<sweeps; sweep++){
       openDragSystem();
       closeDragSystem();
       }
}//end setup

void loop(){
    //  Gather information
    double altitude = altimeter.readAltitude() - ground;         //  Get altitude, subtract ground alt
    Serial.print("altitude:");
    Serial.println(altitude);

    unsigned long currTime = millis();                  //  Get time in ms since run began
    //  Open file
    
    //  Determing velocity and acceleration
    double   deltTime = (currTime - prevTime)/1000.00, 
           deltAlt  = altitude - prevAlt,
           currVel  = deltAlt  / deltTime,
           deltVel  = currVel  - prevVel,
           currAcc  = deltVel  / deltTime;
    //  Update previous variables
           prevAlt  = altitude;
           prevTime = currTime;
           prevVel  = currVel;
           prevAcc  = currAcc;
   
    //  Check if ignition
    if(runDrag && !hasFired && (altitude > burnAlt)){ // engine fired if change in alt
        hasFired = true;
        Serial.println("FIRED");
        delayStart = millis();
    }
    if(runDrag && hasFired && !burnout && currTime >= (delayStart + burnDelay)){  //  Burnout if time past ignition
       burnout = true;
       openDragSystem();
       Serial.println("burnout and openDrag");
       openDragTime = millis();      
       openv2 = true;
       motor.run(RELEASE);     // tell motor to stay open
       delay(tstayOpen);       // keep system open
       closeDragSystem();     //now close the drag system
       motor.run(RELEASE);     //once closed, keep it closed
    }
    else {motor.run(RELEASE);
    Serial.println("release");} 
}

void openDragSystem(){
    motor.run(RELEASE);
    delay(100);
    Serial.println("open");
    motor.run(FORWARD);
    delay(tdragOpen);
}

void closeDragSystem(){
    motor.run(RELEASE);
    delay(100);
    Serial.println("open");
    motor.run(BACKWARD);
    delay(tdragClose);
}
