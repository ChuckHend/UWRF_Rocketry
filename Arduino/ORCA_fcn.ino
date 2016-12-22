/*
 *  UWRF ORCA Flight Computer
 *  Written by Adam Hendel, Trevor J Hoglund
 *  
 *  References:
 *  [1] Altimeter data sheet: http://cdn.sparkfun.com/datasheets/Sensors/Pressure/MPL3115A2.pdf
 *  [2] Altimeter commands:   https://learn.sparkfun.com/tutorials/mpl3115a2-pressure-sensor-hookup-guide
 *  
 */

//  Include Libraries
    #include <SPI.h>
    #include <SD.h>
    #include <Servo.h>
    #include <math.h>
    #include <SparkFunMPL3115A2.h>
    MPL3115A2 altimeter;
    Servo servoOne;
    File dataFile;

//TEST PROGRAM 
//double testTime[] = {0, 600,1200,1800,2400,3000,3600,4200,4800,5400,6000,6600,7200,7800,8400,9000,9600,10200,10800,11400};
//double testAlt[] = {0,12,50,160,250,335,415,490,560,620,675,725,770,810,845,875,900,915,925,930};     
//int j = 0;
//END TEST PROGRAM
          
//  Drag system setup
    long    prevAlt     = 0,            //  Set last loop variables
            prevAcc     = 0,
            prevVel     = 0,
            delayStart  = 99999999,
            prevTime    = 0;
      
    boolean runDrag     = true,         //  Run drag system this run
            hasFired    = false,        //  If engine has been fired
            burnout     = false;        //  If engine has burned out


    int     activeTest  = 0,            //  Currently running test
            burnAlt     = 30,
            burnDelay   = 4000,         //  time for motor to burnout
            sdPin       = 10,           //  SD Card Pin
            servoPin    = 9,            //  Servo pin
            sweep       = 0,
            sweeps      = 2,            //  Amount of intitial sweeps
            target      = 1160;         //  Target altitude (2nd launch)
    String  filename,
            extension   = ".txt";
    double  ground      = 0,
            A           = 0.20,         //constants for curve fit A-D
            B           = 8.35,
            C           = 1.67;
    
void setup(){
    //  SD Card Setup
    int    i = 0;
    SD.begin(sdPin);
    do{
        i++;
        filename  = "flight";
        filename += i;
        filename += extension;
    }   while(SD.exists(filename));
    dataFile = SD.open(filename, FILE_WRITE);
    dataFile.println("Time\tChange in time\tAltitude\tVelocity\tAcceleration");
    dataFile.println("s\ts\tm\tm/s\tm/s*s");
    dataFile.println();
    dataFile.close();
    
    //  Servo Setup
    servoOne.attach(servoPin);
    
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
       delay(500);               //wait for arms to extend
       closeDragSystem();
       delay(500);              //wait for arms to retract
       }
}//end setup

void loop(){

  
    //  Gather information
    double altitude = altimeter.readAltitude() - ground;         //  Get altitude, subtract ground alt
    //double altitude = testAlt[j];              //for test program
    //double currTime = testTime[j];            //for test program
    //j++;                                      //for test program

    unsigned long currTime = millis();                  //  Get time in ms since run began
    //  Open file

    dataFile = SD.open(filename, FILE_WRITE);
    
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

           //FOR DEBUG
//    Serial.print("Altitude: ");
//    Serial.print(altitude);
//    Serial.print(" deltaAlt: ");
//    Serial.print(deltAlt);
//    Serial.print(" deltTime: ");
//    Serial.print(deltTime);           
//    Serial.print(" currAlt: ");
//    Serial.print(altitude);
//    Serial.print(" Vel: ");
//    Serial.print(currVel);       
//    Serial.print(" Time: ");
//    Serial.print(currTime);    
//      
    //  Log data
                          dataFile.print(currTime);
    dataFile.print("\t"); dataFile.print(deltTime,5);
    dataFile.print("\t"); dataFile.print(altitude,5);
    dataFile.print("\t"); dataFile.print(currVel,5);
    dataFile.print("\t"); dataFile.print(currAcc,5);

   
    //  Check if ignition
    if(runDrag && !hasFired && (altitude > burnAlt)){ // engine fired if change in alt
        hasFired = true;
        Serial.println("FIRED");
        delayStart = millis();
        dataFile.print("\tENGINE FIRED");
    }
    if(runDrag && hasFired && !burnout && currTime >= (delayStart + burnDelay)){  //  Burnout if time past ignition
        burnout = true;
        dataFile.print("\tBURNOUT");
        openDragSystem();
        Serial.println("burnout and openDrag");
    }
    if(runDrag && ORCA(currVel, altitude)){
      closeDragSystem();
      Serial.println("ORCA close");
      dataFile.print("\tORCA CLOSE");
    }
    //****BEGIN FAILSAFE*****
    if(runDrag && !hasFired && (altitude>750)){          
      openDragSystem();
      Serial.println("open failsafe 1");
      dataFile.print("\topen failsafe 1");      
    }
    if(runDrag && currVel<20 && (altitude>750)){
      closeDragSystem();
      Serial.println("close failsafe 1");
      dataFile.print("\tCLOSE failsafe 1");    
    }

    //****END FAILSAFE****
    
    //  Finish loop
    dataFile.println();
    dataFile.close(); // Maybe just dataFile.flush();
}

void openDragSystem(){
    servoOne.write(35);
}

void closeDragSystem(){
    servoOne.write(120);
}

bool ORCA(double velocity, double currAlt){ //returns true if drag system should close
  double expectedApogee = currAlt+A*pow(abs(velocity-B),C);
  Serial.print("Expected Apogee: ");
  Serial.println(expectedApogee);
  if(expectedApogee > target) return false;
  else return true;
}


