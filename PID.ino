float PID(int targetangle , float kp,float kd,float ki){
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTime))/1.0e6;

//calculating error in angle 
claculate_angles();
int(actualangle)=headingDegrees; //real angle
angleerror=targetangle-actualangle;  //error between traget and real angle
angleslope=deltaE/deltaT;   //calculating slope  (derivative)
deltaE=angleerror-angleerrorold;     //calculating changing in errore
Serial.println(deltaT);
///////////////////////////////// integral caclulation /////
angleErroreAreaOld=angleErroreAreaOld+(angleerror*deltaT);  //calculsting error area to use it with I contrller

float val=(kp*angleerror)+(ki*angleErroreAreaOld)+(kd*angleslope);
//;// PID
previousTime = currentTime;
angleerrorold=angleerror;
//Serial.println(val);



// monetiring values using chart 
Serial1.print(actualangle);
Serial1.print(",");
Serial1.print(targetangle);
Serial1.print(",");
Serial1.print(val);
Serial1.print(",");
Serial1.print(angleerror);
Serial1.print(",");


return val;
}