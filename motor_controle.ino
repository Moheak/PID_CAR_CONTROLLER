void ControleMotors(float pidVal){
float speed=fabs(pidVal);// taking the absulute value of pidVal

if(speed>120)
speed=120;

else if(speed<55)
speed=57;

//direction
int dir =0; 
if(pidVal<0){
dir=1;  //RIGHT
}
Serial1.print(speed);
Serial1.println();
switch(dir){
case 1: //right direction needed
analogWrite(en1,speed);
analogWrite(en2,speed); 
digitalWrite(m1,0);
digitalWrite(m12,1);
digitalWrite(m2,1);
digitalWrite(m22,0);
break;
case 0: //left direction needed
analogWrite(en1,speed);
analogWrite(en2,speed); 
digitalWrite(m1,1);
digitalWrite(m12,0);
digitalWrite(m2,0);
digitalWrite(m22,1);
break;
default:
break;
}
}




