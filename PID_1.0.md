### PID ALGORITHM X OR Y AXIS

##### PID INPUTS

 Acc_x - Angle of rotation about the axis (i.e. x-x) in degrees, zero on level, positive or negative on tilt
 
 Gyr_x - The rate of rotation about the axis (i.e. x-x) in degrees per seconds. Output zero when on static (at any angle),            and positive or negative on rotation.
 
 SetRollsAbout_X - Transmitter stick input, zero on center, and set an tilt angle on movement.
 
 dtx - This is the lapse or loop time a PID is called, the period normally in proper timing or syncronized with motor PWM   
       output.

##### ERROR AND SETPOINT

```
Att_error_X = Acc_x - SetRollsAbout_X;         

XSetpoint_rate = (tau*XSetpoint_rate/(tau+dtx)) + ((SetRollsAbout_X - SetRollsAbout_Xold)/(tau+dtx));
SetRollsAbout_Xold = SetRollsAbout_X;
```

Att_error_X - The attitude error from desired rated setpoint "XSetpoint_rate" from 1. being level or 2. from setpoint angle desired by stick movement.

XSetpoint_rate - The rate of error outputed by stick movement, Zero on center stick, and depending on the direction, it is positive of negative on stick movement.

SetRollsAbout_Xold - Recorded the old value of "SetRollsAbout_X"

##### RATE P
```
Px_Error_rate = XSetpoint_rate * krx + Att_error_X * kpxx + kg*Gyr_x;
```
Gyr_x - It will act as Lag compensation.

Px_Error_rate - Rated error.


##### DERIVATIVE
```
          Att_dErr_X = (tau*Att_dErr_X)/(tau+dtx) + (Px_Error_rate - Px_Error_rateOld) / (tau+dtx);
          Px_Error_rateOld = Px_Error_rate;
```
Att_dErr_X - The rate of change in error overtime i.e. error over time

##### INTEGRAL
```
Iax = Iax + kix*Px_Error_rate*dtx
```
Iax - is summ of error overtime until Att_error_X = 0

##### OUTPUT

OutAtti_x =  kpx*Px_Error_rate + Iax + kdx*Att_dErr_X;

##### Sample Code
```
void Compute_Aileron(unsigned long dtx)
{dtx =dtx*0.001; 
float XSetpoint_rate, SetRollsAbout_X, Att_error_X, Px_Error_rate, Att_dErr_X, OutAtti_x;
float erRangeX=deadBand;
 
  if (AI_Pulse>1523 || AI_Pulse<1520)
  {AI_Rate=true;
    SetRollsAbout_X = map(AI_Pulse, 1522,1940,0,mapxlim); // pitch output range 0 - 8 degrees  "EL_Pulse"
  }else {SetRollsAbout_X=0;AI_Rate=false;}

   if   (Acc_x<=erRangeX && Acc_x>=-erRangeX) {Acc_x=0;}
   else if   (Acc_x>erRangeX)  {Acc_x=Acc_x-erRangeX;}   
   else if   (Acc_x<-erRangeX) {Acc_x=Acc_x+erRangeX;}  
   
   
  //P.I.D Activation
    if (TH_Pulse>PIDtakeoff)
    {
/****************************RATE****************************************/      
          Att_error_X = Acc_x - SetRollsAbout_X;         
          XSetpoint_rate = (tau*XSetpoint_rate/(tau+dtx)) + ((SetRollsAbout_X - SetRollsAbout_Xold)/(tau+dtx));
          SetRollsAbout_Xold = SetRollsAbout_X;

 /********LEVEL 1 - PID**************************************************************************************************/            
                   
          //PID, LEVEL 1 
          float P = Att_error_X;
               Ix = Ix + Att_error_X*dtx;  
          float D = dx*Gyr_x;  //(Att_error_X - Att_error_Xold)/dtx;
          
            PIDix = P*px + Ix*ix + D*dx;
          Poldx=P; Att_error_Xold=Att_error_X;
 /********LEVEL 2 - PID*******************************************************************************************************/          

         //RATE P + LAG COMPENSATOR:          
          Px_Error_rate = XSetpoint_rate * krx + Att_error_X * kpxx + kg*Gyr_x;  // + kg*Gyr_x  or + PIDix * kpxx 
          
          
          //DERIVATIVE OF RATE P
          Att_dErr_X = (tau*Att_dErr_X)/(tau+dtx) + (Px_Error_rate - Px_Error_rateOld) / (tau+dtx);
          Px_Error_rateOld = Px_Error_rate;
          
          //INTEGRAL OF RATE P
              if(TH_Pulse>Itakeoff)
              {Iax = Iax + kix*Px_Error_rate*dtx;} else{Iax=0;} 
              if  (Iax>=Iax_Limit || Iax<=-Iax_Limit) {Iax -= kix*Px_Error_rate*dtx;}//Stop integration at limits                        

          //REVISED PID      
              OutAtti_x =   Att_error_X * jx +  kpx*Px_Error_rate + Iax + kdx*Att_dErr_X;
         
          //Final Combined Output           
          OutputAtt_x=OutAtti_x;
          //OutputAtt_x=PIDix;
/****************************OUTPUT-X ***************************************************************/          
      if ((OutputAtt_x <= Output_XMax) && (OutputAtt_x >=Output_XMin)){Output_X=OutputAtt_x;} 
          else if  (OutputAtt_x>Output_XMax)  {Output_X= Output_XMax;}//Stop integration at limits
          else if  (OutputAtt_x<Output_XMin)  {Output_X= Output_XMin;}//Stop integration at limits
   }
     else
     {
     OutAtti_x=0;Output_X=0;   
     Px_Error_rate=0;
     Px_Error_rateOld=0;
     Iax=0;Ix=0;
     Att_dErr_X=0;PIDix=0;
     dtx=0;
     XSetpoint_rate=0;
     Att_error_X=0;
     }//End of if TH_Pulse

}
```
