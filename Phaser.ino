//TimerStuff
size_t subPhase_repeat_count = 0;
size_t standGuard_count = 0;
int timeSlices = 50;// how many steps within each phase

//TrigStuff
int kaReverse[2];

void phaseCycle(){//Outputs LegORDERS
  for(int i = 0; i < 8; i++){
    if(legAssignment[i] == 0){//Start at the lowest front leg
      if(i == 0 && legAssignment[7] == 0){
        i = 7;
      }
      Serial.print("FirstFrontLeg:");
      Serial.println(i);
      int legPush;
      for (int m = 0; m<8; m++){
        legPush = i - m;
        if (legPush < 0){
          legPush += 8;
        }
        //Need to swap 1&2, 5&6.
        if(legPush == 1 || legPush == 5){
          legPush += 1;
        }
        else if(legPush == 2 || legPush == 6){
          legPush -= 1;
        }
        moveOrder[m] = legPush;
      }
      // for(int j = 0; j<4; j++){
      //   if(legPush > 7){
      //     legPush -= 8;
      //   }
      //   moveOrder[j] = legPush;
      //   if(legPush+1 > 7){
      //     legPush = -1;
      //   }
      //   moveOrder[j+4] = legPush+1;
      //   legPush +=2;        
      // } 
      i=8;//exits for loop
    }
  }
  // for(int i = 0; i < 8; i++){
  //   Serial.print("LegOrder:");
  //   Serial.println(moveOrder[i]);
  // }
}
void timeCycle(int phase, int speed){//Updates the phase list
  Serial.print("Phase:");
  Serial.print(phase);
  Serial.print(" @cycleSpeed:");
  Serial.print(speed);
  Serial.print(" PhaseEvery:");
  Serial.println(phaseSpeed/(speed/2)/timeSlices);
  for(int i = 0; i<8; i++){
    // Serial.print("Leg:");
    // Serial.print(moveOrder[i]);
    phaseList[i] = phase + i;
    if(phaseList[i] > 7){
      phaseList[i] -= 8;
    }
    // Serial.print(" Phase:");
    // Serial.println(phaseList[i]);
  }
  subPhase_repeat_count = 0;
  //timer.every(phaseSpeed/(speed/2)/timeSlices, subPhaseTimer, (void *)timeSlices-1);//change back to 100/99 later, changed to global variable "timeSlices"
  timer.every(phaseSpeed/2/timeSlices, subPhaseTimer, (void *)timeSlices-1);//change back to 100/99 later, changed to global variable "timeSlices"

}
bool subPhaseTimer(void *opaque) {
  size_t limit = (size_t)opaque;
  Serial.print("subPhaseTimer: ");
  Serial.print(subPhase_repeat_count);
  Serial.print("/");
  Serial.println(limit);
  if (subPhase_repeat_count < timeSlices)
    locomotion();

  return ++subPhase_repeat_count <= limit; // remove this task after limit reached
}
int heightTranslator(int height){
  return map(height, -100, 100, 50, 100);// Translates from height 5cm to 10cm
}
int hipCorrectorFR(int i, int assignment){//Points the front in the direction of travel. Points the rear legs the opposite.
  int hipCorrection;
  int angle = LastAngle;
  if(assignment == 2){
    angle = LastAngle+180;
    if (angle > 359)
      angle -= 360;
  }
  if(abs(angle-hip0Angle[i]) < hipRanges[i]/2 ){
    hipCorrection = angle - hip0Angle[i];
  }
  else if(abs(angle - 360 - hip0Angle[i]) < hipRanges[i] / 2){//Mostly for leg 4
    hipCorrection = angle-360-hip0Angle[i];
  }
  else if(abs(angle+360-hip0Angle[i])<hipRanges[i]/2){//Mostly for leg 3
    hipCorrection = angle + 360 - hip0Angle[i];
  }
  return hipCorrection;
}
void locomotion(){
  // Serial.print("Active Timers:");
  // Serial.println(timer.size());
  for(int i = 0; i < 8; i++){//Sort by move order
    int phase14 = phaseList[i];//Reduce phases from 8 to 4.
    if(phase14 > 3){
      phase14 -= 4;
    }
    switch (legAssignment[i]) {
      case 4://Front Legs
        // Serial.print("Front LEG(s):");
        // Serial.println(i);
        switch (phase14){//0 LiftK, 0.5 LiftA/move hips to start, 1 move1/2, 2 contact, 3 move 2/2,  
          case 0://Move servos to 0 degrees
            // Serial.print("Phase:0 (Lift/Align) Knee position:");
            // Serial.println(map(subPhase_repeat_count, 0, timeSlices-1, FR_Angles[0], 0));//Report on knee movement.
            //Lift Shin, Knee to 0 degrees.
            moveLeg(1, i, 1, findJointAngle(i, 1, map(subPhase_repeat_count, 0, timeSlices-1, FR_Angles[0], 0))); //FR_Angles reference -> minXkAngle, minXaAngle, maxXkAngle, maxXaAngle, distance in mm.
            if(subPhase_repeat_count < timeSlices/2){//0
              moveLeg(1, i, 2, findJointAngle(i, 2, 90));//Tuck Foot
            }
            else{// 0.5
              moveLeg(1, i, 2, findJointAngle(i, 2, map(subPhase_repeat_count, timeSlices/2, timeSlices-1, 90, 0)));//Streighten Foot
              moveLeg(1, i, 0, findJointAngle(i, 0, hipCorrectorFR(i, legAssignment[i]) )); //Align hip
            }
            break;
          case 1://Contact Ground
            // Serial.print("Phase 2: (Contact) Knee position");
            // Serial.println(map(subPhase_repeat_count, 0, timeSlices-1, 0, FR_Angles[2]));//FR_Angles reference -> minXkAngle, minXaAngle, maxXkAngle, maxXaAngle, distance in mm.
            moveLeg(1, i, 1, findJointAngle(i, 1, map(subPhase_repeat_count, 0, timeSlices-1, 0, FR_Angles[2])));// Lower knee to max distance angle.
            moveLeg(1, i, 2, findJointAngle(i, 2, map(subPhase_repeat_count, 0, timeSlices-1, 0, FR_Angles[3])));// Lower ankle to max dist angle.
            break;
          case 2://Move 1/2
            // Serial.print("Phase 1(Move 1/2):");
            // Serial.println(map(subPhase_repeat_count, 0, timeSlices-1, FR_Angles[2], (FR_Angles[0]+FR_Angles[2])/2 ));
            moveLeg(1, i, 1, findJointAngle(i, 1, map(subPhase_repeat_count, 0, timeSlices-1, FR_Angles[2], (FR_Angles[0]+FR_Angles[2])/2 )));//Move knee halfway from max to min
            moveLeg(1, i, 2, findJointAngle(i, 2, map(subPhase_repeat_count, 0, timeSlices-1, FR_Angles[3], (FR_Angles[1]+FR_Angles[3])/2 )));//Move ankle halfway from max to min
            break;
          case 3://Move 2/2
            // Serial.print("Phase 3(Move 2/2):");
            // Serial.println(map(subPhase_repeat_count, 0, timeSlices-1, (FR_Angles[0]+FR_Angles[2])/2, FR_Angles[0]));
            moveLeg(1, i, 1, findJointAngle(i, 1, map(subPhase_repeat_count, 0, timeSlices-1, (FR_Angles[0]+FR_Angles[2])/2, FR_Angles[0])));
            moveLeg(1, i, 2, findJointAngle(i, 2, map(subPhase_repeat_count, 0, timeSlices-1, (FR_Angles[1]+FR_Angles[3])/2, FR_Angles[1])));
            break;
        }
        break;
      case 5://left leg(s)
        switch (phase14){
          case 0://Lift and move hips
            moveLeg(1, i, 1, findJointAngle(i, 1, map(subPhase_repeat_count, 0, timeSlices-1, lastAngles[i*3+1], 0)));
            moveLeg(1, i, 2, findJointAngle(i, 2, map(subPhase_repeat_count, 0, timeSlices-1, lastAngles[i*3+2], 10)));
            //Hips
            if( subPhase_repeat_count > timeSlices/2){
              //Serial.println("moveHipsBack to start");
              moveLeg(1, i, 0, findJointAngle(i, 0, map(subPhase_repeat_count, timeSlices/2, timeSlices-1, lastAngles[i*3], find_hipMovement1(1, i, FR_Angles[4]/2))));
            }
            break;
          case 1://Contact
            moveSave( i, 1, map(subPhase_repeat_count, 0, timeSlices-1, 0, atk90[0]));
            moveSave( i, 2, map(subPhase_repeat_count, 0, timeSlices-1, 10, atk90[1]));
            break;
          case 2://Move 1/2
            moveSave(i, 0, find_hipMovement1(1, i, FR_Angles[4]/2 - map(subPhase_repeat_count, 0, timeSlices-1, 0, FR_Angles[4]/2)));
           break;
          case 3://Move 2/2
            moveSave(i, 0, -find_hipMovement1(2, i, FR_Angles[4] - map(subPhase_repeat_count, 0, timeSlices-1,  FR_Angles[4], FR_Angles[4]/2)));
            break;
        }
        break;
      case 6://Rear leg(s)
        // Serial.print("Rear LEG(s):");
        // Serial.println(i);
        switch (phase14){//0 LiftK, 0.5 LiftA/move hips to start, 1 move1/2, 2 contact, 3 move 2/2,  
          case 0://Lift
            // Serial.print("Phase:0 (Lift/Align) Knee position:");
            // Serial.println(map(subPhase_repeat_count, 0, timeSlices-1, FR_Angles[2], 0));//Report on knee movement.
            //Lift Shin, Knee to 0 degrees.
            moveLeg(1, i, 1, findJointAngle(i, 1, map(subPhase_repeat_count, 0, timeSlices-1, FR_Angles[2], 0))); //FR_Angles reference -> minXkAngle, minXaAngle, maxXkAngle, maxXaAngle, distance in mm.
            if(subPhase_repeat_count < timeSlices/2){//0
              moveLeg(1, i, 2, findJointAngle(i, 2, map(subPhase_repeat_count, 0, timeSlices/2-1, FR_Angles[3], 15)));//kick off foot
            }
            else{// 0.5
              // Serial.print("Rear LEG(s):");
              // Serial.println(i);
              // Serial.print("tick:");
              // Serial.print(subPhase_repeat_count);
              // Serial.print(" out of:");
              // Serial.println(timeSlices/2);
              // Serial.print("Phase 0.5: TuckFOOT");
              // Serial.println(map(subPhase_repeat_count, timeSlices/2, timeSlices-1, 0, 90));
              moveLeg(1, i, 2, findJointAngle(i, 2, map(subPhase_repeat_count, timeSlices/2, timeSlices-1, 15, 90)));//Tuck foot
              moveLeg(1, i, 0, findJointAngle(i, 0, hipCorrectorFR(i, legAssignment[i]) )); //Align hip
            }
            break;
          case 1://Contact Ground
            if(subPhase_repeat_count < timeSlices/2){//First half of phase
              // Serial.print("Phase 1: (Contact) Knee position");
              // Serial.print(map(subPhase_repeat_count, 0, timeSlices-1, 0, FR_Angles[0]));//FR_Angles reference -> minXkAngle, minXaAngle, maxXkAngle, maxXaAngle, distance in mm.
              // Serial.print("/");
              // Serial.println(FR_Angles[0]);
              moveLeg(1, i, 1, findJointAngle(i, 1, map(subPhase_repeat_count, 0, timeSlices/2-1, 0, FR_Angles[0])));// Lower knee to min distance angle.
            }
            else{//Second half of phase X 
              moveLeg(1, i, 2, findJointAngle(i, 2, map(subPhase_repeat_count, timeSlices/2, timeSlices-1, 90, FR_Angles[1])));// Lower ankle to min dist angle.
            }
            break;
          case 2://Move 1/2
            // Serial.print("Phase 2(Move 1/2):");
            // Serial.print(map(subPhase_repeat_count, 0, timeSlices-1, FR_Angles[0], (FR_Angles[0]+FR_Angles[2])/2 ));//FR_Angles reference -> minXkAngle, minXaAngle, maxXkAngle, maxXaAngle, distance in mm.
            // Serial.print("/");
            // Serial.println((FR_Angles[0]+FR_Angles[2])/2);
            moveLeg(1, i, 1, findJointAngle(i, 1, map(subPhase_repeat_count, 0, timeSlices-1, FR_Angles[0], (FR_Angles[0]+FR_Angles[2])/2 )));//Move knee halfway from min to max

            // Serial.print("Phase 2(Move Ankles 1/2):");
            // Serial.print(map(subPhase_repeat_count, 0, timeSlices-1, FR_Angles[1], (FR_Angles[1]+FR_Angles[3])/2 ));//FR_Angles reference -> minXkAngle, minXaAngle, maxXkAngle, maxXaAngle, distance in mm.
            // Serial.print("/");
            // Serial.println((FR_Angles[1]+FR_Angles[3])/2);

            moveLeg(1, i, 2, findJointAngle(i, 2, map(subPhase_repeat_count, 0, timeSlices-1, FR_Angles[1], (FR_Angles[1]+FR_Angles[3])/2 )));//Move ankle halfway from min to max
            break;
          case 3://Move 2/2
            // Serial.print("Phase 3(Move 2/2) Ankles:");
            // Serial.print(map(subPhase_repeat_count, 0, timeSlices-1, (FR_Angles[1]+FR_Angles[3])/2, FR_Angles[3]));//FR_Angles reference -> minXkAngle, minXaAngle, maxXkAngle, maxXaAngle, distance in mm.
            // Serial.print("/");
            // Serial.println(FR_Angles[3]);
            moveLeg(1, i, 1, findJointAngle(i, 1, map(subPhase_repeat_count, 0, timeSlices-1, (FR_Angles[0]+FR_Angles[2])/2, FR_Angles[2])));
            moveLeg(1, i, 2, findJointAngle(i, 2, map(subPhase_repeat_count, 0, timeSlices-1, (FR_Angles[1]+FR_Angles[3])/2, FR_Angles[3])));
            break;
        }
        break;
      case 3://Right leg(s)
        switch (phase14){
          case 0://Lift and move hips
            moveLeg(1, i, 1, findJointAngle(i, 1, map(subPhase_repeat_count, 0, timeSlices-1, lastAngles[i*3+1], 0)));
            moveLeg(1, i, 2, findJointAngle(i, 2, map(subPhase_repeat_count, 0, timeSlices-1, lastAngles[i*3+2], 10)));
            //Hips
            if( subPhase_repeat_count > timeSlices/2){
              // Serial.print("move RIGHTHips back to start:");
              // Serial.println(findJointAngle(i, 0, map(subPhase_repeat_count, timeSlices/2, timeSlices-1, lastAngles[i*3], -find_hipMovement1(1, i, FR_Angles[4]/2))));

              moveLeg(1, i, 0, findJointAngle(i, 0, map(subPhase_repeat_count, timeSlices/2, timeSlices-1, lastAngles[i*3], -find_hipMovement1(1, i, FR_Angles[4]/2))));
            }
            break;
          case 1://Contact at a 90 degree angle
            moveSave(i, 1, map(subPhase_repeat_count, 0, timeSlices-1, 0, atk90[0]));
            moveSave(i, 2, map(subPhase_repeat_count, 0, timeSlices-1, 10, atk90[1]));
            break;
          case 2://Move 1/2
            Serial.print("RightSide Move 1/2 Travel(mm):");
            Serial.println(map(subPhase_repeat_count, 0, timeSlices-1, 0, FR_Angles[4]/2));
            moveSave(i, 0, -find_hipMovement1(1, i, FR_Angles[4]/2 - map(subPhase_repeat_count, 0, timeSlices-1, 0, FR_Angles[4]/2)));
            moveSave(i, 1, kaReverse[0]);
            moveSave(i, 2, kaReverse[1]);
           break;
          case 3://Move 2/2
            Serial.print("RightSide Move 2/2 Travel(mm):");
            Serial.println(FR_Angles[4] - map(subPhase_repeat_count, 0, timeSlices-1, FR_Angles[4], FR_Angles[4]/2));
            moveSave(i, 0, find_hipMovement1(2, i, FR_Angles[4] - map(subPhase_repeat_count, 0, timeSlices-1,  FR_Angles[4], FR_Angles[4]/2)));
            moveSave(i, 1, kaReverse[0]);
            moveSave(i, 2, kaReverse[1]);
            break;
        }
        break;
    }
  }
  // if(subPhase_repeat_count == timeSlices-1){
  //   pwmReset();    
  // }
}
bool standGuardSteps(void *opaque) {//Move to standing guard position. This runs on a timer.
  size_t limit = (size_t)opaque;
  //These prints will crash the program if commented out...
  Serial.print("Guarding: ");
  Serial.print(standGuard_count);
  Serial.print("/");
  Serial.print(limit);
  Serial.print(" ");
  if(standGuard_count == 0){
    for(int i; i < 8; i++){
      for(int j = 0; j < 3; j++){
        moveSave(i, j, 0);
      }
    }
  }
  if(standGuard_count == limit/10){
    for(int j = 0; j<8; j++)
      moveLeg(1, j, 2, 4096);
  }
  if(standGuard_count < limit && standGuard_count > 0){
    for(int i = 0; i < 8; i++){
      moveLeg(1,i,1,findJointAngle(i,1,map(standGuard_count, 0, limit-1, 0, 90 )));
    }
  }
  if(standGuard_count == limit){
    //Serial.println("SavingKnees...");
    for(int k = 0; k < 8; k++){
      moveSave(k, 1, 95);
    }
    Serial.println(" ");
  }
  return ++standGuard_count <= limit; // remove this task after limit reached
}