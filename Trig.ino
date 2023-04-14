int thighLength = 75;
int shinLength = 75;
int footLength = 50;
float combinedX[91*91];
float combinedZ[91*91];
//int hipSolutions[360*];//HUGE MAP OF DEGREES*Heights*leg start/finish an

float a2r(float b){// Quick conversion from angle system to radians
  return b / 180.0 * PI;
}
float r2a(float r){// Rad to DEG
  return r * 180.0 / PI;
}
float otheracos(float x){
  float negate = float(x < 0);
  float ret = -0.0187293;
  x = abs(x);
  ret = x * -0.0187293;
  ret += 0.0742610;
  ret *= x;
  ret -= 0.2121144;
  ret *= x;
  ret += 1.5707288;
  ret *= sqrt(1.0 - x);
  ret = ret - 2.0 * negate * ret;
  return negate * 3.14159265358979 + ret;
}
void setHipRanges(){//Sets the hip0Angle and hipRanges
  float angleOrigin = 0;
  int actualLeg = 0;
  for(int l = 0; l < 8; l++){//Setting up leg angles and ranges of motion for hips
    angleOrigin = 22.5 + 45 * l;
    if(angleOrigin > 359){
      angleOrigin = 360 - 22.5 - 45 *l;
    }
    actualLeg = 7-4-l;
    if(actualLeg <0){
      actualLeg = 8+ actualLeg;
    }
    //Serial.print(actualLeg);
    hip0Angle[actualLeg] = angleOrigin;
    if(actualLeg != 3 && actualLeg != 4){
      hipRanges[actualLeg] = 90;
    }
    else{
      hipRanges[actualLeg] = 70;
    }
  }
  // Serial.println(" HIPSTUFF: ");
  // for(int i = 0; i < 8; i++){
  //   Serial.print(hip0Angle[i]);
  //   Serial.println(hipRanges[i]);
  // }

}
void generate_HeightTable(){
  for(int kAngle = 0; kAngle <= 90; kAngle++){
    int TA = 90 - kAngle;//Thigh/ankle angle of missing angle for knee triangle
    float shinX = shinLength*sin(a2r(TA))/sin(PI/2);
    float shinZ = shinLength*sin(a2r(kAngle))/sin(PI/2);
    Serial.print("kAngle:");
    Serial.println(kAngle);
    Serial.print(" shinX:");
    Serial.println(shinX);
    // Serial.print(" ShinZ:");
    // Serial.println(shinZ);
    for(int aAngle = 0; aAngle <=90; aAngle++){
      // Serial.print("aAngle:");
      // Serial.println(aAngle);
      int combinedAngle = kAngle+aAngle;
      // Serial.print("CombinedAngle:");
      // Serial.println(combinedAngle);
      float compound_footX = 0;
      float compound_footZ = 0;
      int internalSF = 0;
      int internalFG = 0;
      if(combinedAngle < 90){//This adds to overall x
        internalSF = combinedAngle;
        internalFG = 90 - internalSF;
        compound_footX = footLength*sin(a2r(internalFG))/sin(PI/2);
        compound_footZ = footLength*sin(a2r(internalSF))/sin(PI/2);
      }
      else if(combinedAngle == 90){//not right
        compound_footX = 0;
        compound_footZ = footLength;
      }
      else if(combinedAngle > 90){
        internalSF = 180 - combinedAngle;
        internalFG = 90 - internalSF;
        compound_footX = -footLength*sin(a2r(internalFG))/sin(PI/2);
        compound_footZ = footLength*sin(a2r(internalSF))/sin(PI/2);
      }
      if(compound_footZ < 12){
        compound_footZ = 12;
      }
      // Serial.print("compoundX");
      // Serial.println(compound_footX);
      // Serial.print("compoundZ");
      // Serial.println(compound_footZ);
      // Serial.print("StorageSpot:");
      // Serial.println(kAngle*91+aAngle);
      // Serial.print("Find it@ kangle:");
      // Serial.print((kAngle*91+aAngle)/91);
      // Serial.print(" aNgle:");
      // Serial.println((kAngle*91+aAngle)%91);
      combinedX[kAngle*91+aAngle] = thighLength+shinX+compound_footX;
      combinedZ[kAngle*91+aAngle] = shinZ+compound_footZ;
    }
  }
}
void find_frontPath(int height){//Finds maximum distance the front legs can move while in contact @ height. Outputs FR_angles kneemin anklemin kneemax anklemax && Distance traveled.
  int minX = 200;
  int maxX = 0;
  for(int i = 0; i < 91*91; i++){
    if(round(combinedZ[i]) == height){
      if(combinedX[i] < minX){
        minX = combinedX[i];
        //FR_Angles is in pinkBLE.ino
        FR_Angles[0] = i/91;//Finds the KNEE angle for closest x position.
        FR_Angles[1] = i%91;//Finds the ANKLE angle for closest xposition.
      }
      if(round(combinedX[i]) > maxX){
        maxX = combinedX[i];
        FR_Angles[2] =i/91;//Finds the KNEE angle for Farthest x position.
        FR_Angles[3] = i%91;//Finds the ANKLE angle for Farthest x position.
      }
    }
  }
  FR_Angles[4] = maxX - minX;
  if(debugPrint){         
    Serial.print("Translated height(mm from hips(without flex)):");
    Serial.println(height);
    // Serial.print("minX:");
    // Serial.println(minX);
    // Serial.print("minXkAngle:");
    // Serial.println(FR_Angles[0]);
    // Serial.print("minXaAngle:");
    // Serial.println(FR_Angles[1]);
    // Serial.print("maxX:");
    // Serial.println(maxX);
    // Serial.print("maxXkAngle:");
    // Serial.println(FR_Angles[2]);
    // Serial.print("maxXaAngle:");
    // Serial.println(FR_Angles[3]);
    Serial.print("maxTravel Front/Rear legs(mm):");
    Serial.println(FR_Angles[4]);
  }
}
void find_90attack(int height){//Finds a combo of ankle and knee joints that combine to 90 and meet the height requirement
  int bestA;
  int bestK;
  float closestH = 5;
  int bestX;
  for(int k = 0; k < 91; k++){//Knee angles
    for(int a = 0; a < 91; a++){
      if (a+k == 90 && combinedZ[k*91+a] == height){
        closestH = 0;
        bestA = a;
        bestK = k;
        bestX = combinedX[k*91+a];
        if(debugPrint){
          Serial.print("solved! (K_A):");
          Serial.print(bestK);
          Serial.print(" ");
          Serial.println(bestA);
        }
      }
      else if (a+k == 90){
        if(abs(height - combinedZ[k*91+a]) < closestH){
          closestH = abs(height - combinedZ[k*91+a]);
          bestA = a;
          bestK = k;
          bestX = combinedX[k*91+a];
          if(debugPrint){
            Serial.print("Closest(K_A):");
            Serial.print(bestK);
            Serial.print(" ");
            Serial.print(bestA);
            Serial.print(" Height:");
            Serial.println(combinedZ[k*91+a]);
          }
        }
      }     
    }
  }
  atk90[0] = bestK;
  atk90[1] = bestA;
  atk90[2] = bestX;
  Serial.print("Distance from hip @ 90:");
  Serial.println(atk90[2]);
}
void orientLegs(int angle){
  numOf_leftLegs = 0;//Reset the side count
  numOf_rightLegs = 0;
  for(int i=0; i<8; i++){
    hipTrajectory(i, angle);//Figure out leg orientation for the angle of attack
  }
  sideOrder();//In Trig.ino. Finds the order of side legs from front to back.
  phaseCycle();//In Phaser.ino. Outputs the phase order for legs based upon the first front leg.
}
void hipTrajectory(int leg, int angle_desired){
  Serial.print("Leg:");
  Serial.print(leg);
  Serial.print(" AngleDesired:");
  Serial.print(angle_desired);
  float legHipRangeMin= hip0Angle[leg]-hipRanges[leg]/2;
  float legHipRangeMax = hip0Angle[leg]+hipRanges[leg]/2;
  float legHipRangeMin2 = legHipRangeMin;
  float legHipRangeMax2 = legHipRangeMax;

  //Legs 3&4 need two ranges
  if(legHipRangeMin < 0){
    legHipRangeMax2 = 359;
    legHipRangeMin2 = 360 + legHipRangeMin;
    legHipRangeMin = 0;
  }
  else if(legHipRangeMax > 359){
    legHipRangeMax2 = legHipRangeMax - 360;
    legHipRangeMin2 = 0;
    legHipRangeMax = 359;
  }
  // Serial.print(" LegMin:");
  // Serial.print(legHipRangeMin);
  // Serial.print(" LegMax:");
  // Serial.print(legHipRangeMax);
  // Serial.print(" LegMin2:");
  // Serial.print(legHipRangeMin2);
  // Serial.print(" LegMax2:");
  // Serial.println(legHipRangeMax2);
  //Is the angle desired within the legs range?
  if ( ( (angle_desired >= legHipRangeMin) && (angle_desired <= legHipRangeMax) ) || ( (angle_desired >= legHipRangeMin2) && (angle_desired <= legHipRangeMax2) )){
    //Serial.println("Front Legs, Set Hip servos to this angle");//Pulling legs
    legAssignment[leg] = 0;
  }
  //Find rear legs
  else{
    int angle_rear = angle_desired + 180;
    if (angle_rear > 359){
      angle_rear -= 360;
    }
    if ( ( (angle_rear >= legHipRangeMin) && (angle_rear <= legHipRangeMax) ) || ( (angle_rear >= legHipRangeMin2) && (angle_rear <= legHipRangeMax2) )){
      //Serial.println("Rear Legs, Set Hip servos to this angle");//Pushing legs
      legAssignment[leg] = 2;
    }
    else{//Set servos to the closest angle
      float dist_minside;
      float dist_maxside;
      //This Stuff is needed for finding the closest angle to the desired angle for dragging legs
      if(angle_desired < hip0Angle[leg]){
        dist_minside  = hip0Angle[leg]-angle_desired;//Verified@ 0,90,180,270
        dist_maxside = angle_desired + 360 - hip0Angle[leg];//Verified@ 0,90,180,270
      }
      else{
        dist_minside  = hip0Angle[leg]+ 360 - angle_desired;//Verified@ 0,90,180,270
        dist_maxside = angle_desired - hip0Angle[leg];//Verified@ 0,90,180,270
      }
      // Serial.print("HipOrigin:");
      // Serial.print(hip0Angle[leg]);
      // Serial.print(" Range on min side:");
      // Serial.print(dist_minside);
      // Serial.print(" Range on max side:");
      // Serial.println(dist_maxside);
      if(dist_minside < dist_maxside){//This is a Right leg
        //Serial.println("Move to min");
        legAssignment[leg] = 3;
        hipDist2Angle[leg] = dist_minside;
        numOf_rightLegs++;
      }
      else{//This is a left Leg
        //Serial.println("Move to max");
        legAssignment[leg] = 1;
        hipDist2Angle[leg] = dist_maxside;
        numOf_leftLegs++;
      }
    }
  }
  Serial.print(" Assignment:");
  Serial.println(legAssignment[leg]);
}
void sideOrder(){
  //Order the side legs by Angular Distance from the desired angle(front to back)
  int distChecker1 = 180;
  int distChecker2 = 180;
  int distCheckerR1 = 180;
  int distCheckerR2 = 180;
  for(int i = 0; i < 8; i++){
    // Serial.println(legAssignment[i]);
    if (legAssignment[i] == 1){//This is a leftLeg
      // Serial.println("Comparing Lefts");
      if (hipDist2Angle[i] < distChecker1){
        leftOrder[2] = leftOrder[1];
        leftOrder[1] = leftOrder[0];
        leftOrder[0] = i;
        distChecker1 = hipDist2Angle[i];
      }
      else if(hipDist2Angle[i] < distChecker2){
        leftOrder[2] = leftOrder[1];
        leftOrder[1] = i;
        distChecker2 = hipDist2Angle[i];
      }
      else{
        leftOrder[2] = i;
      }
    }
    if ( legAssignment[i] == 3){//This is a RIGHT leg
      // Serial.println("Comparing Rights");
      if (hipDist2Angle[i] < distCheckerR1){
        rightOrder[2] = rightOrder[1];
        rightOrder[1] = rightOrder[0];
        rightOrder[0] = i;
        distCheckerR1 = hipDist2Angle[i];
      }
      else if(hipDist2Angle[i] < distCheckerR2){
        rightOrder[2] = rightOrder[1];
        rightOrder[1] = i;
        distCheckerR2 = hipDist2Angle[i];
      }
      else{
        rightOrder[2] = i;
      }
    }
  }
  //If the angular difference is < 90 they are front side legs, if angular dist > 90, they are rear side legs.
  // for(int i= 0; i < numOf_leftLegs; i++){
  //   Serial.print("LeftLegs Front->Back:");
  //   Serial.print(leftOrder[i]);
  //   Serial.print(" OriginAngle:");
  //   Serial.print(hip0Angle[leftOrder[i]]);
  //   if(i == 0){//This is the most front left side leg
      
  //     Serial.print(" LeftSide_Front -> angular difference:");
  //     Serial.print(hipDist2Angle[leftOrder[i]]);
  //   }
  //   else if(i == numOf_leftLegs-1){// This is the most rear left side leg
    
  //     Serial.print(" LeftSideRear -> angular difference:");
  //     Serial.print(hipDist2Angle[leftOrder[i]]);
  //   }
  //   else if( i == 1 && numOf_leftLegs == 3){// This is the middle left side leg
  //     Serial.print(" LeftSide_Mid -> angular difference:");
  //     Serial.print(hipDist2Angle[leftOrder[i]]);
  //   }
  //   Serial.println(" ");
  // }
  // for(int i= 0; i< numOf_rightLegs; i++){
  //   Serial.print("RightLegs Front->Back:");
  //   Serial.print(rightOrder[i]);
  //   Serial.print(" OriginAngle:");
  //   Serial.print(hip0Angle[rightOrder[i]]);
  //   if(i == 0){//This is the most front right side leg
  //     Serial.print(" RightSide_Front -> angular difference:");
  //     Serial.print(hipDist2Angle[rightOrder[i]]);
  //   }
  //   else if(i == numOf_rightLegs-1){// This is the most rear left side leg
  //     Serial.print(" RightSide_Rear -> angular difference:");
  //     Serial.print(hipDist2Angle[rightOrder[i]]);
  //   }
  //   else if( i == 1 && numOf_rightLegs == 3){// This is the middle left side leg
  //     Serial.print(" RightSide_Mid -> angular difference:");
  //     Serial.print(hipDist2Angle[rightOrder[i]]);
  //   }
  //   Serial.println(" ");
  // }
}
void find_kaReverse(float distance){//Input how long the end-effector(foot) should be from the hip origin, output the best angle combo to achieve this.
  float closestX = 25;
  float closestZ = 25;
  int bestA = 0;
  int bestK = 45;
  //Search through the combined x/z tables// SEARCH ONLY WITHIN 5 degrees of the last position.
  for(int k = 0; k <= 90; k++){
    for(int a = 0; a <= 90; a++){
      if (abs(LastHeight - combinedZ[k*91+a]) < closestZ && abs(distance - combinedX[k*91+a]) < closestX){
        closestX = abs(distance - combinedX[k*91+a]);
        closestZ = abs(LastHeight - combinedZ[k*91+a]);
        bestA = a;
        bestK = k;
      }
    }
  }
  kaReverse[0] = bestK;
  kaReverse[1] = bestA;

  Serial.print("find_kaReverse:");
  Serial.print(" bestK:");
  Serial.print(bestK);
  Serial.print(" bestA:");
  Serial.print(bestA);
  Serial.print(" closestHeight:");
  Serial.print(combinedZ[kaReverse[0]*91+kaReverse[1]]);
  Serial.print(" closestDistance");
  Serial.println(combinedX[kaReverse[0]*91+kaReverse[1]]);
}
float find_hipMovement1(int movement, int leg, int travel){ // find all hip positions to walk in a line
  //RightLegs
  float legLengthb;
  float angleA;
  //float angleC;
  if (movement == 1){
    legLengthb = sqrt(pow(travel,2) + pow(atk90[2],2) - (2* travel * atk90[2]) * cos(a2r(180 - hipDist2Angle[leg] )));
    angleA = r2a(otheracos( (pow(legLengthb,2) + pow(atk90[2],2) - pow(travel,2)) / (2 * legLengthb * atk90[2])));
    //angleC = r2a(otheracos( (pow(legLengthb,2) + pow(travel,2) - pow(atk90[2],2)) / (2 * legLengthb * travel)));
    // Serial.print("Leg:");
    // Serial.print(leg);
    // Serial.print(" angle_A:");
    // Serial.print(angleA);
    // Serial.print(" angle_B:");
    // Serial.print(180 - hipDist2Angle[leg]);
    // // Serial.print(" angle_C:");
    // // Serial.print(angleC);
    // Serial.print(" Required legX:");
    // Serial.println(legLengthb);
    find_kaReverse(legLengthb);
    return angleA;
  }
  else if (movement == 2){
    legLengthb = sqrt(pow(travel,2) + pow(atk90[2],2) - (2* travel * atk90[2]) * cos(a2r( hipDist2Angle[leg] )));
    angleA = r2a(otheracos( (pow(legLengthb,2) + pow(atk90[2],2) - pow(travel,2)) / (2 * legLengthb * atk90[2])));
    //angleC = r2a(otheracos( (pow(legLengthb,2) + pow(travel,2) - pow(atk90[2],2)) / (2 * legLengthb * travel)));
    // Serial.print("Leg:");
    // Serial.print(leg);
    // Serial.print(" angle_A:");
    // Serial.print(angleA);
    // Serial.print(" angle_B:");
    // Serial.print(180 - hipDist2Angle[leg]);
    // // Serial.print(" angle_C:");
    // // Serial.print(angleC);
    // Serial.print(" Required legX:");
    // Serial.println(legLengthb);
    find_kaReverse(legLengthb);
    return angleA;
  }
}