#include "limb.h"

//vector<Limb*> Limb::instances;
//int Limb::twistRecordingsCurrentlyRunning = 0;

Limb::Limb()
{
    //ctor
}

void Limb::init(){
    currentCommands.node = nodeId;
    for(int i = 0; i < nrOfSerialCommands; ++i){
        currentCommands.commands[i] = 0;
    }
}

void Limb::updateData(dataPack &packet){

    pollsReceived = packet.radioStats[0];
    relaysReceived = packet.radioStats[1];
    pollsFailed = packet.radioStats[2];
    relaysFailed = packet.radioStats[3];
    retrieveDuration = packet.radioStats[4];
    nodeId = packet.nodeNr;

    //Timing and update variables
    if(packet.updateStamp == updateStamp){ // Not a new packet
        millisSinceLastUpdate = ofGetElapsedTimeMillis() - updateStamp;
        this->updated = false;
        if(millisSinceLastUpdate > 1000){
            if(isResponding){
                ofLog() << "no response from node " << nodeId;
            }
            isResponding = false;
        }
        return;
    }
    this->updated = true;
    if(!isResponding){
        ofLog() << "node " << nodeId << "now started responding";
        this->isResponding = true;
    }
    updateStamp = packet.updateStamp;
    millisSinceLastUpdate = 0;

    //Quaternion
    //We turn the coord system upside down
//    ofLog() << "received q: " << ofToString(packet.q[2]) << ",  " << ofToString(packet.q[1]) << ",  " << ofToString(-packet.q[3]) << ",  " << ofToString(packet.q[0]);
//    absoluteRotation.set(-packet.q[2], packet.q[1], packet.q[3], packet.q[0]);
    absoluteRotation.set(packet.q[2], packet.q[1], packet.q[3], packet.q[0]);
    absoluteRotation.normalize();
//    rotation = rotation.inverse();

    //magnetom
//    magnetomSamples.push_front(ofVec3f(packet.magnetom[0], packet.magnetom[1], packet.magnetom[2]));
//    if(magnetomSamples.size() > 50){
//        magnetomSamples.pop_back();
//    }
//    averageMagnetom = ofVec3f::zero();
//    for(int i = 0; i < magnetomSamples.size(); ++i){
//        averageMagnetom += magnetomSamples[i];
//    }
//    averageMagnetom /= magnetomSamples.size();

    //Accelerometer
    vec3i accelInputInt(packet.accel[0], packet.accel[1], packet.accel[2]);
    ofVec3f accelInputFloat(packet.accel[0], packet.accel[1], packet.accel[2]);


    gravityFloat = ofVec3f(0,0,1000); //ofVec3f(0,0,((gravityFloat*.99999).length() + (accelInputFloat*.00001).length()));
    gravityFloat = absoluteRotation.inverse()*gravityFloat;
    vec3i gravityInt(gravityFloat.x, gravityFloat.y, gravityFloat.z);
//    accelInputInt -= gravityInt;
//    accelInputFloat -= gravityFloat;

    //int version for keeping track of sum
    accelSamplesInt.push_front(accelInputInt);
    accelSum += accelInputInt;
    if(accelSamplesInt.size() > Limb::accelNrOfSamples){
        accelSum -= accelSamplesInt.back();
        accelSamplesInt.pop_back();
    }
    accelAverage = ofVec3f(accelSum.x, accelSum.y, accelSum.z)/accelSamplesInt.size();

    //float version
    accelSamplesFloat.push_front(accelInputFloat);
    if(accelSamplesFloat.size() > accelNrOfSamples){
        accelSamplesFloat.pop_back();
    }

    //Acceltrigger from node
    if(accelTriggerCounter != packet.accelTriggerId){
//        ofLog() << "setting accelTriggeredFlag";
        accelTriggeredFlag = true;
    }
    accelTriggerCounter = packet.accelTriggerId;
    accelTriggerVelocity = packet.accelTriggerVelocity;

    magnetomSamples.push_front(ofVec3f(packet.magnetom[0], packet.magnetom[1], packet.magnetom[2]));
    if(magnetomSamples.size() > magnetomNrOfSamples){
        magnetomSamples.pop_back();
    }

    relayRole = packet.relayRole;
}

void Limb::update(){
    calibratedRotation = absoluteRotation*yawOffset;

    ofVec3f up = ofVec3f(0,0,1);
//    ofVec3f rotVec = up*absoluteRotation;
//    angleFromLayingFlat = rotVec.angle(up);

    globalDirectionVector = limbVector*calibratedRotation;
    pitch = -(globalDirectionVector.angle(up) - 90);

    ofVec3f grv = calibratedRotation*rollVector;
//    ofVec3f norollRV = grv;
//    norollRV.z = 0;
    ofVec3f gdv = globalDirectionVector;
    if(isLefty){
        gdv = -gdv;
    }
    ofVec3f norollRV = up.getCrossed(gdv);
    roll = grv.angle(norollRV);
    if(grv.z < 0){
        roll = -roll;
    }

//    static int snapYaw;
//    int oldSnapYaw = snapYaw;
//    snapYaw = ofMap(yaw, -180, 180, 0, 12);
//
//
//    ofVec3f xyDirectionVector = globalDirectionVector;
//    xyDirectionVector.z = 0.0;
//    yaw = (ofVec3f(0,1,0)).angle(xyDirectionVector);
//
//    if(snapYaw != oldSnapYaw){
//        currentCommands.commands[3]++;
//    }



    box.setScale(1);

    if(this->parent){
//        ofLog() << "parent pointer is not NULL";
        if(assignedBodypart == LOVERARM){
            globalStartPoint = -parent->globalEndPoint;
//            box.setGlobalPosition(-parent->globalEndPoint);
        }else{
            globalStartPoint = parent->globalEndPoint;
//            box.setGlobalPosition(parent->globalEndPoint);
        }

        //Set quadrant and rotationcounter
        if(globalEndPoint.x < 0){
            if(globalEndPoint.y < 0){
                if(inQuadrant == 4){
                    rotationCount++;
                }
                inQuadrant = 1;
            }else{
                inQuadrant = 2;
            }
        }else{
            if(globalEndPoint.y >= 0){
                inQuadrant = 3;
            }else{
                if(inQuadrant == 1){
                    rotationCount--;
                }
                inQuadrant = 4;
            }
        }
    }else{
        if(assignedBodypart == CHEST){
//            box.setPosition(ofVec3f::zero());
            globalStartPoint = ofVec3f::zero();
        }else if(!nodeDataOverSerial){//Initial position of cubes
            box.setScale(1.5);
            globalStartPoint = ofVec3f(nodeId*50,-500,0);
//            box.setPosition(ofVec3f(nodeId*50,-500,0));
        }
    }
    globalEndPoint = globalStartPoint + globalDirectionVector*length;

    ofVec3f HGSP = globalEndPoint; //globalStartPoint;
    HGSP.z = 0;
    horizontalDistanceFromCenter = HGSP.length();

    ofVec3f y = ofVec3f(0,1,0);
    horizontalPositionAngle = y.angle(HGSP);
    if(HGSP.x < 0){
        horizontalPositionAngle = -horizontalPositionAngle;
    }
    horizontalPositionAngle += 180.0;

    //Position and scale the boxes
    if(assignedBodypart == LHAND || assignedBodypart == RHAND){
//        box.setGlobalPosition(globalEndPoint);
        box.setHeight(25);
        box.setDepth(length);
//        box.setGlobalPosition(globalStartPoint+0.5*(globalEndPoint - globalStartPoint));
        box.setGlobalPosition(globalEndPoint);
    }else{
        box.setGlobalPosition(globalStartPoint);
    }
    box.setGlobalOrientation(calibratedRotation);

    line.clear();
    line.addVertex(globalStartPoint);
    line.addVertex(globalEndPoint);

    //accelTrigger
    if(accelTriggeredFlag){
        globalPositionLastTrigger = globalEndPoint;
        uint64_t newStamp = ofGetElapsedTimeMillis();
        if(newStamp - accelTriggerStamp < doubleTapThreshold){
            lastTriggerWasDoubleTap = true;
        }else{
            lastTriggerWasDoubleTap = false;
        }
        accelTriggerStamp = newStamp;
        sendNoteOn = true;
//        ofLog() << "setting sendNoteOn";
        accelTriggeredFlag = false;
    }

    //Flash if acctriggered
    if(currentTriggerState == TRANSITIONTOTRIGGERACTIVE){
            triggerLerp = 1.0;
    }
    triggerLerp-=0.02;
    triggerLerp = ofClamp(triggerLerp, 0.0, 1.0);
    if(isResponding){
        if(relayRole == 'e'){
            staticColor = ofColor::pink;
        }else{
            staticColor.setSaturation(255);
            staticColor.setBrightness(255);
            staticColor.setHue(nodeId*30.0);
        }
    }else{
        staticColor = ofColor::grey;
    }
    triggerColor = staticColor.getInverted();
    currentColor = staticColor.getLerped(triggerColor, triggerLerp);


    //DEBUGGING WHEN USING ONLY ONE NODE
//    if(nodeDataOverSerial){
//        ofVec3f accVec = accelSamplesFloat.front();
//        ofVec3f magVec = magnetomSamples.front();
//        accVec = -accVec.normalize();
//        northFromAccMagVectorSamples[northFromAccMagVectorIndex] = magVec - ( magVec.dot(accVec) * accVec);
//        northFromAccMagVectorIndex++;
//        northFromAccMagVectorIndex %= northFromAccMagVectorSamples.size();
//        averageNorthVector.average(&northFromAccMagVectorSamples.front(), northFromAccMagVectorSamples.size());
//    }
}

void Limb::updateCommands(){
    //Sets color depending on note
    currentCommands.commands[1] = nearestTriggerPoint % notesPerLap * (255/notesPerLap);
    //sets brightness depending on proximity
    currentCommands.commands[2] = ofMap(angleFromNearestTriggerPoint, maxAngleFromTriggerPoint/2, 0, 0, 255, true);
    //increment command counter if it's a new state
    if(nearestTriggerPoint != previousNearestTriggerPoint){
        currentCommands.commands[3]++;
    }
//    currentCommands.node = nodeId;
    if(assignedBodypart == LHAND || assignedBodypart == RHAND){
        currentCommands.sendMe = true;
    }else if(assignedBodypart == CHEST){

    }
}

void Limb::turnOffLed(){
    currentCommands.commands[1] = 0;
    currentCommands.commands[2] = 0;
    currentCommands.sendMe = true;
}

void Limb::updateTriggerPointProximity(){

    const int startAngle = (360/notesPerLap)/2;
    int offset = ofMap(horizontalPositionAngle, -startAngle, 360-startAngle, 0, notesPerLap, false);// Seems like the mapping function acts weird if given negative boundaries on output? So let's offset the values afterwards.

    float targetAngle = offset*startAngle*2;
    angleFromNearestTriggerPoint = abs(targetAngle - horizontalPositionAngle);

    //Let's have zeroOffset in center. Also include laps
    offset -= notesPerLap/2;
    offset += (notesPerLap) * rotationCount;



    //two rows
    offset *= 2;
//    if(horizontalDistanceFromCenter > 600){
//        offset++;
//    }
    if(globalEndPoint.z > 180){
        offset ++;
    }
    if(globalEndPoint.z > -240){
        offset++;
    }

    previousNearestTriggerPoint = nearestTriggerPoint;
    nearestTriggerPoint = offset;
//    ofLog() << "offset: " << offset;
//    return getNoteFromOffset(offset);
}

void Limb::draw(){
    ofSetColor(currentColor);
    box.draw();
    line.draw();
//    if(globalPositionLastTrigger){
        ofDrawSphere(globalPositionLastTrigger, 15);
//    }
//    ofVec3f accVec = accelSamplesFloat.front()*0.5;
//    ofVec3f magVec = magnetomSamples.front();

    ofVec3f up = ofVec3f(0,0,1);
    ofVec3f gdv = globalDirectionVector;
    if(isLefty){
        gdv = -gdv;
    }
    ofVec3f norollRV = up.getCrossed(gdv);
    ofDrawArrow(globalStartPoint, globalStartPoint+(200*norollRV), 24);

    ofVec3f globalRollvector = calibratedRotation*rollVector;
    ofDrawArrow(globalStartPoint, globalStartPoint+(200*globalRollvector), 24);

    string infoString;
//    infoString += "magAccelAngle: " + ofToString(accVec.angle(magVec))+ "\n";
    infoString += "inQuadrant: " + ofToString(inQuadrant) + "\n";
    infoString += "rotationCount: " + ofToString(rotationCount) + "\n";
    infoString += "pitch: " + ofToString(pitch) + "\n";
    infoString += "roll: " + ofToString(roll) +"\n";
    infoString += "yaw: " + ofToString(yaw) + "\n";
    ofSetColor(ofColor::white);
    ofDrawBitmapString(infoString, globalStartPoint+ofVec3f(50, 0, 0));
//    ofDrawArrow(globalStartPoint, (globalStartPoint+accVec), 30);
//    ofDrawArrow(globalStartPoint, (globalStartPoint+magVec), 30);
//    ofSetColor(ofColor::pink);
//    ofDrawArrow(globalStartPoint, (globalStartPoint+ofVec3f(500,0,0)*calibratedRotation), 25);
//    ofDrawArrow(globalStartPoint, (globalStartPoint+ofVec3f(0,500,0)*calibratedRotation), 25);
//    ofDrawArrow(globalStartPoint, (globalStartPoint+ofVec3f(0,0,500)*calibratedRotation), 25);
//    ofDrawArrow(ofVec3f::zero(), averageTwistAxis*300, 24);
//    ofVec3f north = magnetom*rotation;
//    north.z = 0;
//    ofSetColor(ofColor::blue);
//    ofDrawArrow(ofVec3f::zero(), 5*north, 30);
//    ofSetColor(ofColor::yellow);
//    ofDrawArrow(ofVec3f::zero(), 5*ofVec3f(averageMagnetom.x, averageMagnetom.y, 0), 30);
//    ofDrawArrow(ofVec3f::zero(), averageMagnetom, 30);
//    ofSetColor(ofColor::coral);
//    ofDrawArrow(ofVec3f::zero(), accelSamplesFloat.front(), 24);
//    ofDrawArrow(ofVec3f::zero(), -accelSamplesFloat.front(), 24);
//    ofSetColor(ofColor::red);
//    ofDrawArrow(ofVec3f::zero(), 10*averageNorthVector, 30);
//    ofDrawArrow(box.getPosition(), box.getPosition()+accelAverage*rotation, 24);
//    ofDrawArrow(box.getPosition(), (box.getPosition()+accelSamplesFloat.front()), 24);
//    ofSetColor(ofColor::orange);
//    ofDrawArrow(box.getPosition(), (box.getPosition()+magnetomSamples.front()), 24);
//    ofSetColor(ofColor::green);
//    ofDrawArrow(box.getPosition(), box.getPosition()+10*averageNorthVector, 24);

}

 // 0 = transition to triggered state, 1 = no change, 2 = transition from triggered state
//void Limb::updateTriggerState(){
//    if(!accelSamplesFloat.size()){
//        currentTriggerState = TRIGGERINACTIVE;
//        return;
//    }
//
//    float distanceMax = 0;
//    float distanceSum = 0;
//    float currentDistance;
//    float previousDistance;
//    float currentLength = (accelSamplesFloat[0]/10).length();
//    for(int i = 0; i < 4; ++i){
//        currentDistance = (accelSamplesFloat[0]/10).distance(accelSamplesFloat[i]/10);
//        distanceMax = max(currentDistance, distanceMax);
//        distanceSum += currentDistance;
//    }
//
//    if(currentLength > 100 && distanceMax > 210.0 && ofGetElapsedTimeMillis()-triggerStamp > 120){
//        if(accelTriggerActive){
//            currentTriggerState = TRIGGERACTIVE;
//            return;
//        }
//        ofLog() << "acc triggered " << ofToString(ofGetElapsedTimef());
//        ofLog() << "distanceMax is " << distanceMax;
//        ofLog() << "duration between triggers " << ofGetElapsedTimeMillis()-triggerStamp;
//        accelTriggerActive = true;
//        triggerStamp = ofGetElapsedTimeMillis();
//        globalPositionLastTrigger = globalStartPoint;
//        currentTriggerState = TRANSITIONTOTRIGGERACTIVE;
//        return;
//    }
//    if(accelTriggerActive){
//        if(distanceSum < 250.0 && currentLength < 250){
//            ofLog() << "restoring acctrigger state";
//            accelTriggerActive = false;
//            currentTriggerState = TRANSITIONTOTRIGGERINACTIVE;
//            return;
//        }
//        currentTriggerState = TRIGGERACTIVE;
//        return;
//    }
//    currentTriggerState = TRIGGERINACTIVE;
//    return;
//}
//
//triggerState Limb::getTriggerState(){
//    return currentTriggerState;
//}

void Limb::startTwistRecording(){
    //Clear some stufff here!!!
    clearParent();
    box.setPosition(ofVec3f::zero());
    twistState = INIT;
    twistRecorded = false;
}

void Limb::stopTwistRecording(){
    if(twistState == EARLY){

    }
    twistState = INACTIVE;
}

// Returns a number corresponding to the state of the twist motion. 0 = inactive, 1 = init 2 = early, 3 = waiting, 4 = finished
int Limb::recordTwist(){
    if(twistState == INACTIVE){
        return twistState;
    }
    if(twistState == FINISHED){//EXIT
        return twistState;
    }
    if(twistState == INIT)
    {
        ofLog() << "inititating recording of twist";
//        Limb::twistRecordingsCurrentlyRunning++;//Static counter

        // INIT ALL THE STUFF
        averageTwistAxis = ofVec3f::zero();
        twistAxis.clear();
        limbVector = ofVec3f::zero();
        twistAngleDistance = 0;
        rotationCount = 0;

        twistOrigin = absoluteRotation;
        //TODO: Catch nodes that are turned off
//        if(!isResponding){
//            ofLog() << "Skipping recording of non responsive node " << ofToString(nodeId);
////            Limb::twistRecordingsCurrentlyRunning--;
//            twistState = FINISHED;
//            return;
//        }

        previousRotation = twistOrigin;
        twistState = EARLY;
    }

    //Calculate this q-sample
    currentRotation = absoluteRotation;
    ofQuaternion deltaRot = currentRotation.inverse() * previousRotation;
    float angle;
    ofVec3f axis;
    deltaRot.getRotate(angle, axis);

    if(angle > 3){// Ok this is a big enough difference between two sampled rotations to save a new one.
        previousRotation = currentRotation;
        ofQuaternion twistQuat = currentRotation.inverse() * twistOrigin; //Whats the rotation from the twistorigin
        twistQuat.getRotate(angle, axis);
        twistAngleDistance = max(angle, twistAngleDistance);
        if(angle > 15){//If we have reached far enough, start waiting for return.
            twistAxis.push_back(axis);
            limbVector += axis * currentRotation.inverse();
            averageTwistAxis += axis;
            twistState = WAITING;
        }else if(angle < 14 && twistState == WAITING){
            ofLog() << "recording of node " << ofToString(nodeId) << " twist done";
            averageTwistAxis.normalize();
//            ofLog() << "AverageTwistAxis is " << averageTwistAxis.x << ", " << averageTwistAxis.y << ", " << averageTwistAxis.z;
            ofVec3f tempVec = averageTwistAxis;
            tempVec.z = 0;
            float yaw = tempVec.angle(ofVec3f(0,1,0));
            if(averageTwistAxis.x < 0){
                yaw = -yaw;
            }
            yawOffset = ofQuaternion(yaw, ofVec3f(0,0,1));

            //Calculate rollvector
            rollVector = averageTwistAxis.getCrossed(ofVec3f(0,0,1));
            rollVector.cross(averageTwistAxis);
            rollVector = rollVector * twistOrigin.inverse();
            rollVector.normalize();

            limbVector.normalize();
//            Limb::twistRecordingsCurrentlyRunning--;
            twistRecorded = true;
            twistState = FINISHED;
//            if(!Limb::twistRecordingsCurrentlyRunning){
//                ofLog() << "all twists finished";
//            }
        }
    }
    //We are in EARLY state
    return twistState;
}

void Limb::clearParent(){
    parent = NULL;
    box.clearParent();
}

bool Limb::setParent(Limb* newParent){
    if(!newParent){
            return false;
    }
    parent = newParent;
    box.setParent((newParent->box));
    return true;
}

Limb* Limb::getParent(){
    return parent;
}

void Limb::assignChestRole(){
    assignedBodypart = CHEST;
    length = partLengths[1];
    ofVec3f tempVec = absoluteRotation*ofVec3f(0,0,1);
    tempVec.z = 0;
    float yaw = tempVec.angle(ofVec3f(0,1,0));
    if(tempVec.x < 0){
        yaw = -yaw;
    }
    yawOffset = ofQuaternion(yaw, ofVec3f(0,0,1));
    limbVector = ofVec3f(-1,0,0);
}
