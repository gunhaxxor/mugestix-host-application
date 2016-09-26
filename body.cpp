#include "body.h"
//#include <vector>
#include "limb.h"

Body::Body()
{
    //ctor
}

Body::~Body()
{
    //dtor
}

void Body::threadedFunction(){
    while(isThreadRunning()){
        int limbIndex = 0;
//        if(mySerialThread.isThreadRunning()){
//            mySerialThread.lock();
//            for(int i = 0; i < nrOfNodes; ++i){
//                if(mySerialThread.packet[i].contactEstablished){
//                        if(limbIndex >= limbs.size()){
//                            limbs.push_back(Limb());
//                            ofLog() << "new limb object with index " << limbIndex << " created";
//                        }
//                        limbs[limbIndex++].updateData(mySerialThread.packet[i]);
//                }
//            }
//            for(Limb& lmb : limbs){
//                if(lmb.currentCommands.sendMe){
//                    mySerialThread.nodeCommands[lmb.nodeId] = lmb.currentCommands;
//                }
//            }
//    //        skippedPacketsFromSerial = mySerialThread.skippedPackets - 1;
//            mySerialThread.skippedPackets = 0;
//            mySerialThread.unlock();
//        }


        updateSerial();

        lock();
        for(int i = 0; i < nrOfNodes; ++i){
            if(packet[i].contactEstablished){
                    if(limbIndex >= limbs.size()){
                        limbs.push_back(Limb());
                        limbs[limbIndex].updateData(packet[i]);
                        limbs[limbIndex].init();
                        ofLog() << "new limb object with index " << limbIndex << " created";
                    }else{
                        limbs[limbIndex].updateData(packet[i]);
                    }
                    limbIndex++;
            }
        }

        for(Limb& lmb : limbs){
            if(lmb.currentCommands.sendMe){
                nodeCommands[lmb.nodeId] = lmb.currentCommands;
//                ofLog() << "transfering command for node " << ofToString(lmb.nodeId) << " to serial queue. Commands are: " << ofToString((int)nodeCommands[lmb.nodeId].commands[0]) << " " << ofToString((int)nodeCommands[lmb.nodeId].commands[1])<< " " << ofToString((int)nodeCommands[lmb.nodeId].commands[2])<< " " << ofToString((int)nodeCommands[lmb.nodeId].commands[3]);
            }
        }
        update();
        unlock();
    }
}

void Body::updateSerial(){
    if(!serial.isInitialized()){
        startSerial();
    }


    serial.writeByte('r'); //Request a dataset!!!
    for(const commandGroup& cmd : nodeCommands){
            if(cmd.sendMe){
//                    ofLog() << "writing command to serial";
//                        uint8_t node = 3;
//                        uint8_t comms[4] = {0, 120, 255, 0};
//                        serial.writeByte(node);
//                        serial.writeBytes((uint8_t*) comms, 4);
                    serial.writeByte(cmd.node);
//                    uint8_t testarray[4] = {0,255,255,0};
//                    serial.writeBytes((uint8_t*) testarray, nrOfSerialCommands);
                    serial.writeBytes((uint8_t*) cmd.commands, nrOfSerialCommands);
            }
    }
    serial.writeByte('>');


    //Check for message header
    while(serial.available() < 4){//We don't want to update if there is no new data in serial buffer
        //block
    }
    bool headerReceived = true;
    for(int i = 0; i < 4; ++i){
            if(serial.readByte() != '#'){
                headerReceived = false;
                serial.flush();
                break;
            }
    }

    if(headerReceived){
        if(!nodeDataOverSerial){
            readFromBasestaion();
        }else{
            readFromNode();
        }
    }
}

void Body::readFromBasestaion(){
    while(!serial.available() && serial.isInitialized()){
            //block until we have the first byte
    }
    int nrOfDataSets = serial.readByte();
    int nrOfBytes = 30 * nrOfDataSets + 1;
    while(serial.available() < nrOfBytes && serial.isInitialized()){
            //block until we have a complete dataset
    }
//            ofLog() << "Header received!";

    for(int i = 0; i < nrOfDataSets; ++i){

        int nodeNr = serial.readByte();
        int incStamp = serial.readByte();//A byte with incStamp
//            ofLog() << "nrOfBytes is: " << nrOfBytes;

//                int nrOfElements = nrOfBytes / 2;//The data is stored as fixed deciaml point int16_t. I.E. two bytes per element
//                        ofLog() << "nrOfElements is: " << nrOfElements;

        unsigned char tempData[32]; // This will store the received data before it's handled
        serial.readBytes(tempData, 26); //Read it all!
        unsigned char relayInfo = serial.readByte();
        uint8_t endingCharacter = serial.readByte();//At this point there should be an ending character. If not, something's wrong!

        if(endingCharacter != ']'){
            ofLog() <<  "The individual ending character was not correct! flushing";
            serial.flush();
            break;
        }
        else{
//                    ofLog() << "complete message received";
        }

        if(nodeNr < 0 || nodeNr > nrOfNodes-1){//Something's wrong. Index out of bound.
            ofLog() << "Something's wrong. Node index out of bound.";
            continue;
        }



        lock();
        packet[nodeNr].contactEstablished = true;
        packet[nodeNr].nodeNr = nodeNr;
        int index = 0;
        for(int i = 0; i < 4; i++){
            binaryInt16 number;
            number.c[0] = tempData[index*2];
            number.c[1] = tempData[index*2+1];
//                    if(i < 4){
            packet[nodeNr].q[i] = Q14ToFloat(number.i);
//                    }else{
//
//                    }
            index++;
        }

        packet[nodeNr].accelTriggerId = tempData[index*2];
        packet[nodeNr].accelTriggerVelocity = tempData[index*2+1];
        index++;

        for(int i = 0; i < 3; ++i){
            binaryInt16 number;
            number.c[0] = tempData[index*2];
            number.c[1] = tempData[index*2+1];
            packet[nodeNr].accel[i] = number.i;
            index++;
        }

//        for(int i = 0; i < 3; ++i){
//            binaryInt16 number;
//            number.c[0] = tempData[index*2];
//            number.c[1] = tempData[index*2+1];
//            packet[nodeNr].magnetom[i] = (float) number.i;
//            index++;
//        }

        //Radio stats
        for(int i = 0; i < 5; ++i){
            binaryInt16 number;
            number.c[0] = tempData[index*2];
            number.c[1] = tempData[index*2+1];
            packet[nodeNr].radioStats[i] = number.i;
            index++;
        }
        packet[nodeNr].relayRole = relayInfo;
        packet[nodeNr].updateStamp = ofGetElapsedTimeMillis();
        unlock();
        //put the incoming data in their corresponding variables
//                copyFloatsFromTo(data, q[nodeNr], 0, 4);
//                copyFloatsFromTo(data, mag, 4, 3);
    }

    if(serial.readByte() != '>'){
        ofLog() <<  "The global ending character was not correct! flushing";
        serial.flush();
    }
}

void Body::readFromNode(){
    int nodeNr = 0;
    while(!serial.available() && serial.isInitialized()){
            //block until we have the first byte
    }
    int nrOfBytes = serial.readByte();
    while(serial.available() < nrOfBytes && serial.isInitialized()){
            //block until we have a complete dataset
    }
//            ofLog() << "Header received!";



//            ofLog() << "nrOfBytes is: " << nrOfBytes;

//                int nrOfElements = nrOfBytes / 2;//The data is stored as fixed deciaml point int16_t. I.E. two bytes per element
//                        ofLog() << "nrOfElements is: " << nrOfElements;

    unsigned char tempData[nrOfBytes]; // This will store the received data before it's handled
    serial.readBytes(tempData, nrOfBytes); //Read it all!
    uint8_t endingCharacter = serial.readByte();//At this point there should be an ending character. If not, something's wrong!

    if(endingCharacter != '>'){
        ofLog() <<  "The individual ending character was not correct! flushing";
        serial.flush();
    }
    else{
//                    ofLog() << "complete message received";
    }

    lock();
    packet[nodeNr].contactEstablished = true;
    packet[nodeNr].nodeNr = 0;
    int index = 0;
    for(int i = 0; i < 4; i++){
        binaryInt16 number;
        number.c[0] = tempData[index*2];
        number.c[1] = tempData[index*2+1];
        packet[nodeNr].q[i] = Q14ToFloat(number.i);
        index++;
    }
    for(int i = 0; i < 3; ++i){
        binaryInt16 number;
        number.c[0] = tempData[index*2];
        number.c[1] = tempData[index*2+1];
        packet[nodeNr].accel[i] = (number.i);
        index++;
    }
//    for(int i = 0; i < 3; ++i){
//        binaryInt16 number;
//        number.c[0] = tempData[index*2];
//        number.c[1] = tempData[index*2+1];
//        packet[nodeNr].magnetom[i] = number.i;
//        index++;
//    }
    packet[nodeNr].updateStamp = ofGetElapsedTimeMillis();
    unlock();
}

//TODO: Check that the uint64 comparison below is ok.
void Body::update(){
    for(int i = 0; i < limbs.size(); ++i){
            limbs[i].update();

            if(!inTriggerFreeZone && outputNotes){
                limbs[i].updateTriggerPointProximity();
                limbs[i].updateCommands();
            }else{
                limbs[i].turnOffLed();
            }
    }

    updateTriggerFreeZone();

    if(haveTwoHands){
        uint64_t millisSinceLastTrigger = ofGetElapsedTimeMillis() - max(lHand->accelTriggerStamp, rHand->accelTriggerStamp);
        if(millisSinceLastTrigger < gestureWaitTime){
            waitingForGesture = true;
        }else{waitingForGesture = false;}


        if(abs((int64_t)(lHand->accelTriggerStamp - rHand->accelTriggerStamp)) < syncTapThreshold){
            lastTriggerWasSync = true;
        }else{
            lastTriggerWasSync = false;
        }
    }
    lHandIsTriggable = false;
    rHandIsTriggable = false;
    if(lHand && isTriggable(lHand)){
        lHandIsTriggable = true;
    }
    if(rHand && isTriggable(rHand)){
        rHandIsTriggable = true;
    }
    updateGestures();
    updateMidi();

    if(twistRecordingActive){
        recordTwist();
    }else if(sideChooserActive){
        sideChooser();
    }
}

void Body::recordTwist(){//Ok. A bit complex perhaps. Those limbs that never left the early state of twistrecording should be ignored and the recording considered finished.
    int nrOfInactiveLimbs = 0;
    int nrOfEarlyLimbs = 0;
    int nrOfWaitingLimbs = 0;
    int nrOfFinishedLimbs = 0;
    int indexOfLatestLimbInEarlyState;
    for(int i = 0; i < limbs.size(); ++i){
        int state =  limbs[i].recordTwist();
        if(state == 0){
            nrOfInactiveLimbs++;
        }
        if(state == 2){
            nrOfEarlyLimbs++;
            indexOfLatestLimbInEarlyState = i;
        }
        if(state == 3){
            nrOfWaitingLimbs++;
        }

        if(state == 4){
            nrOfFinishedLimbs++;
        }
    }
//    ofLog() << "nrOfInactiveLimbs " << nrOfInactiveLimbs;
//    ofLog() << "nrOfEarlyLimbs " << nrOfEarlyLimbs;
//    ofLog() << "nrOfWaitingLimbs " << nrOfWaitingLimbs;
//    ofLog() << "nrOfFinishedLimbs " << nrOfFinishedLimbs;
    bool allFinished = false;
    if(nrOfFinishedLimbs){
        if(nrOfFinishedLimbs + nrOfEarlyLimbs + nrOfInactiveLimbs == limbs.size()){
            allFinished = true;
            if(nrOfEarlyLimbs){
                ofLog() << "asssigning chest to node " << indexOfLatestLimbInEarlyState;
                setChestNode(&limbs[indexOfLatestLimbInEarlyState]);
            }
        }
    }
    if(allFinished){
        stopTwistRecording();
        ofLog() << "finished all twists";


        calculateNodeOnBodyLocations();
        startSideChooser();


    }
}

void Body::startTwistRecording(){
    chest = NULL;
    twistRecordingActive = true;
    for(int i = 0; i < limbs.size(); ++i){
        if(limbs[i].isResponding){
            limbs[i].startTwistRecording();
        }
    }
}

void Body::stopTwistRecording(){
    twistRecordingActive = false;
    for(int i = 0; i < limbs.size(); ++i){
        limbs[i].stopTwistRecording();
    }
}

void Body::calculateNodeOnBodyLocations(){
    ofLog() << "calculateNodeOnBodyLocations() started";
    limbGroups.clear();
    for(int i = 0; i < limbs.size(); ++i){
        ofLog() << "Starting comparing iteration for limb " << i << " out of " << limbs.size()-1 << " with nodeId " << limbs[i].nodeId;
        limbs[i].clearParent();
        if(!limbs[i].twistRecorded){
            //If this node is missing a recording, ignore it!
            ofLog() << "skipped limb " << i <<", nodeId " << limbs[i].nodeId << " since it's not recorded";
            continue;
        }
        if(!limbGroups.size()){
            limbGroups.push_back(limbGroup());
            limbGroups.back().group.push_back(&limbs[i]);
            ofLog() << "Creating first limbGroup";
        }
        else{
            for(int groupIndex = 0; groupIndex < limbGroups.size(); ++groupIndex){
                ofLog() << "Comparing alignment of node " << limbs[i].nodeId << " with limbGroup " << groupIndex << " comparing against node " << limbGroups[groupIndex].group.front()->nodeId;
                if(limbs[i].averageTwistAxis.isAligned( limbGroups[groupIndex].group.front()->averageTwistAxis, 90)){
                    ofLog() << "they are aligned";
                    //Ok. we found the correct limbgroup
                    for(int limbIndex = 0; limbIndex < limbGroups[groupIndex].group.size(); ++limbIndex){
                        ofLog() << "is the angle " << limbs[i].twistAngleDistance << " less than " << limbGroups[groupIndex].group[limbIndex]->twistAngleDistance;
                        if(limbs[i].twistAngleDistance < limbGroups[groupIndex].group[limbIndex]->twistAngleDistance){
                            ofLog() << "inserting node " << limbs[i].nodeId << " at position "<< limbIndex << " in limbGroup " << groupIndex;
                            ofLog() << "limbGroups has size " << limbGroups.size() << " and groupindex is " << groupIndex;
                            //BE AWARE. THE limbit ITERATOR BECOMES INVALIDATED HERE
                            limbGroups[groupIndex].group.insert(limbGroups[groupIndex].group.begin() + limbIndex, &limbs[i]);

                            break;
                        }else
                        if(limbIndex == limbGroups[groupIndex].group.size()-1){
                            //BE AWARE. THE ITERATOR BECOMES INVALIDATED HERE
                            limbGroups[groupIndex].group.push_back(&limbs[i]);
                            ofLog() << "inserting node " << limbs[i].nodeId << " at end in limbGroup " << groupIndex;
                            ofLog() << "limbGroups has size " << limbGroups.size() << " and groupindex is " << groupIndex;
                            break;
                        }
                    }
                    break;
                }else if(groupIndex == limbGroups.size()-1){
                    limbGroups.push_back(limbGroup());
                    limbGroups.back().group.push_back(&limbs[i]);
                    ofLog() << "Creating new limbGroup";
                    break;
                }

            }
        }

    }

    //Set parent relationships and assign bodyparts.
    for(auto groupIt = limbGroups.begin(); groupIt != limbGroups.end(); ++groupIt){
        Limb* parent = chest;
        for(int i = 0; i < groupIt->group.size(); ++i){
            if(parent){
                groupIt->group[i]->setParent(parent);
                ofLog() << "parent for node " << groupIt->group[i]->nodeId << " is " << parent->nodeId;
            }
            //Bodypart starts with overarm and goes outwards
            int partIndex = i*2+3;
            groupIt->group[i]->assignedBodypart = static_cast<bodyPart>(partIndex);
            groupIt->group[i]->length = partLengths[partIndex];
            ofLog() << "node " << groupIt->group[i]->nodeId << " is bodypart " << groupIt->group[i]->assignedBodypart;
            parent = groupIt->group[i]; //Make this the next limbs parent
        }
    }
}

void Body::startSideChooser(){
    for(int i = 0; i < limbGroups.size(); ++i){
        limbGroups[i].sideChosen = false;
    }
    sideChooserActive = true;
}

void Body::sideChooser(){
//    ofLog() << "SideChooser entered";
    int nrOfFinishedLimbGroups = 0;
    for(int i = 0; i < limbGroups.size(); ++i){
        if(limbGroups[i].sideChosen){
            nrOfFinishedLimbGroups++;
            continue;
        }
        if(limbGroups[i].group[0]){
            ofVec3f startVector = (limbGroups[i].group[0]->averageTwistAxis*limbGroups[i].group[0]->yawOffset).normalize();
            ofVec3f currentVector = (limbGroups[i].group[0]->globalDirectionVector).normalize();
            if(startVector.angle(currentVector) > 20){
                float z = currentVector.z - startVector.z;
                if(z < 0){
                    ofLog() << "directionVector lowering after twist";
                    if(limbGroups[i].group.size() == 3){
                        rHand = limbGroups[i].group.back();
                    }
                }else{
                    ofLog() << "directionVector raising after twist";
                    if(limbGroups[i].group.size() == 3){
                        lHand = limbGroups[i].group.back();
                    }
                    flipLimbGroup(limbGroups[i].group);
                }
                limbGroups[i].sideChosen = true;
                nrOfFinishedLimbGroups++;
            }
        }
    }

    if(nrOfFinishedLimbGroups == limbGroups.size()){
        sideChooserActive = false;
        if(lHand && rHand){
            haveTwoHands = true;
        }
        //This is the last place to do stuff before calibration is finished

        //Calculate average yawOffset and set all to this.
        //Apparently this doesn't work so well! Fuck!
//        float averageYaw = 0, angle;
//        ofVec3f axis;
//        for(int i = 0; i < limbs.size(); ++i){
//            limbs[i].yawOffset.getRotate(angle, axis);
//            averageYaw += angle;
//        }
//        averageYaw /= limbs.size();
//        for(int i = 0; i < limbs.size(); ++i){
//            limbs[i].yawOffset = ofQuaternion(averageYaw, ofVec3f(0,0,1));
//        }
    }
}

void Body::flipLimbGroup(vector<Limb*> &limbGroup){
    for(int i = 0; i < limbGroup.size(); ++i){
        limbGroup[i]->isLefty = true;
        limbGroup[i]->limbVector = -limbGroup[i]->limbVector;
        limbGroup[i]->yawOffset = ofQuaternion(180, ofVec3f(0,0,1)) * limbGroup[i]->yawOffset;
        int partIndex = limbGroup[i]->assignedBodypart-1;
        limbGroup[i]->assignedBodypart = static_cast<bodyPart>(partIndex);
        limbGroup[i]->length = partLengths[partIndex];
        if(limbGroup[i]->assignedBodypart == LOVERARM){
            ofLog() << "LOVERARM";
        }
    }
}

void Body::setChestNode(Limb* node){
    chest = node;
    chest->assignChestRole();
}

void Body::setup(){
    ofSetLogLevel(OF_LOG_NOTICE);
    midiOut.listPorts();
	midiOut.openPort(1);
}

void Body::exit(){
//    mySerialThread.waitForThread();
}

void Body::startSerial(){
//    mySerialThread.setup();
//    mySerialThread.startThread();

    //TODO: Go through available devices and do handshakes until we identify the basestation
    serial.enumerateDevices();
    int baud = 115200;
//        vector<ofSerialDeviceInfo> deviceList = serial.getDeviceList();
//        for(auto device:deviceList){
//            device
//        }
    serial.setup(0, baud); //open the basestation!
    serial.writeByte('s'); //s for stop printing radio info to serial port
    serial.flush();
}

void Body::stopSerial(){
//    mySerialThread.waitForThread();
    serial.close();
}

void Body::updateGestures(){
    //for midi learn
    Limb* h = NULL;
    if(lHand){
        h = lHand;
    }else if(rHand){
        h = rHand;
    }
    if(h && h->globalEndPoint.z > zMaxThreshold){
        if(h->pitch < pitchMaxThreshold){
            midiOut.sendControlChange(midiChannel, 1, 1);
            ofLog() << "sending modwheel";
        }else{
            midiOut.sendControlChange(midiChannel, 2, 1);
            ofLog() << "sending breath cc";
        }
    }
    if(haveTwoHands && !waitingForGesture){
        if(!lHandIsTriggable && !rHandIsTriggable && (lHand->sendNoteOn || rHand->sendNoteOn)){
            if(lHand->lastTriggerWasDoubleTap && rHand->lastTriggerWasDoubleTap && lastTriggerWasSync){
                outputNotes = !outputNotes;
                ofLog() << "outputNotes: " + ofToString(outputNotes);
            }else if(lHand->sendNoteOn && lHand->lastTriggerWasDoubleTap){
                currentProgramChange = max(min(127, (--currentProgramChange)), 0); //Weird clamp method. Fuuun! :-)
                midiOut.sendProgramChange(midiChannel, currentProgramChange);
                ofLog() << "programchange: " + ofToString(currentProgramChange);
            }else if(rHand->sendNoteOn && rHand->lastTriggerWasDoubleTap){
                currentProgramChange = max(min(127, (++currentProgramChange)), 0); //Weird clamp method. Fuuun! :-)
                midiOut.sendProgramChange(midiChannel, currentProgramChange);
                ofLog() << "programchange: " + ofToString(currentProgramChange);
            }
        }
        lHand->sendNoteOn = false;
        rHand->sendNoteOn = false;
    }

}

void Body::updateMidi(){
    if(!outputNotes || inTriggerFreeZone){
        if(noteActive){
            stopNote();
        }
        return;
    }
    switch (activePlayMode){
        case TWISTLEGATO:
            updateTwistLegato();
            break;
        case AIRDRUMMING:
            updateAirDrumming();
            break;
    }

}

void Body::updateTwistLegato(){

//    bool triggerNote = false;
    Limb* triggeringHand;
//    bool lHandTriggable = isTriggable(lHand);
//    bool rHandTriggable = isTriggable(rHand);

    //guards for escaping if no trigger
    if(!lHandIsTriggable && !rHandIsTriggable){
        if(noteActive){
            stopNote();
        }
        return;
    }else if(lHandIsTriggable && !rHandIsTriggable){
        if(noteActive && rHand == lastTriggerHand){
            stopNote();
        }
        triggeringHand = lHand;
    }else if(!lHandIsTriggable && rHandIsTriggable){
        if(noteActive && lHand == lastTriggerHand){
            stopNote();
        }
        triggeringHand = rHand;
    }//Guards end here. Now normal checks
    else{//We have two hands!
        if(lHand->pitch < rHand->pitch){//Is left pitching more down?
            triggeringHand = lHand;
        }else{
            triggeringHand = rHand;
        }
        if(triggeringHand->pitch > pitchMinThreshold){// Should we compare roll instead?
            if(lHand->roll < rHand->roll){
                triggeringHand = lHand;
            }else{
                triggeringHand = rHand;
            }
        }
    }

    if((triggeringHand->roll < rollThreshold || triggeringHand->pitch < pitchMinThreshold) && triggeringHand->pitch < (pitchMaxThreshold)){
        float mod = ofMap(triggeringHand->pitch, -5, -90, 0, 127, true);
        float vol = ofMap(triggeringHand->roll, rollThreshold, 0, 0, 127, true);
        midiOut.sendControlChange(midiChannel, 1, mod);
        midiOut.sendControlChange(midiChannel, 2, vol);

//        int noteValue = calculateAngularNote(triggeringHand);
        int noteValue = triggeringHand->nearestTriggerPoint;
        if(noteValue != currentPitch){
            lastTriggerHand = triggeringHand;
            startNote(noteValue);
        }
    }else if(noteActive){
        stopNote();
    }
}

void Body::updateAirDrumming(){
    if(noteActive && lastTriggerHand){
        float mod = ofMap(lastTriggerHand->pitch, -5, -90, 0, 127, true);
        midiOut.sendControlChange(midiChannel, 1, mod);
//        float distance = lastTriggerHand->globalStartPoint.distanceSquared(lastTriggerHand->globalPositionLastTrigger);
        float zDistance = lastTriggerHand->globalStartPoint.z - lastTriggerHand->globalPositionLastTrigger.z;
        ofVec2f posNow = ofVec2f(lastTriggerHand->globalStartPoint.x, lastTriggerHand->globalStartPoint.y);
        ofVec2f posTrigg = ofVec2f(lastTriggerHand->globalPositionLastTrigger.x, lastTriggerHand->globalPositionLastTrigger.y);
        float horizontalDistance = posNow.distance(posTrigg);
        if((zDistance > 150 || horizontalDistance > 150) && lastTriggerHand->roll > 45){
            stopNote();
        }else{
            int noteValue = calculateAngularNote(lastTriggerHand);

            if(noteValue != currentPitch){
                startNote(noteValue);
            }
        }
    }

    if(lHand && lHand->sendNoteOn){
        lastTriggerHand = lHand;
        lHand->sendNoteOn = false;
    }else if(rHand && rHand->sendNoteOn){
        lastTriggerHand = rHand;
        rHand->sendNoteOn = false;
    }else{
        return;
    }

    //Ok. There was a trigger. Continue and do shit
    startNote(calculateAngularNote(lastTriggerHand));
}

bool Body::isTriggable(Limb* hand){
//    if(!hand){//NULL pointer
//        return false;
//    }
    if(hand->globalEndPoint.z < zMinThreshold || hand->globalEndPoint.z > zMaxThreshold || hand->pitch > pitchMaxThreshold){
        return false;
    }
    return true;
}

void Body::updateTriggerFreeZone(){
    for(const limbGroup& group: limbGroups){
        ofVec3f xyPos = group.group.front()->globalStartPoint;
        xyPos.z = 0;

        ofVec3f v = group.group.back()->globalStartPoint;
        v.z = 0;
        if(xyPos.distance(v) > 250){
            inTriggerFreeZone = false;
            return;
        }

        for(const Limb* lmb: group.group){
//            ofVec3f v = lmb->globalStartPoint;
//            v.z = 0;
//            if(xyPos.distance(v) > 250){
//                inTriggerFreeZone = false;
//                return;
//            }
            if(lmb->assignedBodypart == LHAND || lmb->assignedBodypart == RHAND){
                continue;
            }
            if(lmb->pitch > -68){
                inTriggerFreeZone = false;
                return;
            }
        }
    }
    ofLog() << "trigger free zone";
    inTriggerFreeZone = true;
    return;
}

void Body::startNote(int pitch){
    //Let's not retrigger same note if it's already "pressed"
    if(noteActive && currentPitch == pitch){
        return;
    }
    midiOut.sendNoteOn(midiChannel, pitch, 120);
    ofLog() << "noteOn";
    if(noteActive && currentPitch != pitch){
        midiOut.sendNoteOff(midiChannel, currentPitch, 0);
        ofLog() << "noteOff";
    }
    noteActive = true;
    currentPitch = pitch;
}

void Body::stopNote(){
    midiOut.sendNoteOff(midiChannel, currentPitch, 0);
    ofLog() << "noteOff";
    noteActive = false;
    currentPitch = -1;
}

int Body::calculateAngularNote(Limb* hand){
    ofVec3f y = ofVec3f(0,1,0);
    ofVec3f v = hand->globalEndPoint;
    v.z = 0;
    float angle = y.angle(v);
    if(v.x < 0){
        angle = -angle;
    }
    angle += 180.0;
//    ofLog() << "angle: " << ofToString(angle);
    //map angle to note
    const int notesPerLap = 12;
    const int startAngle = (360/notesPerLap)/2;
    int offset = ofMap(angle, -startAngle, 360-startAngle, 0, notesPerLap, false);// Seems like the mapping function acts weird if given negative boundaries on output. So let's offset the values afterwards.
    offset -= notesPerLap/2;
    offset += (notesPerLap) * hand->rotationCount;

    //two rows
    offset *= 2;
//    if(hand->horizontalDistanceFromCenter > 600){
//        offset++;
//    }
    if(hand->globalStartPoint.z > 200){
        offset ++;
    }
    if(hand->globalStartPoint.z > -200){
        offset++;
    }
//    ofLog() << "offset: " << offset;
    return getNoteFromOffset(offset);
}

int Body::getNoteFromOffset(int offset){
    int note = offset;
    int octOffset;
    if(offset < 0){
        octOffset = (offset+1) / scaleSize[scale];
        octOffset--;
        note =  scales[scale][scaleSize[scale] + note%scaleSize[scale]];
    }else{
        octOffset = offset / scaleSize[scale];
        note = scales[scale][note%(scaleSize[scale])];
    }
    int semiToneOffset = note + 12*octOffset;
//    ofLog() << "semitoneOfset: " << semiToneOffset;
    return rootNote + semiToneOffset;
}


