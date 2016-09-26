#include "ofApp.h"
#include <vector>
//#include "serialThread.h"
using namespace std;

/*
	rotateToNormal will rotate everything using ofRotate. the rotation amount
	and axis are generated using an ofQuaternion. the trick is to use ofQuaternion
	to determine the rotation that is required from a standard axis (0,0,1) to the
	desired normal vector, then apply that rotation.
*/
//--------------------------------------------------------------

//--------------------------------------------------------------
void ofApp::setup(){
    body.setup();

    cyan = ofColor::fromHex(0x00abec);
	magenta = ofColor::fromHex(0xec008c);
	yellow = ofColor::fromHex(0xffee00);
	green = ofColor::fromHex(0x00ff00);
	blue = ofColor::fromHex(0x0000ff);

    for(int i = 0; i < nrOfNodes; ++i){
        startQuat[i].zeroRotation();
    }
	bSendQRequest = false;
	ofBackground(255);
//	ofSetLogLevel(OF_LOG_VERBOSE);
	ofSetWindowShape(1200, 1000);

	font.loadFont("DIN.otf", 15);


	easyCam.setDistance(1500);



	//serial.listDevices();
	//vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();

	ofSetSmoothLighting(true);
	ofSetGlobalAmbientColor(ofFloatColor(0.7,0.7,0.7));
    pointLight.setDiffuseColor( ofFloatColor(.85, .85, .55) );
    pointLight.setSpecularColor( ofFloatColor(1.f, 1.f, 1.f));

    directionalLight.setDirectional();
//    directionalLight.setDiffuseColor( ofFloatColor(.55, .70, .80) );
    directionalLight.setSpecularColor( ofFloatColor(.8f, .8f, .8f));
    directionalLight.setOrientation(ofVec3f(-0.5,1,0.8));

    directionalLight2.setDirectional();
    directionalLight2.setSpecularColor(ofFloatColor(0.0f, .4f, 1.0f));
    directionalLight2.setOrientation(ofVec3f(1,-1,-0.8));
    ofEnableDepthTest();

    backgroundColor = ofColor(magenta);

	ofSetVerticalSync(false);
}

//--------------------------------------------------------------
void ofApp::update(){
    loopDuration = ofGetElapsedTimeMillis() - loopStamp;
    loopStamp = ofGetElapsedTimeMillis();
//    directionalLight.setOrientation(startQuat[0]);

    flashAmount+=0.05;
    flashAmount = ofClamp(flashAmount, 0.0, 1.0);
    backgroundColor = ofColor::azure.getLerped(magenta, flashAmount);

    if(bRecordTwist){
        bRecordTwist = false;
        body.lock();
        body.startTwistRecording();
        body.unlock();
    }
    if(bNoStartQuat){
        ofLog() << "resetting startquat";
        for(int i =0; i < nrOfNodes; ++i){
            startQuat[i].zeroRotation();
            yawOffset[i] = 0;
        }
        bNoStartQuat = false;
    }

    if(bCaptureOneDataset || bStreamData){
        bCaptureOneDataset = false;

        //Update the limbs with serial data from the sensor
//        int limbIndex = 0;
//        mySerialThread.lock();
//        for(int i = 0; i < nrOfNodes; ++i){
//            if(mySerialThread.packet[i].contactEstablished){
//                    if(limbIndex >= body.limbs.size()){
//                        body.limbs.push_back(Limb());
//                        ofLog() << "new limb object with index " << limbIndex << " created";
//                    }
//                    body.limbs[limbIndex++].updateData(mySerialThread.packet[i]);
//            }
//        }
//        skippedPacketsFromSerial = mySerialThread.skippedPackets - 1;
//        mySerialThread.skippedPackets = 0;
//        mySerialThread.unlock();

        //Take care of the rest limb stuff
//        body.update();


        //Play stuff
//        for(limbGroup group : body.limbGroups){
//            Limb* limb = group.group.back();
//            ofPoint currentPos = limb->globalStartPoint;
//
//            if(noteActive && lastTriggerNode == limb->nodeId){
//                float distance = currentPos.distanceSquared(limb->globalPositionLastTrigger);
//                if(distance > 20000 && limb->angleFromLayingFlat > 35){
//                    stopNote();
//                }
//            }
////            int noteValue = calculateAngularNote(currentPos);
//            int noteValue = 60+limb->nodeId;
//            if(noteActive && lastTriggerNode == limb->nodeId && noteValue != currentPitch && limb->angleFromLayingFlat < 35){
//                startNote(noteValue);
//            }
//            triggerState trgState = limb->getTriggerState();
//            if(trgState == TRANSITIONTOTRIGGERACTIVE){
//                ofLog() << "noteValue: " << noteValue;
//                startNote(noteValue);
//                lastTriggerNode = limb->nodeId;
//            }
//        }

//        if(runAccelGraph){
//            int graphLength = 300;
//            if(body.limbGroups.size() == 2){
//                for(int i = 0; i < 2; ++i){
//                    Limb* hand = body.limbGroups[i].group.back();
//                    accelArrows[i].push_front((hand->accelSamplesFloat[0]*hand->calibratedRotation)/10);
//                    if(accelArrows[i].size() > graphLength){
//                        accelArrows[i].pop_back();
//                    }
//
//
//                    beatMarkers[i].push_front(hand->getTriggerState());
//                    if(beatMarkers[i].size() > graphLength){
//                        beatMarkers[i].pop_back();
//                    }
//                }
//            }
//        }
    }//if Streamdata

    //access the body and retrieve stuffz
    body.lock();
    for(int i = 0; i < body.limbs.size(); ++i){
        if(bAddAMagStamp){
            magStamps.push_back(body.limbs[i].magnetomSamples.front());
        }
    }
    bAddAMagStamp = false;
    body.unlock();
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackgroundGradient(backgroundColor * .6, backgroundColor * .4);

    ofSetColor(ofColor::yellow);

    string msg;
	msg += "Press s to request stream of Quaternions\n";
//	msg += "Node responding " + ofToString(body.limbs[1].isResponding) + "\n";
//	msg += "poll fails for node 0: " + ofToString(body.limbs[0].pollsFailed) + "\n";
//	msg += "bodyPart for node 0 " + ofToString(body.limbs[0].bodyPart) + "\n";
////	msg += "q0 " + ofToString(q[5][0]) + "\n";
////	msg += "q1 " + ofToString(q[5][1]) + "\n";
////	msg += "q2 " + ofToString(q[5][2]) + "\n";
////	msg += "q3 " + ofToString(q[5][3]) + "\n";
//	msg += "quat0 " + ofToString(body.limbs[0].rotation.x()) + "\n";
//	msg += "quat1 " + ofToString(body.limbs[0].rotation.y()) + "\n";
//	msg += "quat2 " + ofToString(body.limbs[0].rotation.z()) + "\n";
//	msg += "quat3 " + ofToString(body.limbs[0].rotation.w()) + "\n";
//	msg += "yawOffset  " + ofToString(yawOffset[0]) + "\n";
//	msg += "accel float x " + ofToString(body.limbs[0].accelSamplesFloat.front().x) + "\n";
//	msg += "accel float y " + ofToString(body.limbs[0].accelSamplesFloat.front().y) + "\n";
//	msg += "accel float z " + ofToString(body.limbs[0].accelSamplesFloat.front().z) + "\n";
////	msg += "mag vector size " + ofToString(sqrt(mag[0][0]*mag[0][0] + mag[0][1]*mag[0][1] + mag[0][2]*mag[0][2])) + "\n";
//	msg += "orientation filter frequency: " + ofToString(filterFrequency) + " Hz \n";
	msg += "loop duration is: " + ofToString(loopDuration) + "ms\n";
	msg += "fps is: " + ofToString(ofGetFrameRate()) + "Hz \n";
//	msg += "target fps is: " + ofToString(ofGetTargetFrameRate()) + "Hz \n";
    msg += "Skipped packets from serial: " + ofToString(skippedPacketsFromSerial) + "\n";
	msg += "bytes in serial buffer: " + ofToString(bytesInSerialBuffer);

	font.drawString(msg, 50, 100);






//	return;
	//ofNoFill();


ofEnableLighting();
    pointLight.setPosition(cos(ofGetElapsedTimef())*400, cos(ofGetElapsedTimef()*4)*200, (sin(ofGetElapsedTimef()))*400);
//    pointLight.setAttenuation(1.0);
//    ofDrawSphere(cos(ofGetElapsedTimef())*400, cos(ofGetElapsedTimef()*4)*200, (sin(ofGetElapsedTimef()))*400, 25);
    pointLight.enable();
    directionalLight.enable();
    directionalLight2.enable();


    easyCam.begin();

	ofRotateX(15); // Make us look slightly from above
	ofDrawGrid(500, 5, false, false, true, false);


    ofRotateX(-90); // Use sensor coord system with z axis up





//	ofSetColor(yellow);


//    ofRotateY(180);



    //Draw axis so we can see the coordinate system
//    ofTranslate(-10,-10,-10);
	ofDrawAxis(1000);
	ofDrawArrow(ofVec3f::zero(), ofVec3f(0,0,-1000), 10);
	ofDrawArrow(ofVec3f::zero(), ofVec3f(0,500,0), 25);
//	ofTranslate(10,10,10);

    float rotationAmount;
	ofVec3f rotationAngle;
	body.lock();
	for(int i = 0; i < body.limbs.size(); ++i){
        body.limbs[i].draw();
        for(auto magStamp:magStamps){
            ofDrawArrow(body.limbs[i].globalStartPoint, body.limbs[i].globalStartPoint + magStamp, 24);
        }

	}
	body.unlock();
//        ofLog() << "drawing accel arrows for debugging";


    for(int i = 0; i<2; ++i){
        float offsetX = 5;
        ofVec3f startPoint = ofVec3f(-500, 0, 350*i-150);
        ofPolyline line;
        for(int pIndex = 0; pIndex < accelArrows[i].size(); ++pIndex){
            ofPoint p = accelArrows[i][pIndex];
            triggerState state = beatMarkers[i][pIndex];
            if(state == TRANSITIONTOTRIGGERACTIVE){
                ofSetColor(ofColor::white);
                ofDrawCone(startPoint,20, 200);
            }else if(state  == TRANSITIONTOTRIGGERINACTIVE){
                ofSetColor(ofColor::black);
                ofDrawCone(startPoint,15, 150);
            }else if(state  == TRIGGERACTIVE){
                ofSetColor(ofColor::aqua);
            }else{
                ofSetColor(ofColor::orange);
            }
            line.addVertex(startPoint+p);
            ofDrawArrow(startPoint, startPoint+p, 5);

            startPoint.x+=offsetX;
        }
        ofSetColor(ofColor::white);
        line.draw();
    }

//    ofRotateZ(body.limbs[0].yawOffset);
//    for(int i = 0; i < body.limbs[0].twistAxis.size(); i++){
//        ofDrawArrow(ofVec3f::zero(), 300*body.limbs[0].twistAxis[i].normalize(), 10);
//    }
//
//	body.limbs[0].rotation.getRotate(rotationAmount, rotationAngle);
//	ofRotate(rotationAmount, rotationAngle.x, rotationAngle.y, rotationAngle.z);
//    ofSetColor(green);
//	ofDrawArrow(ofVec3f::zero(), body.limbs[0].limbVector*500, 24);
//    ofSetColor(90,0,220);
//	ofDrawAxis(150);
//	if(body.limbs[0].relayRole == 'r'){
//        ofSetColor(ofColor::yellow);
//	}else if(body.limbs[0].relayRole == 'e'){
//        ofSetColor(ofColor::red);
//    }
//	ofDrawBox(100);
//
////    ofDrawArrow(ofVec3f::zero(), gravity, 50);
//
//    for(int i = 0; i < body.limbs[0].accelSamplesInt.size(); ++i){
//        ofVec3f accVector = ofVec3f(body.limbs[0].accelSamplesInt[i].x, body.limbs[0].accelSamplesInt[i].y, body.limbs[0].accelSamplesInt[i].z);
////        accVector+=gravity;
//        accVector /= 5;
//        int color = (int) ofMap(i, 0, body.limbs[0].accelSamplesInt.size(), 0, 255);
//        ofSetColor(color, 255-color, 40);
//        ofDrawArrow(ofVec3f::zero(), accVector, 30);
//    }
//
//	ofTranslate(body.limbs[0].limbVector*500);
//	ofRotate(-rotationAmount, rotationAngle.x, rotationAngle.y, rotationAngle.z);
//	ofRotateZ(-body.limbs[0].yawOffset);


	ofPopMatrix();


	pointLight.disable();
	ofDisableLighting();

	easyCam.end();

//	ofSetColor(255);

}

void ofApp::exit(){
    body.exit();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key){
        case 'q':
            bSendQRequest = true;
            break;
    //    if(key == 'e'){
    //        bSendERequest = true;
    //    }
        case '1':
            bCaptureOneDataset = true;
            break;
        case 's':
    //        serial.flush();
            bStreamData = !bStreamData;
            ofLog() << "toggling streaming to " + ofToString(bStreamData);
            if(bStreamData){
                body.startSerial();
                body.startThread();
            }else{
    //            mySerialThread.stopThread();
                body.stopSerial();
                body.waitForThread();
            }
            break;
    //    if(key == 'z'){
    //        bSetStartQuat = true;
    //    }
        case '0':
            bNoStartQuat = true;
            break;
        case 't':
            bRecordTwist = !bRecordTwist;
    //        twistRecordingsCurrentlyRunning = nrOfNodes;
    //        for(int i = 0; i < nrOfNodes; ++i){
    //            setTwistOrigin[i] = true;
    //
    //        }
            break;
        case 'a':
            runAccelGraph = !runAccelGraph;
            break;
        case 'm':
            bAddAMagStamp = true;
            break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
//    if(x < 300 && y < 300){
//        bSetStartQuat = true;
//    }else
    if(x < 300 && y < 600){
        bRecordTwist = !bRecordTwist;
//        twistRecordingsCurrentlyRunning = nrOfNodes;
//        for(int i = 0; i < nrOfNodes; ++i){
//            setTwistOrigin[i] = true;
//
//        }
    }

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}

//
//uint8_t ofApp::readChar(){
////    latestBytes[3] = latestBytes[2];
////    latestBytes[2] = latestBytes[1];
////    latestBytes[1] = latestBytes[0];
////    latestBytes[0] = serial.readByte();
////    byteCounter++;
//////    ofLog() << "reading byte " << ofToString(byteCounter) << " as ascii: " << latestBytes[0] << ", as number: " << ofToString((int)latestBytes[0]);
//    return latestBytes[0];
//
//}

