#include "serialThread.h"

void serialThread::setup()
    {
        ofSetLogLevel(OF_LOG_NOTICE);
        //TODO: Go through available devices and do handshakes until we identify the basestation
        threadedSerial.enumerateDevices();
        int baud = 115200;
//        vector<ofSerialDeviceInfo> deviceList = threadedSerial.getDeviceList();
//        for(auto device:deviceList){
//            device
//        }
        threadedSerial.setup(0, baud); //open the basestation!
        threadedSerial.writeByte('s'); //s for stop printing radio info to serial port
        threadedSerial.flush();
    }

void serialThread::threadedFunction(){

    while(isThreadRunning()){
        if(!threadedSerial.isInitialized()){
            setup();
            continue;
        }


        threadedSerial.writeByte('r'); //Request a dataset!!!
        for(const commandGroup& cmd : nodeCommands){
                if(cmd.sendMe){
//                        uint8_t node = 3;
//                        uint8_t comms[4] = {0, 120, 255, 0};
//                        threadedSerial.writeByte(node);
//                        threadedSerial.writeBytes((uint8_t*) comms, 4);
                        threadedSerial.writeByte(cmd.node);
                        threadedSerial.writeBytes((uint8_t*) cmd.commands, nrOfSerialCommands);
                }
        }
        threadedSerial.writeByte('>');


        //Check for message header
        while(threadedSerial.available() < 4){//We don't want to update if there is no new data in serial buffer
            //block
        }
        bool headerReceived = true;
        for(int i = 0; i < 4; ++i){
                if(threadedSerial.readByte() != '#'){
                    headerReceived = false;
                    threadedSerial.flush();
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

    threadedSerial.close();
}

void serialThread::readFromBasestaion(){
    while(!threadedSerial.available() && threadedSerial.isInitialized()){
            //block until we have the first byte
    }
    int nrOfDataSets = threadedSerial.readByte();
    int nrOfBytes = 30 * nrOfDataSets + 1;
    while(threadedSerial.available() < nrOfBytes && threadedSerial.isInitialized()){
            //block until we have a complete dataset
    }
//            ofLog() << "Header received!";

    for(int i = 0; i < nrOfDataSets; ++i){

        int nodeNr = threadedSerial.readByte();
        int incStamp = threadedSerial.readByte();//A byte with incStamp
//            ofLog() << "nrOfBytes is: " << nrOfBytes;

//                int nrOfElements = nrOfBytes / 2;//The data is stored as fixed deciaml point int16_t. I.E. two bytes per element
//                        ofLog() << "nrOfElements is: " << nrOfElements;

        unsigned char tempData[32]; // This will store the received data before it's handled
        threadedSerial.readBytes(tempData, 26); //Read it all!
        unsigned char relayInfo = threadedSerial.readByte();
        uint8_t endingCharacter = threadedSerial.readByte();//At this point there should be an ending character. If not, something's wrong!

        if(endingCharacter != ']'){
            ofLog() <<  "The individual ending character was not correct! flushing";
            threadedSerial.flush();
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
        skippedPackets++;
        unlock();
        //put the incoming data in their corresponding variables
//                copyFloatsFromTo(data, q[nodeNr], 0, 4);
//                copyFloatsFromTo(data, mag, 4, 3);
    }

    if(threadedSerial.readByte() != '>'){
        ofLog() <<  "The global ending character was not correct! flushing";
        threadedSerial.flush();
    }
}

void serialThread::readFromNode(){
    int nodeNr = 0;
    while(!threadedSerial.available() && threadedSerial.isInitialized()){
            //block until we have the first byte
    }
    int nrOfBytes = threadedSerial.readByte();
    while(threadedSerial.available() < nrOfBytes && threadedSerial.isInitialized()){
            //block until we have a complete dataset
    }
//            ofLog() << "Header received!";



//            ofLog() << "nrOfBytes is: " << nrOfBytes;

//                int nrOfElements = nrOfBytes / 2;//The data is stored as fixed deciaml point int16_t. I.E. two bytes per element
//                        ofLog() << "nrOfElements is: " << nrOfElements;

    unsigned char tempData[nrOfBytes]; // This will store the received data before it's handled
    threadedSerial.readBytes(tempData, nrOfBytes); //Read it all!
    uint8_t endingCharacter = threadedSerial.readByte();//At this point there should be an ending character. If not, something's wrong!

    if(endingCharacter != '>'){
        ofLog() <<  "The individual ending character was not correct! flushing";
        threadedSerial.flush();
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
