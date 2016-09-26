#ifndef SERIALTHREAD_H
#define SERIALTHREAD_H

#include "ofMain.h"
#include "globals.h"


class serialThread: public ofThread{

    public:
        dataPack packet[nrOfNodes];
        commandGroup nodeCommands[nrOfNodes];
        int skippedPackets = 0;

        void setup();
        void threadedFunction();

    private:
        ofSerial threadedSerial;

        void readFromBasestaion();
        void readFromNode();
};

#endif // SERIALTHREAD_H
