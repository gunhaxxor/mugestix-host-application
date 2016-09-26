#ifndef BODY_H
#define BODY_H


#include "ofMain.h"
//#include <vector>
#include "globals.h"
#include "limbgroup.h"
#include "limb.h"
//#include "serialThread.h"
#include "ofxMidi.h"


class Body: public ofThread
{
    public:
        /** Default constructor */
        Body();
        /** Default destructor */
        virtual ~Body();

        vector<limbGroup> limbGroups;
        vector<Limb> limbs;
        bool haveTwoHands = false;
        bool lHandIsTriggable = false;
        bool rHandIsTriggable = false;
        Limb* rHand;
        Limb* lHand;
        bool lastTriggerWasSync = false;
        int nrOfActiveLimbs;


//        int skippedPackets = 0;

        void startSerial();
        void stopSerial();
        void updateSerial();
        void threadedFunction();
        void update();
        void recordTwist();
        void startTwistRecording();
        void stopTwistRecording();

        void calculateNodeOnBodyLocations();
        void startSideChooser();
        void sideChooser();
        void setChestNode(Limb* node);
        void flipLimbGroup(vector<Limb*> &limbGroup);
        void setup();
        void exit();

        void updateCommands();
        void updateGestures();
        void updateMidi();
        void updateTwistLegato();
        void updateAirDrumming();
        bool isTriggable(Limb* hand);
        void updateTriggerFreeZone();
        void startNote(int pitch);
		void stopNote();
		int getNoteFromOffset(int offset);
		int calculateAngularNote(Limb* hand);
    protected:
    private:
//        ofVec3f notePointRight;
//        ofVec3f notePointLeft;
//        bool waitingForGesture = false;

        //MidiStuff
        bool outputNotes = true;
        bool inTriggerFreeZone = false;
        playMode activePlayMode = TWISTLEGATO;
        bool noteActive = false;
        Limb* lastTriggerHand;
        int currentPitch;
		ofxMidiOut midiOut;
		int midiChannel = 1;
		int currentProgramChange = 0;

		//hand stuff
		int currentNoteRHand = 0;
		int currentNoteLHand = 0;
		int previousNoteRHand = 0;
		int previousNoteLHand = 0;
		const float rollThreshold = 45;
        const float pitchMinThreshold = -65;
        const float pitchMaxThreshold = 45;
        const float zMaxThreshold = 660;
        const float zMinThreshold = -720;
        bool waitingForGesture = false;

        dataPack packet[nrOfNodes];
        commandGroup nodeCommands[nrOfNodes];
        ofSerial serial;
        void readFromBasestaion();
        void readFromNode();
        bool twistRecordingActive = false;
        bool sideChooserActive = false;
        Limb* chest = NULL;
};

#endif // BODY_H
