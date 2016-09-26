#ifndef LIMB_H
#define LIMB_H


#include "ofMain.h"
//#include <vector>
//#include <deque>
#include "globals.h"

//Class for describing a wireless sensor node mounted on the body.
class Limb
{
    public:
        bodyPart assignedBodypart;
        bool isLefty = false;

        Limb();
        void init();
        void updateData(dataPack &packet);
        void update();
        void updateCommands();
        void turnOffLed();
        void updateTriggerPointProximity();
        void draw();

        void  clearParent();
        bool  setParent(Limb* newParent);
        Limb* getParent();
        void  assignChestRole();

        void startTwistRecording();
        void stopTwistRecording();
        int recordTwist(); // Returns a number corresponding to the state of the twist motion. 0 = inactive, 1 = init 2 = early, 3 = waiting, 4 = finished

//        void updateTriggerState(); // 0 = transition to triggered state, 1 = no change, 2 = transition from triggered state
//        triggerState getTriggerState();

//        static set<Limb*> instances;

        int nodeId;
        commandGroup currentCommands;

        //Radio stats
        bool isResponding = false;
        uint16_t pollsReceived = 0;
        uint16_t relaysReceived = 0;
        uint16_t pollsFailed = 0;
        uint16_t relaysFailed = 0;
        uint16_t retrieveDuration = 0;

        //Body info
        float length = 400;//Just a default value.  Should be set from globals.h when assigning bodyparts

        //Orientation
		ofVec3f         limbVector = ofVec3f(0,1,0); //vector of limbdirection in the coord system of the device.
		ofVec3f         globalDirectionVector; //vector of limbdirection in the world coord system.
		ofQuaternion    yawOffset;
		ofQuaternion    absoluteRotation;
		ofQuaternion    calibratedRotation; // with yawoffset applied
		float           yaw;
		float           pitch;
		float           roll;
//		float           angleFromLayingFlat;

        //Position
		ofVec3f         globalStartPoint = ofVec3f(0,0,0);;
		ofVec3f         globalEndPoint = ofVec3f(0,0,0);
		float           horizontalPositionAngle;
		int             nearestTriggerPoint;
		int             previousNearestTriggerPoint;
		float           angleFromNearestTriggerPoint;

		float           horizontalDistanceFromCenter;

		int             rotationCount = 0;


//		vector<ofVec3f> northFromAccMagVectorSamples = vector<ofVec3f>(100);
//		int northFromAccMagVectorIndex = 0;
//		ofVec3f averageNorthVector;

        //Accel stuff
        ofVec3f gravityFloat = ofVec3f(0,0,1000);
		deque<ofVec3f>  accelSamplesFloat;
		deque<vec3i>    accelSamplesInt;
		vec3i           accelSum;
		ofVec3f         accelAverage;
		ofVec3f         globalPositionLastTrigger;
		uint64_t        accelTriggerStamp = 0;
		int             accelTriggerCounter;
		int             accelTriggerVelocity;
		bool            accelTriggeredFlag = false;
		bool            lastTriggerWasDoubleTap = false;
		bool            sendNoteOn = false;


        //Magnetometer stuff
		deque<ofVec3f>  magnetomSamples;
		ofVec3f         averageMagnetom;

		unsigned char   relayRole;

		//drawing stuff
		ofBoxPrimitive box = ofBoxPrimitive(50,50,50,2,2,2);
		ofPolyline line = ofPolyline();

		//twistRecording stuff
		vector<ofVec3f> twistAxis;
		ofVec3f averageTwistAxis;
		float twistAngleDistance;
		bool twistRecorded = false;

    protected:
    private:
        //color stuff
        ofColor currentColor = ofColor(ofColor::green);
        ofColor staticColor = ofColor(ofColor::green);
        ofColor triggerColor = ofColor(ofColor::greenYellow);
        float triggerLerp = 0;
        Limb* parent = NULL;

		int             inQuadrant = 0;

        static const int accelNrOfSamples = 40;
        static const int magnetomNrOfSamples = 40;

        triggerState        currentTriggerState = TRIGGERINACTIVE;
        bool                accelTriggerActive = false;
        unsigned long long  triggerStamp;

        bool                updated = false;
        unsigned long long  updateStamp;
        unsigned long long  millisSinceLastUpdate;

        //twist stuff
        enum twistState {INACTIVE, INIT, EARLY, WAITING, FINISHED}twistState = FINISHED;
        ofQuaternion twistOrigin;
		ofQuaternion currentRotation;
		ofQuaternion previousRotation;
		ofVec3f rollVector;
//		bool waitForTwistReturn = false;
//		bool setTwistOrigin = false;
//		bool twistRecordingFinished = false;
//		static int twistRecordingsCurrentlyRunning;
};

#endif // LIMB_H
