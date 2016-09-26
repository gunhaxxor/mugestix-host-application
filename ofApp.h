#pragma once

#include "ofMain.h"
#include "globals.h"

#include "body.h"



class ofApp : public ofBaseApp{
	public:

		void setup();
		void update();
		void draw();
		void exit();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);


//		ofVec3f previous, current;
		ofEasyCam easyCam;

		ofColor cyan;
        ofColor magenta;
        ofColor yellow;
        ofColor green;
        ofColor blue;


		ofTrueTypeFont		font;
		ofLight pointLight;
		ofLight directionalLight;
		ofLight directionalLight2;
		ofColor backgroundColor;
		float flashAmount = 1.0;

		//******************
		//debug stuff
		//accelGraph
		bool runAccelGraph = true;
		deque<triggerState> beatMarkers[2];
		deque<ofVec3f> accelArrows[2];

		int skippedPacketsFromSerial = 0;

        unsigned long long loopStamp = 0;
		unsigned long long loopDuration = 0;
		int bytesInSerialBuffer = 0;

		//debug mag
		deque<ofVec3f> magStamps;
		//*****************

		Body body;

		bool		bSendQRequest;			// a flag for sending serial
//		bool		bSendERequest;
		bool        bCaptureOneDataset;
		bool        bStreamData = false;
//		bool        bSetStartQuat = false;
		bool        bRecordTwist = false;
		bool        bNoStartQuat = false;
		bool        bAddAMagStamp = false;
		char		bytesRead[3];				// data from serial, we will be trying to read 3
		char		bytesReadString[4];			// a string needs a null terminator, so we need 3 + 1 bytes
		int			nBytesRead;					// how much did we read?
		int			nTimesRead;					// how many times did we read?
		float		readTime;					// when did we last read?
		ofQuaternion startQuat[6];
		ofQuaternion originalQuat;
		ofVec3f     calVector[6];
		ofVec3f     limbVector[6];
		float       yawOffset[6];
		float       q[nrOfNodes][4];
		float       e[nrOfNodes][3];
		float       accel[nrOfNodes][3];
		float       gyro[nrOfNodes][3];
		float       mag[nrOfNodes][3];
		unsigned char relayRole[nrOfNodes];
		float       filterFrequency;

		//twistRecording stuff
//		int twistAxisIndex = 0;
		vector<ofVec3f> twistAxis[nrOfNodes];
		ofVec3f averageTwistAxis[nrOfNodes];
		ofQuaternion twistOrigin[nrOfNodes];
		ofQuaternion currentRotation[nrOfNodes];
		ofQuaternion previousRotation[nrOfNodes];
//		ofQuaternion twistQuat;
		bool waitForTwistReturn[nrOfNodes] = {false};
		bool setTwistOrigin[nrOfNodes] = {false};
		bool twistRecordingFinished[nrOfNodes] = {false};
//		int twistRecordingsCurrentlyRunning = 0;


//        uint8_t readChar();
//		void copyFloatsFromTo(float* source, float* target, int index, int amount);
//		inline int16_t floatToQ14( float value);
//		inline float Q14ToFloat( int16_t value);


//		ofSerial	serial;
//		unsigned char latestBytes[4];
//		bool headerReceived = false;
//		int byteCounter = 0;
};
