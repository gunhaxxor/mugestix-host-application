#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED

const int nrOfNodes = 7;
const bool nodeDataOverSerial = false;
const int nrOfSerialCommands = 4;

enum bodyPart{NONE, CHEST, LOVERARM, ROVERARM, LUNDERARM, RUNDERARM, LHAND, RHAND};
const int partLengths[8] = {0, 200, 400, 400, 350, 350, 50, 50};
enum triggerState{TRIGGERINACTIVE, TRANSITIONTOTRIGGERACTIVE, TRIGGERACTIVE, TRANSITIONTOTRIGGERINACTIVE};
enum playMode{AIRDRUMMING, TWISTLEGATO, THERMINE};
const uint64_t doubleTapThreshold = 500;
const uint64_t syncTapThreshold = 200;
const uint64_t gestureWaitTime = 500;

const int notesPerLap = 12;
const int maxAngleFromTriggerPoint = 360/notesPerLap/2;

const int rootNote = 60; //C3
const int scale = 0;
const int scales[][12] = {
                            {0,1,2,3,4,5,6,7,8,9,10,11}//Chromatic
                            ,{0, 2, 4, 5, 7, 9, 11}//Major
                            ,{0, 2, 3, 5, 7, 8, 10}//Minor
                            ,{0, 3, 5, 7, 10}//Penta
                            ,{0, 3, 5, 6, 7, 10}//Blues
                        };
const int scaleSize[] = {12,7,7,5,6};

struct commandGroup{
    bool sendMe = false;
    int node;
    //sensorcommand
    //hue
    //brightness
    //motor
    uint8_t commands[nrOfSerialCommands];
};

struct dataPack{
    bool contactEstablished = false;
    int nodeNr;
    unsigned long long updateStamp;
    float q[4]; //this will store one nodes data when it's converted to float
    int accel[3];
    int accelTriggerId;
    int accelTriggerVelocity;
    float magnetom[3];
    unsigned char relayRole;
    uint16_t radioStats[5];
};

union charToFloat{
    float f;
    unsigned char c[4];
};

union binaryInt16 {
    int16_t i;
    uint8_t c[2];
};

static  int16_t floatToQ14( float value)
{
	return value * (0x01 << 14);
};

static float Q14ToFloat( int16_t value)
{
	return ((float) value) / (0x01 << 14);
};

static void copyFloatsFromTo(float* source, float* target, int index, int amount){
    int k = 0;
    for(int i = index; i < amount + index; i++){
        target[k++] = source[i];
    }
}

class vec3i{
    public:
        int x;
        int y;
        int z;

        vec3i();
        vec3i( int _x, int _y, int _z=0 );

        vec3i  operator+( const vec3i& vec ) const;
        vec3i& operator+=( const vec3i& vec );
        vec3i  operator-( const vec3i& vec ) const;
        vec3i& operator-=( const vec3i& vec );

//        ofVec3f  operator*( const float f ) const;
//        ofVec3f& operator*=( const float f );

        int length();
        float lengthSquared();
};

inline vec3i::vec3i(): x(0), y(0), z(0) {};
inline vec3i::vec3i( int _x, int _y, int _z ):x(_x), y(_y), z(_z) {}

inline vec3i vec3i::operator+( const vec3i& vec ) const {
	return vec3i( x+vec.x, y+vec.y, z+vec.z );
}

inline vec3i& vec3i::operator+=( const vec3i& vec ) {
	x+=vec.x;
	y+=vec.y;
	z+=vec.z;
	return *this;
}

inline vec3i vec3i::operator-( const vec3i& vec ) const {
	return vec3i( x-vec.x, y-vec.y, z-vec.z );
}

inline vec3i& vec3i::operator-=( const vec3i& vec ) {
	x -= vec.x;
	y -= vec.y;
	z -= vec.z;
	return *this;
}

inline int vec3i::length(){
    return (float) sqrt((float) (x*x+y*y+z*z));
}

inline float vec3i::lengthSquared(){
    return int (x*x+y*y+z*z);
}

//--------------------------------------------------------------

/**
 * @brief   Returns Cardan = roll/pitch/yaw angles for the given
 *          rotation.
 *
 * @param   q_ an ofQuaternion (unit quaternion) denoting a rotation.
 *
 * @details Cardan Angles == Tail-Bryan-A. == Nautical-A. == Euler
 *          Angle Sequence 1,2,3.
 *
 *          The angle sequence is relative to the object's current
 *          reference frame and to be applied in sequence. That is:
 *          roatation first around the object's x-axis, then its new
 *          y'-axis, then its new z''-axis. This is equivalent to the
 *          angle notation you'd expect for a plane or for a ship,
 *          thus the name 'Nautical Angles'.
 *
 *          Math implemented based on: Diebel, James: Representing
 *          Attitude: Euler Angles, Unit Quaternions, and Rotation
 *          Vectors, Stanford University, Stanford, California, 2006,
 *          p. 12.
 *          http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.110.5134
 *
 * @warning This method returns undefined results when approaching
 *          pitch PI/2 or -PI/2
 *
 * @author  tig
 */

static ofVec3f getCardanAngles(const ofQuaternion& q_){

	// tig: ok, so in math textbooks a quaternion is defined as:
	//
	// q = a + ib + jc + kd
	//
	// whereas openFrameworks stores quaternions as in:
	//
	// q = ix + jy + kz + w
	//
	// therefore, when textbooks uses quaternion indices, these will *NOT*
	// correspond to ofQuaternion's indices, but need to be shifted
	// by 1 to the left, which is what we do in the next step.

	float
	q0(q_[3]),	// w -> a
	q1(q_[0]),	// x -> b
	q2(q_[1]),  // y -> c
	q3(q_[2]);  // z -> d

	float sq0 = q0*q0;
	float sq1 = q1*q1;
	float sq2 = q2*q2;
	float sq3 = q3*q3;

	// we can now use the same terms as in the textbook.
	float roll	=	atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1, sq3 - sq2 - sq1 + sq0);
	float pitch	=	-asin(2.0f * q1 * q3 - 2.0f * q0 * q2);
	float yaw	=	atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3, sq1 + sq0 - sq3 - sq2);

	return (ofVec3f(roll,pitch,yaw) * RAD_TO_DEG);
}

#endif // GLOBALS_H_INCLUDED
