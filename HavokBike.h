#ifndef HAVOK_BIKE_H
#define HAVOK_BIKE_H


#include "camera.h"
//#include "LowFilter.h"
#include "filter.h"

#include <hkdefaultvehicle/hkdefaultvehicle.h>
#include "HavokVehicleObject.h"
#include <hkdynamics/defs.h>
#include <hkdynamics/havok.h>
#include <hktoolkit/toolkit.h>
#include "bikeSteering.h"
#include "BikeAttributes.h"
#include "RiderAttributes.h"
#include "stats.h"
#include "BikeDefaultWheel.h"
#include "TrackBonus.h"
#include <hkdynamics/action/angularconstraint1d.h>
#include <hkdynamics/collide/intersectionsolver.h>
#include "BikeFlags.h"
//#include "TerrainType.h"

#include "PowerBands.h"
#include "SpeedBands.h"

#include "HavokConversions.h"

extern "C"
{
	#include "display.h"
}


#define TRACK_TO_HUD_NONE				0
#define TRACK_TO_HUD_WRONG_WAY			1
#define TRACK_TO_HUD_OFF_TRACK			2
#define TRACK_TO_HUD_RIGHT_WAY			4
#define TRACK_TO_HUD_ON_TRACK			8
#define	TRACK_TO_HUD_MISSED_C_POINT		10
#define TRACK_TO_HUD_MADE_C_POINT		12
#define TRACK_TO_HUD_MISSED_CHECKPOINT	16


#define TO_MILES_PER_HOUR(cmPerSec)			((cmPerSec) * 3.6f * 0.6f * 0.01f)
#define TO_METERS_PER_SEC(milesPerHour)		((milesPerHour)  / (3.6f * 0.6f))
#define SECS_TO_FRAMES(seconds)				((u32)((seconds) * 60.0f))




#define CANT_DIDNT_BOOST	5
#define CANT_DID_BOOST 		4
#define EARLY_BOOST			3
#define DID_BOOST			2
#define CAN_BOOST			1
#define CANT_BOOST			0


typedef enum 
{
	PRELOAD_NO_ATTEMPT_PENDING = 0,
	PRELOAD_PRE_WINDOW,
	PRELOAD_DURING_WINDOW,
	PRELOAD_POST_WINDOW
} PreLoadState;





typedef enum
{
	FW = 0,
	BW
} BikeWheel;



typedef enum 
{
	PRELOAD_AVAILABLE = 0,
	PRELOAD_SUCCESSFUL,
	PRELOAD_FAILED
} PreLoadReportState;





typedef enum
{
	BIKESTATE_NO_TURN_NO_ACCEL = 0,
	BIKESTATE_NO_TURN_ACCEL,
	BIKESTATE_NO_TURN_BRAKE,
	BIKESTATE_LEFT_TURN,
	BIKESTATE_RIGHT_TURN,
	BIKESTATE_LEFT_POWERSLIDE_ACCEL,
	BIKESTATE_RIGHT_POWERSLIDE_ACCEL,
	BIKESTATE_LEFT_POWERSLIDE_BRAKE,
	BIKESTATE_RIGHT_POWERSLIDE_BRAKE,
	BIKESTATE_POWERSLIDE_INAIR,
	BIKESTATE_INAIR,
	BIKESTATE_LEFT_WHEELIE,
	BIKESTATE_RIGHT_WHEELIE,
	BIKESTATE_WHEELIE,
	BIKESTATE_LEFT_STOPPIE,
	BIKESTATE_RIGHT_STOPPIE,
	BIKESTATE_STOPPIE,
	BIKESTATE_LEFT_BRAKE,
	BIKESTATE_RIGHT_BRAKE,
	BIKESTATE_LEFT_ACCEL,
	BIKESTATE_RIGHT_ACCEL,
	BIKESTATE_NUM_STATES
} BikeState;


typedef enum 
{
	COMSS_NORMAL = 0,
	COMSS_FORWARD,
	COMSS_BACK,
	COMSS_LEFT,
	COMSS_RIGHT
} CenterOfMassShiftState;


typedef enum
{
	FROM_STOP_SLIDE_TURN = 0,
	SHARP_SLIDE_TURN,
	NORMAL_TURN
} TurnType;





typedef enum
{
	NO_NITRO = 0,
	RAMP_UP_NITRO,
	MID_NITRO,
	RAMP_DOWN_NITRO,
	NUM_NITRO_STATES
} NitroState;




typedef struct 
{
	f32 normal;
	f32 ang;
	f32 force;
} TurnFriction;


typedef struct
{
	f32 accel;
	f32 coast;
	f32 brake;
	f32 torque;
}DriveFriction;


typedef struct
{
	TurnFriction turn;
	TurnFriction fromStop;
	TurnFriction slide;
	TurnFriction ps;
	TurnFriction psb;
	TurnFriction pse;
	DriveFriction drive;
} TerrainFriction;







typedef enum 
{
	FRICTION_LOW_L = 0,
	FRICTION_LOW_H,
	FRICTION_MED_L,
	FRICTION_MED_H,
	FRICTION_HIGH_L,
	FRICTION_HIGH_H,
	NUM_FRICTION_LEVELS
} FrictionLevel;


typedef enum
{
	RIDING_STATE = 0,
	START_CRASH_STATE,
	CRASH_STATE,
	START_RECOVER_STATE,
	RECOVER_STATE,
	NUM_CRASH_STATES
} CrashState;


typedef enum
{
	GROUND_CRASH = 0,
	OBJECT_CRASH,
	OTHER_BIKE_CRASH,
	NUM_CRASH_TYPES
} CrashType;


typedef enum
{
	GROUND_COLLISION = 0,
	OBJECT_COLLISION,
	OTHER_BIKE_COLLISION,
	NUM_COLLISION_TYPES
} CollisionType;



#define NUM_SAFE_INDEXES	20
#define NUM_PITCH_ROLL_INDEXES	5
#define NUM_VELOCITY_INDEXES	6


const f32 COM_X				= 0.0f;//0.0f;
const f32 COM_Y				= 0.05f;//-0.25f;//0.10f;//0.10f;
const f32 COM_Z				= -0.5f;//-0.80f;//-0.40f;//-0.80f;


const f32 BACK_SUSP_COM_OFFSET_X = 0.0f;
const f32 BACK_SUSP_COM_OFFSET_Y = -0.79f;
const f32 BACK_SUSP_COM_OFFSET_Z = -0.46f+0.1f;


const f32 FRONT_SUSP_COM_OFFSET_X = 0.0f;
const f32 FRONT_SUSP_COM_OFFSET_Y = 1.48f - 0.59f;
const f32 FRONT_SUSP_COM_OFFSET_Z = -0.46f+0.1f;



f32 getSlopePoint(f32 min, f32 minPercent, f32 opt, f32 optPercent, f32 max, f32 maxPercent, f32 current);









class HavokBike : public HavokVehicleObject 
{
public:
	HavokBike( objectid_t descId, objectid_t camId, RigidCollection* rc, u32 CPU);
	~HavokBike();

	void updateModelWorld();
	void updateHavokFromModelObject();
	void updateModelObjectFromHavok();

	void limitMaximumSpeed();
	void getTerrainType();
	void getWheelieCondition();
	void jumpSpeed();
	void inAirAdjust();
	void landSpeed();
	void getBaseVectors();
	void zeroVelocity();
	void forceZeroVelocity();
	void unpinBike();
	void pinBike();
	void control();
	f32 getMaximumSuspensionLength();
	void zeroThrottleBoost();
	void zeroLinearVelocity();
	void dampenAngularVelocity();
	void preLoad();
	void bikeTerrainInfo();
	void getMovingInfo();
	void collide();
	virtual void collisionHandler( CModelObject* collidedWith, const Havok::CollisionEvent& ce );

	sxVector3d getCameraForward();

	void checkPoint();

	void doAction(const CPacket& pkt, sxtime_t currentTime); //only here because of inheritance
	virtual void update(sxtime_t _time);
	

	inline f32 getWheelRadius();


	inline f32 getTerrainPitch();
	inline f32 getTerrainRoll();
	inline f32 getBikePitch();
	inline f32 getBikeRoll();
	inline f32 getBikeRelativePitch();
	inline f32 getBikeRelativeRoll();

	inline objectid_t getCameraId();

	inline void enableLeanTarget();
	inline void disableLeanTarget();
	inline void setLeanTarget(f32 lean);
	inline void setLeanTargetSteps(f32 steps);
	inline f32 getLean();

	inline void	setBoostBit( u32 val );
	inline void	setStartRaceBit( u32 val );
	inline u32 getBoostBit( void );
	inline u32 getStartRaceBit( void );
	inline const f32& getWayPointTime();
	inline void setWayPointTime(f32 wayTime);
	inline const s32& getCurLap();
	inline f32 getSpeed();
	inline f32 getSpeedometerReading();
	inline sxVector3d getVelocity();
	inline CBikeAttributes* getBikeAttributes();
	inline CRiderAttributes* getRiderAttributes();
	inline bool getFlag(int flagId) const;
	inline void setFlag(int flagId);
	inline void clearFlags();
	inline CStats* getBikeStats();
	inline CTrackBonus* getBikeTrackBonus();

	inline sxVector3d getUpVector();
	inline sxVector3d getForwardVector();
	inline sxVector3d getRightVector();
	inline sxVector3d getFUpVector();
	inline sxVector3d getFForwardVector();
	inline sxVector3d getFRightVector();
	inline RigidBody& getChassis();
	inline RigidBody& getFrontWheel();
	inline RigidBody& getBackWheel();
	inline RigidBody& getCrashBike();

	inline s32 getLastMicroTileIndex();
	inline void setLastMicroTileIndex( s32 index );




	//-- Functions used to handle track to hud protocall
	inline u32 getTrackToHudWrongWay( void );
	inline u32 getTrackToHudOffTrack( void );
	inline u32 getTrackToHudMissedCheckPoint( void );
	inline void setTrackToHudWrongWay( u32 data );
	inline void setTrackToHudOffTrack( u32 data );
	inline void setTrackToHudMissedCheckPoint( u32 data );

	inline u32 getPreloadReportState();
	inline u32 getCurrentCheckPoint( void );
	inline CModelObject* getCollisionObject();

	void setCurLap(s32 lapNum);
	void setStartPos(sxVector3d sPos, sxVector3d sDir);



	inline sxVector3d getBackWheelPosition();
	inline sxVector3d getFrontWheelPosition();
	inline sxVector3d getBackWheelSuspensionPosition();
	inline sxVector3d getFrontWheelSuspensionPosition();
	inline f32 getBackWheelSuspensionCompression();
	inline f32 getFrontWheelSuspensionCompression();
	inline f32 getFrontWheelAngle();
	inline f32 getWheelSpeed();
	inline f32 getWheelRPM(u32 wheel);
	void setCenterOfMass(sxVector3d com);

	inline matrix4 getBikeMatrix();
	inline bool onGround();
	f32 getDistToGround(sxVector3d pos);
	f32 getDistToGround();
	f32 getDistToGround(u32 wheel);
	inline s32 stuntButtonPressed(xbool& reversed);
	inline bool loadButtonPressed();
	inline bool isBikeCPU();
	inline void setIsBikeCPU(bool CPU);
	
	bool		inStunt;	//-- If In stunt then disallow directional controller input
	s32	lastMicroTileIndex;
	

	inline void setParticleTextureID(objectid_t id);
	//*******************pyw
	inline void setParticleMudTextureID(objectid_t id);
	inline void setParticleTrailTextureID(objectid_t id);
	//************************

	inline sxVector3d getCameraLookAtPos();

	void emitExhaust();
	

//*****************************pyw
	void emitTrail();
	void emitMud();
//*********************************







	void emitDust(sxVector3d startPos, f32* numParticles, f32 dustScale);
	void emitParticleTrail();
	void restitution();

	bool isCrashing(){return crashing;}
	bool isRecovering(){return recovering;}

	void setPosition(const sxVector3d& pos);
	void setOrientation(const sxVector3d& orient);
	void setOrientationQ(const sxQuaternion& Q);



	inline char* getPlayerName( void );
	inline void  setPlayerName( char* name);
	
	void maybeFlashWrongWay(const sxVector3d & rightWay);
	

	inline void addNitro();
	inline u32 getNumNitros();
	inline void setNumNitros(u32 num);
	inline u32 getCrashCounter();
	void nitroBoost();

	sxVector3d getRiderPos();
	sxVector3d getRiderDisplacement();
	void riderDisplacement();


	inline u32 getCrashState();

	void groundCollision();

	inline void setStuntCrashFlag(bool flag);
	inline void setStuntFrames(u32 frames);
	inline void setStuntPointValue(u32 value);
	inline u32 getStuntPointValue( void );

	void limitInitialCrashVelocity(RigidBody* rb);
	void dampenCrashVelocity(RigidBody* rb);
	void startCrash(u32 type);
	inline void disableCrashes();
	inline void enableCrashes();
	void setAttributes();

protected:
	void checkCurrentGate();
	void checkTerrainPoint();

private:

	char playerName[20];


	DefaultWheels*					m_wheels;
	DefaultChassis*					m_chassis;
	SteeringComponent*				m_steering;
	EngineComponent*				m_engine;
	TransmissionComponent*			m_transmission;
	BrakeComponent*					m_brakes;
	SuspensionComponent*			m_suspension;
	AerodynamicsComponent*			m_aerodynamics;
	DriverInputComponent*			m_input;	// Set externally by user

	RigidBody* frontWheel;
	RigidBody* backWheel;
	RigidBody* chassis;
	RigidBody* crashBike;


	AngularConstraint1D* constraint;
	FastConstraintSolver* csolver;
	RigidCollection* rc;

    Geometry* g;






//-- start state variables

	f32 leanRot;
	bool leanTargetEnabled;
	f32	leanTarget;
	s32	leanTargetSteps;
	f32 leanPercent;
	




	sxQuaternion q;
	s32 pressed[BIKEACTION_NUM_STATES];
	sxVector3d forwardDir, upDir, rightDir;
	sxVector3d lastForwardDir, lastUpDir, lastRightDir;
	sxVector3d fForwardDir, fUpDir, fRightDir;
	sxVector3d lastFForwardDir, lastFUpDir, lastFRightDir;

	f32 throttle;
	f32 turnStep;
	s32 turnCounter;
	bool powerSlideEnd;
	s32 powerSlideBrakeTurnCounter;
	s32 powerSlideAccelTurnCounter;
	f32 turnPressedPercent;

	u32 lastFrameBikeState;
	u32 bikeState;
	u32 lastBikeState;
	s32 bikeStateCounter[BIKESTATE_NUM_STATES];
	bool bikeStateSuspended[BIKESTATE_NUM_STATES];
	s32 bikeStateCounter2;
	s32 bikeStateCounter3;
	s32 lastBikeStateCounter;
	f32 brakingScale;


	u32 preLoadPressedCounter;
	u32 preLoadWaitDelay;
	PreLoadState preLoadJumpState;
	bool preLoadJump;
	u32 preLoadJumpCounter;
	bool preLoadJumpPending;
	u32 preLoadReportState;
	s32 preLoadForceCounter;
	
	u32 comShiftState;


	f32 linearVelocityMag;
	f32 forwardVelocityMag;

	sxVector3d lastLinearVelocity;
	f32 lastLinearVelocityMag;
	f32 inAirLinearVelocityMag;
	sxVector3d eventStartVelocity;
	f32 eventStartVelocityMag;
//	sxVector3d eventStartForwardDir, eventStartUpDir, eventStartRightDir;
	bool powerSlideLeftGround;
	bool powerSlideAccelBoostEligible;
	bool powerSlideBrakeBoostEligible;
	f32 powerSlideAccelBoostVelocityMag;
	f32 powerSlideBrakeBoostVelocityMag;

	f32 eventStartAngle;
	f32 eventDeltaAngle;


	f32 throttleZero;
	f32 lastThrottleZero;
	u32 framesInAir;
	u32 crashFramesInAir;
	bool inAir;
	bool crashLand;
	u32 changeInVelCounter;
	bool jumpAssistInAir;
	sxVector3d takeOffPos;


	f32		curWayTime;
	s32		curLap;
	u32		currentCheckPoint;

	u32		canBoost;	//-- If True then Bike can try for Start Race Boost
	u32		startRace;	//-- If True then the race has started and all of the controll can go to the bikes
	bool	isCPU;		//-- If true then the bike is a CPU and should not flash messages on screen!
	bool	flashWrong; // -- If true then flash a wrong way message!
	bool	isOffTrack; // -- If true the bike has gone offTrack


	s32 dustCloudCounter;

	f32 numExhaustParticles;

	//pyw
	f32 numTrailParticles;
	f32 numMudParticles;
	///
	f32 numFrontDustParticles;
	f32 numBackDustParticles;
	f32	numDirtParticles;


	u32 frameCountForMessage;
	sxVector3d lastPos;

	bool collided;
	u32 collidedCounter;
	bool crashing;
	bool recovering;
	sxVector3d crashPos;
	sxVector3d crashDir;
	u32 crashState;
	u32 crashType;
	u32 collisionType;
	u32 crashStartCounter;
	bool willCrashFlag;
	bool lockSuspension;




	CTrackBonus trackBonus;
//	u32 terrainType;

	s32 wheelieCounter;

	WheelInfo wheelInfo[2];
	WheelInfo lastWheelInfo[2];


//	CLowFilter filterSAV;
//	CLowFilter leanFilter;
//	CLowFilter frictionFilter[2];

	u32 bikePacket;
	CFilter filterSAV;
	CFilter leanFilter;
	CFilter frictionFilter[2];
	CFilter speedometerFilter;
	CFilter camLookatPosFilterX;
	CFilter camLookatPosFilterY;
	CFilter camLookatPosFilterZ;

	CFilter frontSuspension;
	CFilter backSuspension;



	f32 throttleUp;
	bool throttleUpActive;
	u32 leftTurnType;
	u32 rightTurnType;
	s32 sharpTurnCounter;
	s32 lastLeftTurnPressed;
	s32 lastRightTurnPressed;
	s32 sharpRightTurnPressedCounter;
	s32 sharpLeftTurnPressedCounter;


	s32 landingSpeedCounter;
	f32 landingSpeedMag;
	f32 landingSpeedScale;

	f32 jumpSpeedTarget;
	s32 jumpSpeedCounter;

	s32 wheelFramesInContact[2];
	s32 wheelFramesInAir[2];

	
	u32 terrainType;
	u32 lastTerrainType;
	u32 lastValidTerrainType;

	bool jumpAssist;

	bool wheelieCondition;
	bool wheelieRequested;
	
	NitroState nitroState;
	s32 nitrosAvailable;
	s32 nitroCounter;


	sxVector3d riderRestPos;
	sxVector3d riderLastPos;
	sxVector3d riderCurrentPos;
	sxVector3d riderDelta;
	sxVector3d riderLastVelocity;
	sxVector3d riderVelocity;
	sxVector3d riderPos;


	bool movingForward;


	bool stuntCrashFlag;
	u32 crashCounter;
	u32 postCrashCounter;
	bool crashEnabled;


	sxVector3d safeDir[NUM_SAFE_INDEXES];
	sxVector3d safePos[NUM_SAFE_INDEXES];
	f32 lastSafeTime;
	u32 safeIndex;


//-- end state variables




	CPowerBands* bikePowerBand;
	CSpeedBands* bikeSpeedBand;


	sxVector3d wheelNormVec[2];


	f32 terrainPitchAvg;
	f32 terrainPitch[2];
	f32 terrainRoll;
	f32 bikePitch;
	f32 bikeRoll;
	f32 bikeRelativePitch;
	f32 bikeRelativeRoll;

	f32 lastBikePitch[NUM_PITCH_ROLL_INDEXES];
	f32 lastBikeRoll[NUM_PITCH_ROLL_INDEXES];
	f32 lastBikeRelativePitch[NUM_PITCH_ROLL_INDEXES];
	f32 lastBikeRelativeRoll[NUM_PITCH_ROLL_INDEXES];
	s32 lastBikePitchRollIndex;


	sxVector3d lastVelocity[NUM_VELOCITY_INDEXES];
	f32 lastVelocityMag[NUM_VELOCITY_INDEXES];
	s32 lastVelocityMagIndex;


	f32 speedScale;


	objectid_t cameraId;
	sxVector3d cameraLookAtPos;



	objectid_t dustTexID;

	//**************pyw
	objectid_t MudTexID;
	objectid_t TrailTexID;
	//**************

	objectid_t dirtTexID;
	u32 dustParticleBlockID;
	u32 dirtParticleBlockID;
	u32 dirt2ParticleBlockID;

// For trail purpose ************************************PYW
	u32 mudParticleBlockID;
	u32 trailParticleBlockID;
//*******************************************************




	CStats bikeStats;
	CBikeAttributes bikeAttributes;
	CRiderAttributes riderAttributes;



	TerrainFriction tFriction;

	class Attributes
	{
	public:
		f32 maximumSpeed;
		f32 nitroMaximumSpeed;
		f32 corneringPercent;
		f32 landingSpeedRecoveryJumps;
		f32 landingSpeedRecoveryWhoops;
		f32 jumpSpeedTolerance;
		s32 traction;
		f32 stability;
		f32 stabilityChangeInVel;
		f32 maxJumpSpeed;
	};
	Attributes attributes;



	u32 trackToHudWrongWay;
	u32 trackToHudOffTrack;
	u32 trackToHudMissedCheckPoint;

	u32 flags;
	CModelObject* collisionObject;
	sxVector3d collisionObjectPos;
	u32 collisionObjectCounter;
	sxVector3d collisionNormal;
	f32 collisionStartTime;
	f32 wheelBase;
	f32 bikeLength;



//	u32 stuntFramesHeld;
	u32 stuntPointValue;


	void crash();
	void recover();
	void crashUpdate();
	
	bool checkMessage(bikeActions action);

//	static IntersectionSolver		m_raycaster;	// Shared by all raycast vehicles
//	IntersectionSolver		m_raycaster;	// Shared by all raycast vehicles


//	bool preLoadCondition;

};


inline char* HavokBike::getPlayerName( void )
{
	return( playerName );
}
inline void  HavokBike::setPlayerName( char* name)
{
	x_strcpy( playerName, name );
}



inline objectid_t HavokBike::getCameraId()
{
	return cameraId;
}



inline f32 HavokBike::getWheelRadius()
{
	const WheelsComponent *wheels_comp = vehicle->getWheelsComponent();
	return (wheels_comp->getWheelRadius(1) * HAVOK_TO_SX);

//	const CBikeDefaultWheel* wheel = dynamic_cast<const CBikeDefaultWheel*> (vehicle->getWheelsComponent());
//	if( wheel != NULL )
//		return( wheel->getWheelRadius(1) * HAVOK_TO_SX);
//	return 0.0f;
}





inline void HavokBike::enableLeanTarget()
{
	leanTargetEnabled = TRUE;
}

inline void HavokBike::disableLeanTarget()
{
	leanTargetEnabled = FALSE;
}

inline void HavokBike::setLeanTarget(f32 lean)
{
	leanTarget = lean;
}

inline void HavokBike::setLeanTargetSteps(f32 steps)
{
	leanTargetSteps = steps;
}

inline f32 HavokBike::getLean()
{
	return leanRot;
}



inline f32 HavokBike::getTerrainPitch()
{
	return (terrainPitchAvg);
}

inline f32 HavokBike::getTerrainRoll()
{
	return (terrainRoll);
}

inline f32 HavokBike::getBikePitch()
{
	return (bikePitch);
}

inline f32 HavokBike::getBikeRoll()
{
	return (bikeRoll);
}

inline f32 HavokBike::getBikeRelativePitch()
{
	return (bikeRelativePitch);
}

inline f32 HavokBike::getBikeRelativeRoll()
{
	return (bikeRelativeRoll);
}







inline void	HavokBike::setBoostBit( u32 val )
{
	canBoost = val;
}

inline void	HavokBike::setStartRaceBit( u32 val )
{
	startRace = val;
}

inline u32 HavokBike::getBoostBit( void )
{
	return canBoost;
}

inline u32 HavokBike::getStartRaceBit( void )
{
	return startRace;
}

inline const f32& HavokBike::getWayPointTime()
{
	return curWayTime;
}

inline void HavokBike::setWayPointTime(f32 wayTime)
{
	curWayTime = wayTime;
}

inline const s32& HavokBike::getCurLap()
{
	return curLap;
}


inline f32 HavokBike::getSpeedometerReading()
{
	return (speedometerFilter.applyToOneSample(linearVelocityMag * HAVOK_TO_SX * speedScale));
}

inline f32 HavokBike::getSpeed()
{
	return (linearVelocityMag * HAVOK_TO_SX);
}

inline sxVector3d HavokBike::getVelocity()
{
	return (linearVelocity* HAVOK_TO_SX);
}


inline CBikeAttributes* HavokBike::getBikeAttributes()
{
	return &bikeAttributes;
}

inline CRiderAttributes* HavokBike::getRiderAttributes()
{
	return &riderAttributes;
}


inline bool HavokBike::getFlag(int flagId) const
{
	ASSERT(flagId>=0 && flagId<32);
	return (bool)(flags&(1<<flagId));
}

inline void HavokBike::setFlag(int flagId)
{
	ASSERT(flagId>=0 && flagId<32);
	flags|=(1<<flagId);
}

inline void HavokBike::clearFlags()
{
	flags=0;
}

inline CStats* HavokBike::getBikeStats()
{
	return( &bikeStats );
}

inline CTrackBonus* HavokBike::getBikeTrackBonus()
{
	return( &trackBonus );
}

inline sxVector3d HavokBike::getUpVector()
{
	return upDir;
}

inline sxVector3d HavokBike::getForwardVector()
{
	return forwardDir;
}

inline sxVector3d HavokBike::getRightVector()
{
	return rightDir;
}

inline sxVector3d HavokBike::getFUpVector()
{
	return fUpDir;
}

inline sxVector3d HavokBike::getFForwardVector()
{
	return fForwardDir;
}

inline sxVector3d HavokBike::getFRightVector()
{
	return fRightDir;
}


inline RigidBody& HavokBike::getChassis()
{
	return (*chassis);
}

inline RigidBody& HavokBike::getFrontWheel()
{
	return (*frontWheel);
}

inline RigidBody& HavokBike::getBackWheel()
{
	return (*backWheel);
}

inline RigidBody& HavokBike::getCrashBike()
{
	return (*crashBike);
}




//-------------------------------------------------------------------
inline void HavokBike:: setParticleTextureID(objectid_t id)
{
	dustTexID = id;
	dirtTexID = id;
}

//**********************pyw
inline void HavokBike:: setParticleMudTextureID(objectid_t id)
{
	MudTexID = id;
}
//*************************


//**********************pyw
inline void HavokBike:: setParticleTrailTextureID(objectid_t id)
{
	TrailTexID = id;
}
//*************************


inline sxVector3d HavokBike::getCameraLookAtPos()
{
	return cameraLookAtPos * HAVOK_TO_SX;
}


inline u32 HavokBike::getTrackToHudOffTrack( void )
{
	return( trackToHudOffTrack );
}
inline u32 HavokBike::getTrackToHudWrongWay( void )
{
	return( trackToHudWrongWay );
}
inline u32 HavokBike::getTrackToHudMissedCheckPoint( void )
{
	return( trackToHudMissedCheckPoint );
}


inline void HavokBike::setTrackToHudOffTrack( u32 data )
{
	trackToHudOffTrack = data;
}
inline void HavokBike::setTrackToHudWrongWay( u32 data )
{
	trackToHudWrongWay = data;
}
inline void HavokBike::setTrackToHudMissedCheckPoint( u32 data )
{
	trackToHudMissedCheckPoint = data;
}





inline u32 HavokBike::getPreloadReportState()
{
	return preLoadReportState;
}


inline u32 HavokBike::getCurrentCheckPoint( void )
{
	return( currentCheckPoint );
}


inline CModelObject* HavokBike::getCollisionObject()
{
	return collisionObject;
}


//inline void HavokBike::Collide()
//{
//	if(collided)
//		setFlag(BIKE_COLLIDE);
//	collided=false;
//}


inline sxVector3d HavokBike::getBackWheelPosition()
{
	sxVector3d wheelPos;
	const Transform& t = wheelInfo[BW].getTransform();
	const Vector3& v = t.getTranslation();
	wheelPos = V3_TO_SXV(v);

	wheelPos *= HAVOK_TO_SX;

	if (fabs(wheelPos.x) > 100000.0f || fabs(wheelPos.z) > 100000.0f)
		wheelPos = getPosition();
	

	return wheelPos;
}

//-------------------------------------------------------------------
inline sxVector3d HavokBike::getFrontWheelPosition()
{
	sxVector3d wheelPos;
	const Transform& t = wheelInfo[FW].getTransform();
	const Vector3& v = t.getTranslation();
	wheelPos = V3_TO_SXV(v);

	wheelPos *= HAVOK_TO_SX;

	if (fabs(wheelPos.x) > 100000.0f || fabs(wheelPos.z) > 100000.0f)
		wheelPos = getPosition();


	return wheelPos;
}


//-------------------------------------------------------------------
inline sxVector3d HavokBike::getBackWheelSuspensionPosition()
{
	sxVector3d suspDir = sxVector3d(0, -1, 0);
	const Vector3 comPosV3 = chassis->getPosition();
	sxVector3d comPos = V3_TO_SXV(comPosV3);

	sxVector3d pos = comPos + (forwardDir * (BACK_SUSP_COM_OFFSET_Y)) - (suspDir * (BACK_SUSP_COM_OFFSET_Z));
	return (pos * HAVOK_TO_SX);
}



//-------------------------------------------------------------------
inline sxVector3d HavokBike::getFrontWheelSuspensionPosition()
{
	sxVector3d suspDir = sxVector3d(0, -1, 0);
	const Vector3 comPosV3 = chassis->getPosition();
	sxVector3d comPos = V3_TO_SXV(comPosV3);

	sxVector3d pos = comPos + (forwardDir * (FRONT_SUSP_COM_OFFSET_Y)) - (suspDir * (FRONT_SUSP_COM_OFFSET_Z));
	return (pos * HAVOK_TO_SX);
}


//-------------------------------------------------------------------
inline f32 HavokBike::getMaximumSuspensionLength()
{
	return (m_suspension->getWheelLength(FW) * HAVOK_TO_SX);
}

//-------------------------------------------------------------------
inline f32 HavokBike::getFrontWheelSuspensionCompression()
{
	f32 dist;
	if (!lockSuspension)
	{
		f32 deltaY = getFrontWheelPosition().y - getFrontWheelSuspensionPosition().y;
		dist = (1.0f * HAVOK_TO_SX * m_suspension->getWheelLength(FW)) - deltaY;
		dist = MAX(-18.0f, MIN(10.0f, dist));
	}
	else
	{
		dist = 8.0f;
	}
	return (frontSuspension.applyToOneSample(-10.0f + (1.2f * dist)) );
}


//-------------------------------------------------------------------
inline f32 HavokBike::getBackWheelSuspensionCompression()
{
	f32 dist;
	if (!lockSuspension)
	{
		f32 deltaY = getBackWheelPosition().y - getBackWheelSuspensionPosition().y;
		dist = (1.0f * HAVOK_TO_SX * m_suspension->getWheelLength(BW)) - deltaY;
		dist = MAX(-18.0f, MIN(10.0f, dist));
	}
	else
	{
		dist = 8.0f;
	}
	return  (backSuspension.applyToOneSample(-5.0f + (1.5f * dist)) );
}


//-------------------------------------------------------------------
inline f32 HavokBike::getFrontWheelAngle()
{
//	const SteeringComponent& stComp = vehicle->getSteeringComponent();
//
//	f32 angle = RAD_TO_DEG(stComp.calcSteeringAngle());
//	return (angle);
	return 0.0f;
}




//-------------------------------------------------------------------
inline f32 HavokBike::getWheelSpeed()
{
//	f32 speed = wheelInfo[BW].getSpinVelocity() / (3.6f * 0.6f);
//	return (speed * HAVOK_TO_SX);

	if (wheelInfo[FW].getIsInContact() || wheelInfo[BW].getIsInContact())
	{
		return (getSpeed());
	}
	else
	{
		return (inAirLinearVelocityMag * HAVOK_TO_SX);
	}
}


inline f32 HavokBike::getWheelRPM(u32 wheel)
{
	const f32 sec_min = 60.0f;
	const f32 rev_rad = 1.0f / (2.0f * PI);

//	f32 torqueRatio = ((CBikeTransmission*)m_transmission)->getWheelTorqueRatio(wheel);

	f32 wheelRPM = ((f32)wheelInfo[wheel].getSpinVelocity()) * sec_min * rev_rad;

	if (x_fabs(wheelRPM) < TO_METERS_PER_SEC(5.0f))
		wheelRPM = 0.0f;
	if (x_fabs(wheelRPM) > TO_METERS_PER_SEC(100.0f))
		wheelRPM = TO_METERS_PER_SEC(100.0f);

	return wheelRPM;
}




//-------------------------------------------------------------------
inline matrix4 HavokBike::getBikeMatrix()
{
	matrix4 m;
	Q_SetupMatrix(&q, &m);
    M4_ClearTranslations( &m );
	return m;
}


//-------------------------------------------------------------------
inline bool HavokBike::onGround()
{

	if (wheelInfo[FW].getIsInContact() || wheelInfo[BW].getIsInContact())
		return TRUE;
	else
		return FALSE;

}

//-------------------------------------------------------------------
inline s32 HavokBike::stuntButtonPressed(xbool& reversed)
{
	if (checkMessage(BIKEACTION_STUNT))
	{
		if (pressed[BIKEACTION_STUNT] < 0)
			reversed = TRUE;
		else
			reversed = FALSE;

		return abs(pressed[BIKEACTION_STUNT]);
	}
	else
		return -1;
}


// These next few functions are temporary Motion Triggers
inline bool HavokBike::loadButtonPressed()
{
//	return FALSE;
	if (getFlag(BIKE_PRELOAD))
//	if (buttons & PRELOAD_BIKE_BUTTON)
		return TRUE;
	else
		return FALSE;
}


inline s32 HavokBike::getLastMicroTileIndex()
{
	return lastMicroTileIndex;
}

inline void HavokBike::setLastMicroTileIndex( s32 index )
{
	lastMicroTileIndex = index;
}

inline void HavokBike::setIsBikeCPU(bool CPU)
{
	isCPU = CPU;
}

inline bool HavokBike::isBikeCPU()
{
	return isCPU;
}


inline void HavokBike::setStuntCrashFlag(bool flag)
{
	stuntCrashFlag = flag;
}

//inline void HavokBike::setStuntFrames(u32 frames)
//{
//	stuntFramesHeld = frames;
//}

inline void HavokBike::setStuntPointValue(u32 value)
{
	stuntPointValue = value;
}

inline u32 HavokBike::getStuntPointValue( void )
{
	return stuntPointValue;
}

inline void HavokBike::disableCrashes()
{
	crashEnabled = FALSE;
}
inline void HavokBike::enableCrashes()
{
	crashEnabled = TRUE;
	postCrashCounter = 0;
}



inline void HavokBike::addNitro()
{
	nitrosAvailable++;
}

inline u32 HavokBike::getNumNitros()
{
	return nitrosAvailable;
}

inline void HavokBike::setNumNitros(u32 num)
{
	nitrosAvailable = num;
}

inline u32 HavokBike::getCrashCounter()
{
	return crashCounter;
}

inline u32 HavokBike::getCrashState()
{
	return crashState;
}



#endif 