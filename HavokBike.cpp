// HavocBike.cpp: implementation of the CBike class.
//
//////////////////////////////////////////////////////////////////////

#include "stdhdr.h"
#include "HavokBike.h"
#include "HavokCollide.h"
#include "globaltime.h"
#include "ResourceStorage.h"
#include "ObjectDescription.h"
#include "BikeSteering.h"
#include "bikeDriverInput.h"
#include "model.h"
#include "debugdraw.h"
#include "camera.h"
#include "CollisionGroups.h"
#include "BikeEngine.h"
#include "BikeAerodynamics.h"
#include "BikeTransmission.h"
#include "BikeBrake.h"
#include "BikeSuspension.h"
//#include "PowerBands.h"
//#include "SpeedBands.h"
#include "SXModel.h"
#include "WorldModel.h"
#include "BikeFlags.h"
#include "particleSystem.h"
#include "TerrainType.h"
#include "random.h"
#include "Util.h"
#include <vector>
#include "viewdefs.h"
#include "Tweaker.h"
#include "SplineManager.h"
#include "SubtileModel.h"
#include "gameend.h"
#include "GameOptions.h"
#include "sxgame.h"

// Havok
#include <hkdynamics/havok.h>


// vehicle includes
#include <hkdefaultvehicle/hkdefaultvehicle.h>
#include <hkdynamics/collide/intersectionsolver.h>

#include "intersection.h"
//extern "C"
//{
//	extern matrix4 *asEngineTempMatrix;				// engine temp matrix
//}


using namespace Havok;

const f32 TURN_STEP_DETERIORATE = 0.75f;



const f32 MAX_VELOCITY = 50.0f; // meters per sec

const f32 MAX_BRAKING = 1.0f;
const f32 NORMAL_BRAKING = 1.0f;//0.125f;





const f32 BIKE_MASS = 1500.0f;


const u32 IN_AIR_FRAME = 5;
const u32 PRELOAD_INAIR_FRAME = 1;







const f32 MIN_ZERO_LINEAR_VEL = TO_METERS_PER_SEC(2.0f);

const f32 CRASH_THRESHOLD = TO_METERS_PER_SEC(10.0f);

const f32 AC_TAU = .075f;
const f32 AC_STR = 1.f;



const f32 PRELOAD_INAIR_FRAME_OFFSET = 0.0f;



const s32 COLLIDED_COUNTER_TRIGGER_FRAME = 5;
const s32 COLLIDED_COUNTER_NO_COLLISION_FRAME = 20;
const s32 COLLIDED_COUNTER_MAX_TURN_FRAME = 40;

const f32 GRAVITY = 15.1074f;


#define TEST_CRASH 0//set to 1 causes R2 to crash the bike--hit R2 again to recover

//IntersectionSolver HavokBike::m_raycaster;	// Shared by all raycast vehicles

//------------------------------------------------------------------------
HavokBike::HavokBike( objectid_t descId, objectid_t camId, RigidCollection* _rc, u32 CPU ) : HavokVehicleObject(descId )
{


	clearFlags();
	CResourceStorage<CObjectDescription>* descriptionResource=CResourceStorage<CObjectDescription>::getInstance();
	CObjectDescription* objectDescription = descriptionResource->getResource( getDescriptionId() );
	ASSERT( objectDescription != NULL );
	CResourceStorage<CHavokCollide>* havokCollideResource=CResourceStorage<CHavokCollide>::getInstance();
	objectid_t collideChassisId = objectDescription->getCollisionId(0);
	objectid_t collideWheelId = objectDescription->getCollisionId(1);
	objectid_t collideCrashId = objectDescription->getCollisionId(2);
	rc=_rc;

	isCPU = (CPU) ? true : false;
//	isCPU = 0;
	isOffTrack = false;


	//-- Track To Hud data
	trackToHudOffTrack = 0;
	trackToHudWrongWay = 0;
	trackToHudMissedCheckPoint = 0;

	startRace = 0; //-- GameStart Code
	canBoost = 0;
	curWayTime = 0.0f;
	curLap = 0.0f;
	frameCountForMessage = 0;	
	
	collided = false;
	collidedCounter = COLLIDED_COUNTER_MAX_TURN_FRAME;
	collisionObjectCounter = 0;
	collisionObject = (CModelObject*)NULL;
	crashing = false;
	recovering = false;
	crashState = RIDING_STATE;
	postCrashCounter = 0;
	currentCheckPoint = 0;

	cameraId = camId;
	chassis = new RigidBody(); // cannot track havok *
    SET_FILE_LINE(chassis);
	chassis->setUserInfo(this);

	frontWheel = new RigidBody(); // cannot track havok *
    SET_FILE_LINE(frontWheel);
	frontWheel->setUserInfo(this);

	backWheel = new RigidBody(); // cannot track havok *
    SET_FILE_LINE(backWheel);
	backWheel->setUserInfo(this);


	crashBike = new RigidBody(); // cannot track havok *
    SET_FILE_LINE(crashBike);
	crashBike->setUserInfo(this);



	if( collideCrashId != NULL_ID )
	{
		CHavokCollide* collideCrash = havokCollideResource->getResource( collideCrashId );
		ASSERT( collideCrash != NULL );	
		collideCrash->addCollisionToRigidBody( crashBike );
	}

	if( collideWheelId != NULL_ID )
	{
		CHavokCollide* collideWheel = havokCollideResource->getResource( collideWheelId );
		ASSERT( collideWheel != NULL );		
		collideWheel->addCollisionToRigidBody( frontWheel );
		collideWheel->addCollisionToRigidBody( backWheel );
	}

	if( collideChassisId != NULL_ID )
	{
		CHavokCollide* collideChassis = havokCollideResource->getResource( collideChassisId );
		ASSERT( collideChassis != NULL );	
		collideChassis->addCollisionToRigidBody( chassis );
	}


	
//	chassis->setLocalToDisplayTranslation(Vector3(0, 0.70f, 0.55f));
	chassis->setLocalToDisplayTranslation(Vector3(0, 0.70f, 0.45f));



	// Setup wheels
	frontWheel->setPinned(true);
//	frontWheel->setActive(false);
	backWheel->setPinned(true);
//	backWheel->setActive(false);
		

	crashBike->setPinned(true);
	crashBike->setActive(false);

	crashBike->setMass(1500.0f);


	frontWheel->setMass(10.0f);
	backWheel->setMass(10.0f);

	// define collision groups
	for (int p=0; p< chassis->getNumPrimitives(); p++)
	{
		if( CPU )
			chassis->getPrimitive(p)->setCollisionMask(COLLISION_MASK_CHASSIS_AI);	
		else 
			chassis->getPrimitive(p)->setCollisionMask(COLLISION_MASK_CHASSIS_PLAYER);	
	}
	
	// Wheels only have 1 prim
	frontWheel->getPrimitive(0)->setCollisionMask(COLLISION_MASK_WHEELS);
	backWheel->getPrimitive(0)->setCollisionMask(COLLISION_MASK_WHEELS);


	for (s32 p=0; p< crashBike->getNumPrimitives(); p++)
		crashBike->getPrimitive(p)->setCollisionMask(COLLISION_MASK_CHASSIS_PLAYER);




	/****************************** CONSTRUCTION OF THE CAR ******************************/

	hkDefaultWheelsBP wheelBP(2);

	wheelBP.m_wheelsAxle[0] = 0;
	wheelBP.m_wheelsAxle[1] = 1;
	wheelBP.m_wheelsRadius[0] = 0.36f;
	wheelBP.m_wheelsRadius[1] = 0.36f;
	wheelBP.m_wheelsWidth[0] = 0.2f;
	wheelBP.m_wheelsWidth[1] = 0.2f;
	wheelBP.m_wheelsFriction[0] = 8.1f;
	wheelBP.m_wheelsFriction[1] = 8.1f;
	wheelBP.m_wheelsViscosityFriction[0] = 0.0f;
	wheelBP.m_wheelsViscosityFriction[1] = 0.0f;
	wheelBP.m_wheelsRigidBody[0] = frontWheel;
	wheelBP.m_wheelsRigidBody[1] = backWheel;


	m_wheels = new DefaultWheels(wheelBP);
	SET_FILE_LINE(m_wheels );

	// Chassis
	hkDefaultChassisBP chassisBP;
	chassisBP.m_mass		 		= BIKE_MASS;
	chassisBP.m_coordinateSystem	= CCS_Z_UP_X_RIGHT;

	m_chassis = new DefaultChassis(chassisBP);
	SET_FILE_LINE(m_chassis );



	//Steering
	hkDefaultSteeringBP steeringBP(2);

    steeringBP.m_maxSteeringAngle			= 30.0f * (2.0f * PI) / 360.0f;
    steeringBP.m_maxSpeedFullSteeringAngle	= 20000.0f;

	steeringBP.m_wheelsDoesSteer[0] = true;
	steeringBP.m_wheelsDoesSteer[1] = false;

	m_steering = new CBikeSteering (steeringBP);
	SET_FILE_LINE(m_steering );	


	// Engine
	hkDefaultEngineBP engineBP;

	engineBP.m_minRPM = 1000.0f;
	engineBP.m_optRPM = 6500.0f;//3500.0f;//5500.0f;
	engineBP.m_maxRPM = 8000.0f;

	engineBP.m_torque = 1500.0f;

	engineBP.m_torqueFactorAtMinRPM = 1.0f;//0.8f;
	engineBP.m_torqueFactorAtMaxRPM = 1.0f;//0.8f;

	engineBP.m_resistanceFactorAtMinRPM = 0.20f;//0.01f;//0.05;//0.35f;//0.05f;
	engineBP.m_resistanceFactorAtOptRPM = 0.45f;//0.275f;//0.10f;//0.35f;//0.1f;
	engineBP.m_resistanceFactorAtMaxRPM = 0.65f;//0.3f;

	bikePowerBand = new CPowerBands;
	SET_FILE_LINE(bikePowerBand);

	bikePowerBand->setTorque(0, 0);
	bikePowerBand->setTorque(1000, 2600);
	bikePowerBand->setTorque(9000, 2600);
	bikePowerBand->setTorque(10000, 0);

	bikeSpeedBand = new CSpeedBands;
	SET_FILE_LINE(bikeSpeedBand);
	bikeSpeedBand->setBoost(0, 3.0f);
	bikeSpeedBand->setBoost(10, 1.8f);
	bikeSpeedBand->setBoost(20, 1.6f);
	bikeSpeedBand->setBoost(30, 1.4f);
	bikeSpeedBand->setBoost(40, 1.2f);
	bikeSpeedBand->setBoost(50, 1.0f);
	bikeSpeedBand->setBoost(60, 0.8f);
	bikeSpeedBand->setBoost(70, 0.5);
	bikeSpeedBand->setBoost(80, 0.2f);
	bikeSpeedBand->setBoost(90, 0.0f);
	bikeSpeedBand->setBoost(100, 0.0f);
	bikeSpeedBand->setBoost(110, 0.0f);
	bikeSpeedBand->setBoost(120, 0.0f);	
	bikeSpeedBand->setBoost(130, 0.0f);




	m_engine = new CBikeEngine (engineBP, bikePowerBand, bikeSpeedBand);
	SET_FILE_LINE(m_engine );


	// Transmission
	const unsigned int numGears = 4;

	hkDefaultTransmissionBP transmissionBP(2, numGears);

	transmissionBP.m_downshiftRPM				= 3500.0f;
	transmissionBP.m_upshiftRPM					= 6550.0f;
	transmissionBP.m_primaryTransmissionRatio	= 2.4f;//1.8f;//4.8f;//4.8f;//3.8f;//3.8f;
	transmissionBP.m_clutchDelayTime			= 0.0f;//0.1f;
	transmissionBP.m_reverseGearRatio			= 3.5f;

	transmissionBP.m_gearsRatio[0] = 2.0f;
	transmissionBP.m_gearsRatio[1] = 1.5f;
	transmissionBP.m_gearsRatio[2] = 1.0f;
	transmissionBP.m_gearsRatio[3] = 0.75f;

	transmissionBP.m_wheelsTorqueRatio[0] = 0.0f;
	transmissionBP.m_wheelsTorqueRatio[1] = 0.5f;//0.25f;

	m_transmission = new CBikeTransmission (transmissionBP);
	SET_FILE_LINE(m_transmission );


	// Brakes
	hkDefaultBrakeBP brakesBP(2);

	brakesBP.m_wheelsMaxBrakingTorque [0] = 8000.0f;
	brakesBP.m_wheelsMaxBrakingTorque [1] = 8000.0f;
	
	brakesBP.m_wheelsMinTimeToBlock [0] = 10.5f;//0.7f;
	brakesBP.m_wheelsMinTimeToBlock [1] = 10.5f;//0.7f;

	brakesBP.m_wheelsMinPedalInputToBlock [0] = 0.9f;
	brakesBP.m_wheelsMinPedalInputToBlock [1] = 0.9f;

	brakesBP.m_wheelsIsConnectedToHandbrake [0] = true;
	brakesBP.m_wheelsIsConnectedToHandbrake [1] = false;
	
//	m_brakes = new DefaultBrake (brakesBP);
	m_brakes = new CBikeBrake (brakesBP);
	SET_FILE_LINE(m_brakes );

    // Suspension
	hkDefaultSuspensionBP suspensionBP(2);

	const Vector3 massCenterShift = Vector3(0.0f, COM_Y, COM_Z);
	chassis->shiftCentreOfMass(massCenterShift);

	suspensionBP.m_wheelsHardpointCS [0] = Vector3( FRONT_SUSP_COM_OFFSET_X - COM_X, FRONT_SUSP_COM_OFFSET_Y - COM_Y, FRONT_SUSP_COM_OFFSET_Z - COM_Z) ;
	suspensionBP.m_wheelsHardpointCS [1] = Vector3( BACK_SUSP_COM_OFFSET_X - COM_X, BACK_SUSP_COM_OFFSET_Y - COM_Y, BACK_SUSP_COM_OFFSET_Z - COM_Z);

//	suspensionBP.m_wheelsHardpointCS [0] = Vector3( FRONT_SUSP_COM_OFFSET_X, FRONT_SUSP_COM_OFFSET_Y, FRONT_SUSP_COM_OFFSET_Z) - massCenterShift;
//	suspensionBP.m_wheelsHardpointCS [1] = Vector3( BACK_SUSP_COM_OFFSET_X, BACK_SUSP_COM_OFFSET_Y, BACK_SUSP_COM_OFFSET_Z) - massCenterShift;


	suspensionBP.m_wheelsDirectionCS [0] = Vector3 (0.0f, 0.0f, -1.0f);
	suspensionBP.m_wheelsDirectionCS [1] = Vector3 (0.0f, 0.0f, -1.0f);

	suspensionBP.m_wheelsLength [0] = 0.40f;
	suspensionBP.m_wheelsLength [1] = 0.40f;

	suspensionBP.m_wheelsStrength [0] = 40.0f;
	suspensionBP.m_wheelsStrength [1] = 40.0f;

	suspensionBP.m_wheelsDampingCompression [0] = 30.0f;
	suspensionBP.m_wheelsDampingCompression [1] = 30.0f;

	suspensionBP.m_wheelsDampingRelaxation [0] = 35.0f;
	suspensionBP.m_wheelsDampingRelaxation [1] = 35.0f;

//	m_suspension = new DefaultSuspension (suspensionBP);
	m_suspension = new CBikeSuspension(suspensionBP);
	SET_FILE_LINE(m_suspension );



	// Aerodynamics
	hkDefaultAerodynamicsBP aerodynamicsBP;

	aerodynamicsBP.m_airDensity      =  0.15f;
	aerodynamicsBP.m_frontalArea     =  4.0f;
	aerodynamicsBP.m_dragCoefficient =  0.55;
	aerodynamicsBP.m_liftCoefficient = 0.0f;
	aerodynamicsBP.m_extraGravity    =  Vector3(0.0f, 0.0f, 0.0f);

	m_aerodynamics = new CBikeAerodynamics (aerodynamicsBP);
	SET_FILE_LINE(m_aerodynamics );




	// Input
	hkDriverInputComponentBP inputBP;
	m_input = new CBikeDriverInput (inputBP);
	SET_FILE_LINE(m_input );



//	hkRaycastVehicleBP bp(1);
	hkRaycastVehicleBP bp(0);


	// Components
	bp.m_wheels			= m_wheels;
	bp.m_chassis		= m_chassis;
	bp.m_driverInput	= m_input;
	bp.m_steering		= m_steering;
	bp.m_engine			= m_engine;
	bp.m_transmission	= m_transmission;
	bp.m_brake			= m_brakes;
	bp.m_suspension		= m_suspension;
	bp.m_aerodynamics	= m_aerodynamics;

	// Raycast friction parameters & playability params
	bp.m_chassisRigidBody			= chassis;
	bp.m_intersectionSolver			= CIntersection::getIntersectionSolver();//&m_raycaster;
	bp.m_frictionEqualizer			= 0.5f;//1.0f;//1.00f;//0.5f;
	
	bp.m_torqueRollFactor			= 1.0f;//0.125;//0.5f;//0.5f;
	bp.m_torquePitchFactor			= 1.0f;//-0.25f;//-0.125;//-0.125f;//-0.125f;//0.125f;//-0.125f;//-0.25f;
	bp.m_torqueYawFactor			= 1.0f;//1.0f;//0.5f;

	bp.m_extraTorqueFactor			= 0.0f;//-35.0f;//-12.0f;//-4.75f;//0.5f;
	bp.m_extraAngularImpulseFactor	= 0.0f;//1.5f;//-0.5f;//-10.5f;//0.7f;

	hkAngularVelocityDamperBP avdbp;
	avdbp.m_normalSpinDamping			= 0.0f;//0.5f;//0.5f;
	avdbp.m_collisionSpinDamping		= 0.0f;//0.5f;//2.0f;
 	avdbp.m_collisionThreshold			= 4.0f;//4.0f;

	//AngularVelocityDamper* damper = new AngularVelocityDamper(avdbp);
	//SET_FILE_LINE(damper );
//	bp.m_externalVehicleControllers[0] = damper;
//	bp.m_externalVehicleControllers[0] = HK_NULL;


	bp.m_chassisUnitInertiaYaw		= 0.5f;//7.5f;//5.5f;//2.0f;//3.5f;//2.0f;
	bp.m_chassisUnitInertiaRoll		= 0.5f;//0.9f;//0.9f;//0.6f;
	bp.m_chassisUnitInertiaPitch	= 0.5f;//1.5f;//1.5f;//1.5f;//0.75f;//1.5f;//3.5f;//2.0f;




	constraint = new AngularConstraint1D(chassis); // cannot track havok *
    SET_FILE_LINE(constraint);


	constraint->setTau(0.075f);

	const Vector3 up_ws    = Vector3(0.0f,-1.0f,0.0f);
	const Vector3 fwd_os   = Vector3(0.0f,1.0f,0.0f);
	const Vector3 right_os = Vector3(1.0f,0.0f,0.0f);

	constraint->setAxis(up_ws, fwd_os, right_os);
	csolver = new FastConstraintSolver(); // cannot track havok *
    SET_FILE_LINE(csolver);

	csolver->addConstraint(constraint);
	rc->addAction(csolver);
	csolver->setDeactivationThreshold(0.0f);
	



	vehicle = new RaycastVehicle (bp);
	SET_FILE_LINE(vehicle );




	/************************************************************************************/

//	buttons = 0;


	for(u32 i = 0; i < BIKEACTION_NUM_STATES; i++)
	{
		pressed[i] = 0.0f;
	}

	throttle = 0.0f;
	throttleUp = 0.0f;
	turnStep = 0.0f;
	brakingScale = 0.0f;
	framesInAir = 0;


//	enableDustParticles();
//	disableDirtParticles();
	dustCloudCounter = 0;

//	disableLeanDelay();
//	leanDelay = 0;
	currentCheckPoint = 0;
	cameraId = camId;

	canBoost = 0;

	startRace = 0;	

	CParticleSystem* particleSystem = CParticleSystem::getInstance();


	numExhaustParticles = 0.0f;

	
	numTrailParticles = 0.0f;
	numMudParticles= 0.0f;







//	numDustParticles = 0.0f;
	numFrontDustParticles = 0.0f;
	numBackDustParticles = 0.0f;
//	terrainType = 0;



	preLoadPressedCounter = 0;
	preLoadWaitDelay = 0;
	preLoadJumpState = PRELOAD_NO_ATTEMPT_PENDING;
	preLoadJump = FALSE;
	preLoadJumpCounter = 0;
	preLoadJumpPending = FALSE;
	preLoadReportState = PRELOAD_AVAILABLE;



	for(u32 i = 0; i < BIKESTATE_NUM_STATES; i++)
	{
		bikeStateCounter[i] = -1000;
		bikeStateSuspended[i] = FALSE;
	}

	lastFrameBikeState = lastBikeState = bikeState = BIKESTATE_INAIR;
	lastBikeStateCounter = 0;


	comShiftState = COMSS_NORMAL;

	((CBikeTransmission*)m_transmission)->setMinRPM(2000.0f);
	((CBikeTransmission*)m_transmission)->setMaxRPM(8000.0f);

	((CBikeTransmission*)m_transmission)->setWheelRPMScale(40.0f/*35.0f*/);


	bikePowerBand->setTorque(1000, 3000);
	bikePowerBand->setTorque(9000, 3000);






	getBaseVectors();

	lastForwardDir = forwardDir;
	lastUpDir = upDir;
	lastRightDir = rightDir;

	const Vector3 comPos = chassis->getPosition();
	cameraLookAtPos = V3_TO_SXV(comPos);

	turnCounter = 0;
	powerSlideAccelTurnCounter = 0;
	powerSlideBrakeTurnCounter = 0;
	turnPressedPercent = 0.0f;
	preLoadForceCounter = 0;


	powerSlideAccelBoostEligible = FALSE;
	powerSlideBrakeBoostEligible = FALSE;

	forceZeroVelocity();
	pinBike();
	unpinBike();

	filterSAV.setUp(5);
	leanFilter.setUp(5);
	frictionFilter[FW].setUp(5);
	frictionFilter[BW].setUp(5);
	speedometerFilter.setUp(10);

	camLookatPosFilterX.setUp(5);
	camLookatPosFilterY.setUp(5);
	camLookatPosFilterZ.setUp(5);

	frontSuspension.setUp(5);
	backSuspension.setUp(5);



	wheelieCounter = 0;
	bikePacket = 0;


	landingSpeedScale = 1.0f;
	jumpSpeedCounter = 0;
	landingSpeedCounter = 6;

	wheelFramesInContact[FW] = 0;
	wheelFramesInContact[BW] = 0;
	wheelFramesInAir[FW] = 0;
	wheelFramesInAir[BW] = 0;

	jumpAssist = FALSE;
	nitroState = NO_NITRO;




	riderLastPos = riderCurrentPos = riderRestPos = sxVector3d(0, 0, 0);
	riderLastVelocity = riderVelocity = sxVector3d(0, 0, 0);

	stuntCrashFlag = FALSE;
	lockSuspension = FALSE;
	changeInVelCounter = 0;
	willCrashFlag = false;

	wheelBase  = 0.0f;
	bikeLength = 0.0f;



	safeIndex = 0;
	for(u32 i = 0; i < NUM_SAFE_INDEXES; i++)
	{
		safePos[i] = getPosition();
		safeDir[i] = forwardDir;
	}

	lastBikePitchRollIndex = 0;
	for(u32 i = 0; i < NUM_PITCH_ROLL_INDEXES; i++)
	{
		lastBikePitch[i] = bikePitch;
		lastBikeRoll[i] = bikeRoll;
		lastBikeRelativePitch[i] = bikeRelativePitch;
		lastBikeRelativeRoll[i] = bikeRelativeRoll;
	}


	for(u32 i = 0; i < NUM_VELOCITY_INDEXES; i++)
	{
		lastVelocityMag[i] = lastLinearVelocityMag;
		lastVelocity[i] = lastLinearVelocity;
	}
	lastVelocityMagIndex = 0;
	
	lastThrottleZero = throttleZero = 1.0f;
	jumpAssistInAir = false;


	setNumNitros(0);
//	addNitro();



//-- for debugging

	bikeAttributes.setTopSpeed(2);
	bikeAttributes.setAcceleration(1);
	bikeAttributes.setTraction(1);
	bikeAttributes.setSuspension(2);

	riderAttributes.setJumping(5);
	riderAttributes.setCornering(3);
	riderAttributes.setStunt(3);
	riderAttributes.setStability(4);


//-- end debugging

	setAttributes();
}





//------------------------------------------------------------------------
HavokBike::~HavokBike()
{

	CModel *model = CModel::getInstance();
	Toolkit* toolkit = model->getToolkit();
    
	if( chassis != NULL )
	{
		toolkit->removeRigidBody( chassis );
//		chassis->notifyDependants(ReferenceObject::REF_DELETE);
		delete chassis;
	}

	if( frontWheel != NULL )
	{
		toolkit->removeRigidBody( frontWheel );
//		frontWheel->notifyDependants(ReferenceObject::REF_DELETE);
		delete frontWheel;
	}

	if( backWheel != NULL )
	{
		toolkit->removeRigidBody( backWheel );
//		backWheel->notifyDependants(ReferenceObject::REF_DELETE);
		delete backWheel;
	}
	if (crashBike != NULL)
	{
		toolkit->removeRigidBody( crashBike );
//		crashBike->notifyDependants(ReferenceObject::REF_DELETE);
		delete crashBike;
	}


    if( constraint != NULL )
    {
        csolver->removeConstraint(constraint);
        delete constraint;
    }

    if( csolver != NULL )
    {
        toolkit->m_defaultRigidCollection->removeAction( csolver );
        delete csolver;
    }
    

	//((IntersectionSolver*)vehicle->getIntersectionSolver())->clearBodyList();
	//((IntersectionSolver*)vehicle->getIntersectionSolver())->notifyDependants(REF_DELETE);
	//((IntersectionSolver*)vehicle->getIntersectionSolver())->destroyReferences();


        
    if(vehicle != NULL)
        delete vehicle;

    
#if PRINT_DESTRUCTOR_MESSAGE
    x_printf("HavokBike Destructor\n");
#endif

}



void HavokBike::setAttributes()
{


//	bikeAttributes.setTopSpeed(2);


	switch(bikeAttributes.getTopSpeed())
	{
	case 1:
		attributes.maximumSpeed = 68.0f;
		attributes.maxJumpSpeed = 79.92f;
		break;
	case 2:
		attributes.maximumSpeed = 70.0f;
		attributes.maxJumpSpeed = 79.92f;
		break;
	case 3:
		attributes.maximumSpeed = 72.0f;
		attributes.maxJumpSpeed = 79.92f;
		break;
	}
	attributes.nitroMaximumSpeed = attributes.maximumSpeed + 30.0f;

	switch(bikeAttributes.getAcceleration())
	{
	case 1:
		bikeSpeedBand->setTurbo(0, 40.0f, 2.0f);
		bikeSpeedBand->setTurbo(40.0f, 55.0f, 1.0f);
		bikeSpeedBand->setTurbo(55.0f, 75.0f, 1.0f);
		break;
	case 2:
		bikeSpeedBand->setTurbo(0, 20, 1.0f);
		bikeSpeedBand->setTurbo(20.0f, 60.0f, 2.0f);
		bikeSpeedBand->setTurbo(60.0f, 75.0f, 1.0f);
		break;
	case 3:
		bikeSpeedBand->setTurbo(0, 30.0f, 1.0f);
		bikeSpeedBand->setTurbo(30.0f, 55.0f, 0.9f);
		bikeSpeedBand->setTurbo(55.0f, 75.0f, 1.5f);
		break;
	}
	switch(bikeAttributes.getSuspension())
	{
	case 1:
		attributes.landingSpeedRecoveryJumps = 0.85f;
		attributes.landingSpeedRecoveryWhoops = 0.933f;
		break;
	case 2:
		attributes.landingSpeedRecoveryJumps = 0.925f;
		attributes.landingSpeedRecoveryWhoops = 0.9667f;
		break;
	case 3:
		attributes.landingSpeedRecoveryJumps = 1.0f;
		attributes.landingSpeedRecoveryWhoops = 1.0f;
		break;
	}

	switch(bikeAttributes.getTraction())
	{
	case 1:
		attributes.traction = 0;
		break;
	case 2:
		attributes.traction = 1;
		break;
	case 3:
		attributes.traction = 2;
		break;
	}







	switch(riderAttributes.getCornering())
	{
	case 1:
		attributes.corneringPercent = 0.75f;
		break;
	case 2:
		attributes.corneringPercent = 0.875f;
		break;
	case 3:
		attributes.corneringPercent = 1.0f;
		break;
	case 4:
		attributes.corneringPercent = 1.125f;
		break;
	case 5:
		attributes.corneringPercent = 1.25f;
		break;
	}


	switch(riderAttributes.getJumping())
	{
	case 1:
		attributes.jumpSpeedTolerance = 4.0f;
		break;
	case 2:
		attributes.jumpSpeedTolerance = 8.0f;
		break;
	case 3:
		attributes.jumpSpeedTolerance = 12.0f;
		break;
	case 4:
		attributes.jumpSpeedTolerance = 16.0f;
		break;
	case 5:
		attributes.jumpSpeedTolerance = 20.0f;
		break;
	}


	switch(riderAttributes.getStability())
	{
	case 1:
		attributes.stability = 0.80f;
		attributes.stabilityChangeInVel = 12.0f;
		break;
	case 2:
		attributes.stability = 0.90f;
		attributes.stabilityChangeInVel = 11.0f;
		break;
	case 3:
		attributes.stability = 1.0f;
		attributes.stabilityChangeInVel = 10.0f;
		break;
	case 4:
		attributes.stability = 1.1f;
		attributes.stabilityChangeInVel = 9.0f;
		break;
	case 5:
		attributes.stability = 1.2f;
		attributes.stabilityChangeInVel = 8.0f;
		break;
	}

	switch(riderAttributes.getStunt())
	{
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		break;
	case 5:
		break;
	}


	CGameOptions* gameOptions = CSxGame::getGameOptions();	
	if(gameOptions->getTrackType() == TRACKTYPE_BAJA)
	{
		attributes.maximumSpeed += 5.0f;
		attributes.stabilityChangeInVel -= 2.0f;
		speedScale = 1.0f;//1.35f;
	}
	else
	{
		speedScale = 1.0f;//1.15f;
	}
}




const f32 MAX_TURN_BUTTON_PRESSED = 248;//232;
const f32 MAX_PITCH_BUTTON_PRESSED = 232;
const f32 MAX_ROLL_BUTTON_PRESSED = 244;
const f32 MAX_POWERSLIDE_BUTTON_PRESSED = 244;

bool HavokBike::checkMessage(bikeActions action)
{
	if (action < BIKEACTION_NUM_STATES && action >= 0)
	{
		if (bikePacket & (1<<action))
			return TRUE;
	}
	return FALSE;
}


//------------------------------------------------------------------------
void HavokBike::doAction(const CPacket& pkt, sxtime_t currentTime)
{
	bikePacket = pkt.getAction();

	const real accInc  = 0.01f;
	const real turnInc = 0.01f;
	bool useAnalog = TRUE;


	//-- If we are able to boost and boost has not happend then set canBoost to DID boost
//	if(canBoost == CAN_BOOST && canBoost < DID_BOOST && (pkt.getAction() == LACT_SEL1_PRESSED))
	if(canBoost == CAN_BOOST && canBoost < DID_BOOST && (checkMessage(BIKEACTION_ACCEL)))
		canBoost = DID_BOOST;
	//-- if player tryed to boost befor they should then dont let later..
	else if(canBoost == CANT_BOOST && (checkMessage(BIKEACTION_ACCEL)))
		canBoost = EARLY_BOOST;



	
	lastThrottleZero = throttleZero = 1.0f;
	if (checkMessage(BIKEACTION_ACCEL))
	{
		if (isCPU)
		{
			lastThrottleZero = throttleZero = 0.01f * (f32)pkt.getActionParameter(ANALOG_ACCEL);
			pressed[BIKEACTION_ACCEL] = 255;
//			pressed[BIKEACTION_ACCEL] = pkt.getActionParameter(ANALOG_ACCEL);
		}
		else
		{
			pressed[BIKEACTION_ACCEL] = pkt.getActionParameter(ANALOG_ACCEL);
		}
	}
	if (checkMessage(BIKEACTION_BRAKE))
	{
		pressed[BIKEACTION_BRAKE] = pkt.getActionParameter(ANALOG_BRAKE);
	}
	if (checkMessage(BIKEACTION_FORWARD))
	{
		pressed[BIKEACTION_FORWARD] = pkt.getActionParameter(ANALOG_FWD);
	}
	else if (checkMessage(BIKEACTION_BACK))
	{
		pressed[BIKEACTION_BACK] = pkt.getActionParameter(ANALOG_BACK);
	}
	if (checkMessage(BIKEACTION_LEFT_TURN))
	{
		pressed[BIKEACTION_LEFT_TURN] = pkt.getActionParameter(ANALOG_LEFT);
	}
	else if (checkMessage(BIKEACTION_RIGHT_TURN))
	{
		pressed[BIKEACTION_RIGHT_TURN] = pkt.getActionParameter(ANALOG_RIGHT);
	}
	if (checkMessage(BIKEACTION_LEFT_POWERSLIDE_ACCEL))
	{
		pressed[BIKEACTION_LEFT_POWERSLIDE_ACCEL] = pkt.getActionParameter(ANALOG_LEFT);
		if (isCPU)
		{
			lastThrottleZero = throttleZero = 0.01f * (f32)pkt.getActionParameter(ANALOG_ACCEL);
			pressed[BIKEACTION_ACCEL] = 255;
		}
		else
		{
			throttleZero = 1.0f;
			pressed[BIKEACTION_ACCEL] = pkt.getActionParameter(ANALOG_ACCEL);
		}
	}
	if (checkMessage(BIKEACTION_RIGHT_POWERSLIDE_ACCEL))
	{
		pressed[BIKEACTION_RIGHT_POWERSLIDE_ACCEL] = pkt.getActionParameter(ANALOG_RIGHT);
		if (isCPU)
		{
			lastThrottleZero = throttleZero = 0.01f * (f32)pkt.getActionParameter(ANALOG_ACCEL);
			pressed[BIKEACTION_ACCEL] = 255;
		}
		else
		{
			throttleZero = 1.0f;
			pressed[BIKEACTION_ACCEL] = pkt.getActionParameter(ANALOG_ACCEL);
		}
	}
	if (checkMessage(BIKEACTION_LEFT_POWERSLIDE_BRAKE))
	{
		pressed[BIKEACTION_LEFT_POWERSLIDE_BRAKE] = pkt.getActionParameter(ANALOG_LEFT);
		pressed[BIKEACTION_BRAKE] = pkt.getActionParameter(ANALOG_BRAKE);
	}
	if (checkMessage(BIKEACTION_RIGHT_POWERSLIDE_BRAKE))
	{
		pressed[BIKEACTION_RIGHT_TURN] = pkt.getActionParameter(ANALOG_RIGHT);
		pressed[BIKEACTION_BRAKE] = pkt.getActionParameter(ANALOG_BRAKE);
	}
	if (checkMessage(BIKEACTION_STUNT))
	{
		if (pkt.getActionParameter(ANALOG_STUNT_REVERSED))
			pressed[BIKEACTION_STUNT] = -pkt.getActionParameter(ANALOG_STUNT);
		else
			pressed[BIKEACTION_STUNT] = pkt.getActionParameter(ANALOG_STUNT);
	}
}



sxVector3d HavokBike::getCameraForward()
{
	sxVector3d cameraLookAt;

	if (getFlag(BIKE_INAIR) || crashing)
	{
		cameraLookAt = lastForwardDir;
	}
	else
	{
		lastForwardDir = forwardDir;
		lastUpDir = upDir;
		lastRightDir = rightDir;
		cameraLookAt = lastForwardDir;
	}
	
	return cameraLookAt;
}





//------------------------------------------------------------------------
void HavokBike::getBaseVectors()
{
	Vector3 pos;		// Current Position
	Quaternion quat;	// Current Orientation
	chassis->getDisplayToWorldTransform(pos,quat);

	sxQuaternion q( quat.Imag().x, quat.Imag().y, quat.Imag().z, quat.Real());
	
	matrix4 hM;
	Q_SetupMatrix( &q, &hM );
    M4_ClearTranslations( &hM );


	upDir.x =	hM.M[2][0];
	upDir.y = 	hM.M[2][1];
	upDir.z =	hM.M[2][2];

	rightDir.x =	hM.M[0][0];
	rightDir.y = 	hM.M[0][1];
	rightDir.z =	hM.M[0][2];

	forwardDir.x =	hM.M[1][0];
	forwardDir.y = 	hM.M[1][1];
	forwardDir.z =	hM.M[1][2];


	lastFForwardDir = fForwardDir;
	lastFUpDir = fUpDir;
	lastFRightDir = fRightDir;



	fRightDir = rightDir;
	fRightDir.y = 0.0f;
	fRightDir.normalize();

	fUpDir = sxVector3d(0, -1, 0);
//	fUpDir = fRightDir.cross(forwardDir);
	fForwardDir = fUpDir.cross(fRightDir);
}





void HavokBike::bikeTerrainInfo()
{
	CWorldModel*	worldModel = ((CSXModel*) CModel::getInstance())->getWorldModel();

	sxVector3d triVertex[3];
	f32 xPos = getBackWheelPosition().x;
	s32 x = CUtil::f32_to_s32(xPos);
	f32 zPos = getBackWheelPosition().z;
	s32 z = CUtil::f32_to_s32(zPos);

	worldModel->getLocation(triVertex, x, z);
//	TRI_Normal(&wheelNormVec[BW], &triVertex[0], &triVertex[1], &triVertex[2]);
	wheelNormVec[BW].setupVectorFromTriNormal( &triVertex[0], &triVertex[1], &triVertex[2] );



	worldModel->getLocation(triVertex,  CUtil::f32_to_s32(getFrontWheelPosition().x),  CUtil::f32_to_s32(getFrontWheelPosition().z));
//	TRI_Normal(&wheelNormVec[FW], &triVertex[0], &triVertex[1], &triVertex[2]);
	wheelNormVec[FW].setupVectorFromTriNormal( &triVertex[0], &triVertex[1], &triVertex[2] );




	bool inverted;
	f32 sign;
	sxVector3d forwardDirFlat;
	sxVector3d up = sxVector3d(0.0f, -1.0f, 0.0f);
	if (upDir.y < 0.0f)
	{
		inverted = false;
		if (getFrontWheelPosition().y < getBackWheelPosition().y)
		{
			sign = 1.0f;
			//-- leaning back
			if (fabs(forwardDir * up) < 0.5f)
				forwardDirFlat = forwardDir;
			else
				forwardDirFlat = -upDir;
		}
		else //-- leaning forward
		{
			sign = -1.0f;
			if (fabs(forwardDir * up) < 0.5f)
				forwardDirFlat = forwardDir;
			else
				forwardDirFlat = upDir;
		}
	}
	else //-- inverted
	{
		inverted = true;
		if (getFrontWheelPosition().y < getBackWheelPosition().y)
		{
			sign = 1.0f;
			//-- leaning back
			if (fabs(forwardDir * up) < 0.5f)
				forwardDirFlat = -forwardDir;
			else
				forwardDirFlat = -upDir;
		}
		else //-- leaning forward
		{
			sign = -1.0f;
			if (fabs(forwardDir * up) < 0.5f)
				forwardDirFlat = -forwardDir;
			else
				forwardDirFlat = upDir;
		}
	}


	forwardDirFlat.y = 0.0f;
	forwardDirFlat.normalize();

	sxVector3d rightDirFlat = rightDir;
	rightDirFlat.y = 0.0f;
	rightDirFlat.normalize();

	f32 normAngle;


//	normAngle = RAD_TO_DEG(x_acos(wheelNormVec[BW] * rightDirFlat)); 
//	terrainRoll = (90.0f - ((2.0f * (90.0f - normAngle)) + normAngle));


	if (wheelNormVec[BW] * rightDirFlat < 0.0f)
	{
		normAngle = RAD_TO_DEG(x_acos( (-wheelNormVec[BW]) * rightDirFlat));
		terrainRoll = 90.0f - normAngle;
	}
	else
	{
		normAngle = RAD_TO_DEG(x_acos( (wheelNormVec[BW]) * rightDirFlat));
		terrainRoll = -(90.0f - normAngle);
	}




	if (wheelNormVec[BW] * forwardDirFlat < 0.0f)
	{
		normAngle = RAD_TO_DEG(x_acos( (-wheelNormVec[BW]) * forwardDirFlat));
		terrainPitch[BW] = 90.0f - normAngle;
	}
	else
	{
		normAngle = RAD_TO_DEG(x_acos( (wheelNormVec[BW]) * forwardDirFlat));
		terrainPitch[BW] = -(90.0f - normAngle);
	}

	if (wheelNormVec[FW] * forwardDirFlat < 0.0f)
	{
		normAngle = RAD_TO_DEG(x_acos( (-wheelNormVec[BW]) * forwardDirFlat));
		terrainPitch[FW] = 90.0f - normAngle;
	}
	else
	{
		normAngle = RAD_TO_DEG(x_acos( (wheelNormVec[FW]) * forwardDirFlat));
		terrainPitch[FW] = -(90.0f - normAngle);
	}

	terrainPitchAvg = 0.5f * (terrainPitch[FW] + terrainPitch[BW]);


	sxVector3d fixedNorm = sxVector3d(0.0f, -1.0f, 0.0f);
	normAngle = RAD_TO_DEG(x_acos(fixedNorm * rightDir)); 
	bikeRoll = -(90.0f - ((2.0f * (90.0f - normAngle)) + normAngle));

//	normAngle = RAD_TO_DEG(x_acos(fixedNorm * forwardDir)); 
//	bikePitch = -(90.0f - ((2.0f * (90.0f - normAngle)) + normAngle));
//	if (inverted)
//		bikePitch = 180.0f - bikePitch;

	normAngle = RAD_TO_DEG(x_acos(forwardDirFlat * forwardDir)); 
	bikePitch = normAngle * sign;





	bikeRelativeRoll = terrainRoll - bikeRoll;
	bikeRelativePitch = terrainPitchAvg - bikePitch;	





	lastBikePitch[lastBikePitchRollIndex] = bikePitch;
	lastBikeRoll[lastBikePitchRollIndex] = bikeRoll;
	lastBikeRelativePitch[lastBikePitchRollIndex] = bikeRelativePitch;
	lastBikeRelativeRoll[lastBikePitchRollIndex] = bikeRelativeRoll;

	lastBikePitchRollIndex++;
	lastBikePitchRollIndex %= NUM_PITCH_ROLL_INDEXES;
}


void HavokBike::getMovingInfo()
{
	if (forwardVelocityMag < TO_METERS_PER_SEC(-0.0f))
		movingForward = FALSE;
	else
		movingForward = TRUE;
}



void HavokBike::pinBike()
{
	chassis->setPinned(true);
	chassis->setActive(false);
}


void HavokBike::unpinBike()
{
	chassis->setPinned(false);
	chassis->setActive(true);
}





void HavokBike::forceZeroVelocity()
{
	Vector3 angVelV3 = chassis->getAngularVelocity();
	sxVector3d angVel = V3_TO_SXV(angVelV3);
	f32 projAngVelForward = angVel * forwardDir;
	angVel = forwardDir * projAngVelForward;
	angVelV3 = SXV_TO_V3(angVel);
	chassis->setAngularVelocity(angVelV3);

	Vector3 lVelV3  = chassis->getLinearVelocity();
	sxVector3d lVel = V3_TO_SXV(lVelV3);
	f32 upVel = lVel * upDir;
	lVel = upDir * upVel;
	lVelV3 = SXV_TO_V3(lVel);
	chassis->setLinearVelocity(lVelV3);

	wheelInfo[FW].setSpinVelocity(0.0f);
	wheelInfo[BW].setSpinVelocity(0.0f);


//	pinBike();
//	unpinBike();

	
//	linearVelocity = lastLinearVelocity = lVel;

//	linearVelocityMag = lastLinearVelocityMag = lVel.magnitude();
//	forwardVelocityMag = 0.0f;

//	inAirLinearVelocityMag = 0.0f;
//	eventStartVelocity = sxVector3d(0, 0, 0);
//	eventStartVelocityMag = 0.0f;
}



//------------------------------------------------------------------------
void HavokBike::zeroVelocity()
{
	Vector3 lVel;

//	sxVector3d up = sxVector3d(0, -1, 0);
//	bool level = TRUE;
//	f32 levelVal = upDir * up;
//	if (levelVal < 0.95f)
//		level = FALSE;


	bool level = FALSE;
	if (fabs(terrainPitchAvg) < 5.0f)
		level = TRUE;


	if (m_input->getAcceleratorPedalInput() == 0.0f && 
		(m_input->getBrakePedalInput() >= NORMAL_BRAKING || level) && 
		wheelInfo[FW].getIsInContact() && 
		wheelInfo[BW].getIsInContact() && 
		linearVelocityMag < MIN_ZERO_LINEAR_VEL)
	{
		lVel = Vector3(0.0f, 0.0f, 0.0f);
		chassis->setLinearVelocity(lVel);
		wheelInfo[FW].setSpinVelocity(0.0f);
		wheelInfo[BW].setSpinVelocity(0.0f);

		Vector3 angVelV3 = chassis->getAngularVelocity();
		sxVector3d angVel = V3_TO_SXV(angVelV3);
		f32 projAngVelForward = angVel * forwardDir;

		angVel = forwardDir * projAngVelForward;
		angVelV3 = SXV_TO_V3(angVel);
		chassis->setAngularVelocity(angVelV3);
	}

	if (forwardVelocityMag < TO_METERS_PER_SEC(-0.0f) && level)
	{
//		sxVector3d force = (fForwardDir) * 10.0f;
//		Vector3 forceV3 = SXV_TO_V3(force);
//		chassis->applyImpulse(forceV3);

//		Vector3 lVel = chassis->getLinearVelocity();
//		lVel = lVel * 0.98f;
//		chassis->setLinearVelocity(lVel);
	}



	if (m_input->getAcceleratorPedalInput() == 0.0f && linearVelocityMag < TO_METERS_PER_SEC(15.0f))
		throttleUp = 0.0f;

	if (m_input->getAcceleratorPedalInput() == 0.0f)
		throttleUpActive = FALSE;


}




//------------------------------------------------------------------------
void HavokBike::zeroLinearVelocity()
{
	if (linearVelocityMag < MIN_ZERO_LINEAR_VEL)
	{
		linearVelocityMag = 0.0f;
		linearVelocity = sxVector3d(0.0f, 0.0f, 0.0f);
	}
}

void HavokBike::zeroThrottleBoost()
{
	((CBikeEngine*)m_engine)->setEngineBoost(throttleZero);
}


void HavokBike::limitMaximumSpeed()
{
	f32 maxSpeed;
	if (nitroState != NO_NITRO)
		maxSpeed = attributes.nitroMaximumSpeed;
	else
		maxSpeed = attributes.maximumSpeed;

	if (linearVelocityMag > TO_METERS_PER_SEC(maxSpeed) && !isCPU)
	{
		Vector3 lVel = chassis->getLinearVelocity();
		lVel = lVel * TO_METERS_PER_SEC(maxSpeed) / linearVelocityMag;
		chassis->setLinearVelocity(lVel);
	}
}



void HavokBike::getWheelieCondition()
{
	wheelieCondition = FALSE;
	if (!wheelInfo[FW].getIsInContact() && getDistToGround(FW) > 35.0f && wheelInfo[BW].getIsInContact())
//	if (!wheelInfo[FW].getIsInContact() && getDistToGround(FW) > 20.0f && wheelInfo[BW].getIsInContact())
	{
		wheelieCondition = TRUE;
	}
}





void HavokBike::nitroBoost()
{
	bool nitroPossible = FALSE;
	if (nitroState == NO_NITRO || nitroState == RAMP_DOWN_NITRO)
		nitroPossible = TRUE;

	if (checkMessage(BIKEACTION_NITRO) && nitrosAvailable && nitroPossible)
	{
		nitrosAvailable--;

		nitroState = RAMP_UP_NITRO;
		nitroCounter = 0;
	}

	sxVector3d force;
	Vector3 forceV3;
	f32 nitroScale = 1.0f;
	if (isCPU)
		nitroScale = 0.125f;

	switch(nitroState)
	{
	case RAMP_UP_NITRO:

		if (wheelInfo[FW].getIsInContact() || wheelInfo[BW].getIsInContact())
		{
			force = fForwardDir * nitroScale * 2000.0f * MIN(1.0f, (f32)(nitroCounter) / 15.0f);
			forceV3 = SXV_TO_V3(force);
			chassis->applyImpulse(forceV3);
		}
		
		if (nitroCounter >= SECS_TO_FRAMES(1.0f))
			nitroState = MID_NITRO;

		break;
	case MID_NITRO:
		if (nitroCounter >= SECS_TO_FRAMES(2.0f))
			nitroState = RAMP_DOWN_NITRO;

		break;
	case RAMP_DOWN_NITRO:
		force = (-fForwardDir) * 1500.0f;
		forceV3 = SXV_TO_V3(force);
		chassis->applyImpulse(forceV3);


		if (linearVelocityMag < attributes.maximumSpeed)
			nitroState = NO_NITRO;
		break;


	}
	if (nitroState != NO_NITRO)
		nitroCounter++;

}







const f32 THROTTLE_MIN		= 0.1f;
const f32 THROTTLE_MAX		= 1.0f;
const f32 TERRAIN_BOOST = 0.5f;//1.25f;//5.25f;//6.0f;//5.0f;//10.0f;//6.95f;




const f32 POWERSLIDE_TAU = 0.0048f;//;4.8f;
const f32 TURN_TAU = 0.075f;
const f32 NORMAL_TAU = 0.25f;
const f32 POWERSLIDE_TURN_STEP = 1.0f;










f32 getSlopePoint(f32 min, f32 minPercent, f32 opt, f32 optPercent, f32 max, f32 maxPercent, f32 current)
{
	f32 percent;
	if (current < opt)
	{
		if (current < min)
			percent = 0.0f;
		else
			percent = (opt - current) / opt;

		percent = minPercent + percent * (optPercent - minPercent);
	}
	else
	{
		if (current > max)
			percent = 1.0f;
		else
			percent = (current - opt) / (max - opt);

		percent = optPercent + percent * (maxPercent - optPercent);
	}
	

	return percent;
}


f32 getTargetPoint(f32 currentPoint, f32 targetPoint, f32 stepPoint)
{
	f32 delta = currentPoint - targetPoint;

	if (x_fabs(delta) <= stepPoint)
		return targetPoint;
	else if (delta > 0.0f)
		return (currentPoint - stepPoint);
	else
		return (currentPoint + stepPoint);
}




//------------------------------------------------------------------------
void HavokBike::control()
{


	bool onGround = FALSE;
	if (getDistToGround() <= 0.0f)
		onGround = TRUE;

	bool onGroundPowerSlide;
	if (getDistToGround(FW) <= 35.0f && getDistToGround(BW) <= 35.0f)
		onGroundPowerSlide = TRUE;
	else 
		onGroundPowerSlide = FALSE;



	wheelieRequested = FALSE;
	bool wheelieRequestAttempted = FALSE;
	if (checkMessage(BIKEACTION_WHEELIE))
	{
		wheelieRequestAttempted = TRUE;

		if (getDistToGround(BW) > 35.0f)
			wheelieCounter = SECS_TO_FRAMES(3.0f);

		if (wheelieCounter >= SECS_TO_FRAMES(3.0f))
		{
			wheelieRequested = FALSE;
		}
		else
		{
			wheelieRequested = TRUE;
			setFlag(BIKE_WHEELIE);
		}
	}





	if (wheelieCondition && !wheelieRequested)
	{
		if ((checkMessage(BIKEACTION_LEFT_POWERSLIDE_BRAKE) && 
			bikeStateCounter[BIKESTATE_LEFT_POWERSLIDE_BRAKE] > 0) ||

			(checkMessage(BIKEACTION_LEFT_POWERSLIDE_ACCEL) &&
			 bikeStateCounter[BIKESTATE_LEFT_POWERSLIDE_ACCEL] > 0) ||
	
			 (checkMessage(BIKEACTION_RIGHT_POWERSLIDE_BRAKE) &&
			 bikeStateCounter[BIKESTATE_RIGHT_POWERSLIDE_BRAKE] > 0) ||
	
			(checkMessage(BIKEACTION_RIGHT_POWERSLIDE_ACCEL) &&
			 bikeStateCounter[BIKESTATE_RIGHT_POWERSLIDE_ACCEL] > 0))
		{
			wheelieCondition = FALSE;
		}
	}

	if (wheelieCondition)
		wheelieCounter++;

	if (!wheelieRequestAttempted && wheelInfo[FW].getIsInContact())
		wheelieCounter = 0;






	f32 pressedPercent = 0.0f;
	f32 slideAngVel;
	f32 turnDirSign;
	f32 maxLeanRot;
	f32 steeringPercent;
	f32 turnPercent;
	f32 invTurnPercent;
	f32 downForceMag;
	f32 friction[2];
	f32 turnAnglePercent[2];
	f32 throttleStep;
	f32 brakeStep;
	sxVector3d lastVel;
	f32 pitchDampening;
	f32 turnScale;
	f32 turnScale2;
	f32 powerSlideCircle;
	bool startTurnCondition;
	bool eligibleBoostState = FALSE;
	s32 pressedDelta;



	if (onGround)
	{
		if ((wheelieCondition || wheelieRequested))
		{
			if (checkMessage(BIKEACTION_LEFT_TURN))
			{
				bikeState = BIKESTATE_LEFT_WHEELIE;
				pressedPercent = pressed[BIKEACTION_LEFT_TURN] / MAX_POWERSLIDE_BUTTON_PRESSED;
			}
			else if (checkMessage(BIKEACTION_LEFT_POWERSLIDE_ACCEL))
			{
				bikeState = BIKESTATE_LEFT_WHEELIE;
				pressedPercent = pressed[BIKEACTION_LEFT_POWERSLIDE_ACCEL] / MAX_POWERSLIDE_BUTTON_PRESSED;
			}
			else if (checkMessage(BIKEACTION_LEFT_POWERSLIDE_BRAKE))
			{
				bikeState = BIKESTATE_LEFT_WHEELIE;
				pressedPercent = pressed[BIKEACTION_LEFT_POWERSLIDE_BRAKE] / MAX_POWERSLIDE_BUTTON_PRESSED;
			}
			else if (checkMessage(BIKEACTION_RIGHT_TURN))
			{
				bikeState = BIKESTATE_RIGHT_WHEELIE;
				pressedPercent = pressed[BIKEACTION_RIGHT_TURN] / MAX_POWERSLIDE_BUTTON_PRESSED;
			}
			else if (checkMessage(BIKEACTION_RIGHT_POWERSLIDE_ACCEL))
			{
				bikeState = BIKESTATE_RIGHT_WHEELIE;
				pressedPercent = pressed[BIKEACTION_RIGHT_POWERSLIDE_ACCEL] / MAX_POWERSLIDE_BUTTON_PRESSED;
			}
			else if (checkMessage(BIKEACTION_RIGHT_POWERSLIDE_BRAKE))
			{
				bikeState = BIKESTATE_RIGHT_WHEELIE;
				pressedPercent = pressed[BIKEACTION_RIGHT_POWERSLIDE_BRAKE] / MAX_POWERSLIDE_BUTTON_PRESSED;
			}
			else
			{
				bikeState = BIKESTATE_WHEELIE;
			}
		}
		else if (checkMessage(BIKEACTION_STOPPIE))
		{
			bikeState = BIKESTATE_LEFT_STOPPIE;
		}
		else if (checkMessage(BIKEACTION_LEFT_TURN))
		{
			if (linearVelocityMag > TO_METERS_PER_SEC(5.0f))
			{
				bikeState = BIKESTATE_LEFT_TURN;
				pressedPercent = pressed[BIKEACTION_LEFT_TURN] / MAX_POWERSLIDE_BUTTON_PRESSED;
			}
			else if (checkMessage(BIKEACTION_BRAKE))
			{
				bikeState = BIKESTATE_NO_TURN_BRAKE;
			}
			else if (checkMessage(BIKEACTION_ACCEL))
			{
				bikeState = BIKESTATE_NO_TURN_ACCEL;
			} 
			else
			{
				bikeState = BIKESTATE_NO_TURN_NO_ACCEL;
			}
		}
		else if (checkMessage(BIKEACTION_RIGHT_TURN))
		{
			if (linearVelocityMag > TO_METERS_PER_SEC(5.0f))
			{
				bikeState = BIKESTATE_RIGHT_TURN;
				pressedPercent = pressed[BIKEACTION_RIGHT_TURN] / MAX_POWERSLIDE_BUTTON_PRESSED;
			}
			else if (checkMessage(BIKEACTION_BRAKE))
			{
				bikeState = BIKESTATE_NO_TURN_BRAKE;
			}
			else if (checkMessage(BIKEACTION_ACCEL))
			{
				bikeState = BIKESTATE_NO_TURN_ACCEL;
			} 
			else
			{
				bikeState = BIKESTATE_NO_TURN_NO_ACCEL;
			}
		}
		else if (checkMessage(BIKEACTION_LEFT_POWERSLIDE_BRAKE))
		{
			bikeState = BIKESTATE_LEFT_POWERSLIDE_BRAKE;
			pressedPercent = pressed[BIKESTATE_LEFT_POWERSLIDE_BRAKE] / MAX_POWERSLIDE_BUTTON_PRESSED;
		}
		else if (checkMessage(BIKEACTION_RIGHT_POWERSLIDE_BRAKE))
		{
			bikeState = BIKESTATE_RIGHT_POWERSLIDE_BRAKE;
			pressedPercent = pressed[BIKEACTION_RIGHT_POWERSLIDE_BRAKE] / MAX_POWERSLIDE_BUTTON_PRESSED;
		}
		else if (checkMessage(BIKEACTION_LEFT_POWERSLIDE_ACCEL))
		{
			bikeState = BIKESTATE_LEFT_POWERSLIDE_ACCEL;
			pressedPercent = pressed[BIKEACTION_LEFT_POWERSLIDE_ACCEL] / MAX_POWERSLIDE_BUTTON_PRESSED;
		}
		else if (checkMessage(BIKEACTION_RIGHT_POWERSLIDE_ACCEL))
		{
			bikeState = BIKESTATE_RIGHT_POWERSLIDE_ACCEL;
			pressedPercent = pressed[BIKEACTION_RIGHT_POWERSLIDE_ACCEL] / MAX_POWERSLIDE_BUTTON_PRESSED;
		}		
		else if (checkMessage(BIKEACTION_BRAKE))
		{
			bikeState = BIKESTATE_NO_TURN_BRAKE;
		}
		else if (checkMessage(BIKEACTION_ACCEL))
		{
			bikeState = BIKESTATE_NO_TURN_ACCEL;
		} 
		else
		{
			bikeState = BIKESTATE_NO_TURN_NO_ACCEL;
		}
	}
	else if (onGroundPowerSlide)
	{
		if (checkMessage(BIKEACTION_LEFT_POWERSLIDE_BRAKE))
		{
			bikeState = BIKESTATE_LEFT_POWERSLIDE_BRAKE;
			pressedPercent = pressed[BIKESTATE_LEFT_POWERSLIDE_BRAKE] / MAX_POWERSLIDE_BUTTON_PRESSED;
		}
		else if (checkMessage(BIKEACTION_RIGHT_POWERSLIDE_BRAKE))
		{
			bikeState = BIKESTATE_RIGHT_POWERSLIDE_BRAKE;
			pressedPercent = pressed[BIKEACTION_RIGHT_POWERSLIDE_BRAKE] / MAX_POWERSLIDE_BUTTON_PRESSED;
		}
		else if (checkMessage(BIKEACTION_LEFT_POWERSLIDE_ACCEL))
		{
			bikeState = BIKESTATE_LEFT_POWERSLIDE_ACCEL;
			pressedPercent = pressed[BIKEACTION_LEFT_POWERSLIDE_ACCEL] / MAX_POWERSLIDE_BUTTON_PRESSED;
		}
		else if (checkMessage(BIKEACTION_RIGHT_POWERSLIDE_ACCEL))
		{
			bikeState = BIKESTATE_RIGHT_POWERSLIDE_ACCEL;
			pressedPercent = pressed[BIKEACTION_RIGHT_POWERSLIDE_ACCEL] / MAX_POWERSLIDE_BUTTON_PRESSED;
		}		
		else
		{
			bikeState = BIKESTATE_INAIR;
		}
	}
	else if (checkMessage(BIKEACTION_LEFT_POWERSLIDE_BRAKE) && bikeStateCounter[BIKESTATE_LEFT_POWERSLIDE_BRAKE] > 0)
	{
		bikeStateSuspended[BIKESTATE_LEFT_POWERSLIDE_BRAKE] = TRUE;
		bikeState = BIKESTATE_POWERSLIDE_INAIR;
		setFlag(BIKE_POWERSLIDE_LEFT);
		powerSlideLeftGround = TRUE;
	}
	else if (checkMessage(BIKEACTION_RIGHT_POWERSLIDE_BRAKE) && bikeStateCounter[BIKESTATE_RIGHT_POWERSLIDE_BRAKE] > 0)
	{
		bikeStateSuspended[BIKESTATE_RIGHT_POWERSLIDE_BRAKE] = TRUE;
		bikeState = BIKESTATE_POWERSLIDE_INAIR;
		setFlag(BIKE_POWERSLIDE_LEFT);
		powerSlideLeftGround = TRUE;
	}
	else if (checkMessage(BIKEACTION_LEFT_POWERSLIDE_ACCEL) && bikeStateCounter[BIKESTATE_LEFT_POWERSLIDE_ACCEL] > 0)
	{
		bikeStateSuspended[BIKESTATE_LEFT_POWERSLIDE_ACCEL] = TRUE;
		bikeState = BIKESTATE_POWERSLIDE_INAIR;
		setFlag(BIKE_POWERSLIDE_LEFT);
		powerSlideLeftGround = TRUE;
	}
	else if (checkMessage(BIKEACTION_RIGHT_POWERSLIDE_ACCEL) && bikeStateCounter[BIKESTATE_RIGHT_POWERSLIDE_ACCEL] > 0)
	{
		bikeStateSuspended[BIKESTATE_RIGHT_POWERSLIDE_ACCEL] = TRUE;
		bikeState = BIKESTATE_POWERSLIDE_INAIR;
		setFlag(BIKE_POWERSLIDE_LEFT);
		powerSlideLeftGround = TRUE;
	}		
	else
	{
		bikeState = BIKESTATE_INAIR;
	}



	pressedPercent = MIN(1.0f, pressedPercent);





	if (lastFrameBikeState != bikeState)
	{
		if (!bikeStateSuspended[bikeState])
		{
			lastBikeState = lastFrameBikeState;
			lastBikeStateCounter = bikeStateCounter[lastBikeState];

			if (!bikeStateSuspended[lastBikeState])
			{
				bikeStateCounter[lastBikeState] = 0;
				powerSlideLeftGround = FALSE;
		
				eventStartVelocity = linearVelocity;
//				eventStartVelocity.x = linearVelocity * rightDir;
//				eventStartVelocity.y = linearVelocity * upDir;
//				eventStartVelocity.z = linearVelocity * forwardDir;

//				eventStartForwardDir = forwardDir;
//				eventStartUpDir = upDir;
//				eventStartRightDir = rightDir;

				eventStartVelocityMag = linearVelocityMag;
				bikeStateCounter[bikeState] = 0;
	
				eventStartAngle = RAD_TO_DEG(x_atan2(fForwardDir.x, fForwardDir.z));
				eventDeltaAngle = 0.0f;
			}
		}
		else
		{
			bikeStateSuspended[bikeState] = FALSE;
		}
	}
	else
	{
		eventDeltaAngle += RAD_TO_DEG(x_acos(lastFForwardDir * fForwardDir));
	}

	lastFrameBikeState = bikeState;


	f32 boostPercent = (linearVelocityMag / MAX_VELOCITY);
	boostPercent = MIN(boostPercent, 1.0f);
	boostPercent = 1.0f - boostPercent;








	f32 velocityPercent = (linearVelocityMag / TO_METERS_PER_SEC(65.0f));
	velocityPercent = MIN(velocityPercent, 1.0f);
	f32 invVelocityPercent = 1.0f - velocityPercent;
	invVelocityPercent = MAX(0.0f, invVelocityPercent);
	invVelocityPercent = MIN(1.0f, invVelocityPercent);

	f32 acceleration = linearVelocityMag - lastLinearVelocityMag;

	f32 psBrakeVelocityPercent = MIN(0.85f, (linearVelocityMag / TO_METERS_PER_SEC(65.0f)));





	((CBikeBrake*)m_brakes)->setWheelIsConnectedToHandbrake(FW, FALSE);
	((CBikeBrake*)m_brakes)->setWheelIsConnectedToHandbrake(BW, FALSE);
	((CBikeDriverInput*)m_input)->setHandbrake(FALSE);

	((CBikeSteering*)m_steering)->setDoesWheelSteer(BW, FALSE);


	sxVector3d terrainUpVector = wheelNormVec[FW];
	sxVector3d terrainForwardVector = wheelNormVec[FW].cross(rightDir);
	sxVector3d terrainRightVector = terrainForwardVector.cross(wheelNormVec[FW]);
	
	Vector3 angVelV3 = chassis->getAngularVelocity();
	sxVector3d angVel = V3_TO_SXV(angVelV3);
	
	f32 projAngVelUp = angVel * terrainUpVector;
	f32 projAngVelForward = angVel * terrainForwardVector;
	f32 projAngVelRight = angVel * terrainRightVector;

	pitchDampening = 1.0f;


	if (comShiftState != COMSS_NORMAL)
	{
		switch(comShiftState)
		{
		case COMSS_FORWARD:
			setCenterOfMass(sxVector3d(0.0f,-0.35f,0.0f));
			break;
		case COMSS_BACK:
			setCenterOfMass(sxVector3d(0.0f,0.35f,0.0f));
			break;
		case COMSS_LEFT:
			setCenterOfMass(sxVector3d(-0.20f,0.0f,0.0f));
			break;
		case COMSS_RIGHT:
			setCenterOfMass(sxVector3d(0.20f,0.0f,0.0f));
			break;
		}
		comShiftState = COMSS_NORMAL;
	}



	if (pressedPercent != 0.0f && pressedPercent < 0.25f)
		pressedPercent = 0.25f;

	switch(bikeState)
	{
	case BIKESTATE_LEFT_TURN:

		if (bikeStateCounter[bikeState] == 0)
			pressedDelta = pressed[BIKEACTION_LEFT_TURN];
		else
			pressedDelta = pressed[BIKEACTION_LEFT_TURN] - lastLeftTurnPressed;
		lastLeftTurnPressed = pressed[BIKEACTION_LEFT_TURN];


		if (pressed[BIKEACTION_LEFT_TURN] > 180)
			sharpLeftTurnPressedCounter++;
		else
			sharpLeftTurnPressedCounter = 0;



		if (bikeStateCounter[bikeState] == 0)
		{
			leftTurnType = NORMAL_TURN;
		}
		else if (bikeStateCounter[bikeState] >= 0)
		{
			if (throttleUp < 1.0f)
			{
				if (linearVelocityMag > TO_METERS_PER_SEC(3.0f))
				{
					leftTurnType = FROM_STOP_SLIDE_TURN;
				}
				else
				{
					leftTurnType = NORMAL_TURN;
				}
			}
//			else if (leftTurnType == NORMAL_TURN && pressedDelta > 60 && pressed[BIKEACTION_LEFT_TURN] > 200 && linearVelocityMag > TO_METERS_PER_SEC(50.0f))
			else if (leftTurnType == NORMAL_TURN && 
					 ((pressedDelta > 60 && pressed[BIKEACTION_LEFT_TURN] > 220) || sharpLeftTurnPressedCounter > 70) && 
					 linearVelocityMag > TO_METERS_PER_SEC(50.0f))
			{
				leftTurnType = SHARP_SLIDE_TURN;
				sharpTurnCounter = 0;
				eventStartVelocityMag = linearVelocityMag;
			}
			else if (leftTurnType == SHARP_SLIDE_TURN)
			{
				if (pressed[BIKEACTION_LEFT_TURN] < 150 || 
					linearVelocityMag < TO_METERS_PER_SEC(50.0f))
					leftTurnType = NORMAL_TURN;
			}
			else if (leftTurnType == FROM_STOP_SLIDE_TURN)
			{
				leftTurnType = NORMAL_TURN;
			}
		}


		setFlag(BIKE_LEFT_TURN);
//		constraint->setTau(TURN_TAU);
		constraint->setTau(0.0005f);
		downForceMag = 90000.0f;
		friction[FW] = tFriction.turn.normal * 2.0f;
		friction[BW] = tFriction.turn.normal * 2.0f;

		turnPercent = (f32)(bikeStateCounter[bikeState]) / 60.0f;
		turnPercent = MIN(1.0f, turnPercent);
		steeringPercent = 0.28f;
		turnScale = (f32)(bikeStateCounter[bikeState]) / 10.0f;
		turnScale = MAX(0.5f,turnScale);
		turnScale = MIN(1.5f, turnScale);

		turnScale2 = (f32)(bikeStateCounter[bikeState]) / 10.0f;
		turnScale2 = MAX(0.5f,turnScale);
		turnScale2 = MIN(1.6f, turnScale);

		turnStep = -POWERSLIDE_TURN_STEP * turnPercent;

		if (linearVelocityMag > TO_METERS_PER_SEC(5.0f))
		{
			slideAngVel = 1.45f * attributes.corneringPercent * pressedPercent * turnScale * getSlopePoint(0.0f, 0.75f, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
			leanPercent = 1.0f * attributes.corneringPercent * pressedPercent * turnScale2 * getSlopePoint(0.0f, 0.75f, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
		}
		else
		{
			slideAngVel = 0.0f;
			leanPercent = 0.0f;
		}

		turnDirSign = 1.0f;
		maxLeanRot = 40.0f;
		turnAnglePercent[FW] = 1.0f;
		turnAnglePercent[BW] = -0.5f;

		if (checkMessage(BIKEACTION_ACCEL) && acceleration < 0)
			((CBikeEngine*)m_engine)->scaleEngineBoost(1.0f + (boostPercent * 3.0f * attributes.corneringPercent));


		if (checkMessage(BIKEACTION_ACCEL))
		{
			if (leftTurnType == FROM_STOP_SLIDE_TURN && bikeStateCounter[bikeState] > 6)
			{
				if (throttleUp < 1.0f)
				{
					throttleUpActive = TRUE;
	
					slideAngVel = 2.0f * tFriction.fromStop.ang * attributes.corneringPercent * pressedPercent * turnScale * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
	
					sxVector3d force = (fRightDir) * 1000.0f * tFriction.fromStop.force * throttleUp;
					Vector3 forceV3 = SXV_TO_V3(force);
					chassis->applyImpulse(forceV3);
				}
			}
			else if (leftTurnType == SHARP_SLIDE_TURN && bikeStateCounter[bikeState] > 6)
			{

				f32 normalTurnPercent = getSlopePoint(0.0f, 1.0f, 40.0f, 1.0f, 120.0f, 0.0f, sharpTurnCounter);
				if (sharpTurnCounter < 120.0f)
				{
//					friction[FW] = 8.0f * getSlopePoint(0.0f, 0.0f, 20.0f, 0.0f, 120.0f, 1.0f, sharpTurnCounter);
//					friction[BW] = 8.0f * getSlopePoint(0.0f, 0.0f, 20.0f, 0.0f, 120.0f, 1.0f, sharpTurnCounter);

					if (sharpLeftTurnPressedCounter < 70)
					{
						friction[FW] = tFriction.slide.normal * 2.0f * getSlopePoint(0.0f, 0.0f, 20.0f, 0.0f, 50.0f, 1.0f, sharpTurnCounter);
						friction[BW] = tFriction.slide.normal * 2.0f * getSlopePoint(0.0f, 0.0f, 20.0f, 0.0f, 50.0f, 1.0f, sharpTurnCounter);
					}


					sxVector3d force = fForwardDir * normalTurnPercent * 5000.0f * tFriction.slide.force * pressedPercent * x_sqrt(velocityPercent) * MIN(1.0f, (f32)(sharpTurnCounter) / 40.0f);
					Vector3 forceV3 = SXV_TO_V3(force);
					chassis->applyImpulse(forceV3);
	
					if (linearVelocityMag > eventStartVelocityMag && normalTurnPercent != 0.0f)
					{
						Vector3 lVel = chassis->getLinearVelocity();
						lVel = lVel * eventStartVelocityMag / linearVelocityMag;
						chassis->setLinearVelocity(lVel);
					}
				}
				else
				{

				}
				sharpTurnCounter++;

			}

			
			throttleStep = 0.01f;
			brakeStep = 0.0f;

		}
		else if (checkMessage(BIKEACTION_BRAKE))
		{
			throttleStep = 0.0f;

			brakeStep = 0.01f;
			friction[FW] = 0.0f;
			friction[BW] = 0.0f;
		}


		bikeStateCounter3 = 0.0f;
		eligibleBoostState = TRUE;
		break;

	case BIKESTATE_RIGHT_TURN:
		if (bikeStateCounter[bikeState] == 0)
			pressedDelta = pressed[BIKEACTION_RIGHT_TURN];
		else
			pressedDelta = pressed[BIKEACTION_RIGHT_TURN] - lastRightTurnPressed;
		lastRightTurnPressed = pressed[BIKEACTION_RIGHT_TURN];


		if (pressed[BIKEACTION_RIGHT_TURN] > 180)
			sharpRightTurnPressedCounter++;
		else
			sharpRightTurnPressedCounter = 0;


		if (bikeStateCounter[bikeState] == 0)
		{
			rightTurnType = NORMAL_TURN;
		}
		else if (bikeStateCounter[bikeState] >= 0)
		{
			if (throttleUp < 1.0f)
			{
				if (linearVelocityMag > TO_METERS_PER_SEC(3.0f))
				{
					rightTurnType = FROM_STOP_SLIDE_TURN;
				}
				else
				{
					leftTurnType = NORMAL_TURN;
				}
			}
//			else if (rightTurnType == NORMAL_TURN && pressedDelta > 60 && pressed[BIKEACTION_RIGHT_TURN] > 200 && linearVelocityMag > TO_METERS_PER_SEC(50.0f))
			else if (rightTurnType == NORMAL_TURN && 
					 ((pressedDelta > 60 && pressed[BIKEACTION_RIGHT_TURN] > 220) || sharpRightTurnPressedCounter > 70) && 
					 linearVelocityMag > TO_METERS_PER_SEC(50.0f))
			{
				rightTurnType = SHARP_SLIDE_TURN;
				sharpTurnCounter = 0;
				eventStartVelocityMag = linearVelocityMag;
			}
			else if (rightTurnType == SHARP_SLIDE_TURN)
			{
				if (pressed[BIKEACTION_RIGHT_TURN] < 150 || 
					linearVelocityMag < TO_METERS_PER_SEC(50.0f))
					rightTurnType = NORMAL_TURN;
			}
//			else if (rightTurnType == SHARP_SLIDE_TURN && pressed[BIKEACTION_RIGHT_TURN] < 150)
//			{
//				rightTurnType = NORMAL_TURN;
//			}
			else if (rightTurnType == FROM_STOP_SLIDE_TURN)
			{
				rightTurnType = NORMAL_TURN;
			}
		}



		setFlag(BIKE_RIGHT_TURN);
//		constraint->setTau(TURN_TAU);
		constraint->setTau(0.0005f);
		downForceMag = 90000.0f;
		friction[FW] = tFriction.turn.normal * 2.0f;
		friction[BW] = tFriction.turn.normal * 2.0f;

		turnPercent = (f32)(bikeStateCounter[bikeState]) / 60.0f;
		turnPercent = MIN(1.0f, turnPercent);
		steeringPercent = 0.28f;
		turnScale = (f32)(bikeStateCounter[bikeState]) / 10.0f;
		turnScale = MAX(0.5f,turnScale);
		turnScale = MIN(1.5f, turnScale);

		turnScale2 = (f32)(bikeStateCounter[bikeState]) / 10.0f;
		turnScale2 = MAX(0.5f,turnScale);
		turnScale2 = MIN(1.6f, turnScale);

		turnStep = POWERSLIDE_TURN_STEP * turnPercent;

		if (linearVelocityMag > TO_METERS_PER_SEC(5.0f))
		{
			slideAngVel = -1.45f * attributes.corneringPercent * pressedPercent * turnScale * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
			leanPercent = 1.0f * attributes.corneringPercent * pressedPercent * turnScale2 * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
		}
		else
		{
			slideAngVel = 0.0f;
			leanPercent = 0.0f;
		}

		turnDirSign = -1.0f;
		maxLeanRot = 40.0f;
		turnAnglePercent[FW] = 1.0f;
		turnAnglePercent[BW] = -0.5f;

		if (checkMessage(BIKEACTION_ACCEL) && acceleration < 0)
			((CBikeEngine*)m_engine)->scaleEngineBoost(1.0f + (boostPercent * 3.0f * attributes.corneringPercent));


		if (checkMessage(BIKEACTION_ACCEL))
		{
			if (rightTurnType == FROM_STOP_SLIDE_TURN && bikeStateCounter[bikeState] > 6)
			{
				if (throttleUp < 1.0f)
				{
					throttleUpActive = TRUE;
	
//					slideAngVel = -2.0f * attributes.corneringPercent * pressedPercent * turnScale * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
					slideAngVel = -2.0f * tFriction.fromStop.ang * attributes.corneringPercent * pressedPercent * turnScale * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
	
					sxVector3d force = (-fRightDir) * 1000.0f * tFriction.fromStop.force * throttleUp;
					Vector3 forceV3 = SXV_TO_V3(force);
					chassis->applyImpulse(forceV3);
				}
			}
			else if (rightTurnType == SHARP_SLIDE_TURN && bikeStateCounter[bikeState] > 6)
			{

				f32 normalTurnPercent = getSlopePoint(0.0f, 1.0f, 40.0f, 1.0f, 120.0f, 0.0f, sharpTurnCounter);
				if (sharpTurnCounter < 120.0f)
				{
//					friction[FW] = 8.0f * getSlopePoint(0.0f, 0.0f, 20.0f, 0.0f, 120.0f, 1.0f, sharpTurnCounter);
//					friction[BW] = 8.0f * getSlopePoint(0.0f, 0.0f, 20.0f, 0.0f, 120.0f, 1.0f, sharpTurnCounter);
					if (sharpLeftTurnPressedCounter < 70)
					{
						friction[FW] = tFriction.slide.normal * 2.0f * getSlopePoint(0.0f, 0.0f, 20.0f, 0.0f, 50.0f, 1.0f, sharpTurnCounter);
						friction[BW] = tFriction.slide.normal * 2.0f * getSlopePoint(0.0f, 0.0f, 20.0f, 0.0f, 50.0f, 1.0f, sharpTurnCounter);
					}


					sxVector3d force = fForwardDir * normalTurnPercent * 5000.0f * tFriction.slide.force * pressedPercent * x_sqrt(velocityPercent) * MIN(1.0f, (f32)(sharpTurnCounter) / 40.0f);
					Vector3 forceV3 = SXV_TO_V3(force);
					chassis->applyImpulse(forceV3);
	
					if (linearVelocityMag > eventStartVelocityMag && normalTurnPercent != 0.0f)
					{
						Vector3 lVel = chassis->getLinearVelocity();
						lVel = lVel * eventStartVelocityMag / linearVelocityMag;
						chassis->setLinearVelocity(lVel);
					}
				}
				sharpTurnCounter++;

			}

			
			throttleStep = 0.01f;
			brakeStep = 0.0f;

		}
		else if (checkMessage(BIKEACTION_BRAKE))
		{
			throttleStep = 0.0f;

			brakeStep = 0.01f;
			friction[FW] = 0.0f;
			friction[BW] = 0.0f;
		}


		bikeStateCounter3 = 0.0f;
		eligibleBoostState = TRUE;
		break;

	case BIKESTATE_LEFT_POWERSLIDE_ACCEL:
		powerSlideAccelBoostVelocityMag = eventStartVelocityMag;
		powerSlideAccelBoostEligible = FALSE;
		pitchDampening = 0.5f;
		comShiftState = COMSS_BACK;

		powerSlideCircle = (270.0f) * (eventStartVelocityMag / TO_METERS_PER_SEC(75.0f));
		startTurnCondition = TRUE;
		if (turnCounter < -18)
			startTurnCondition = FALSE;

		if (startTurnCondition && !powerSlideLeftGround)
		{
			if (pressedPercent < (f32)-turnCounter / 18.0f)
				pressedPercent = MAX(1.0f, (f32)-turnCounter / 18.0f);

			setFlag(BIKE_LEFT_TURN);
//			constraint->setTau(TURN_TAU);
			constraint->setTau(0.0005f);
			downForceMag = 90000.0f;
			friction[FW] = tFriction.turn.normal * 2.0f;
			friction[BW] = tFriction.turn.normal * 2.0f;
	
			turnPercent = (f32)(bikeStateCounter[bikeState]) / 60.0f;
			turnPercent = MIN(1.0f, turnPercent);
			steeringPercent = 0.28f;
			turnStep = -POWERSLIDE_TURN_STEP * turnPercent;
			leanPercent = 1.0f * pressedPercent * velocityPercent;
			slideAngVel = 1.45f * pressedPercent;
			turnDirSign = 1.0f;
			maxLeanRot = 50.0f;//50.0f
			turnAnglePercent[FW] = 1.0f;
			turnAnglePercent[BW] = -0.5f;
	
			if (checkMessage(BIKEACTION_ACCEL) && acceleration < 0)
				((CBikeEngine*)m_engine)->scaleEngineBoost(1.0f + (boostPercent * 1.0f));
	
			throttleStep = 0.01f;
			brakeStep = 0.0f;

			bikeStateCounter3 = 0.0f;
			powerSlideAccelBoostEligible = TRUE;
		}
		else if (eventDeltaAngle < powerSlideCircle && !powerSlideLeftGround && powerSlideBrakeTurnCounter > -18 && !powerSlideEnd)
		{			
			setFlag(BIKE_POWERSLIDE_LEFT);
//			constraint->setTau(POWERSLIDE_TAU);
			constraint->setTau(0.0005f);
			downForceMag = 90000.0f;

			friction[FW] = tFriction.ps.normal * 2.0f * getSlopePoint(0.0f, 0.0f, 60.0f, 0.0f, 120.0f, 1.0f, bikeStateCounter3);
			friction[BW] = tFriction.ps.normal * 2.0f * getSlopePoint(0.0f, 0.0f, 60.0f, 0.0f, 120.0f, 1.0f, bikeStateCounter3);


			steeringPercent = 0.0f;
//			steeringPercent = 0.4f;
//			turnPercent = (f32)(bikeStateCounter[bikeState]) / 60.0f;
//			turnPercent = MIN(1.0f, turnPercent);
			turnStep = 0.0f;
//			turnStep = -POWERSLIDE_TURN_STEP * turnPercent;
			leanPercent = 1.25f * velocityPercent;
			leanPercent = MIN(1.0f, leanPercent);
			slideAngVel = 6.0f * tFriction.ps.ang * velocityPercent * getSlopePoint(0.0f, 0.35f, 30.0f, 0.5f, 60.0f, 0.5f, bikeStateCounter3);
//			slideAngVel = 5.5f * velocityPercent * getSlopePoint(0.0f, 0.35f, 30.0f, 0.5f, 60.0f, 0.5f, bikeStateCounter3);
			throttleStep = 0.01f;
			brakeStep = 0.0f;
			turnDirSign = 1.0f;
			maxLeanRot = 75.0f;//80.0f;
			turnAnglePercent[FW] = 2.0f;
			turnAnglePercent[BW] = -0.5f;


			((CBikeSteering*)m_steering)->setDoesWheelSteer(BW, TRUE);
	
//			if (onGround)
			if (onGroundPowerSlide || onGround)
//			if (wheelInfo[FW].getIsInContact() && wheelInfo[BW].getIsInContact())
			{
				sxVector3d force = fForwardDir * 5000.0f * tFriction.ps.force * pressedPercent * x_sqrt(velocityPercent) * MIN(1.0f, (f32)(bikeStateCounter3) / 40.0f);
				Vector3 forceV3 = SXV_TO_V3(force);
				chassis->applyImpulse(forceV3);

				sxVector3d newDir = (-upDir) + (-rightDir);
				force = newDir * 5000.0f * tFriction.ps.force * pressedPercent * x_sqrt(velocityPercent) * MIN(1.0f, (f32)(bikeStateCounter3) / 40.0f);
				forceV3 = SXV_TO_V3(force);
				chassis->applyImpulse(forceV3);


				if (linearVelocityMag > eventStartVelocityMag)
				{
					Vector3 lVel = chassis->getLinearVelocity();
					lVel = lVel * eventStartVelocityMag / linearVelocityMag;
					chassis->setLinearVelocity(lVel);
				}
			}

//			if (!movingForward)
//			{
//				Vector3 lVel = chassis->getLinearVelocity();
//				lVel = lVel * 0.90f;
//				chassis->setLinearVelocity(lVel);
//			}

			if (!movingForward)
			{
				Vector3 lVelV3 = chassis->getLinearVelocity();
				sxVector3d lVel = V3_TO_SXV(lVelV3);

				sxVector3d vel;
				vel.x = lVel * rightDir;
				vel.y = lVel * upDir;
				vel.z = lVel * forwardDir;

				lVel = (rightDir * vel.x * 1.0f) + 
						(forwardDir * vel.z * 0.25f) + 
						(upDir * vel.y * 0.95f);

				lVelV3 = SXV_TO_V3(lVel);
				chassis->setLinearVelocity(lVelV3);
			}



			powerSlideAccelBoostEligible = TRUE;
			bikeStateCounter2 = 0;
			bikeStateCounter3++;
		}
		else
		{
			powerSlideEnd = TRUE;
			setFlag(BIKE_POWERSLIDE_LEFT);
//			constraint->setTau(TURN_TAU);
			constraint->setTau(0.0005f);
			downForceMag = 90000.0f;
			friction[FW] = tFriction.pse.normal * 0.25f;
			friction[BW] = tFriction.pse.normal * 0.25f;

			turnPercent = 1.0f;
			steeringPercent = 0.28f;
			turnScale = 1.5f;
			turnScale2 = 1.6f;
			turnStep = POWERSLIDE_TURN_STEP * turnPercent;

			if (linearVelocityMag > TO_METERS_PER_SEC(15.0f))
			{
				slideAngVel = 1.45f * tFriction.pse.ang * pressedPercent * turnScale * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
//				slideAngVel = 1.45f * pressedPercent * turnScale * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
				leanPercent = 1.0f * pressedPercent * turnScale2 * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
//				powerSlideAccelBoostEligible = TRUE;
			}
			else
			{
				slideAngVel = 0.0f;
				leanPercent = 0.0f;
			}

			if (!movingForward)
			{
				Vector3 lVel = chassis->getLinearVelocity();
				lVel = lVel * 0.98f;
				chassis->setLinearVelocity(lVel);
			}

			turnDirSign = 1.0f;
			maxLeanRot = 40.0f;
			turnAnglePercent[FW] = 1.0f;
			turnAnglePercent[BW] = -0.5f;

			if (checkMessage(BIKEACTION_ACCEL) && acceleration < 0)
				((CBikeEngine*)m_engine)->scaleEngineBoost(1.0f + (boostPercent * 3.0f));


			throttleStep = 0.0f;
			brakeStep = 0.0f;

			bikeStateCounter3 = 0.0f;
			bikeStateCounter2++;

		}
		break;

	case BIKESTATE_RIGHT_POWERSLIDE_ACCEL:
		powerSlideAccelBoostVelocityMag = eventStartVelocityMag;
		powerSlideAccelBoostEligible = FALSE;
		pitchDampening = 0.5f;
		comShiftState = COMSS_BACK;

		powerSlideCircle = (270.0f) * (eventStartVelocityMag / TO_METERS_PER_SEC(75.0f));
		startTurnCondition = TRUE;
		if (turnCounter > 18)
			startTurnCondition = FALSE;

		if (startTurnCondition && !powerSlideLeftGround)
		{
			if (pressedPercent < (f32)turnCounter / 18.0f)
				pressedPercent = MAX(1.0f, (f32)turnCounter / 18.0f);

			setFlag(BIKE_RIGHT_TURN);
//			constraint->setTau(TURN_TAU);
			constraint->setTau(0.0005f);
			downForceMag = 90000.0f;
			friction[FW] = tFriction.turn.normal * 2.0f;
			friction[BW] = tFriction.turn.normal * 2.0f;
	
			turnPercent = (f32)(bikeStateCounter[bikeState]) / 60.0f;
			turnPercent = MIN(1.0f, turnPercent);
			steeringPercent = 0.28f;
			turnStep = POWERSLIDE_TURN_STEP * turnPercent;
			leanPercent = 1.0f * pressedPercent * velocityPercent;
			slideAngVel = -1.45f * pressedPercent;
			turnDirSign = -1.0f;
			maxLeanRot = 50.0f;//50.0f
			turnAnglePercent[FW] = 1.0f;
			turnAnglePercent[BW] = -0.5f;
	
			if (checkMessage(BIKEACTION_ACCEL) && acceleration < 0)
				((CBikeEngine*)m_engine)->scaleEngineBoost(1.0f + (boostPercent * 1.0f));
	
			throttleStep = 0.01f;
			brakeStep = 0.0f;

			bikeStateCounter3 = 0.0f;
			powerSlideAccelBoostEligible = TRUE;
		}
		else if (eventDeltaAngle < powerSlideCircle && !powerSlideLeftGround && powerSlideBrakeTurnCounter < 18 && !powerSlideEnd)
		{			
			setFlag(BIKE_POWERSLIDE_RIGHT);
//			constraint->setTau(POWERSLIDE_TAU);
			constraint->setTau(0.0005f);
			downForceMag = 90000.0f;

			friction[FW] = tFriction.ps.normal * 2.0f * getSlopePoint(0.0f, 0.0f, 60.0f, 0.0f, 120.0f, 1.0f, bikeStateCounter3);
			friction[BW] = tFriction.ps.normal * 2.0f * getSlopePoint(0.0f, 0.0f, 60.0f, 0.0f, 120.0f, 1.0f, bikeStateCounter3);


			steeringPercent = 0.0f;
			turnStep = 0.0f;
			leanPercent = 1.25f * velocityPercent;
			leanPercent = MIN(1.0f, leanPercent);
			slideAngVel = -6.0f * tFriction.ps.ang * velocityPercent * getSlopePoint(0.0f, 0.35f, 30.0f, 0.5f, 60.0f, 0.5f, bikeStateCounter3);
			throttleStep = 0.01f;
			brakeStep = 0.0f;
			turnDirSign = -1.0f;
			maxLeanRot = 75.0f;//80.0f;
			turnAnglePercent[FW] = 1.0f;
			turnAnglePercent[BW] = 1.0f;


			((CBikeSteering*)m_steering)->setDoesWheelSteer(BW, TRUE);
	
//			if (onGround)
			if (onGroundPowerSlide || onGround)
//			if (wheelInfo[FW].getIsInContact() && wheelInfo[BW].getIsInContact())
			{
				sxVector3d force = fForwardDir * 5000.0f * tFriction.ps.force * pressedPercent * x_sqrt(velocityPercent) * MIN(1.0f, (f32)(bikeStateCounter3) / 40.0f);
				Vector3 forceV3 = SXV_TO_V3(force);
				chassis->applyImpulse(forceV3);

				sxVector3d newDir = (-upDir) + rightDir;
				force = newDir * 5000.0f * tFriction.ps.force * pressedPercent * x_sqrt(velocityPercent) * MIN(1.0f, (f32)(bikeStateCounter3) / 40.0f);
				forceV3 = SXV_TO_V3(force);
				chassis->applyImpulse(forceV3);

				if (linearVelocityMag > eventStartVelocityMag)
				{
					Vector3 lVel = chassis->getLinearVelocity();
					lVel = lVel * eventStartVelocityMag / linearVelocityMag;
					chassis->setLinearVelocity(lVel);
				}
			}

//			if (!movingForward)
//			{
//				Vector3 lVel = chassis->getLinearVelocity();
//				lVel = lVel * 0.90f;
//				chassis->setLinearVelocity(lVel);
//			}
			if (!movingForward)
			{
				Vector3 lVelV3 = chassis->getLinearVelocity();
				sxVector3d lVel = V3_TO_SXV(lVelV3);

				sxVector3d vel;
				vel.x = lVel * rightDir;
				vel.y = lVel * upDir;
				vel.z = lVel * forwardDir;

				lVel = (rightDir * vel.x * 1.0f) + 
						(forwardDir * vel.z * 0.25f) + 
						(upDir * vel.y * 0.95f);

				lVelV3 = SXV_TO_V3(lVel);
				chassis->setLinearVelocity(lVelV3);
			}





			powerSlideAccelBoostEligible = TRUE;
			bikeStateCounter2 = 0;
			bikeStateCounter3++;
		}
		else
		{
			powerSlideEnd = TRUE;
			setFlag(BIKE_POWERSLIDE_RIGHT);
//			constraint->setTau(TURN_TAU);
			constraint->setTau(0.0005f);
			downForceMag = 90000.0f;
			friction[FW] = tFriction.pse.normal * 0.25f;
			friction[BW] = tFriction.pse.normal * 0.25f;

			turnPercent = 1.0f;
			steeringPercent = 0.28f;
			turnScale = 1.5f;
			turnScale2 = 1.6f;
			turnStep = POWERSLIDE_TURN_STEP * turnPercent;

			if (linearVelocityMag > TO_METERS_PER_SEC(15.0f))
			{
				slideAngVel = -1.45f * tFriction.pse.ang * pressedPercent * turnScale * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
				leanPercent = 1.0f * pressedPercent * turnScale2 * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
//				powerSlideAccelBoostEligible = TRUE;
			}
			else
			{
				slideAngVel = 0.0f;
				leanPercent = 0.0f;
			}

			if (!movingForward)
			{
				Vector3 lVel = chassis->getLinearVelocity();
				lVel = lVel * 0.98f;
				chassis->setLinearVelocity(lVel);
			}

			turnDirSign = -1.0f;
			maxLeanRot = 40.0f;
			turnAnglePercent[FW] = 1.0f;
			turnAnglePercent[BW] = -0.5f;

			if (checkMessage(BIKEACTION_ACCEL) && acceleration < 0)
				((CBikeEngine*)m_engine)->scaleEngineBoost(1.0f + (boostPercent * 3.0f));


			throttleStep = 0.0f;
			brakeStep = 0.0f;

			bikeStateCounter3 = 0.0f;
			bikeStateCounter2++;

		}
		break;


	case BIKESTATE_LEFT_POWERSLIDE_BRAKE:
		powerSlideBrakeBoostVelocityMag = eventStartVelocityMag;
		powerSlideBrakeBoostEligible = FALSE;
		pitchDampening = 0.5f;
		comShiftState = COMSS_BACK;
		powerSlideCircle = (200.0f) * (eventStartVelocityMag / TO_METERS_PER_SEC(75.0f));

		startTurnCondition = TRUE;
		if (turnCounter < -18)
			startTurnCondition = FALSE;

		if (startTurnCondition && !powerSlideLeftGround)
		{
			if (pressedPercent < (f32)-turnCounter / 18.0f)
				pressedPercent = MAX(1.0f, (f32)-turnCounter / 18.0f);

			setFlag(BIKE_LEFT_TURN);
//			constraint->setTau(TURN_TAU);
			constraint->setTau(0.0005f);
			downForceMag = 90000.0f;
			friction[FW] = tFriction.turn.normal * 2.0f;
			friction[BW] = tFriction.turn.normal * 2.0f;
	
			turnPercent = (f32)(bikeStateCounter[bikeState]) / 60.0f;
			turnPercent = MIN(1.0f, turnPercent);
			steeringPercent = 0.28f;
			turnStep = -POWERSLIDE_TURN_STEP * turnPercent;
			leanPercent = 1.0f * pressedPercent * velocityPercent;
			slideAngVel = 1.45f * pressedPercent;
			turnDirSign = 1.0f;
			maxLeanRot = 50.0f;
			turnAnglePercent[FW] = 1.0f;
			turnAnglePercent[BW] = -0.5f;
	
			if (checkMessage(BIKEACTION_ACCEL) && acceleration < 0)
				((CBikeEngine*)m_engine)->scaleEngineBoost(1.0f + (boostPercent * 1.0f));
	
			throttleStep = 0.01f;
			brakeStep = 0.0f;

			bikeStateCounter3 = 0.0f;
			powerSlideBrakeBoostEligible = TRUE;
		}
//		else if (eventDeltaAngle < powerSlideCircle && !powerSlideLeftGround)
//		else if (eventDeltaAngle < powerSlideCircle && !powerSlideLeftGround && powerSlideAccelTurnCounter > -18 && !powerSlideEnd)
		else if (eventDeltaAngle < powerSlideCircle && !powerSlideLeftGround && !powerSlideEnd)
		{			
			setFlag(BIKE_POWERSLIDE_LEFT);
//			constraint->setTau(POWERSLIDE_TAU);
			constraint->setTau(0.0005f);
			downForceMag = 90000.0f;

//			friction[FW] = 0.0f;
//			friction[BW] = 0.0f;

			friction[FW] = tFriction.psb.normal * 3.0f * getSlopePoint(0.0f, 0.0f, 40.0f, 0.0f, 80.0f, 1.0f, bikeStateCounter3);
			friction[BW] = tFriction.psb.normal * 3.0f * getSlopePoint(0.0f, 0.0f, 40.0f, 0.0f, 80.0f, 1.0f, bikeStateCounter3);


			steeringPercent = 0.0f;
			turnStep = 0.0f;
			leanPercent = 1.25f * velocityPercent;
			leanPercent = MIN(1.0f, leanPercent);
//			slideAngVel = -6.5f * velocityPercent * getSlopePoint(0.0f, 0.35f, 30.0f, 0.5f, 60.0f, 1.0f, bikeStateCounter3);
			slideAngVel = 8.0f * tFriction.psb.ang * psBrakeVelocityPercent * getSlopePoint(0.0f, 0.35f, 20.0f, 0.7f, 60.0f, 1.0f, bikeStateCounter3);
			throttleStep = 0.01f;
			brakeStep = 0.0f;
			turnDirSign = 1.0f;
			maxLeanRot = 75.0f;//80.0f;
			turnAnglePercent[FW] = 0.0f;
			turnAnglePercent[BW] = 0.0f;


			((CBikeSteering*)m_steering)->setDoesWheelSteer(BW, TRUE);



			if (onGround)
//			if (onGroundPowerSlide || onGround)
//			if (wheelInfo[FW].getIsInContact() && wheelInfo[BW].getIsInContact())
			{
				sxVector3d force = fForwardDir * 5000.0f * tFriction.psb.force * pressedPercent * x_sqrt(velocityPercent) * MIN(1.0f, (f32)(bikeStateCounter3) / 40.0f);
				Vector3 forceV3 = SXV_TO_V3(force);
				chassis->applyImpulse(forceV3);

				if (linearVelocityMag > eventStartVelocityMag)
				{
					Vector3 lVel = chassis->getLinearVelocity();
					lVel = lVel * 0.85f * eventStartVelocityMag / linearVelocityMag;
					chassis->setLinearVelocity(lVel);
				}
			}

			if (!movingForward)
			{
				Vector3 lVelV3 = chassis->getLinearVelocity();
				sxVector3d lVel = V3_TO_SXV(lVelV3);

				sxVector3d vel;
				vel.x = lVel * rightDir;
				vel.y = lVel * upDir;
				vel.z = lVel * forwardDir;

				lVel = (rightDir * vel.x * 1.00f) + 
						(forwardDir * vel.z * 0.5f) + 
						(upDir * vel.y * 0.95f);

				lVelV3 = SXV_TO_V3(lVel);
				chassis->setLinearVelocity(lVelV3);
			}


			powerSlideBrakeBoostEligible = TRUE;
			bikeStateCounter2 = 0;
			bikeStateCounter3++;
		}
		else
		{
			powerSlideEnd = TRUE;
			setFlag(BIKE_POWERSLIDE_LEFT);
//			constraint->setTau(TURN_TAU);
			constraint->setTau(0.0005f);
			downForceMag = 90000.0f;
			friction[FW] = tFriction.pse.normal * 0.25f;
			friction[BW] = tFriction.pse.normal * 0.25f;

			turnPercent = 1.0f;
			steeringPercent = 0.28f;
			turnScale = 1.5f;
			turnScale2 = 1.6f;
			turnStep = -POWERSLIDE_TURN_STEP * turnPercent;

			if (linearVelocityMag > TO_METERS_PER_SEC(15.0f))
			{
				slideAngVel = 1.45f * tFriction.pse.ang * pressedPercent * turnScale * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
				leanPercent = 1.0f * pressedPercent * turnScale2 * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
//				powerSlideBrakeBoostEligible = TRUE;
			}
			else
			{
				slideAngVel = 0.0f;
				leanPercent = 0.0f;
			}

			if (!movingForward)
			{
				Vector3 lVel = chassis->getLinearVelocity();
				lVel = lVel * 0.98f;
				chassis->setLinearVelocity(lVel);
			}


			turnDirSign = 1.0f;
			maxLeanRot = 40.0f;
			turnAnglePercent[FW] = 1.0f;
			turnAnglePercent[BW] = -0.5f;

			if (checkMessage(BIKEACTION_ACCEL) && acceleration < 0)
				((CBikeEngine*)m_engine)->scaleEngineBoost(1.0f + (boostPercent * 3.0f));


			throttleStep = 0.0f;
			brakeStep = 0.0f;

			bikeStateCounter3 = 0.0f;
			bikeStateCounter2++;


			if (bikeStateCounter2 < 40.0f)
				powerSlideBrakeBoostEligible = TRUE;

		}
		break;

	case BIKESTATE_RIGHT_POWERSLIDE_BRAKE:

		powerSlideBrakeBoostVelocityMag = eventStartVelocityMag;
		powerSlideBrakeBoostEligible = FALSE;
		pitchDampening = 0.5f;
		comShiftState = COMSS_BACK;
		powerSlideCircle = (200.0f) * (eventStartVelocityMag / TO_METERS_PER_SEC(75.0f));

		startTurnCondition = TRUE;
		if (turnCounter > 18)
			startTurnCondition = FALSE;

		if (startTurnCondition && !powerSlideLeftGround)
		{
			if (pressedPercent < (f32)turnCounter / 18.0f)
				pressedPercent = MAX(1.0f, (f32)turnCounter / 18.0f);

			setFlag(BIKE_RIGHT_TURN);
//			constraint->setTau(TURN_TAU);
			constraint->setTau(0.0005f);
			downForceMag = 90000.0f;
			friction[FW] = tFriction.turn.normal * 2.0f;
			friction[BW] = tFriction.turn.normal * 2.0f;
	
			turnPercent = (f32)(bikeStateCounter[bikeState]) / 60.0f;
			turnPercent = MIN(1.0f, turnPercent);
			steeringPercent = 0.28f;
			turnStep = POWERSLIDE_TURN_STEP * turnPercent;
			leanPercent = 1.0f * pressedPercent * velocityPercent;
			slideAngVel = -1.45f * pressedPercent;
			turnDirSign = -1.0f;
			maxLeanRot = 50.0f;
			turnAnglePercent[FW] = 1.0f;
			turnAnglePercent[BW] = -0.5f;
	
			if (checkMessage(BIKEACTION_ACCEL) && acceleration < 0)
				((CBikeEngine*)m_engine)->scaleEngineBoost(1.0f + (boostPercent * 1.0f));
	
			throttleStep = 0.01f;
			brakeStep = 0.0f;

			bikeStateCounter3 = 0.0f;
			powerSlideBrakeBoostEligible = TRUE;
		}
//		else if (eventDeltaAngle < powerSlideCircle && !powerSlideLeftGround)
//		else if (eventDeltaAngle < powerSlideCircle && !powerSlideLeftGround && powerSlideAccelTurnCounter > -18 && !powerSlideEnd)
		else if (eventDeltaAngle < powerSlideCircle && !powerSlideLeftGround && !powerSlideEnd)
		{			
			setFlag(BIKE_POWERSLIDE_RIGHT);
//			constraint->setTau(POWERSLIDE_TAU);
			constraint->setTau(0.0005f);
			downForceMag = 90000.0f;

//			friction[FW] = 0.0f;
//			friction[BW] = 0.0f;

			friction[FW] = tFriction.psb.normal * 3.0f * getSlopePoint(0.0f, 0.0f, 40.0f, 0.0f, 80.0f, 1.0f, bikeStateCounter3);
			friction[BW] = tFriction.psb.normal * 3.0f * getSlopePoint(0.0f, 0.0f, 40.0f, 0.0f, 80.0f, 1.0f, bikeStateCounter3);


			steeringPercent = 0.0f;
			turnStep = 0.0f;
			leanPercent = 1.25f * velocityPercent;
			leanPercent = MIN(1.0f, leanPercent);
//			slideAngVel = -6.5f * velocityPercent * getSlopePoint(0.0f, 0.35f, 30.0f, 0.5f, 60.0f, 1.0f, bikeStateCounter3);
			slideAngVel = -8.0f * tFriction.psb.ang * psBrakeVelocityPercent * getSlopePoint(0.0f, 0.35f, 20.0f, 0.7f, 60.0f, 1.0f, bikeStateCounter3);
			throttleStep = 0.01f;
			brakeStep = 0.0f;
			turnDirSign = -1.0f;
			maxLeanRot = 75.0f;//80.0f;
			turnAnglePercent[FW] = 0.0f;
			turnAnglePercent[BW] = 0.0f;


			((CBikeSteering*)m_steering)->setDoesWheelSteer(BW, TRUE);



			if (onGround)
//			if (onGroundPowerSlide || onGround)
//			if (wheelInfo[FW].getIsInContact() && wheelInfo[BW].getIsInContact())
			{
				sxVector3d force = fForwardDir * 5000.0f * tFriction.psb.force * pressedPercent * x_sqrt(velocityPercent) * MIN(1.0f, (f32)(bikeStateCounter3) / 40.0f);
				Vector3 forceV3 = SXV_TO_V3(force);
				chassis->applyImpulse(forceV3);

				if (linearVelocityMag > eventStartVelocityMag)
				{
					Vector3 lVel = chassis->getLinearVelocity();
					lVel = lVel * 0.85f * eventStartVelocityMag / linearVelocityMag;
					chassis->setLinearVelocity(lVel);
				}
			}

			if (!movingForward)
			{
				Vector3 lVelV3 = chassis->getLinearVelocity();
				sxVector3d lVel = V3_TO_SXV(lVelV3);

				sxVector3d vel;
				vel.x = lVel * rightDir;
				vel.y = lVel * upDir;
				vel.z = lVel * forwardDir;

				lVel = (rightDir * vel.x * 1.00f) + 
						(forwardDir * vel.z * 0.5f) + 
						(upDir * vel.y * 0.95f);

				lVelV3 = SXV_TO_V3(lVel);
				chassis->setLinearVelocity(lVelV3);
			}


			powerSlideBrakeBoostEligible = TRUE;
			bikeStateCounter2 = 0;
			bikeStateCounter3++;
		}
		else
		{
			powerSlideEnd = TRUE;
			setFlag(BIKE_POWERSLIDE_RIGHT);
//			constraint->setTau(TURN_TAU);
			constraint->setTau(0.0005f);
			downForceMag = 90000.0f;
			friction[FW] = tFriction.pse.normal * 0.25f;
			friction[BW] = tFriction.pse.normal * 0.25f;

			turnPercent = 1.0f;
			steeringPercent = 0.28f;
			turnScale = 1.5f;
			turnScale2 = 1.6f;
			turnStep = POWERSLIDE_TURN_STEP * turnPercent;

			if (linearVelocityMag > TO_METERS_PER_SEC(15.0f))
			{
				slideAngVel = -1.45f * tFriction.pse.ang * pressedPercent * turnScale * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
				leanPercent = 1.0f * pressedPercent * turnScale2 * getSlopePoint(0.0f, 0.75, TO_METERS_PER_SEC(40.0f), 0.45f, TO_METERS_PER_SEC(65.0f), 1.0f, linearVelocityMag);
//				powerSlideBrakeBoostEligible = TRUE;
			}
			else
			{
				slideAngVel = 0.0f;
				leanPercent = 0.0f;
			}

			if (!movingForward)
			{
				Vector3 lVel = chassis->getLinearVelocity();
				lVel = lVel * 0.98f;
				chassis->setLinearVelocity(lVel);
			}


			turnDirSign = -1.0f;
			maxLeanRot = 40.0f;
			turnAnglePercent[FW] = 1.0f;
			turnAnglePercent[BW] = -0.5f;

			if (checkMessage(BIKEACTION_ACCEL) && acceleration < 0)
				((CBikeEngine*)m_engine)->scaleEngineBoost(1.0f + (boostPercent * 3.0f));


			throttleStep = 0.0f;
			brakeStep = 0.0f;

			bikeStateCounter3 = 0.0f;
			bikeStateCounter2++;


			if (bikeStateCounter2 < 40.0f)
				powerSlideBrakeBoostEligible = TRUE;

		}
		break;


	case BIKESTATE_NO_TURN_NO_ACCEL:
//		constraint->setTau(TURN_TAU);
		constraint->setTau(0.0005f);
		downForceMag = 90000.0f;
		friction[FW] = tFriction.drive.coast * 2.0f;
		friction[BW] = tFriction.drive.coast * 2.0f;

		steeringPercent = 0.0f;
		turnStep = turnStep * TURN_STEP_DETERIORATE;
		leanPercent = 1.0f;
		slideAngVel = 0.0f;
		turnDirSign = 0.0f;
		maxLeanRot = 40.0f;
		turnAnglePercent[FW] = 1.0f;
		turnAnglePercent[BW] = -0.5f;

		((CBikeSteering*)m_steering)->setDoesWheelSteer(BW, FALSE);

		throttleStep = 0.0f;
		brakeStep = 0.0f;
		break;

	case BIKESTATE_NO_TURN_ACCEL:
//		constraint->setTau(TURN_TAU);
		constraint->setTau(0.0005f);
		downForceMag = 90000.0f;
		friction[FW] = tFriction.drive.accel * 2.0f;
		friction[BW] = tFriction.drive.accel * 2.0f;

		steeringPercent = 0.0f;
		turnStep = turnStep * TURN_STEP_DETERIORATE;
		leanPercent = 1.0f;
		slideAngVel = 0.0f;
		turnDirSign = 0.0f;
		maxLeanRot = 40.0f;
		turnAnglePercent[FW] = 1.0f;
		turnAnglePercent[BW] = -0.5f;

		((CBikeSteering*)m_steering)->setDoesWheelSteer(BW, FALSE);

		throttleStep = 0.01f;
		brakeStep = 0.0f;
		eligibleBoostState = TRUE;
		break;

	case BIKESTATE_NO_TURN_BRAKE:
//		constraint->setTau(TURN_TAU);
		constraint->setTau(0.0005f);
		downForceMag = 90000.0f;
		friction[FW] = tFriction.drive.brake * 2.0f;
		friction[BW] = tFriction.drive.brake * 2.0f;

		steeringPercent = 0.0f;
		turnStep = turnStep * TURN_STEP_DETERIORATE;
		leanPercent = 1.0f;
		slideAngVel = 0.0f;
		turnDirSign = 0.0f;
		maxLeanRot = 40.0f;
		turnAnglePercent[FW] = 1.0f;
		turnAnglePercent[BW] = -0.5f;

		((CBikeSteering*)m_steering)->setDoesWheelSteer(BW, FALSE);

		throttleStep = 0.0f;
		brakeStep = 0.01f;
		break;

	case BIKESTATE_LEFT_WHEELIE:
		if (wheelieRequested)
			comShiftState = COMSS_BACK;
		else
			comShiftState = COMSS_FORWARD;

//		if (wheelieRequested)
//			setFlag(BIKE_WHEELIE);

		setFlag(BIKE_LEFT_TURN);
//		constraint->setTau(TURN_TAU);
		constraint->setTau(0.0005f);
		downForceMag = 0.0f;//90000.0f;
		friction[FW] = tFriction.drive.accel * 1.0f;
		friction[BW] = tFriction.drive.accel * 1.0f;


		turnPercent = (f32)(bikeStateCounter[bikeState]) / 60.0f;
		turnPercent = MIN(1.0f, turnPercent);
		steeringPercent = 0.010f;
		turnStep = -0.0125f * turnPercent;
		leanPercent = 0.5f * pressedPercent * velocityPercent;
		slideAngVel = 0.3f * pressedPercent;//0.85f * pressedPercent;
		turnDirSign = 1.0f;
		maxLeanRot = 40.0f;
		turnAnglePercent[FW] = 0.5f;
		turnAnglePercent[BW] = 0.0f;

		((CBikeEngine*)m_engine)->scaleEngineBoost(0.25f);
		((CBikeSteering*)m_steering)->setDoesWheelSteer(BW, FALSE);

		throttleStep = 0.0001f;

		break;
	case BIKESTATE_RIGHT_WHEELIE:
		if (wheelieRequested)
			comShiftState = COMSS_BACK;
		else
			comShiftState = COMSS_FORWARD;

//		if (wheelieRequested)
//			setFlag(BIKE_WHEELIE);

		setFlag(BIKE_RIGHT_TURN);
//		constraint->setTau(TURN_TAU);
		constraint->setTau(0.0005f);
		downForceMag = 0.0f;//90000.0f;
		friction[FW] = tFriction.drive.accel * 1.0f;
		friction[BW] = tFriction.drive.accel * 1.0f;


		turnPercent = (f32)(bikeStateCounter[bikeState]) / 60.0f;
		turnPercent = MIN(1.0f, turnPercent);
		steeringPercent = 0.010f;
		turnStep = 0.0125f * turnPercent;
		leanPercent = 0.5f * pressedPercent * velocityPercent;
		slideAngVel = -0.3f * pressedPercent;
		turnDirSign = -1.0f;
		maxLeanRot = 40.0f;
		turnAnglePercent[FW] = 0.5f;
		turnAnglePercent[BW] = 0.5f;

		((CBikeEngine*)m_engine)->scaleEngineBoost(0.25f);
		((CBikeSteering*)m_steering)->setDoesWheelSteer(BW, FALSE);

		throttleStep = 0.0001f;
		break;
	case BIKESTATE_WHEELIE:
		if (wheelieRequested)
			comShiftState = COMSS_BACK;
		else
			comShiftState = COMSS_FORWARD;

//		if (wheelieRequested)
//			setFlag(BIKE_WHEELIE);

//		constraint->setTau(TURN_TAU);
		constraint->setTau(0.0005f);
		downForceMag = 0.0f;//90000.0f;
		friction[FW] = tFriction.drive.accel * 1.0f;
		friction[BW] = tFriction.drive.accel * 1.0f;

		steeringPercent = 0.0f;
		turnStep = turnStep * TURN_STEP_DETERIORATE;
		leanPercent = 1.0f;
		slideAngVel = 0.0f;
		turnDirSign = 0.0f;
		maxLeanRot = 40.0f;
		turnAnglePercent[FW] = 0.0f;
		turnAnglePercent[BW] = 0.0f;

		((CBikeSteering*)m_steering)->setDoesWheelSteer(BW, FALSE);

		throttleStep = 0.01f;
		brakeStep = 0.0f;
		break;

	case BIKESTATE_LEFT_STOPPIE:
	case BIKESTATE_RIGHT_STOPPIE:
	case BIKESTATE_STOPPIE:
		if (linearVelocityMag > TO_METERS_PER_SEC(5.0f))
			comShiftState = COMSS_FORWARD;
		else
			comShiftState = COMSS_BACK;

		setFlag(BIKE_STOPPIE);
//		constraint->setTau(TURN_TAU);
		constraint->setTau(0.0005f);
		downForceMag = 0.0f;//90000.0f;
		friction[FW] = 6.0f;
		friction[BW] = 6.0f;

		steeringPercent = 0.0f;
		turnStep = turnStep * TURN_STEP_DETERIORATE;
		leanPercent = 1.0f;
		slideAngVel = 0.0f;
		turnDirSign = 0.0f;
		maxLeanRot = 40.0f;
		turnAnglePercent[FW] = 0.0f;
		turnAnglePercent[BW] = 0.0f;

		((CBikeSteering*)m_steering)->setDoesWheelSteer(BW, FALSE);

		throttleStep = 0.0f;
		brakeStep = 0.01f;
		break;

	case BIKESTATE_INAIR:
		constraint->setTau(0.0005f);
		turnPercent = (f32)(bikeStateCounter[bikeState]) / SECS_TO_FRAMES(3.0f);
		turnPercent = 1.0f - turnPercent;
		turnPercent = MIN(1.0f, turnPercent);
		turnPercent = MAX(0.25f, turnPercent);


		slideAngVel = 0.0f;

		if (checkMessage(BIKEACTION_ACCEL))
			throttleStep = 0.01f;
		else
			throttleStep = 0.0f;

		if (checkMessage(BIKEACTION_BRAKE))
			brakeStep = 0.01f;
		else
			brakeStep = 0.0f;


		if (((checkMessage(BIKEACTION_BRAKE) && !checkMessage(BIKEACTION_BACK)) || checkMessage(BIKEACTION_FORWARD)) && !inStunt)
		{
			if (checkMessage(BIKEACTION_FORWARD))
			{
				pressedPercent = pressed[BIKEACTION_FORWARD] / MAX_PITCH_BUTTON_PRESSED;
				pressedPercent = MIN(1.0f, pressedPercent);
			}
			else
			{
				pressedPercent = 0.25f;
			}

			if (bikePitch > -65.0f)
			{
				sxVector3d torque = fRightDir * 0.5f * -35000.0f * pressedPercent;
				Vector3 torqueV3 = SXV_TO_V3(torque);
				chassis->applyTorque(torqueV3);
			}
		}
		else if (((checkMessage(BIKEACTION_ACCEL) && !checkMessage(BIKEACTION_FORWARD)) || checkMessage(BIKEACTION_BACK)) && !inStunt)
		{
			if (checkMessage(BIKEACTION_BACK))
			{
				pressedPercent = pressed[BIKEACTION_BACK] / MAX_PITCH_BUTTON_PRESSED;
				pressedPercent = MIN(1.0f, pressedPercent);
			}
			else
			{
				pressedPercent = 0.25f;
			}
			if (bikePitch < 90.0f)
			{
				sxVector3d torque = fRightDir * 0.5f * 35000.0f * pressedPercent;
				Vector3 torqueV3 = SXV_TO_V3(torque);
				chassis->applyTorque(torqueV3);
			}
		}



		if ((checkMessage(BIKEACTION_LEFT_TURN) || 
			 checkMessage(BIKEACTION_LEFT_POWERSLIDE_ACCEL) ||
			 checkMessage(BIKEACTION_LEFT_POWERSLIDE_BRAKE)) && !inStunt)
		{
			if (checkMessage(BIKEACTION_LEFT_TURN))
				pressedPercent = pressed[BIKEACTION_LEFT_TURN] / MAX_ROLL_BUTTON_PRESSED;
			else if (checkMessage(BIKEACTION_LEFT_POWERSLIDE_ACCEL))
				pressedPercent = pressed[BIKEACTION_LEFT_POWERSLIDE_ACCEL] / MAX_ROLL_BUTTON_PRESSED;
			else 
				pressedPercent = pressed[BIKEACTION_LEFT_POWERSLIDE_BRAKE] / MAX_ROLL_BUTTON_PRESSED;

			pressedPercent = MIN(1.0f, pressedPercent);

			sxVector3d torque = fUpDir * 3500.0f * pressedPercent;
			Vector3 torqueV3 = SXV_TO_V3(torque);
			chassis->applyTorque(torqueV3);

			sxVector3d force = fRightDir * -90.0f * pressedPercent * turnPercent;
			Vector3 forceV3 = SXV_TO_V3(force);
			chassis->applyImpulse(forceV3);

			constraint->setAngle(DEG_TO_RAD(30.0f * pressedPercent));
			constraint->setTau(0.75f);
		}
		else if ((checkMessage(BIKEACTION_RIGHT_TURN) || 
				  checkMessage(BIKEACTION_RIGHT_POWERSLIDE_ACCEL) ||
				  checkMessage(BIKEACTION_RIGHT_POWERSLIDE_BRAKE)) && !inStunt)
		{
			if (checkMessage(BIKEACTION_RIGHT_TURN))
				pressedPercent = pressed[BIKEACTION_RIGHT_TURN] / MAX_ROLL_BUTTON_PRESSED;
			else if (checkMessage(BIKEACTION_RIGHT_POWERSLIDE_ACCEL))
				pressedPercent = pressed[BIKEACTION_RIGHT_POWERSLIDE_ACCEL] / MAX_ROLL_BUTTON_PRESSED;
			else 
				pressedPercent = pressed[BIKEACTION_RIGHT_POWERSLIDE_BRAKE] / MAX_ROLL_BUTTON_PRESSED;

			pressedPercent = MIN(1.0f, pressedPercent);

			sxVector3d torque = fUpDir * -3500.0f * pressedPercent;
			Vector3 torqueV3 = SXV_TO_V3(torque);
			chassis->applyTorque(torqueV3);

			sxVector3d force = fRightDir * 90.0f * pressedPercent * turnPercent;
			Vector3 forceV3 = SXV_TO_V3(force);
			chassis->applyImpulse(forceV3);

			constraint->setAngle(DEG_TO_RAD(-30.0f * pressedPercent));
			constraint->setTau(0.75f);
		}
		else
		{

/*			if (preLoadJumpState != PRELOAD_DURING_WINDOW && 
				preLoadReportState != PRELOAD_SUCCESSFUL)
			{
				f32 velScale = 1.0f - (bikeStateCounter[bikeState] * (1.0f / 1000.0f));
				velScale = MAX(0.50f, velScale);
				sxVector3d newVel = linearVelocity;
				newVel.normalize();
				newVel = newVel * eventStartVelocityMag * velScale;
				Vector3 velV3 = SXV_TO_V3(newVel);
//				chassis->setLinearVelocity(velV3);
			}
			else
			{
*/
				eventStartVelocityMag = linearVelocityMag;
//			}
			constraint->setAngle(DEG_TO_RAD(0));
		}
		

		break;
	case BIKESTATE_POWERSLIDE_INAIR:
		constraint->setAngle(DEG_TO_RAD(0));

		if (checkMessage(BIKEACTION_ACCEL))
			throttleStep = 0.01f;
		else
			throttleStep = 0.0f;
		break;
	}

	if (throttleUpActive)
	{
		if (throttleUp != 0.0f && throttleUp < 1.0f)
		{
			friction[FW] = MIN(2.0f, 8.0f * throttleUp * velocityPercent * tFriction.fromStop.normal);
			friction[BW] = MIN(2.0f, 8.0f * throttleUp * velocityPercent * tFriction.fromStop.normal);
		}
		else
		{
			throttleUpActive = TRUE;
		}
	}







	if (getFlag(BIKE_LEFT_TURN))
	{
		if (turnCounter > 0)
			turnCounter = 0;
		turnCounter--;

//		powerSlideTurnCounter = 0;
		powerSlideAccelTurnCounter = 0;
		powerSlideBrakeTurnCounter = 0;
		powerSlideEnd = FALSE;

		turnPressedPercent = pressedPercent;
	}
	else if (getFlag(BIKE_POWERSLIDE_LEFT))
	{
		if (turnCounter > 0)
			turnCounter = 0;
		turnCounter--;

//		if (powerSlideTurnCounter > 0)
//			powerSlideTurnCounter = 0;
//		powerSlideTurnCounter--;

		if (bikeState == BIKESTATE_LEFT_POWERSLIDE_BRAKE)
		{
			if (powerSlideBrakeTurnCounter > 0)
				powerSlideBrakeTurnCounter = 0;
			powerSlideBrakeTurnCounter--;

			powerSlideAccelTurnCounter = 0;
		}
		else
		{
			if (powerSlideAccelTurnCounter > 0)
				powerSlideAccelTurnCounter = 0;
			powerSlideAccelTurnCounter--;

			powerSlideBrakeTurnCounter = 0;
		}

		turnPressedPercent = pressedPercent;
	}
	else if (getFlag(BIKE_RIGHT_TURN))
	{
		if (turnCounter < 0)
			turnCounter = 0;
		turnCounter++;

//		powerSlideTurnCounter = 0;
		powerSlideAccelTurnCounter = 0;
		powerSlideBrakeTurnCounter = 0;
		powerSlideEnd = FALSE;

		turnPressedPercent = pressedPercent;
	}
	else if (getFlag(BIKE_POWERSLIDE_RIGHT))
	{
		if (turnCounter < 0)
			turnCounter = 0;
		turnCounter++;

//		if (powerSlideTurnCounter < 0)
//			powerSlideTurnCounter = 0;
//		powerSlideTurnCounter++;

		if (bikeState == BIKESTATE_RIGHT_POWERSLIDE_BRAKE)
		{
			if (powerSlideBrakeTurnCounter < 0)
				powerSlideBrakeTurnCounter = 0;
			powerSlideBrakeTurnCounter++;

			powerSlideAccelTurnCounter = 0;
		}
		else
		{
			if (powerSlideAccelTurnCounter < 0)
				powerSlideAccelTurnCounter = 0;
			powerSlideAccelTurnCounter++;

			powerSlideBrakeTurnCounter = 0;
		}

		turnPressedPercent = pressedPercent;
	}
	else
	{
		turnCounter = 0;
//		powerSlideTurnCounter = 0;
		powerSlideAccelTurnCounter = 0;
		powerSlideBrakeTurnCounter = 0;
		turnPressedPercent = 0.0f;
		powerSlideEnd = FALSE;
	}

	if (bikeState != BIKESTATE_LEFT_TURN)
		sharpLeftTurnPressedCounter = 0;

	if (bikeState != BIKESTATE_RIGHT_TURN)
		sharpRightTurnPressedCounter = 0;


	if (powerSlideAccelBoostEligible && eligibleBoostState && linearVelocityMag < powerSlideAccelBoostVelocityMag)
	{
		if ((bikeStateCounter[BIKESTATE_RIGHT_POWERSLIDE_ACCEL] < -5 && bikeStateCounter[BIKESTATE_RIGHT_POWERSLIDE_ACCEL] > -45) ||
			(bikeStateCounter[BIKESTATE_LEFT_POWERSLIDE_ACCEL] < -5 && bikeStateCounter[BIKESTATE_LEFT_POWERSLIDE_ACCEL] > -45))
		{
			if (checkMessage(BIKEACTION_ACCEL))
			{
				// if (linearVelocityMag < TO_METERS_PER_SEC(maximumSpeed)
				throttle = THROTTLE_MAX;
	//			((CBikeEngine*)m_engine)->scaleEngineBoost(1.0f + (boostPercent * 5.0f));
				((CBikeEngine*)m_engine)->scaleEngineBoost(4.5f);

				bikeStateCounter[BIKESTATE_LEFT_POWERSLIDE_BRAKE] = -1000;
				bikeStateCounter[BIKESTATE_RIGHT_POWERSLIDE_BRAKE] = -1000;
			}
		}
	}
	else if (powerSlideBrakeBoostEligible && eligibleBoostState && linearVelocityMag < powerSlideBrakeBoostVelocityMag)
	{
		if ((bikeStateCounter[BIKESTATE_RIGHT_POWERSLIDE_BRAKE] < -5 && bikeStateCounter[BIKESTATE_RIGHT_POWERSLIDE_BRAKE] > -55) ||
			(bikeStateCounter[BIKESTATE_LEFT_POWERSLIDE_BRAKE] < -5 && bikeStateCounter[BIKESTATE_LEFT_POWERSLIDE_BRAKE] > -55))
		{
			if (checkMessage(BIKEACTION_ACCEL))
			{
				throttle = THROTTLE_MAX;
//				((CBikeEngine*)m_engine)->scaleEngineBoost(1.0f + (boostPercent * 3.5f));
//				((CBikeEngine*)m_engine)->scaleEngineBoost(4.0f * invVelocityPercent);
				((CBikeEngine*)m_engine)->scaleEngineBoost(4.0f);

				bikeStateCounter[BIKESTATE_LEFT_POWERSLIDE_ACCEL] = -1000;
				bikeStateCounter[BIKESTATE_RIGHT_POWERSLIDE_ACCEL] = -1000;
			}
		}
	}




	if (comShiftState != COMSS_NORMAL)
	{
		switch(comShiftState)
		{
		case COMSS_FORWARD:
			setCenterOfMass(sxVector3d(0.0f,0.35f,0.0f));
			break;
		case COMSS_BACK:
			setCenterOfMass(sxVector3d(0.0f,-0.35f,0.0f));
			break;
		case COMSS_LEFT:
			setCenterOfMass(sxVector3d(0.0f,20.0f,0.0f));
			break;
		case COMSS_RIGHT:
			setCenterOfMass(sxVector3d(-0.0f,20.0f,0.0f));
			break;
		}
	}



//	((CBikeBrake*)m_brakes)->setWheelMaxBrakingTorque(FW, 8000.0f);
//	((CBikeBrake*)m_brakes)->setWheelMaxBrakingTorque(BW, 8000.0f);

	if (wheelieRequested && linearVelocityMag > TO_METERS_PER_SEC(20.0f))
	{

		f32 pitchPercent = getSlopePoint(0.0f, 0.1f, 22.5f, 1.0f, 45.0f, 0.1f, bikePitch);

		sxVector3d torque;
		torque = fRightDir * (45000.0f * pitchPercent);
		Vector3 torqueV3 = SXV_TO_V3(torque);
		chassis->applyTorque(torqueV3);
	}
	else if (getFlag(BIKE_STOPPIE))
	{
		pitchDampening = 1.00f;//0.75f;

		if (linearVelocityMag > TO_METERS_PER_SEC(5.0f))
		{
			if (bikePitch > -40.0f)
			{
				f32 pitchPercent = getSlopePoint(0.0f, 0.1f, 22.5f, 1.0f, 40.0f, 0.1f, -bikePitch);
		
				sxVector3d torque;
				torque = fRightDir * (-45000.0f * pitchPercent * invVelocityPercent);
				Vector3 torqueV3 = SXV_TO_V3(torque);
			}
			else
			{
				pitchDampening = 0.25f;
			}
		}
//		((CBikeBrake*)m_brakes)->setWheelMaxBrakingTorque(FW, 50000.0f);
//		((CBikeBrake*)m_brakes)->setWheelMaxBrakingTorque(BW, 8000.0f);
//		}

//		if (bikeStateCounter[bikeState] > SECS_TO_FRAMES(3.0f))
//		{
//			((CBikeBrake*)m_brakes)->setWheelIsConnectedToHandbrake(FW, TRUE);
//			((CBikeBrake*)m_brakes)->setWheelIsConnectedToHandbrake(BW, FALSE);
//			((CBikeDriverInput*)m_input)->setHandbrake(TRUE);
//		}

	}
	else if (bikeState == BIKESTATE_RIGHT_POWERSLIDE_BRAKE || 
			 bikeState == BIKESTATE_LEFT_POWERSLIDE_BRAKE ||
			 bikeState == BIKESTATE_RIGHT_POWERSLIDE_ACCEL ||
			 bikeState == BIKESTATE_LEFT_POWERSLIDE_ACCEL)
	{
		if (wheelInfo[FW].getIsInContact() && !wheelInfo[BW].getIsInContact())
		{
			sxVector3d torque;
			torque = fRightDir * (45000.0f);
			Vector3 torqueV3 = SXV_TO_V3(torque);
			chassis->applyTorque(torqueV3);
		}
		else if (!wheelInfo[FW].getIsInContact() && wheelInfo[BW].getIsInContact())
		{
			sxVector3d torque;
			torque = fRightDir * (-45000.0f);
			Vector3 torqueV3 = SXV_TO_V3(torque);
			chassis->applyTorque(torqueV3);
		}
	}







	bikeStateCounter[bikeState]++;
	for(u32 i = 0; i < BIKESTATE_NUM_STATES; i++)
	{
		if (i != bikeState && !bikeStateSuspended[i])
			bikeStateCounter[i]--;
	}


	if (onGroundPowerSlide || onGround)
//	if (onGround)
	{
//		if ((wheelInfo[FW].getIsInContact() && wheelInfo[BW].getIsInContact()) || getFlag(BIKE_STOPPIE))
		{
			f32 slideAngVelApply = filterSAV.applyToOneSample(slideAngVel);

//			sxVector3d newAngVelUp = terrainUpVector * (slideAngVel * 1.0f);
			sxVector3d newAngVelUp = terrainUpVector * (slideAngVelApply);
			sxVector3d newAngVelForward = terrainForwardVector * projAngVelForward;
			sxVector3d newAngVelRight = terrainRightVector * projAngVelRight * pitchDampening;
			sxVector3d newAngVel = newAngVelUp + newAngVelForward + newAngVelRight;
			angVelV3 = SXV_TO_V3(newAngVel);
			chassis->setAngularVelocity(angVelV3);
		}
	



		f32 frictionTerrainScaleFW = getSlopePoint(0.0f, 1.0f, 25.0f, 1.0f, 90.0f, 0.0f, terrainPitch[FW]);
		f32 frictionTerrainScaleBW = getSlopePoint(0.0f, 1.0f, 25.0f, 1.0f, 90.0f, 0.0f, terrainPitch[BW]);


		m_wheels->setWheelFriction(FW, frictionFilter[FW].applyToOneSample(friction[FW] * frictionTerrainScaleFW));
		m_wheels->setWheelFriction(BW, frictionFilter[BW].applyToOneSample(friction[BW] * frictionTerrainScaleBW));










		((CBikeSteering*)m_steering)->setSteeringAnglePercent(FW, turnAnglePercent[FW]);
		((CBikeSteering*)m_steering)->setSteeringAnglePercent(BW, turnAnglePercent[BW]);









		if ((wheelInfo[FW].getIsInContact() && wheelInfo[BW].getIsInContact()) && (!wheelieRequested && !wheelieCondition) && !getFlag(BIKE_STOPPIE))
		{
			Vector3 downForceVec3;
			sxVector3d downDir = sxVector3d(0.0f, 1.0f, 0.0f);
		
//			sxVector3d normVector;
//			const Vector3 normVector3;
//			if (wheelInfo[FW].getIsInContact())
//				normVector = wheelNormVec[FW];
//			else
//				normVector = wheelNormVec[BW];
//			downDir = -normVector;
			
			if (checkMessage(BIKEACTION_ACCEL))
			{
				downDir = -(wheelNormVec[FW] + wheelNormVec[BW]) * 0.5f;
				downDir.normalize();
			}
			else
			{
				downDir = -fUpDir;
			}

			downForceVec3 = SXV_TO_V3(downDir * downForceMag);
			chassis->applyForce(downForceVec3);
		}









		f32 leanRotTarget;
		f32 terrainLean = 0.0f;
		f32 terrainSlope = 0.0f;
		if (linearVelocityMag != 0.0f)
		{
			leanRotTarget = leanPercent * maxLeanRot;

			f32 terrainPressedPercent = MAX(0.2f, MIN(0.65f, pressedPercent));
			terrainLean = terrainPressedPercent * terrainRoll;
		}
		else
		{
			leanRotTarget = 0.0f;
			terrainLean = 0.0f;
		}
		leanRotTarget *= turnDirSign;

		leanRotTarget += terrainLean;


		if (leanTargetEnabled)
		{
			f32 leanStep = (leanTarget - leanRot) / leanTargetSteps;
			if (leanTargetSteps > 0)
			{
				leanTargetSteps--;
				leanRot += leanStep;
			}
			else
			{
				leanRot = leanTarget;
			}
			leanFilter.stabilize(leanRot);
		}
		else
		{
			leanRot = leanRotTarget;
			leanRot = leanFilter.applyToOneSample(leanRot);
		}


		constraint->setAngle(DEG_TO_RAD(leanRot));
	}











	if (throttleStep == 0.0 || (wheelieCondition && bikePitch > 65.0f))
	{
		throttle = 0.0f;

		((CBikeEngine*)m_engine)->setResistanceFactorAtMinRPM(0.20f);
		((CBikeEngine*)m_engine)->setResistanceFactorAtOptRPM(0.45f);
		((CBikeEngine*)m_engine)->setResistanceFactorAtMaxRPM(0.65f);
	}
	else
	{
		if (!wheelieCondition && !wheelieRequested)
		{
			((CBikeEngine*)m_engine)->setResistanceFactorAtMinRPM(0.01f);
			((CBikeEngine*)m_engine)->setResistanceFactorAtOptRPM(0.0275f);
			((CBikeEngine*)m_engine)->setResistanceFactorAtMaxRPM(0.065f);
		}
		else
		{
			((CBikeEngine*)m_engine)->setResistanceFactorAtMinRPM(0.9f);
			((CBikeEngine*)m_engine)->setResistanceFactorAtOptRPM(2.0f);
			((CBikeEngine*)m_engine)->setResistanceFactorAtMaxRPM(2.0f);
		}

		if (throttle == 0.0f)
		{
			throttle = ((CBikeEngine*)m_engine)->getThrottle();
		}
		else
		{
			throttle += throttleStep;
			throttleUp += 1.0f / SECS_TO_FRAMES(1.5f);
		}
		throttle = MAX(THROTTLE_MIN, throttle);
		throttle = MIN(THROTTLE_MAX, throttle);
		throttleUp = MIN(1.0f, throttleUp);
	}


	f32 upPercent = terrainPitchAvg / 90.0f;

	upPercent *= 10.0f;
	upPercent = (f32)(CUtil::f32_to_s32(upPercent));
	upPercent *= 0.1f;

	upPercent = MAX(0.0f, upPercent);
	if ((wheelInfo[FW].getIsInContact() || wheelInfo[BW].getIsInContact()) && !(wheelieRequested || wheelieCondition))
		((CBikeEngine*)m_engine)->scaleEngineBoost(1.0f + (TERRAIN_BOOST * upPercent));



	bool level = FALSE;
	if (fabs(terrainPitchAvg) < 5.0f)
		level = TRUE;


	if (!movingForward && !getFlag(BIKE_POWERSLIDE_LEFT) && !getFlag(BIKE_POWERSLIDE_RIGHT))
	{
		((CBikeEngine*)m_engine)->setResistanceFactorAtMinRPM(-0.50f);
		((CBikeEngine*)m_engine)->setResistanceFactorAtOptRPM(-0.50f);
		((CBikeEngine*)m_engine)->setResistanceFactorAtMaxRPM(-0.50f);
	}





	f32 brakingValue;

	if (brakeStep != 0.0f)
	{
		brakingScale += brakeStep;
		brakingValue = NORMAL_BRAKING * brakingScale;
		brakingValue = MIN(NORMAL_BRAKING, brakingValue);
	}
	else
	{
		brakingValue = 0.0f;
	}




//	if (onGround)
	if (onGroundPowerSlide || onGround)
	{
		f32 targetWheelStrength[2];
		f32 targetWheelRelax[2];
		f32 targetWheelComp[2];
		f32 currentWheelStrength[2];
		f32 currentWheelRelax[2];
		f32 currentWheelComp[2];

		f32 wheelStrength[2];
		f32 wheelRelax[2];
		f32 wheelComp[2];



		if (brakingValue != 0.0f && linearVelocityMag > TO_METERS_PER_SEC(10.0f))
		{
			targetWheelStrength[FW] = 30.0f;
			targetWheelStrength[BW] = 90.0f;
			targetWheelRelax[FW] = 25.0f;
			targetWheelRelax[BW] = 50.0f;
			targetWheelComp[FW] = 20.0f;
			targetWheelComp[BW] = 60.0f;

			if (bikePitch > -15.0f)
			{
				sxVector3d torque = rightDir * (-8000.0f);
				Vector3 torqueV3 = SXV_TO_V3(torque);
				chassis->applyTorque(torqueV3);
			}
		}
		else if ((throttle < 1.0f || acceleration > 0.1f) && throttle > 0.0f && !getFlag(BIKE_WHEELIE))
		{
			targetWheelStrength[FW] = 90.0f;
			targetWheelStrength[BW] = 60.0f;
			targetWheelRelax[FW] = 50.0f;
			targetWheelRelax[BW] = 35.0f;
			targetWheelComp[FW] = 60.0f;
			targetWheelComp[BW] = 40.0f;

			if (bikePitch < 15.0f)
			{
				sxVector3d torque = rightDir * (18000.0f);
				Vector3 torqueV3 = SXV_TO_V3(torque);
				chassis->applyTorque(torqueV3);
			}
		}
		else
		{

			if (getFlag(BIKE_POWERSLIDE_LEFT) || 
				getFlag(BIKE_POWERSLIDE_RIGHT) || 
				getFlag(BIKE_LEFT_TURN) || 
				getFlag(BIKE_RIGHT_TURN))
			{
				targetWheelStrength[FW] = 170.0f;
				targetWheelStrength[BW] = 170.0f;
				targetWheelRelax[FW] = 80.0f;
				targetWheelRelax[BW] = 80.0f;
				targetWheelComp[FW] = 90.0f;
				targetWheelComp[BW] = 90.0f;
			}
			else
			{
				targetWheelStrength[FW] = 170.0f;
				targetWheelStrength[BW] = 170.0f;
				targetWheelRelax[FW] = 80.0f;
				targetWheelRelax[BW] = 80.0f;
				targetWheelComp[FW] = 90.0f;
				targetWheelComp[BW] = 90.0f;
			}
		}

		
		currentWheelStrength[FW] = ((CBikeSuspension*)m_suspension)->getWheelStrength(FW);
		currentWheelStrength[BW] = ((CBikeSuspension*)m_suspension)->getWheelStrength(BW);
		currentWheelRelax[FW] = ((CBikeSuspension*)m_suspension)->getWheelDampingRelaxation(FW);
		currentWheelRelax[BW] = ((CBikeSuspension*)m_suspension)->getWheelDampingRelaxation(BW);
		currentWheelComp[FW] = ((CBikeSuspension*)m_suspension)->getWheelDampingCompression(FW);
		currentWheelComp[BW] = ((CBikeSuspension*)m_suspension)->getWheelDampingCompression(BW);



		wheelStrength[FW] = getTargetPoint(currentWheelStrength[FW], targetWheelStrength[FW], 5.0f);
		wheelStrength[BW] = getTargetPoint(currentWheelStrength[BW], targetWheelStrength[BW], 5.0f);
		wheelRelax[FW] = getTargetPoint(currentWheelRelax[FW], targetWheelRelax[FW], 5.0f);
		wheelRelax[BW] = getTargetPoint(currentWheelRelax[BW], targetWheelRelax[BW], 5.0f);
		wheelComp[FW] = getTargetPoint(currentWheelComp[FW], targetWheelComp[FW], 5.0f);
		wheelComp[BW] = getTargetPoint(currentWheelComp[BW], targetWheelComp[BW], 5.0f);


		((CBikeSuspension*)m_suspension)->setWheelStrength(FW, wheelStrength[FW]);
		((CBikeSuspension*)m_suspension)->setWheelStrength(BW, wheelStrength[BW]);
		((CBikeSuspension*)m_suspension)->setWheelDampingRelaxation(FW, wheelRelax[FW]);
		((CBikeSuspension*)m_suspension)->setWheelDampingRelaxation(BW, wheelRelax[BW]);
		((CBikeSuspension*)m_suspension)->setWheelDampingCompression(FW, wheelComp[FW]);
		((CBikeSuspension*)m_suspension)->setWheelDampingCompression(BW, wheelComp[BW]);

	}
	else
	{

		((CBikeSuspension*)m_suspension)->setWheelStrength(0, 120.0f);
		((CBikeSuspension*)m_suspension)->setWheelStrength(1, 120.0f);
		((CBikeSuspension*)m_suspension)->setWheelDampingRelaxation(0, 25.0f);
		((CBikeSuspension*)m_suspension)->setWheelDampingRelaxation(1, 25.0f);
		((CBikeSuspension*)m_suspension)->setWheelDampingCompression(0, 30.0f);
		((CBikeSuspension*)m_suspension)->setWheelDampingCompression(1, 30.0f);

	}








	((CBikeDriverInput*)m_input)->setSteeringInput(turnStep * steeringPercent);
	((CBikeDriverInput*)m_input)->setBrakeInput(brakingValue);


	if (collidedCounter > COLLIDED_COUNTER_NO_COLLISION_FRAME)
		((CBikeDriverInput*)m_input)->setAcceleratorInput(throttle);
	else
		((CBikeDriverInput*)m_input)->setAcceleratorInput(0.0f);
}


TurnFriction turnFriction[NUM_FRICTION_LEVELS] = 
{
//-- NORMAL				ANG				FORCE
	{0.125f,			0.0f,			0.0f},
	{0.20f,				0.0f,			0.0f},
	{0.35f,				0.0f,			0.0f},
	{0.25f,				0.0f,			0.0f},
	{0.5f,				0.0f,			0.0f},
	{1.0f,				0.0f,			0.0f}
};

TurnFriction fromStopFriction[NUM_FRICTION_LEVELS] = 
{
//-- NORMAL				ANG				FORCE
	{0.25f,				0.25f,			0.0625f},
	{0.37f,				0.37f,			0.125f},
	{0.50f,				0.50f,			0.25f},
	{0.62f,				0.67f,			0.50f},
	{0.75f,				0.85f,			0.85f},
	{1.0f,				1.0f,			1.0f}
};

TurnFriction slideFriction[NUM_FRICTION_LEVELS] = 
{
//-- NORMAL				ANG				FORCE
	{0.0625f,			0.0f,			0.125f},
	{0.125f,			0.0f,			0.25f},
	{0.25f,				0.0f,			0.40f},
	{0.35f,				0.0f,			0.70f},
	{0.50f,				0.0f,			0.90f},
	{1.0f,				0.0f,			1.0f}
};

TurnFriction psFriction[NUM_FRICTION_LEVELS] = 
{
//-- NORMAL				ANG				FORCE
	{0.0625f,			0.65f,			0.125f},
	{0.125f,			0.65f,			0.25f},
	{0.25f,				0.65f,			0.40f},
	{0.35f,				0.65f,			0.70f},
	{0.25f,				0.75f,			0.90f},
	{1.0f,				1.0f,			1.0f}
};

TurnFriction psbFriction[NUM_FRICTION_LEVELS] = 
{
//-- NORMAL				ANG				FORCE
	{0.0625f,			0.5f,			0.125f},
	{0.125f,			0.5f,			0.25f},
	{0.25f,				0.5f,			0.40f},
	{0.25f,				0.5f,			0.40f},
	{0.25f,				0.75f,			0.90f},
	{1.0f,				1.0f,			1.0f}
};

TurnFriction pseFriction[NUM_FRICTION_LEVELS] = 
{
//-- NORMAL				ANG				FORCE
	{0.125f,			1.0f,			0.0f},
	{0.25f,				1.0f,			0.0f},
	{0.4f,				1.0f,			0.0f},
	{0.5f,				1.0f,			0.0f},
	{0.75f,				1.0f,			0.0f},
	{1.0f,				1.0f,			0.0f}
};



DriveFriction driveFriction[NUM_FRICTION_LEVELS] = 
{
//-- ACCEL				COAST				BRAKE				TORQUE
	{0.0625f,			0.0625f,			0.0325f,			0.0625f},
	{0.125f,			0.125f,				0.0625f,			0.125f},
	{0.25f,				0.25f,				0.125f,				0.25f},
	{0.5f,				0.5f,				0.33f,				0.5f},
	{0.75f,				0.75f,				0.66f,				0.75f},
	{1.0f,				1.0f,				1.0f,				1.0f}
};




u32 terrainFriction[15][2] = 
{
//--	TURN					DRIVE
//	hardpack = 0,
		{FRICTION_HIGH_H,		FRICTION_HIGH_H}, 
//	cement = 2,
		{FRICTION_HIGH_H,		FRICTION_HIGH_H},
//	loam = 4,
		{FRICTION_HIGH_H,		FRICTION_HIGH_H},
//	clay = 6,
		{FRICTION_HIGH_L,		FRICTION_HIGH_L},
//	sand = 8,
		{FRICTION_MED_L,		FRICTION_LOW_H}, 
//	grass = 10,
		{FRICTION_MED_H,		FRICTION_HIGH_L}, 
//	gravel = 12,
		{FRICTION_HIGH_L,		FRICTION_MED_H}, 
//	oil = 14,
		{FRICTION_LOW_L,		FRICTION_LOW_L}, 



//	wetHardpack = 1,
		{FRICTION_HIGH_L,		FRICTION_HIGH_L}, 
//	wetCement = 3,
		{FRICTION_HIGH_L,		FRICTION_HIGH_L}, 
//	wetLoam = 5,
		{FRICTION_HIGH_L,		FRICTION_HIGH_L}, 
//	wetClay = 7,
		{FRICTION_MED_H,		FRICTION_MED_H}, 
//	wetSand = 9,
		{FRICTION_LOW_H,		FRICTION_LOW_H}, 
//	wetGrass = 11,
		{FRICTION_MED_L,		FRICTION_MED_L}, 
//	wetGravel = 13,
		{FRICTION_HIGH_H,		FRICTION_HIGH_H}, 

};


u32 getTerrainFrictionType(u32 terrainType)
{
	if ((CTerrainType::TypeList)terrainType <= CTerrainType::oil)
		return terrainType;
	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetHardpack || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offHardpack ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetHardpack)
		return (u32)(CTerrainType::hardpack);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetCement || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offCement ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetCement)
		return (u32)(CTerrainType::cement);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetLoam || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offLoam ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetLoam)
		return (u32)(CTerrainType::loam);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetClay || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offClay ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetClay)
		return (u32)(CTerrainType::clay);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetSand || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offSand ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetSand)
		return (u32)(CTerrainType::sand);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetGrass || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offGrass ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetGrass)
		return (u32)(CTerrainType::grass);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetGravel || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offGravel ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetGravel)
		return (u32)(CTerrainType::gravel);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::offOil) 
		return (u32)(CTerrainType::oil);

	else
		return 0;
}



u32 getTerrainParticleType(u32 terrainType)
{
	if ((CTerrainType::TypeList)terrainType <= CTerrainType::oil)
		return terrainType;
	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetHardpack || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offHardpack ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetHardpack)
		return (u32)(CTerrainType::hardpack);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetCement || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offCement ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetCement)
		return (u32)(CTerrainType::cement);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetLoam || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offLoam ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetLoam)
		return (u32)(CTerrainType::loam);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetClay || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offClay ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetClay)
		return (u32)(CTerrainType::clay);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetSand || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offSand ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetSand)
		return (u32)(CTerrainType::sand);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetGrass || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offGrass ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetGrass)
		return (u32)(CTerrainType::grass);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::wetGravel || 
			 (CTerrainType::TypeList)terrainType == CTerrainType::offGravel ||
			 (CTerrainType::TypeList)terrainType == CTerrainType::offwetGravel)
		return (u32)(CTerrainType::gravel);

	else if ((CTerrainType::TypeList)terrainType == CTerrainType::offOil) 
		return (u32)(CTerrainType::oil);

	else
		return 0;
}





void HavokBike::getTerrainType()
{
	lastTerrainType = terrainType;

	CTerrainType* terrain = CTerrainType::getInstance();
//	CTerrainType::TypeList terrainType = terrain->getTypeAtPoint(getPosition().x, getPosition().z);
	terrainType = (u32)terrain->getTypeAtPoint(getPosition().x, getPosition().z);
//	if (terrainType < 15)

	lastValidTerrainType = getTerrainFrictionType(terrainType);





	u32 turnIndex = MIN(FRICTION_HIGH_H, terrainFriction[lastValidTerrainType][0] + attributes.traction);
	u32 driveIndex = MIN(FRICTION_HIGH_H, terrainFriction[lastValidTerrainType][1] + attributes.traction);


	//-- for debug
//	turnIndex = FRICTION_HIGH_H;
//	driveIndex = FRICTION_LOW_H;
//	turnIndex = FRICTION_HIGH_H;
//	driveIndex = FRICTION_HIGH_H;


	tFriction.turn.normal =				turnFriction[turnIndex].normal;
	tFriction.turn.ang =				turnFriction[turnIndex].ang;
	tFriction.turn.force =				turnFriction[turnIndex].force;

	tFriction.fromStop.normal =			fromStopFriction[turnIndex].normal;
	tFriction.fromStop.ang =			fromStopFriction[turnIndex].ang;
	tFriction.fromStop.force =			fromStopFriction[turnIndex].force;

	tFriction.slide.normal =			slideFriction[turnIndex].normal;
	tFriction.slide.ang =				slideFriction[turnIndex].ang;
	tFriction.slide.force =				slideFriction[turnIndex].force;

	tFriction.ps.normal =				psFriction[turnIndex].normal;
	tFriction.ps.ang =					psFriction[turnIndex].ang;
	tFriction.ps.force =				psFriction[turnIndex].force;

	tFriction.psb.normal =				psbFriction[turnIndex].normal;
	tFriction.psb.ang =					psbFriction[turnIndex].ang;
	tFriction.psb.force =				psbFriction[turnIndex].force;

	tFriction.pse.normal =				pseFriction[turnIndex].normal;
	tFriction.pse.ang =					pseFriction[turnIndex].ang;
	tFriction.pse.force =				pseFriction[turnIndex].force;


	tFriction.drive.accel = driveFriction[driveIndex].accel;
	tFriction.drive.coast = driveFriction[driveIndex].coast;
	tFriction.drive.brake = driveFriction[driveIndex].brake;
	tFriction.drive.torque = driveFriction[driveIndex].torque;


	bikePowerBand->setTorque(1000, 3000 * tFriction.drive.torque);
}















const f32 PRELOAD_FORCE = 17000.0f;//7000.0f;
const f32 PRELOAD_MIN_SEC_ACTIVATE = 0.2f;//0.2f;//0.25f;//0.1f;
const f32 PRELOAD_MAX_SEC_ACTIVATE = 2.0f;
const f32 PRELOAD_BETWEEN_ATTEMPTS_SEC = 2.0f;
const f32 PRELOAD_JUMP_WINDOW_SEC = 0.75f;//0.5f;//1.0f;//0.75f;
const f32 PRELOAD_MIN_VELOCITY = TO_METERS_PER_SEC(30.0f);





void HavokBike::landSpeed()
{
	if (nitroState != NO_NITRO)
		return;

	if (inAir && (wheelInfo[FW].getIsInContact() || wheelInfo[BW].getIsInContact()) && landingSpeedCounter > 5)
	{
		landingSpeedCounter = 0;
		if (framesInAir > 20)
			landingSpeedScale = attributes.landingSpeedRecoveryJumps;
		else
			landingSpeedScale = attributes.landingSpeedRecoveryWhoops;

		Vector3 lVel3 = chassis->getLinearVelocity();
		sxVector3d lVel = V3_TO_SXV(lVel3);
		landingSpeedMag = lVel.magnitude();
	}

	if (landingSpeedCounter < 5)
	{
		Vector3 lVel3 = chassis->getLinearVelocity();
		sxVector3d lVel = V3_TO_SXV(lVel3);

//		f32 lVelMag = lVel.magnitude() * attributes.landingSpeedRecovery;
		f32 lVelMag = landingSpeedMag * landingSpeedScale;
		lVel.normalize();
		
		sxVector3d newLVel = lVel * lVelMag;

		lVel3 = SXV_TO_V3(newLVel);
		chassis->setLinearVelocity(lVel3);
	}


	landingSpeedCounter++;
}



void HavokBike::jumpSpeed()
{

	if (nitroState != NO_NITRO)
		return;

	if (!checkMessage(BIKEACTION_ACCEL))
		return;


	sxVector3d point, tangent;
	CSplineManager *splineManager = CSplineManager::getSplineManager();

	f32 rightWay;
	if (splineManager->getPoint(curWayTime, CSplineManager::WayPoint, point, tangent)) 
	{
		tangent.normalize();
		rightWay = forwardDir * tangent;
	}
	else
	{
		rightWay = 1.0f;
	}





	if (!getFlag(BIKE_POWERSLIDE_LEFT) && !getFlag(BIKE_POWERSLIDE_RIGHT))
	{
		Vector3 lVel3 = chassis->getLinearVelocity();
		sxVector3d lVel = V3_TO_SXV(lVel3);

		f32 lVelMag = lVel.magnitude();
		lVel.normalize();


		if ((wheelInfo[FW].getIsInContact() || wheelInfo[BW].getIsInContact()) &&
			(((CTerrainType::TypeList)terrainType == CTerrainType::aiPreLoad || 
			  (CTerrainType::TypeList)terrainType == CTerrainType::jumpLand) && 
			rightWay > 0.8f))
		{
			jumpAssist = TRUE;

//			jumpSpeedTarget = TO_METERS_PER_SEC(attributes.maximumSpeed-4.0f);
			jumpSpeedTarget = TO_METERS_PER_SEC(70.0f-4.0f);

			if ((linearVelocityMag > TO_METERS_PER_SEC(attributes.maximumSpeed - attributes.jumpSpeedTolerance)) ||
				(linearVelocityMag > jumpSpeedTarget))
			{
				sxVector3d newLVel = lVel * jumpSpeedTarget;
				lVel3 = SXV_TO_V3(newLVel);
				chassis->setLinearVelocity(lVel3);					
			}
		}
		else if (jumpAssist && ((CTerrainType::TypeList)lastTerrainType == CTerrainType::aiPreLoad || 
								(CTerrainType::TypeList)lastTerrainType == CTerrainType::jumpLand))
		{
			jumpAssist = FALSE;
			sxVector3d flatForward = fForwardDir;
			flatForward.y = 0.0f;
			flatForward.normalize();
			f32 velAngle = RAD_TO_DEG(x_acos(lVel * flatForward));

			jumpSpeedTarget = x_sqrt((37.0f * GRAVITY) /(x_sin(2.0f * DEG_TO_RAD(velAngle)) ) );
			
//			jumpSpeedTarget = x_sqrt((TO_METERS_PER_SEC(attributes.maxJumpSpeed) * 15.1074f) /(x_sin(2.0f * DEG_TO_RAD(velAngle)) ) );

			if ((linearVelocityMag > TO_METERS_PER_SEC(attributes.maximumSpeed - attributes.jumpSpeedTolerance)) ||
				(linearVelocityMag > jumpSpeedTarget))
//			if (linearVelocityMag > TO_METERS_PER_SEC(attributes.maximumSpeed - attributes.jumpSpeedTolerance))
			{

				sxVector3d newLVel = lVel * jumpSpeedTarget;
				lVel3 = SXV_TO_V3(newLVel);
				chassis->setLinearVelocity(lVel3);					
			}

//#if 0			
			takeOffPos = getPosition();
			jumpAssistInAir = true;
//#endif

		}
	}
}

//#if 0
void HavokBike::inAirAdjust()
{
#if 0
	if (jumpAssistInAir)
	{

		sxVector3d p1 = takeOffPos;
		sxVector3d p2 = getPosition();
		p1.y = 0.0f;
		p2.y = 0.0f;

		sxVector3d delta = p1 - p2;
		f32 currDist = x_sqrt(delta * delta) * SX_TO_HAVOK;

		f32 range;
		if (preLoadReportState == PRELOAD_SUCCESSFUL)
			range = 20.0f - currDist;
		else 
			range = 10.0f - currDist;


		if (range > 0.0f)
		{
			Vector3 lVel3 = chassis->getLinearVelocity();
			sxVector3d lVel = V3_TO_SXV(lVel3);
	
			f32 lVelMag = lVel.magnitude();
			lVel.normalize();
	
	
			sxVector3d flatForward = fForwardDir;
			flatForward.y = 0.0f;
			flatForward.normalize();
			f32 velAngle = RAD_TO_DEG(x_acos(lVel * flatForward));
	
			f32 extraGravity = (linearVelocityMag * x_sin(2.0f * DEG_TO_RAD(velAngle)) / range) - GRAVITY;
	
	
			((CBikeAerodynamics*)m_aerodynamics)->setExtraGravity(Vector3(0.0f, extraGravity, 0.0f));

			if (wheelInfo[FW].getIsInContact() || wheelInfo[BW].getIsInContact())
				jumpAssistInAir = false;
		}
		else
		{
			jumpAssistInAir = false;
		}

	}
	else
	{
		((CBikeAerodynamics*)m_aerodynamics)->setExtraGravity(Vector3(0.0f, 0.0f, 0.0f));
	}
#endif
}

//#endif




void HavokBike::preLoad()
{
	CGameOptions* gameOptions = CSxGame::getGameOptions();		

	if (nitroState != NO_NITRO)
		return;

	bool preLoadCondition = FALSE;

	if (gameOptions->getTrackType() == TRACKTYPE_BAJA || 
		gameOptions->getTrackType() == TRACKTYPE_FREESTYLE)
	{
		if (!wheelInfo[FW].getIsInContact() && wheelInfo[BW].getIsInContact())
		{
			if (terrainPitch[BW] - terrainPitch[FW] > 10.0f)
			{
				preLoadCondition = TRUE;
			}
		}
		if (framesInAir == PRELOAD_INAIR_FRAME + PRELOAD_INAIR_FRAME_OFFSET)
		{
			preLoadCondition = TRUE;
		}
	}
	else
	{
		if ((CTerrainType::TypeList)lastTerrainType == CTerrainType::aiPreLoad || 
			(CTerrainType::TypeList)lastTerrainType == CTerrainType::jumpLand)
			preLoadCondition = TRUE;
	}


	((CBikeAerodynamics*)m_aerodynamics)->setExtraGravity(Vector3(0.0f, 0.0f, 0.0f));

	if (!preLoadWaitDelay)
	{
		preLoadReportState = PRELOAD_AVAILABLE;

		if (checkMessage(BIKEACTION_PRELOAD) && !preLoadJumpPending)
		{
			setFlag(BIKE_PRELOAD);

			preLoadPressedCounter++;

			if (preLoadPressedCounter <= SECS_TO_FRAMES(PRELOAD_MIN_SEC_ACTIVATE))
			{
				preLoadJumpState = PRELOAD_PRE_WINDOW;
			}
			else if (preLoadPressedCounter >= SECS_TO_FRAMES(PRELOAD_MIN_SEC_ACTIVATE) &&
					 preLoadPressedCounter <= SECS_TO_FRAMES(PRELOAD_MAX_SEC_ACTIVATE))
			{
				preLoadJumpState = PRELOAD_DURING_WINDOW;
			}
			else
			{
				preLoadWaitDelay = SECS_TO_FRAMES(PRELOAD_BETWEEN_ATTEMPTS_SEC);
				preLoadJumpState = PRELOAD_NO_ATTEMPT_PENDING;
				preLoadReportState = PRELOAD_FAILED;
			}
		}
		else
		{
			switch(preLoadJumpState)
			{
			case PRELOAD_NO_ATTEMPT_PENDING:
				preLoadPressedCounter = 0;
				preLoadJumpPending = FALSE;
				preLoadJump = FALSE;
				preLoadJumpCounter = 0;
				break;
			case PRELOAD_PRE_WINDOW:
				preLoadJumpState = PRELOAD_NO_ATTEMPT_PENDING;
				preLoadWaitDelay = SECS_TO_FRAMES(PRELOAD_BETWEEN_ATTEMPTS_SEC);
				preLoadReportState = PRELOAD_FAILED;
				break;
			case PRELOAD_DURING_WINDOW:
				setFlag(BIKE_PRELOAD);

				if (!preLoadJump)
				{
					preLoadJump = TRUE;
					preLoadJumpCounter = 0;
				}
				else
				{
					preLoadJumpCounter++;
				}				


//				if (preLoadCondition)
//				{
				if ((framesInAir >  PRELOAD_INAIR_FRAME + PRELOAD_INAIR_FRAME_OFFSET) || (preLoadJumpCounter > SECS_TO_FRAMES(PRELOAD_JUMP_WINDOW_SEC)))
//				if (preLoadJumpCounter > SECS_TO_FRAMES(PRELOAD_JUMP_WINDOW_SEC))
				{
					preLoadJumpState = PRELOAD_NO_ATTEMPT_PENDING;
					preLoadWaitDelay = SECS_TO_FRAMES(PRELOAD_BETWEEN_ATTEMPTS_SEC);
					preLoadReportState = PRELOAD_FAILED;
				}
				else
				{
					if (preLoadCondition && 
						preLoadJumpCounter <= SECS_TO_FRAMES(PRELOAD_JUMP_WINDOW_SEC) && linearVelocityMag >= PRELOAD_MIN_VELOCITY)						
					{
						sxVector3d torque;
						torque = fRightDir * (45000.0f);
						Vector3 torqueV3 = SXV_TO_V3(torque);
						chassis->applyTorque(torqueV3);
	

						preLoadJumpState = PRELOAD_NO_ATTEMPT_PENDING;
						preLoadWaitDelay = SECS_TO_FRAMES(PRELOAD_BETWEEN_ATTEMPTS_SEC);
						preLoadReportState = PRELOAD_SUCCESSFUL;
						preLoadForceCounter = 20;
					}
				}
				break;
			}
		}
	}
	else
	{
		preLoadWaitDelay--;

		if (preLoadForceCounter)
		{
			f32 forcePercent = 1.0f - ((f32)preLoadForceCounter / (20.0f));
			forcePercent = MAX(0.0f, MIN(1.0f, forcePercent));

			sxVector3d upForce = sxVector3d(1.35f * fForwardDir.x, -1.5f, 1.35f * fForwardDir.z);
			upForce.normalize();
			upForce = upForce * 1080.0f * forcePercent;
		
			Vector3 upForceV3 = SXV_TO_V3(upForce);
			chassis->applyImpulse(upForceV3);

			preLoadForceCounter--;
		}


		if (preLoadReportState == PRELOAD_SUCCESSFUL && framesInAir > PRELOAD_INAIR_FRAME)
		{
			((CBikeAerodynamics*)m_aerodynamics)->setExtraGravity(Vector3(0.0f, -0.3f, 0.0f));
		}
	}
}



const f32 IN_AIR_ANGULAR_DAMPENING_X = 0.75f;
const f32 IN_AIR_ANGULAR_DAMPENING_Y = 0.75f;
const f32 IN_AIR_ANGULAR_DAMPENING_Z = 0.75f;

const f32 BACK_ON_FRONT_OFF_ANGULAR_DAMPENING_X = 0.95f;
const f32 BACK_ON_FRONT_OFF_ANGULAR_DAMPENING_Y = 1.0f;//0.75f;
const f32 BACK_ON_FRONT_OFF_ANGULAR_DAMPENING_Z = 0.95f;

const f32 FRONT_ON_BACK_OFF_ANGULAR_DAMPENING_X = 0.95f;
const f32 FRONT_ON_BACK_OFF_ANGULAR_DAMPENING_Y = 1.0f;//0.75f;
const f32 FRONT_ON_BACK_OFF_ANGULAR_DAMPENING_Z = 0.95f;

const f32 ON_GROUND_ANGULAR_DAMPENING_X = 0.95f;//1.0f;//0.95f;
const f32 ON_GROUND_ANGULAR_DAMPENING_Y = 1.0f;//0.95f;
const f32 ON_GROUND_ANGULAR_DAMPENING_Z = 0.95f;//1.0f;//0.95f;



//------------------------------------------------------------------------
void HavokBike::dampenAngularVelocity()
{
	sxVector3d angDampening;

//	Vector3 angVelVec3 = chassis->getAngularVelocity();
//	sxVector3d angVel = V3_TO_SXV(angVelVec3);

	Vector3 angVelV3 = chassis->getAngularVelocity();
	sxVector3d angVel = V3_TO_SXV(angVelV3);
	
	f32 projAngVelUp = angVel * upDir;
	f32 projAngVelForward = angVel * forwardDir;
	f32 projAngVelRight = angVel * rightDir;


	//-- in air
	if (!wheelInfo[FW].getIsInContact() && !wheelInfo[BW].getIsInContact())
	{
		angDampening = sxVector3d(	IN_AIR_ANGULAR_DAMPENING_X,
									IN_AIR_ANGULAR_DAMPENING_Y,
									IN_AIR_ANGULAR_DAMPENING_Z);
	} 
	//-- back on ground & front in air
	else if (!wheelInfo[FW].getIsInContact() && wheelInfo[BW].getIsInContact())
	{
		angDampening = sxVector3d(	BACK_ON_FRONT_OFF_ANGULAR_DAMPENING_X, 
									1.0f/*BACK_ON_FRONT_OFF_ANGULAR_DAMPENING_Y*/,
									BACK_ON_FRONT_OFF_ANGULAR_DAMPENING_Z);
	}
	//-- front on ground & back in air
	else if (wheelInfo[FW].getIsInContact() && !wheelInfo[BW].getIsInContact())
	{
		angDampening = sxVector3d(	FRONT_ON_BACK_OFF_ANGULAR_DAMPENING_X,
									1.0f/*FRONT_ON_BACK_OFF_ANGULAR_DAMPENING_Y*/,
									FRONT_ON_BACK_OFF_ANGULAR_DAMPENING_Z);
	}
	//-- on ground
	else if (wheelInfo[FW].getIsInContact() && wheelInfo[BW].getIsInContact())
	{
		angDampening = sxVector3d(	ON_GROUND_ANGULAR_DAMPENING_X, 
									1.0f/*ON_GROUND_ANGULAR_DAMPENING_Y*/, 
									ON_GROUND_ANGULAR_DAMPENING_Z);
	}

	angVel = (upDir * projAngVelUp * angDampening.x) + 
			 (forwardDir * projAngVelForward * angDampening.y) + 
			 (rightDir * projAngVelRight * angDampening.z);

//	angVel.x *= angDampening.x;
//	angVel.y *= angDampening.y;
//	angVel.z *= angDampening.z;

	angVelV3 = SXV_TO_V3(angVel);
	chassis->setAngularVelocity(angVelV3);
}







sxVector3d HavokBike::getRiderDisplacement()
{
	return riderDelta;

}

sxVector3d HavokBike::getRiderPos()
{
	return riderPos;
}



void HavokBike::riderDisplacement()
{
	riderLastPos = riderCurrentPos;
	riderLastVelocity = riderVelocity;

	sxVector3d bikeAccel = linearVelocity - lastLinearVelocity;

	sxVector3d accel;
	accel.y = bikeAccel * upDir;
	accel.x = bikeAccel * rightDir;
	accel.z = bikeAccel * forwardDir;

	const f32 riderMass = 100.0f; // kg
	const sxVector3d springK = sxVector3d(400.0f, 400.0f, 400.0f);
//	const f32 springK = 400.0f;
	const f32 time = 10.0f * 0.016666f;
	const sxVector3d velocityDampen = sxVector3d(0.75f, 0.75f, 0.75f);
	const sxVector3d deltaLimitMin = sxVector3d(-0.125f, -0.25f, -0.5f);
	const sxVector3d deltaLimitMax = sxVector3d(0.125f, 0.0f, 0.5f);

	sxVector3d accelForce = accel * riderMass;
	sxVector3d springForce;
	springForce.x = -((riderCurrentPos.x - riderRestPos.x) * springK.x);
	springForce.y = -((riderCurrentPos.y - riderRestPos.y) * springK.y);
	springForce.z = -((riderCurrentPos.z - riderRestPos.z) * springK.z);

	sxVector3d force = accelForce + springForce;

	sxVector3d acceleration = force / riderMass;

	riderLastVelocity.x = riderLastVelocity.x * velocityDampen.x;
	riderLastVelocity.y = riderLastVelocity.y * velocityDampen.y;
	riderLastVelocity.z = riderLastVelocity.z * velocityDampen.z;


	riderVelocity = riderLastVelocity + (acceleration * time);
	riderCurrentPos = riderLastPos + (riderVelocity * time);

	riderCurrentPos.x = MIN(MAX(riderCurrentPos.x, deltaLimitMin.x), deltaLimitMax.x);
	riderCurrentPos.y = MIN(MAX(riderCurrentPos.y, deltaLimitMin.y), deltaLimitMax.y);
	riderCurrentPos.z = MIN(MAX(riderCurrentPos.z, deltaLimitMin.z), deltaLimitMax.z);
	
	riderDelta = (riderRestPos - riderCurrentPos) * HAVOK_TO_SX;


//	riderPos = getPosition() + (upDir * 95.0f) + (forwardDir * 55.0f) + 
//				((upDir * riderDelta.y) + (forwardDir * riderDelta.z) + (rightDir * riderDelta.x));


}













void HavokBike::restitution()
{
	f32 restitution = (linearVelocityMag / MAX_VELOCITY);
	restitution = MIN(restitution, 1.0f);
	restitution = 1.0f - restitution;
//	restitution *= 8.0f;
//	if (linearVelocityMag < TO_METERS_PER_SEC(5.0f))
//		restitution = 30.0f;
	if (linearVelocityMag < TO_METERS_PER_SEC(5.0f))
		restitution = 4.0f;

	restitution = 0.25f;
	chassis->setRestitution(restitution);
//	chassis->setFriction(1.0f);
}


const f32 IN_AIR_VELOCITY_ACCEL_STEP = 0.0825f;
const f32 IN_AIR_VELOCITY_BRAKE_STEP = 0.125f;
const f32 IN_AIR_VELOCITY_STEP = 0.25f;


const f32 DUST_PARTICLE_DIST_SCALAR_BOTH_WHEEL = 6.0f;
const f32 DUST_PARTICLE_DIST_SCALAR_ONE_WHEEL = 3.0f;
const f32 DUST_MIN_VELOCITY = 3.0f; // mph



//------------------------------------------------------------------------
void HavokBike::updateHavokFromModelObject()
{
	//-- get the model object position
	sxVector3d oldPos = getPosition();
	Havok::Vector3 newPos(oldPos.x,oldPos.y,oldPos.z);  //namespace just in case
	newPos *= SX_TO_HAVOK;
	
	//-- convert to havok coordinate system
//	sxVector3d oldOrient = getOrientation();
//	Havok::Vector3 newOrient(oldOrient.x,oldOrient.y,oldOrient.z);
//	Havok::Quaternion qx(newOrient.x,Vector3(1,0,0)); 
//	Havok::Quaternion qy(newOrient.y,Vector3(0,1,0)); 
//	Havok::Quaternion qz(newOrient.z,Vector3(0,0,1)); 
//	Havok::Quaternion qx90(0.5f*PI,Vector3(1,0,0)); 
//
//	Havok::Quaternion qf = (qx)*(qy);
//	qf = qf*(qz);
//	qf = qf * qx90;


	Havok::Quaternion qx90(0.5f*PI,Vector3(1,0,0)); 
	Havok::Quaternion qf = SXQ_TO_Q(getOrientationQ());
	qf = qf * qx90;

	//-- set the havok rigid body position & orientation
	vehicle->getChassisRigidBody()->setDisplayToWorldTransform(newPos,qf);
	crashBike->setDisplayToWorldTransform(newPos,qf);

}

//------------------------------------------------------------------------
void HavokBike::updateModelObjectFromHavok()
{
    
	//retrieve the current state of the objects from 
	//the physics world
	Vector3 pos;		// Current Position
	Quaternion quat;	// Current Orientation
	vehicle->getChassisRigidBody()->getDisplayToWorldTransform(pos,quat);
	crashBike->setDisplayToWorldTransform(pos,quat);	

	//-- set the orientation
	Havok::Quaternion qx90(0.5f*PI,Vector3(1,0,0)); 
    quat = quat * qx90;
	sxQuaternion q = Q_TO_SXQ(quat);
    setOrientationQ( q );

	// Set the position
	pos *= HAVOK_TO_SX;		
	setPosition(V3_TO_SXV(pos));
}


sxVector3d startPosition;
sxQuaternion startOrientation;
//CGameOptions* gameOptions = CSxGame::getGameOptions();

//------------------------------------------------------------------------
void HavokBike::update(sxtime_t _time)
{
	sxVector3d Temp;



	static int Done = 0;

	if (isOffTrack && !isCPU)
	{
		setTrackToHudOffTrack( TRACK_TO_HUD_OFF_TRACK );
		
//		x_printfxy(100, 100, "Off Track or Missed Checkpoint");
//		x_printfxy(100, 125, "Press L1 & L2 to return to track");
	}
	else
	{
		setTrackToHudOffTrack( TRACK_TO_HUD_ON_TRACK );
	}


	frameCountForMessage++;
/*
Tyson help
	if (getButton(POWERSLIDE_BIKE_BUTTON) && 
		getButton(PRELOAD_BIKE_BUTTON) &&
		getButton(WANNA_'_FAKE_BUTTON))
	{
		CSplineManager *pSM = CSplineManager::getSplineManager();
		sxVector3d newPos, newDir;
		pSM->getPoint(getWayPointTime(), CSplineManager::WayPoint, newPos, newDir);
		resetButton(WANNA_CRASH_FAKE_BUTTON);
		setStartPos(newPos, newDir);
	}
*/

	if (wheelBase == 0.0f)
	{
		wheelBase = getBackWheelPosition().distance(&getFrontWheelPosition());
		bikeLength = wheelBase + (2.0f * getWheelRadius());

		setAttributes();
	}

	if (isCPU)
	{
		if (jumpAssist || nitroState != NO_NITRO)
			throttleZero = 1.0f;
		else
			throttleZero = lastThrottleZero;
	}


	clearFlags();

	getBikeStats()->setRaceTime( CGlobalTime::getGlobalTime() );
	getBikeStats()->calculateHoleShot();

	//retrieve the current state of the objects from 
	//the physics world


	getBaseVectors();

	lastWheelInfo[FW] = wheelInfo[FW];
	lastWheelInfo[BW] = wheelInfo[BW];

	wheelInfo[FW] = m_wheels->getWheelInfo(FW);
	wheelInfo[BW] = m_wheels->getWheelInfo(BW);

	if (wheelInfo[FW].getIsInContact())
	{
		wheelFramesInContact[FW]++;
		wheelFramesInAir[FW] = 0;
	}
	else
	{
		wheelFramesInContact[FW] = 0;
		wheelFramesInAir[FW]++;
	}
	if (wheelInfo[BW].getIsInContact())
	{
		wheelFramesInContact[BW]++;
		wheelFramesInAir[BW] = 0;
	}
	else
	{
		wheelFramesInContact[BW] = 0;
		wheelFramesInAir[BW]++;
	}


	if (crashState == RIDING_STATE)
	{
		bikeTerrainInfo();

		getTerrainType();
		getWheelieCondition();
		getMovingInfo();
	}

	Vector3 pos;		// Current Position
	sxVector3d sxPos;
	Quaternion quat;	// Current Orientation


	if (crashState == RIDING_STATE)
	{
		vehicle->getChassisRigidBody()->getDisplayToWorldTransform(pos,quat);
		crashBike->setDisplayToWorldTransform(pos,quat);	
	}
	else
	{
		crashBike->getDisplayToWorldTransform(pos,quat);
		vehicle->getChassisRigidBody()->setDisplayToWorldTransform(pos,quat);

//			//-- set the orientation
//			Havok::Quaternion qx90(0.5f*PI,Vector3(1,0,0)); 
//		    quat = quat * qx90;
//			sxQuaternion q = Q_TO_SXQ(quat);
//		    setOrientationQ( q );
//		
//			// Set the position
//			pos *= HAVOK_TO_SX;		
//			setPosition(V3_TO_SXV(pos));

	}


	q = Q_TO_SXQ(quat);
	sxPos = V3_TO_SXV(pos);





	cameraLookAtPos = sxPos + (upDir * 0.15f) + (forwardDir * 1.0f);


	// Set the position
	pos *= HAVOK_TO_SX;		
	CModelObject::setPosition(V3_TO_SXV(pos));


    setOrientationQ( q );

	CGameOptions* gameOptions = CSxGame::getGameOptions();		


	//-- if the race has yet to start then just return
//	if( !startRace )
//	{
//		startPosition = getPosition();
//		startOrientation = q;
//		return;
//	}

	zeroVelocity();
	limitMaximumSpeed();
//	if (!wheelieCondition)
		jumpSpeed();
	landSpeed();


	lastLinearVelocity = linearVelocity;
	lastLinearVelocityMag = linearVelocityMag;

	lastVelocityMag[lastVelocityMagIndex] = linearVelocityMag;
	lastVelocity[lastVelocityMagIndex] = linearVelocity;
	lastVelocityMagIndex++;
	lastVelocityMagIndex %= NUM_VELOCITY_INDEXES;


	linearVelocity = V3_TO_SXV(chassis->getLinearVelocity());
	linearVelocityMag = linearVelocity.magnitude();

	forwardVelocityMag = linearVelocity * forwardDir;


	if (crashState == RIDING_STATE)
	{


		if (!wheelInfo[FW].getIsInContact() || !wheelInfo[BW].getIsInContact())
			((CBikeEngine*)m_engine)->disableEngineBoost();
		else
			((CBikeEngine*)m_engine)->enableEngineBoost();
	
	
	
	
		if(checkMessage(BIKEACTION_ACCEL))
			setFlag(BIKE_ACCEL);
		else
			setFlag(BIKE_DECEL);
	
		if(checkMessage(BIKEACTION_BRAKE))
			setFlag(BIKE_BRAKING);
	
		if (wheelInfo[FW].getIsInContact() || wheelInfo[BW].getIsInContact())
			setFlag(BIKE_ON_GROUND);
		else
			setFlag(BIKE_OFF_GROUND);
	
		if (inAir && (wheelInfo[FW].getIsInContact() || wheelInfo[BW].getIsInContact()))
		{
			setFlag(BIKE_LAND);
			// This can't go here, cause the land is not necesarrily successfull
//			getBikeStats()->jumpLandSuccessful( getPosition(), CGlobalTime::getGlobalTime() );
			inAir = FALSE;
		}


		if (crashFramesInAir > 5 && (wheelInfo[FW].getIsInContact() || wheelInfo[BW].getIsInContact()))
		{
			crashLand = TRUE;
		}
		else
		{
			crashLand = FALSE;
		}
	
		if (!wheelInfo[FW].getIsInContact() && !wheelInfo[BW].getIsInContact())
		{
			if (getDistToGround() > 15.0f)
				crashFramesInAir++;
		}
		else
		{
			crashFramesInAir = 0;
		}



		if (!wheelInfo[FW].getIsInContact() && !wheelInfo[BW].getIsInContact())
		{
			if (getDistToGround() > 36.0f)
				framesInAir++;
		}
		else
		{
			framesInAir = 0;
		}
	
		if (framesInAir == IN_AIR_FRAME)
		{
			setFlag(BIKE_TAKEOFF);
			inAirLinearVelocityMag = linearVelocityMag;
			getBikeStats()->jumpStart( getPosition(), CGlobalTime::getGlobalTime() );
			inAir = TRUE;
		}
	
		if (inAir)
		{
			setFlag(BIKE_INAIR);
	
			if (getFlag(BIKE_ACCEL))
				inAirLinearVelocityMag += IN_AIR_VELOCITY_ACCEL_STEP;
			else if (getFlag(BIKE_BRAKING))
				inAirLinearVelocityMag -= IN_AIR_VELOCITY_BRAKE_STEP;
			else
				inAirLinearVelocityMag -= IN_AIR_VELOCITY_STEP;
	
			if (inAirLinearVelocityMag > TO_METERS_PER_SEC(attributes.maximumSpeed))
				inAirLinearVelocityMag = TO_METERS_PER_SEC(attributes.maximumSpeed);
		}
	}


	if (crashState == RIDING_STATE)
	{
		groundCollision();


		restitution();
		zeroThrottleBoost();
//		zeroVelocity();
		dampenAngularVelocity();



		if(startRace)
			control();
		else
		{
			forceZeroVelocity();
			startPosition = getPosition();
			startOrientation = q;
		}

//		if (!wheelieCondition)
			preLoad();
		inAirAdjust();
		nitroBoost();
		riderDisplacement();

		((CBikeEngine*)m_engine)->setVelocity(linearVelocityMag);

//		jumpSpeed();
		zeroLinearVelocity();
		if (linearVelocityMag > MIN_ZERO_LINEAR_VEL)
			disableLeanTarget();


		emitParticleTrail();

	#if PRINT_EVERY_FRAME
	//	x_printf("roll = %f\n",bikeRoll); 
	#endif
	}

	collide();

	//-- Check Gates --//
	checkPoint( );

	//-- update the track bonuses
	if( isCPU == FALSE )
		trackBonus.update( getBikeStats() );
	
	crashUpdate();

	if( getCurLap() == gameOptions->getDuration() )
	{
		if( !Done )
		{
			Done = 1;
			forceZeroVelocity();
			setPosition(startPosition);
			startOrientation *= sxQuaternion(sxVector3d(1.0f,0.0f,0.0f),-R_90);
			setOrientationQ(startOrientation);
//			updateModelWorld();
			updateHavokFromModelObject();
		}
	}
	if(Done)
	{
		Temp = getPosition();
//		printf("Start2X = %f, Start2Y = %f, Start2Z = %f\n",Temp.x, Temp.y, Temp.z);
	}


}





//------------------------------------------------------------------------
void HavokBike::updateModelWorld()
{
	//when the DynamicObject is added to model, then we must add
	//whatever we need added to the havok world at that time
		//for the body
	CModel *model = CModel::getInstance();
	Toolkit* toolkit = model->getToolkit();

	Havok::Vector3 newPos(0.0f,0.0f,0.0f);  //namespace just in case
	Havok::Quaternion qf(0.0f,Vector3(1,0,0)); 
	toolkit->addRigidBody(vehicle->getChassisRigidBody(),newPos,qf);	

	toolkit->addRigidBody(frontWheel,newPos,qf);	
	toolkit->addRigidBody(backWheel,newPos,qf);	
	toolkit->addRigidBody(crashBike,newPos,qf);	


	updateHavokFromModelObject();

	
	Subspace* sub = toolkit->m_defaultSubspace;
	RRResolver* res = sub->getRRResolver();

	EventPipe* pipe=res->getEventPipe();
	
	//This Havok object won't work with dynamic_cast, possibly because
	//the Havok libs weren't compiled with RTTI enabled. I'm using an
	//old-style, dangerous C cast instead. Warning! Do not try this at home!
	//Trained professional, closed course!
//	CollisionEventFilter* cef = dynamic_cast<CollisionEventFilter*>(pipe);
	CollisionEventFilter* cef = (CollisionEventFilter*)(pipe);

	for(int i=0;i<chassis->getNumPrimitives();i++)
		cef->registerInterest(chassis->getPrimitive(i), HK_NULL, 1);
	for(int i=0;i<frontWheel->getNumPrimitives();i++)
		cef->registerInterest(frontWheel->getPrimitive(i), HK_NULL, 1);
	for(int i=0;i<backWheel->getNumPrimitives();i++)
		cef->registerInterest(backWheel->getPrimitive(i), HK_NULL, 1);





/*
	sub->registerCallbackObject( 
			new EventCallbackProxy<HavokBike>(this, &HavokBike::collisionReact),
			CollisionEvent::defaultID );
*/
//	setCenterOfMass(sxVector3d(0,0,0));
}



//-------------------------------------------------------------------


//-------------------------------------------------------------------
f32 HavokBike::getDistToGround()
{
	if (wheelInfo[FW].getIsInContact() || wheelInfo[BW].getIsInContact())
//	if (vehicle->getWheelBodyInfoRef(0).getIsInContact() || vehicle->getWheelBodyInfoRef(1).getIsInContact())
	{
		return 0.0f;
	}
	else
	{
		CWorldModel*	worldModel = ((CSXModel*) CModel::getInstance())->getWorldModel();
		f32 y = worldModel->getHeightAt(getBackWheelPosition().x, getBackWheelPosition().z);
		// negative y - more negative y = negative y + bigger positive y = positive result

		sxVector3d backWheelPos = getBackWheelPosition();
		f32 dist = y - (backWheelPos.y + getWheelRadius());
		// return the result
		return dist;
	}
}




f32 HavokBike::getDistToGround(sxVector3d pos)
{
	CWorldModel*	worldModel = ((CSXModel*) CModel::getInstance())->getWorldModel();
	f32 y = worldModel->getHeightAt(pos.x, pos.z);
	f32 dist = y - (pos.y + getWheelRadius());
	if (dist < 0.0f)
		dist = 0.0f;
	return dist;
}

f32 HavokBike::getDistToGround(u32 wheel)
{
	if (wheelInfo[wheel].getIsInContact())
//	if (vehicle->getWheelBodyInfoRef(wheel).getIsInContact())
	{
		return 0.0f;
	}
	else
	{
		sxVector3d pos;
		if (wheel == 0)
			pos = getFrontWheelPosition();
		else
			pos = getBackWheelPosition();
			
		CWorldModel*	worldModel = ((CSXModel*) CModel::getInstance())->getWorldModel();
		f32 y = worldModel->getHeightAt(pos.x, pos.z);
		f32 dist = y - (pos.y + getWheelRadius());
		return dist;
	}
}

void HavokBike::setCenterOfMass(sxVector3d com)
{
	const Vector3 massCenterShift = SXV_TO_V3(com);
	chassis->shiftCentreOfMass(massCenterShift);

	Vector3 wheelsHardpointCS[2];
	wheelsHardpointCS[0] = ((CBikeSuspension*)m_suspension)->getWheelHardpointCS(0)-massCenterShift;
	wheelsHardpointCS[1] = ((CBikeSuspension*)m_suspension)->getWheelHardpointCS(1)-massCenterShift;

	((CBikeSuspension*)m_suspension)->setWheelHardpointCS (0, wheelsHardpointCS[0]);
	((CBikeSuspension*)m_suspension)->setWheelHardpointCS (1, wheelsHardpointCS[1]);
}

//-------------------------------------------------------------------











typedef struct _DustTerrainData
{
	u32 r, g, b, a;
	u32 size;
	f32 expand;
	u32 duration;
	f32 freqScale;
} DustTerrainData;

typedef struct _DustTerrain
{
	DustTerrainData data[2];
} DustTerrain;

#define NUM_DUST_TERRAINS	15
DustTerrain dustTerrainTable[NUM_DUST_TERRAINS] = 
{
	{	130,100,60,38,		40,	0.185f,		9,	1.0f,	//-- hardpack	
		130,100,60,38,		50, 0.33f,		15, 2.0f}, 
//	{	150,120,50,38,		40,	0.185f,		9,	1.0f,	//-- hardpack	
//		150,120,50,38,		50, 0.33f,		15, 2.0f}, 
	{	100,100,100,38,		30,	0.185f,		5,	1.0f,	//-- cement	
		100,100,100,38,		40, 0.25f,		15, 2.0f}, 
	{	130,100,50,38,		50, 0.185f,		9,	1.0f,	//-- loam	
		130,100,50,38,		60, 0.33f,		15, 2.0f}, 
	{	200,120,50,38,		50, 0.185f,		9,	1.0f,	//-- clay	
		200,120,50,38,		60, 0.33f,		15, 2.0f}, 
	{	220,220,80,38,		60, 0.28f,		9,	1.0f,	//-- sand	
		220,220,80,38,		70, 0.38f,		15, 2.0f}, 
	{	114,100,36,15,		30, 0.33f,		6,	1.0f,	//-- grass	
		114,100,36,4,		40, 0.44f,		8, 2.0f}, 
	{	50,50,50,38,		50, 0.185f,		9,	1.0f,	//-- gravel	
		50,50,50,38,		60, 0.25f,		8, 2.0f}, 
	{	10,10,10,38,		10, 0.185f,		9,	1.0f,	//-- oil	
		10,10,10,38,		20, 0.125f,		15, 2.0f}, 
	{	150,120,50,38,		40, 0.185f,		9,	1.0f,	//-- wet hardpack	
		150,120,50,38,		50, 0.33f,		15, 2.0f}, 
	{	100,100,100,38,		30, 0.185f,		5,	1.0f,	//-- wet cement	
		100,100,100,38,		40, 0.25f,		15, 2.0f}, 
	{	130,100,50,38,		50, 0.185f,		9,	1.0f,	//-- wet loam	
		130,100,50,38,		60, 0.33f,		15, 2.0f}, 
	{	200,120,50,38,		50, 0.185f,		9,	1.0f,	//-- wet clay	
		200,120,50,38,		60, 0.33f,		15, 2.0f}, 
	{	220,220,80,38,		60, 0.28f,		9,	1.0f,	//-- wet sand	
		220,220,80,38,		70, 0.38f,		15, 2.0f}, 
	{	114,100,36,15,		30, 0.33f,		6,	1.0f,	//-- wet grass	
		114,100,36,4,		40, 0.44f,		8, 2.0f}, 
	{	50,50,50,38,		50, 0.185f,		9,	1.0f,	//-- wet gravel	
		50,50,50,38,		60, 0.25f,		8, 2.0f}, 
};



const f32 PARTICLE_FRONT_WHEEL_ON_GROUND = 20.0f;
const f32 PARTICLE_BACK_WHEEL_ON_GROUND = 20.0f;

void HavokBike::emitParticleTrail()
{
	CGameOptions* gameOptions = CSxGame::getGameOptions();		
	CTerrainType* terrain = CTerrainType::getInstance();
	CTerrainType::TypeList newTerrainType = terrain->getTypeAtPoint(getPosition().x, getPosition().z);

	if ((newTerrainType > CTerrainType::offtrack && newTerrainType < CTerrainType::special) || newTerrainType == CTerrainType::autoCrash)
	{
		isOffTrack = true;
	}
	else
	{
		isOffTrack = false;
	}

	if( !gameOptions->getGameOptions( GAMEOPTION_PARTICLES_ON ) )
		return;

	f32 frontDistScale;
	f32 backDistScale;

	bool frontWheelOnGround = FALSE;
	bool backWheelOnGround = FALSE;

	if (getDistToGround(0) < PARTICLE_FRONT_WHEEL_ON_GROUND)
		frontWheelOnGround = TRUE;

	if (getDistToGround(1) < PARTICLE_BACK_WHEEL_ON_GROUND)
		backWheelOnGround = TRUE;

	if (frontWheelOnGround && !backWheelOnGround)
	{
		frontDistScale = 2.0f;
		backDistScale = 0.0f;
	}
	else if (!frontWheelOnGround && backWheelOnGround)
	{
		frontDistScale = 0.0f;
		backDistScale = 2.0f;
	}
	else
	{
		frontDistScale = 4.0f;
		backDistScale = 4.0f;
	}
	backDistScale = 4.0f;

	if (getFlag(BIKE_POWERSLIDE_LEFT) || getFlag(BIKE_POWERSLIDE_RIGHT))
	{
		frontDistScale *= 0.33f;
		backDistScale *= 0.33f;
	}


	sxVector3d velocityDir = linearVelocity;
	velocityDir.y = 0.0f;
	velocityDir.normalize();



//	if (frontWheelOnGround && linearVelocityMag > TO_METERS_PER_SEC(DUST_MIN_VELOCITY))
//		emitDust(this->getFrontWheelPosition() + (velocityDir * 1.0f * HAVOK_TO_SX), &numFrontDustParticles, frontDistScale);
	if (backWheelOnGround && linearVelocityMag > TO_METERS_PER_SEC(DUST_MIN_VELOCITY))
//		emitDust(this->getBackWheelPosition() + (velocityDir * 1.0f * HAVOK_TO_SX), &numBackDustParticles, backDistScale);
		emitDust(this->getBackWheelPosition() + (velocityDir * 2.0f * HAVOK_TO_SX), &numBackDustParticles, backDistScale);
	emitExhaust();


	//*********************pyw
	getMovingInfo();
	if(wheelInfo[BW].getIsInContact() && movingForward)
	{
		emitTrail();
		emitMud();
	}
	//*************************

}



//-------------------------------------------------------------------
void HavokBike::emitDust(sxVector3d startPos, f32* numParticles, f32 distScale)
{
	CGameOptions* gameOptions = CSxGame::getGameOptions();		
	if( !gameOptions->getGameOptions( GAMEOPTION_PARTICLES_ON ) )
		return;

	CQuadParticle	dust;
	CQuadParticle	dust2;

/*	CModel *model = CModel::getInstance();
	CCamera * cam = dynamic_cast<CCamera*>(model->getObjectByID(cameraId));
	if (!cam)
	{
		return;
	}

	sxVector3d forward = cam->getForwardVec();
	sxVector3d right = cam->getRightVec();
	sxVector3d up = cam->getUpVec(); 

	forward = -forward; */


//	CTerrainType* terrain = CTerrainType::getInstance();
//	CTerrainType::TypeList newTerrainType = terrain->getTypeAtPoint(getPosition().x, getPosition().z);

	u32 dustTerrainType = getTerrainParticleType(terrainType);

	u32 dustTerrainI = (dustCloudCounter > 0)? 1:0;
//	dustTerrainI = 0;

	DustTerrainData* terrainData;
//	if (newTerrainType < NUM_DUST_TERRAINS)
//		terrainType = (u32)newTerrainType;
//	else
//		newTerrainType = (CTerrainType::TypeList)terrainType;

	//terrainType = 0;
/*
	if (terrainType > CTerrainType::offtrack && terrainType < CTerrainType::special)
	{
		terrainType -= CTerrainType::offtrack+1;
	}

	if (terrainType < CTerrainType::wetstuff)
	{
		terrainData = &dustTerrainTable[terrainType].data[dustTerrainI];
	}
	else if (terrainType < CTerrainType::offtrack)
	{
		terrainType -= CTerrainType::wetstuff+1;
		terrainType += 8; // ACK!  Hardcoded from looking at the dust table.
		terrainData = &dustTerrainTable[terrainType].data[dustTerrainI];
	}
*/
	terrainData = &dustTerrainTable[0].data[dustTerrainI];
//	terrainData = &dustTerrainTable[dustTerrainType].data[dustTerrainI];

	dust.color = CColorRGBA(terrainData->r, terrainData->g, terrainData->b, terrainData->a);
	dust.size = sxVector3d(terrainData->size, 0, terrainData->size);
	dust.expand = terrainData->expand;
//		dust.duration = 0.5f * terrainData->duration;
//	dust.duration = 0.5f * terrainData->duration;
	dust.duration = 1.0f * terrainData->duration;

//	f32 particleDist = 3.0f * terrainData->size;
//	f32 particleDist = 5.0f * terrainData->size;
	f32 particleDist = distScale * terrainData->size;
//		f32 particleDist = 4.0f * terrainData->size;

//	particleDist = 150.0f;

	sxVector3d pos;
	sxVector3d dustStartPt2;
	sxVector3d dustStartPt3;
	f32 xOffset = terrainData->size/100.0f;
	f32 zOffset = particleDist/100.0f;

//	sxVector3d forwardDir = -forwardDir;
//	forwardDir = -linearVelocity;
//	forwardDir.normalize();

	pos =	((startPos * SX_TO_HAVOK) + 
			(upDir * 0.0f) + 
//			(up * 0.0f) + 
			((fForwardDir) * -1.25f)) * HAVOK_TO_SX;
//			((forward) * -1.25f)) * HAVOK_TO_SX;

//	sxVector3d rightNoLeanDir = rightDir;
//	rightNoLeanDir.y = 0.0f;
//	rightNoLeanDir.normalize();


	sxVector3d offset[3];
	offset[0] = sxVector3d(0, 0, 0);
	offset[1] = ((upDir * 0.0f) + 
				 (fRightDir * (0.33f * xOffset)) + 
				 ((fForwardDir) * (0.5f * zOffset))) * HAVOK_TO_SX;
//	offset[1] = ((up * 0.0f) + 
//				 (right * (0.33f * xOffset)) + 
//				 ((forward) * (0.5f * zOffset))) * HAVOK_TO_SX;

	offset[2] = ((upDir * 0.0f) + 
				 (fRightDir * -(0.33f * xOffset)) + 
				 ((fForwardDir) * ((0.5f * zOffset) - 0.05f))) * HAVOK_TO_SX;

//	offset[2] = ((up * 0.0f) + 
//				 (right * -(0.33f * xOffset)) + 
//				 ((forward) * ((0.5f * zOffset) - 0.05f))) * HAVOK_TO_SX;



	f32 linearVelMag = MIN(linearVelocityMag, MAX_VELOCITY);
//	f32 linearVelMag = MIN(10.0f, MAX_VELOCITY);
	(*numParticles) += linearVelMag * 100.0f / 60.0f / particleDist;
	(*numParticles) *= terrainData->freqScale;



	dust.type = QUAD_PARTICLE;
	dust.flags = P_EXPAND_FLAG | P_FACE_CAMERA_FLAG | P_SORT_FLAG | P_EMIT_COPY_FLAG;

	dust.time = 0.0f;
	dust.texID = dustTexID;
	dust.gravity = 0.0f;

	dust.colorRate = CColorRateRGBA(0, 0, 0, -(dust.color.a / (2.0f * dust.duration)));



	sxVector3d velocityDir = linearVelocity;
	velocityDir.normalize();
	sxVector3d delta = (-velocityDir) * particleDist;
//	sxVector3d delta = (-forwardDir) * particleDist;
//	sxVector3d delta = (forward) * particleDist;

	CParticleSystem* particleSystem = CParticleSystem::getInstance();

	sxVector3d sPos = pos;
	while((*numParticles) >= 1.0f)
	{
		pos = sPos + (delta * (*numParticles));
		for(s32 i = 0; i < 3; i++)
		{
			dust.pos = pos + offset[i];
			dust2 = dust;
			dust2.colorRate = CColorRateRGBA(0, 0, 0, -(dust.color.a / (2.0f * dust.duration)));
//			dust.particle = (CParticle*)&dust2;
			dust2.particle = (CParticle*)NULL;
			dust2.gravity = 2.5f;
			if (dustCloudCounter > 0)
				dust2.expand = 0.25f * terrainData->expand;
			else
				dust2.expand = 1.25f * terrainData->expand;

			dust2.flags = P_EXPAND_FLAG | P_FACE_CAMERA_FLAG | P_SORT_FLAG | P_GRAVITY_FLAG | P_MOVE_FLAG;
			particleSystem->emitParticle(&dust);
		}

		(*numParticles) -= 1.0f;
	}
	if (dustCloudCounter > 0)
		dustCloudCounter--;
}



//pyw
// Emit particles when make fast turns and powerslids
////////////////////////////////////////////////////////////////////
void HavokBike::emitMud()
{
	CGameOptions* gameOptions = CSxGame::getGameOptions();		
	if( !gameOptions->getGameOptions( GAMEOPTION_PARTICLES_ON ) )
		return;

	// conditions for emiting 
	if( !checkMessage(BIKEACTION_RIGHT_TURN) && !checkMessage(BIKEACTION_LEFT_TURN)
		&&!checkMessage(BIKEACTION_RIGHT_POWERSLIDE_ACCEL) && !checkMessage(BIKEACTION_LEFT_POWERSLIDE_ACCEL))
	{
		return;
	}

	if(!checkMessage(BIKEACTION_ACCEL))
	{
		return;
	}


	CQuadParticle	mud;

	f32 particleDist;

	particleDist = 30.0f;
	mud.duration = 30; //6;

	//change color here
	mud.color = CColorRGBA(50, 50, 50, 50);


	f32 velocity = linearVelocityMag;

	// emits only it's fast enough
	if (velocity < 10.0f)
	{
		return;
	}

	velocity = MAX(10.0f, velocity);
	velocity = MIN(50.0f, velocity);

	numMudParticles += velocity * 100.0f / 60.0f / particleDist;


	mud.type = QUAD_PARTICLE;
	mud.flags = P_FACE_CAMERA_FLAG | P_SORT_FLAG | P_MOVE_FLAG | P_GRAVITY_FLAG;
	mud.time = 0.0f;
	mud.texID = dustTexID;

	// Deal with Gravity*****************
	mud.gravity = 0;	
	//***********************************


	//Deal with mud velocity******************
	sxVector3d wheelNormVec[2];
	CWorldModel*	worldModel = ((CSXModel*) CModel::getInstance())->getWorldModel();			
	sxVector3d triVertex[3];
	worldModel->getLocation(triVertex, (s32) (getBackWheelPosition().x), (s32) (getBackWheelPosition().z));
	TRI_Normal(&wheelNormVec[BW], &triVertex[0], &triVertex[1], &triVertex[2]);


	sxVector3d terrainUpVector = wheelNormVec[BW];
	sxVector3d terrainForwardVector = wheelNormVec[BW].cross(fRightDir);

	//change the direction here
	sxVector3d mudVector = terrainUpVector - terrainForwardVector * 3.0f;
	//**************************

	mudVector.normalize();

	//Randomize the velocity vector so that it looks like moving
	f32 random = (f32)(GetRandom32Bits() % 1000) / 20.0f;
	mudVector = mudVector * random;
	mud.vel= mudVector;
	//***************************************//

	// Set the size of the particle
	mud.size = sxVector3d(10, 0, 10);
	//***************************


	mud.expand = 0.0f;


	mud.colorRate = CColorRateRGBA(0, 0, 0, -(mud.color.a / (mud.duration)));
//	sxVector3d mudPos =	((getBackWheelPosition() * SX_TO_HAVOK) + 
	sxVector3d mudPos =	((getPosition() * SX_TO_HAVOK) + 
							(-upDir * 0.30f) + 
							((-forwardDir) * 0.125f) + 
							(rightDir * 0.0f)) * HAVOK_TO_SX;


	sxVector3d delta = (-forwardDir) * particleDist;
	CParticleSystem* particleSystem = CParticleSystem::getInstance();

	sxVector3d exSPos = mudPos;
	sxVector3d exPos;
	while(numMudParticles >= 1.0f)
	{
		exPos = exSPos + (delta * numMudParticles);
		mud.pos = exPos;
		particleSystem->emitParticle(&mud);
		numMudParticles -= 1.0f;
	}

}


// Function for tire trail *******************************PYW
void HavokBike::emitTrail()
{
	
	CGameOptions* gameOptions = CSxGame::getGameOptions();		
	if( !gameOptions->getGameOptions( GAMEOPTION_PARTICLES_ON ) )
		return;


	CQuadParticle trail;
	f32 particleDist;

	f32 velocity = linearVelocityMag;
//	velocity = MAX(100.0f, velocity);
//	velocity = MIN(180.0f, velocity);

	if(velocity < 2.0f)
	{
		return;								
	}

//	velocity = MAX(10.0f, velocity);
	velocity = MIN(50.0f, velocity);

	trail.color = CColorRGBA(128, 128, 128, 200);//velocity); // 128 means not transparent


	particleDist =30.0f;//distance =< the size of that dimension of the particle
//	particleDist =5.0f;
//	trail.duration = -1;
//	numTrailParticles += velocity * 75.0f/ particleDist;
	numTrailParticles = 7.0f;
	trail.size = sxVector3d(10,10,0);
	trail.duration = 30; //if set it bigger, the game would be slower

	// Deal with Left and Right Turn
	if(checkMessage(BIKEACTION_RIGHT_TURN) || checkMessage(BIKEACTION_LEFT_TURN))
	{
		particleDist =30.0f;
		numTrailParticles = 7.0f;
		trail.size = sxVector3d(10,10,0);
		trail.duration = 30; //if set it bigger, the game would be slower

		//Deal with Powerslides
		if ( checkMessage(BIKEACTION_RIGHT_POWERSLIDE_ACCEL) || checkMessage(BIKEACTION_LEFT_POWERSLIDE_ACCEL))
		{
			particleDist =30.0f;
			numTrailParticles = 7.0f;
			trail.size = sxVector3d(10,10,0);
			trail.duration = 30; //if set it bigger, the game would be slower
			//	trail.color = CColorRGBA(0, 0, 0, 40); // 128 means not transparent
		}
	}


	trail.type = QUAD_PARTICLE;
	trail.flags = P_SORT_FLAG;
	trail.time = 0.0f;
	trail.texID = TrailTexID;
	trail.gravity = 0.0f;
	
	
	//trail.size = sxVector3d(10,10,0);
	//trail.size = sxVector3d(8, 50, 0);
	//trail.expand = 0.68f;

// Calculate Matrix for tire trail instead of using camera matrix
	
	//get right vector
	/*
	CParticleSystem* ps = CParticleSystem::getInstance();
	CModel* model = CModel::getInstance();
	CModelObject *camInstance;
	camInstance = (CModelObject *)model->getObjectByID(ps->getCameraId());
	CCamera * cam = (CCamera*)camInstance;
					
	sxVector3d rightDir = cam->getRightVec();		
    */

	//get the rest vectors
	//CModelObject* obj = model->getObjectByID( getModelObjectId() );
	//CModelObject* pObj=CModel::getInstance()->getObjectByID(getModelObjectId());
	//HavokBike* havokBike=dynamic_cast<HavokBike*>(pObj);
	matrix4	m;
				
	sxVector3d wheelNormVec[2];
	CWorldModel*	worldModel = ((CSXModel*) CModel::getInstance())->getWorldModel();
	sxVector3d triVertex[3];
	worldModel->getLocation(triVertex, (s32) (getBackWheelPosition().x), (s32) (getBackWheelPosition().z));
	TRI_Normal(&wheelNormVec[BW], &triVertex[0], &triVertex[1], &triVertex[2]);


	// 3 vectors calculated perpendicular to each other
	sxVector3d terrainUpVector = wheelNormVec[BW];
	sxVector3d terrainForwardVector = wheelNormVec[BW].cross(fRightDir);
	sxVector3d terrainRightVector = terrainForwardVector.cross(wheelNormVec[BW]);

	M4_Identity(&m);

	m.M[2][0] = terrainUpVector.x;
	m.M[2][1] = terrainUpVector.y;
	m.M[2][2] = terrainUpVector.z;

	m.M[0][0] = terrainRightVector.x;
	m.M[0][1] = terrainRightVector.y;
	m.M[0][2] = terrainRightVector.z;

	m.M[1][0] = terrainForwardVector.x;
	m.M[1][1] = terrainForwardVector.y;
	m.M[1][2] = terrainForwardVector.z;
//*********************End of calculating the matrix				



	trail.colorRate = CColorRateRGBA(0, 0, 0, -(trail.color.a / (trail.duration)));
//	sxVector3d trailPos =	((getBackWheelPosition() * SX_TO_HAVOK) + 
	sxVector3d trailPos =	((getPosition() * SX_TO_HAVOK) + 
							(-upDir * 0.35f) + 
							((-forwardDir) * 0.125f) + 
							(rightDir * 0.0f)) * HAVOK_TO_SX;

	static sxVector3d prevPos(0.0f, 0.0f, 0.0f);

	sxVector3d delta = (-forwardDir) * particleDist; // bridge the gaps between particles
	//delta = (delta + (prevPos - trailPos)) / 2.0;

	//fill the gap between 2 particles with some particles
	delta = (prevPos - trailPos) / (numTrailParticles-1);
	
	CParticleSystem* particleSystem = CParticleSystem::getInstance();

	sxVector3d exSPos = trailPos;
	sxVector3d exPos;

	while(numTrailParticles >= 1.0f)
	{
		exPos = exSPos + (delta * numTrailParticles);
		trail.pos = exPos;

		particleSystem->emitParticle(&trail, m);
		numTrailParticles -= 1.0f;
	}
	prevPos = trailPos;

}





//-------------------------------------------------------------------
void HavokBike::emitExhaust()
{
	CGameOptions* gameOptions = CSxGame::getGameOptions();		
	if( !gameOptions->getGameOptions( GAMEOPTION_PARTICLES_ON ) )
		return;

	CQuadParticle	exhaust;

	f32 particleDist;
	if (checkMessage(BIKEACTION_ACCEL))
	{
		particleDist = 30.0f;
		exhaust.duration = 5; //6;
		exhaust.color = CColorRGBA(50, 50, 50, 35);
	}
	else
	{
		particleDist = 30.0f;//30.0f;
		exhaust.duration = 5;
//		exhaust.duration = 100000;
//		exhaust.color = CColorRGBA(50, 50, 50, 40);
		exhaust.color = CColorRGBA(50, 50, 50, 50);
//		exhaust.color = CColorRGBA(128, 128, 128, 128);
	}

	f32 velocity = linearVelocityMag;
	velocity = MAX(10.0f, velocity);
	velocity = MIN(50.0f, velocity);

	numExhaustParticles += velocity * 100.0f / 60.0f / particleDist;


	exhaust.type = QUAD_PARTICLE;
//	exhaust.flags = P_FACE_CAMERA_FLAG | P_SORT_FLAG;
//	exhaust.flags = P_SORT_FLAG;
	exhaust.flags = P_EXPAND_FLAG | P_FACE_CAMERA_FLAG | P_SORT_FLAG;

	exhaust.time = 0.0f;
	exhaust.texID = dustTexID;
	exhaust.gravity = 0.0f;

//	exhaust.size = sxVector3d(15, 0, 15);
	exhaust.size = sxVector3d(25, 0, 25);
//	exhaust.size = sxVector3d(40, 0, 40);
	exhaust.expand = 0.68f;


	exhaust.colorRate = CColorRateRGBA(0, 0, 0, -(exhaust.color.a / (exhaust.duration)));
//	sxVector3d exhaustPos =	((getBackWheelPosition() * SX_TO_HAVOK) + 
	sxVector3d exhaustPos =	((getPosition() * SX_TO_HAVOK) + 
							(upDir * 0.45f) + 
							((-forwardDir) * 0.125f) + 
							(rightDir * 0.0f)) * HAVOK_TO_SX;


	sxVector3d delta = (-forwardDir) * particleDist;
	CParticleSystem* particleSystem = CParticleSystem::getInstance();

	sxVector3d exSPos = exhaustPos;
	sxVector3d exPos;
	while(numExhaustParticles >= 1.0f)
	{
		f32 randomUp  = -0.1f + 0.2f * (f32)(GetRandom32Bits() % 1000) / 1000.0f;
		f32 randomRight  = -0.1f + 0.2f * (f32)(GetRandom32Bits() % 1000) / 1000.0f;
		sxVector3d randomRightVec = (rightDir * randomRight) * HAVOK_TO_SX;
		sxVector3d randomUpVec = (upDir * randomUp) * HAVOK_TO_SX;

		exPos = exSPos + (delta * numExhaustParticles);
		exhaust.pos = exPos + randomRightVec + randomUpVec;
		particleSystem->emitParticle(&exhaust);
		numExhaustParticles -= 1.0f;
	}
}




sxVector3d			gatePoints[4];
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------//

void HavokBike::setStartPos(sxVector3d sPos, sxVector3d sDir)
{
	CWorldModel*	worldModel = ((CSXModel*) CModel::getInstance())->getWorldModel();
	f32 y0 = worldModel->getHeightAt(sPos.x, sPos.z);
	sDir.normalize();
	sxVector3d sPos2 = sPos + (sDir * wheelBase);
	f32 y1 = worldModel->getHeightAt(sPos2.x, sPos2.z);
	sPos.y = MIN(y0, y1);
	sPos.y -= 150.0f;

	disableCrashes();
	enableCrashes();

	isOffTrack = false; // we always teleport to an on-track location.
	lastPos = sPos;
	setPosition(sPos);


	// set up from angles
	f32 yAngle;
	yAngle = -x_atan2(- sDir.x, sDir.z);
	sxQuaternion q(0.0f, yAngle, 0.0f);	//-- this is the fix for the starting the direction
	setOrientationQ(q);
	updateModelWorld();

	objectid_t cameraId = getCameraId();
	if (cameraId != NULL_ID)
	{
		CSXModel*			model;
		model = (CSXModel*)CModel::getInstance();
		CCamera* camera = dynamic_cast<CCamera*>(model->getObjectByID(cameraId));
		if (camera)
		{
			// inform the camera that it may need to move
			camera->teleportIfFollowing();
		}
	}
}





void HavokBike::setPosition(const sxVector3d& pos) 
{
	Vector3 v3Pos = Vector3(pos.x, pos.y, pos.z);
	v3Pos *= SX_TO_HAVOK;

	if (chassis)
		chassis->setPosition(v3Pos);

	CModelObject::setPosition(pos);

}

void HavokBike::setOrientation(const sxVector3d& orient)
{
	if (chassis)
	{
		Vector3 v3Orient = SXV_TO_V3(orient);
		Havok::Quaternion qx(v3Orient.x,Vector3(1,0,0));
		Havok::Quaternion qy(v3Orient.y,Vector3(0,1,0));
		Havok::Quaternion qz(v3Orient.z,Vector3(0,0,1));
		
		Havok::Quaternion qf = (qx)*(qy);
		qf = qf*(qz);
		chassis->setRotation(qf);
	}

	CModelObject::setOrientation(orient);
}


void HavokBike::setOrientationQ(const sxQuaternion& Q)
{

	if (chassis)
	{
		Havok::Quaternion hq = SXQ_TO_Q(Q);
		chassis->setRotation(hq);
	}

	CModelObject::setOrientationQ(Q);
	
}

void HavokBike::maybeFlashWrongWay(const sxVector3d &rightWay)
{
	f32 dotProd;
	dotProd = forwardDir * rightWay;
	if (dotProd > 0.7f)
	{
		flashWrong = false;
		setTrackToHudWrongWay( TRACK_TO_HUD_RIGHT_WAY );
	}
	else if (!crashing && !recovering && (dotProd < -0.7f || flashWrong) && !isCPU)
	{
//		x_printfxy(100, 112, "Wrong Way -- Turn Around!");
		flashWrong = true;
		setTrackToHudWrongWay( TRACK_TO_HUD_WRONG_WAY );

	}
}


void HavokBike::checkPoint()
{
	CSXModel*			model;
	model = (CSXModel*)CModel::getInstance();
	CWorldModel* wm = (CWorldModel*) model->getWorldModel();
	CModelObject* GateModel;				
	GateModel = wm->getGateObject( 0 );
	if (!GateModel)
	{
		checkTerrainPoint(); // done with terrain types
	}
	else
	{
		checkCurrentGate(); // done with gates
	}
	lastPos = getPosition();
}

void HavokBike::checkTerrainPoint()
{
	CTerrainType * pTT;
	f32 iMag;
	s32 iCount, iMax;
	u32 curPoint;
	pTT = CTerrainType::getInstance();
	sxVector3d checkPos, intervalVec;
	checkPos = lastPos;
	intervalVec = getPosition() - lastPos;
	intervalVec.y = 0.0f; // not actually a 3-space operation, so zero out vertical
	iMag = intervalVec.magnitude();
	if (iMag > 0.0f)
	{
		intervalVec *= CSubtileModel::getGridSize() * 0.5f / iMag;
		iMax = CUtil::f32_to_s32(iMag / CSubtileModel::getGridSize() * 0.5f) + 1;
	}
	else
	{
		iMax = 1;
	}
	for (iCount = 0; iCount < iMax; iCount++)
	{
		curPoint = pTT->getCheckPoint(checkPos.x, checkPos.z);
		if (curPoint == currentCheckPoint)
		{
			currentCheckPoint++;
			isOffTrack = false;
			setTrackToHudOffTrack(TRACK_TO_HUD_ON_TRACK);
		}
		else if (curPoint < 255 && curPoint > currentCheckPoint)
		{
			if (isOffTrack && !isCPU)
			{
				x_printf("Missed checkpoint: %d, at %d\n", (s32)currentCheckPoint, (s32)curPoint);
				setTrackToHudMissedCheckPoint(TRACK_TO_HUD_MISSED_C_POINT);
			}
			// missed a checkpoint!!!
			// isOffTrack = true;
		}
		else
		{
			setTrackToHudMissedCheckPoint(TRACK_TO_HUD_MADE_C_POINT);
		}

		checkPos += intervalVec;
	}
}

void HavokBike::checkCurrentGate()
{
	const f32			gateW = 1000.0f;
	const f32			gateH = 700.0f;
	sxVector3d			up = sxVector3d(0,-1,0);
	f32					dotResult[4];
	sxVector3d			gateNormal;
	CSXModel*			model;

	model = (CSXModel*)CModel::getInstance();
	CWorldModel* wm = (CWorldModel*) model->getWorldModel();
	CModelObject* GateModel;				
											
	//-- Get Gates Pos,Ori --//
	GateModel = wm->getGateObject( currentCheckPoint );
	if (!GateModel)
	{
		return;
	}
	sxVector3d pos = GateModel->getPosition();
//	sxVector3d ori = GateModel->getOrientation();
	quaternion Q =	GateModel->getOrientationQ();

//	radian RadY = Q_GetAngle( &Q );  
//
//	//-- Get Gates Normal --//
//	gateNormal.x = x_cos( (-RadY+1.57f) );
//	gateNormal.z = x_sin( (-RadY+1.57f) );
//
//	gateNormal.y = 0.0f;


	matrix4 M;
	
	Q_SetupMatrix(&Q, &M);

	gateNormal.x = M.M[2][0];
	gateNormal.y = 0;
	gateNormal.z = M.M[2][2];

	gateNormal.normalize();

	sxVector3d storeV1 = lastPos;
	sxVector3d storeV2 = getPosition();

	sxVector3d v1 = lastPos - pos;
	sxVector3d v2 = getPosition() - pos;

	f32 D1 = v1 * gateNormal;
	f32 D2 = v2 * gateNormal;

	sxVector3d right = gateNormal.cross(up);
	gatePoints[0] = pos + (-right * (gateW/2.0f)) + (-up * (gateH/2.0f));
	gatePoints[1] = pos + ( right * (gateW/2.0f)) + (-up * (gateH/2.0f));
	gatePoints[2] = pos + ( right * (gateW/2.0f)) + ( up * (gateH));
	gatePoints[3] = pos + (-right * (gateW/2.0f)) + ( up * (gateH));

	if( (D1 * D2) > 0)
		return;

	f32 D = ((gateNormal.x * pos.x)+(gateNormal.y * pos.y)+(gateNormal.z * pos.z));

	sxVector3d currentVector = (getPosition() - lastPos);

	currentVector.normalize();

	f32	S = (D - ( gateNormal.x * lastPos.x + gateNormal.y * lastPos.y + gateNormal.z * lastPos.z )) / 
				 ( gateNormal.x * currentVector.x + gateNormal.y * currentVector.y + gateNormal.z * currentVector.z );

	sxVector3d C = lastPos + (currentVector * S);
   
	sxVector3d vect01 = gatePoints[1] - gatePoints[0];
	sxVector3d vect12 = gatePoints[2] - gatePoints[1];
	sxVector3d vect23 = gatePoints[3] - gatePoints[2];
	sxVector3d vect30 = gatePoints[0] - gatePoints[3];

	sxVector3d vectC0 = gatePoints[0] - C;
	sxVector3d vectC1 = gatePoints[1] - C;
	sxVector3d vectC2 = gatePoints[2] - C;
	sxVector3d vectC3 = gatePoints[3] - C;

	dotResult[0] = vectC1 * vect01;
	dotResult[1] = vectC2 * vect12;
	dotResult[2] = vectC3 * vect23;
	dotResult[3] = vectC0 * vect30;

	if (dotResult[0] >= 0.0f && dotResult[1] >= 0.0f && dotResult[2] >= 0.0f && dotResult[3] >= 0.0f)
	{
		//-- Current Gate has been passed -//
		currentCheckPoint++;
		setFlag(BIKE_PASSED_GATE);
		// When the lap gets incremented by the spline code, the currentCheckPoint
		// will go back to zero
	}
}

void HavokBike::setCurLap(s32 lapNum)
{

	if (lapNum != 0)
	{
		TimeStruct tt;

		bikeStats.setLapTime( CGlobalTime::getGlobalTime() );

		f32 lapTime = bikeStats.getLastLapTime();
		x_printf("********LAP Time %f\n",lapTime);

		bikeStats.secondsToTime( &tt, lapTime );
		x_printf("********Time %i.%2.2f\n",tt.Min, tt.Sec_HSec);

		currentCheckPoint = 0;

		//-- If Race is to end then set the finish time of current bike
		CGameOptions* gameOptions = CSxGame::getGameOptions();	

		if( lapNum == gameOptions->getDuration() )
			bikeStats.setFinishTime( bikeStats.getRaceTime() );
	}
	curLap = lapNum;

}


















void HavokBike::groundCollision()
{
	CGameOptions* gameOptions = CSxGame::getGameOptions();	

	sxVector3d lastLinearVelocityNorm = lastLinearVelocity;
	sxVector3d linearVelocityNorm = linearVelocity;
	lastLinearVelocityNorm.normalize();
	linearVelocityNorm.normalize();


	f32 changeInVelFactor;
	if (changeInVelCounter)
	{
		changeInVelCounter--;
		changeInVelFactor = 0.0f;
	}
	else
	{
		f32 changeInVel = (1.0f - (lastLinearVelocityNorm * linearVelocityNorm)) * attributes.stabilityChangeInVel;
		changeInVelFactor = changeInVel * (lastLinearVelocityMag * linearVelocityMag / TO_METERS_PER_SEC(50.0f * 50.0f));
	}


	
//	f32 upChangeInVel = 




//	sxVector3d linearVelocityDir = lastLinearVelocity;
//	linearVelocityDir.normalize();


	sxVector3d forwardVelDir = lastLinearVelocityNorm;
	forwardVelDir.y = 0;
	forwardVelDir.normalize();

	sxVector3d flatForwardDir = forwardDir;
	flatForwardDir.y = 0.0f;
	flatForwardDir.normalize();





	sxVector3d terrainNorm;
	if (!wheelInfo[FW].getIsInContact() && wheelInfo[BW].getIsInContact())
	{
		terrainNorm = wheelNormVec[BW];
	}
	else if (wheelInfo[FW].getIsInContact() && !wheelInfo[BW].getIsInContact())
	{
		terrainNorm = wheelNormVec[FW];
	}
	else //-- both wheels
	{
		terrainNorm = (wheelNormVec[FW] + wheelNormVec[BW]) * 0.5f;
	}

	f32 terrainNormVelFactor = 0.0f;
	if (getDistToGround() <= 0.0f)
	{
		f32 terrainNormVel = lastLinearVelocity * (-terrainNorm);
		terrainNormVelFactor = terrainNormVel / TO_METERS_PER_SEC(60.0f);
	}



	
	f32 rollFactor = 0.0f;
	f32 forwardPitchFactor = 0.0f;
	f32 backPitchFactor = 0.0f;
	f32 forwardVelFactor = 0.0f;
	f32 yawLandVelFactor = 0.0f;

	f32 stuntFactor = 0.0f;



	if (crashLand)
//	if (getFlag(BIKE_LAND))
	{
		s32 pitchRollIndex = lastBikePitchRollIndex - 3;
		if (pitchRollIndex < 0)
			pitchRollIndex += NUM_PITCH_ROLL_INDEXES;


		f32 rollMaxAngle = 60.0f + 15.0f * MIN(1.0f, lastLinearVelocityMag / TO_METERS_PER_SEC(45.0f));
		rollFactor = MIN(1.0f, fabs(lastBikeRelativeRoll[pitchRollIndex]) / rollMaxAngle);


		f32 forwardPitchMaxAngle;
		if (terrainPitchAvg < 0.0f)
			forwardPitchMaxAngle = getSlopePoint(0.0f, 45.0f, 45.0f, 45.0f, 90.0f, 20.0f, -terrainPitchAvg);
		else
			forwardPitchMaxAngle = getSlopePoint(0.0f, 45.0f, 20.0f, 40.0f, 45.0f, 20.0f, terrainPitchAvg);
		forwardPitchFactor = lastBikeRelativePitch[pitchRollIndex] / forwardPitchMaxAngle;


		if (lastBikePitch[pitchRollIndex] < -75.0f)
			forwardPitchFactor = attributes.stability;


		f32 backPitchMaxAngle = 65.0f - 0.0f * MIN(1.0f, lastLinearVelocityMag / TO_METERS_PER_SEC(45.0f));
		backPitchFactor = -lastBikeRelativePitch[pitchRollIndex] / backPitchMaxAngle;

		if (lastBikePitch[pitchRollIndex] > 80.0f)
			backPitchFactor = attributes.stability;


		f32 forwardVel = ((1.0f - (lastLinearVelocityNorm * forwardDir)) * 0.5f);
		forwardVelFactor = forwardVel * (lastLinearVelocityMag / TO_METERS_PER_SEC(50.0f));
	}

	if (getFlag(BIKE_LAND))
	{

		if ((bikeStateCounter[BIKESTATE_RIGHT_POWERSLIDE_ACCEL] < -5 && bikeStateCounter[BIKESTATE_RIGHT_POWERSLIDE_ACCEL] > -45) ||
			(bikeStateCounter[BIKESTATE_LEFT_POWERSLIDE_ACCEL] < -5 && bikeStateCounter[BIKESTATE_LEFT_POWERSLIDE_ACCEL] > -45))

		if (!checkMessage(BIKEACTION_LEFT_POWERSLIDE_BRAKE) && 
			!checkMessage(BIKEACTION_RIGHT_POWERSLIDE_BRAKE) && 
			bikeStateCounter[BIKESTATE_RIGHT_POWERSLIDE_BRAKE] < -30.0f && 
			bikeStateCounter[BIKESTATE_LEFT_POWERSLIDE_BRAKE] < -30.0f)
		{
			f32 yawLandVel = ((1.0f - (forwardVelDir * flatForwardDir)) * 0.5f);
			yawLandVelFactor = yawLandVel * (lastLinearVelocityMag / TO_METERS_PER_SEC(20.0f));
		}


		if (stuntCrashFlag)
			stuntFactor = attributes.stability;
	}


	f32 yawVelFactor = 0.0f;
//	if (!checkMessage(BIKEACTION_LEFT_POWERSLIDE_BRAKE) && !checkMessage(BIKEACTION_RIGHT_POWERSLIDE_BRAKE))
//	{
//		f32 yawVel = ((attributes.stability - (forwardVelDir * flatForwardDir)) * 0.5f);
//		yawVelFactor = yawVel * (lastLinearVelocityMag / TO_METERS_PER_SEC(40.0f));
//	}



	f32 wheelInGroundFactor = 0.0f;
	if (getDistToGround(FW) < -36.0f || getDistToGround(BW) < -36.0f)
		wheelInGroundFactor = attributes.stability;



	
	
	


	if (terrainNormVelFactor >= attributes.stability || 
		rollFactor >= attributes.stability ||
		forwardPitchFactor >= attributes.stability ||
		backPitchFactor >= attributes.stability ||
		yawVelFactor >= attributes.stability ||
		yawLandVelFactor >= attributes.stability ||
		forwardVelFactor >= attributes.stability ||
		changeInVelFactor >= attributes.stability ||
		stuntFactor >= attributes.stability ||
		wheelInGroundFactor >= attributes.stability)
	{
		if( gameOptions->getGameOptions( GAMEOPTION_GROUND_CRASHES_ON ) )
			startCrash(GROUND_CRASH);
	}
	else if ( getFlag(BIKE_LAND) )
	{
		getBikeStats()->jumpLandSuccessful( getPosition(), CGlobalTime::getGlobalTime() );
	}

}


void HavokBike::collide()
{
	CGameOptions* gameOptions = CSxGame::getGameOptions();	

	if (crashState == RIDING_STATE)
	{
		if (collided)
		{
			collided = false;
			collidedCounter = 0;
			setFlag(BIKE_COLLIDE);
		}
		else if (collidedCounter == COLLIDED_COUNTER_TRIGGER_FRAME)
		{
			if (willCrashFlag)
			{
				if( gameOptions->getGameOptions( GAMEOPTION_BIKE_TO_BIKE_CRASHES_ON ) )
					startCrash(collisionType);
			}
			else
			{

				s32 velMagIndex;
				f32 deltaVelMag = 0.0f;
				f32 minVelMag = linearVelocity * forwardDir;
				f32 maxVelMag = lastVelocity[lastVelocityMagIndex] * forwardDir;

				for(u32 i = 0; i < NUM_VELOCITY_INDEXES; i++)
				{
					velMagIndex = lastVelocityMagIndex - i;
					if (velMagIndex < 0)
						velMagIndex += NUM_VELOCITY_INDEXES;

					f32 velMag = lastVelocity[velMagIndex] * forwardDir;
					
					if (velMag < minVelMag)
						minVelMag = velMag;

					if (velMag > maxVelMag)
						maxVelMag = velMag;

				
//					f32 deltaVelMagTemp = fabs(linearVelocityMag - lastVelocityMag[velMagIndex]);
//					if (deltaVelMag < deltaVelMagTemp)
//						deltaVelMag = deltaVelMagTemp;
				}

				deltaVelMag = maxVelMag - minVelMag;
	
	
				if(deltaVelMag >= TO_METERS_PER_SEC(35.0f))
				{
					collided = false;
					if (collisionType == OBJECT_COLLISION)
					{
						if( gameOptions->getGameOptions( GAMEOPTION_OBJECT_CRASHES_ON ) )	
							startCrash(OBJECT_CRASH);
					}
//					else if (collisionType == OTHER_BIKE_COLLISION)
	//				{
	//					if( gameOptions->getGameOptions( GAMEOPTION_BIKE_TO_BIKE_CRASHES_ON ) )
	//						startCrash(OTHER_BIKE_CRASH);
	//				}
				}
			}
		}
		else if (collidedCounter > COLLIDED_COUNTER_TRIGGER_FRAME && collidedCounter < COLLIDED_COUNTER_MAX_TURN_FRAME && linearVelocityMag < TO_METERS_PER_SEC(3.0f))
		{
//			Vector3 forceV3 = SXV_TO_V3(collisionForce);
//			if (collisionForce.magnitude() < 20000.0f)
//			{
//				chassis->applyImpulse(forceV3);
//				chassis->applyForce(forceV3 * 10.0f);
//			}


			sxVector3d terrainUpVector = wheelNormVec[FW];
			sxVector3d terrainForwardVector = wheelNormVec[FW].cross(rightDir);
			sxVector3d terrainRightVector = terrainForwardVector.cross(wheelNormVec[FW]);
	
			Vector3 angVelV3 = chassis->getAngularVelocity();
			sxVector3d angVel = V3_TO_SXV(angVelV3);


			f32 projAngVelUp = angVel * terrainUpVector;
			f32 projAngVelForward = angVel * terrainForwardVector;
			f32 projAngVelRight = angVel * terrainRightVector;


			sxVector3d forcePos;
			Vector3 forcePosV3;
			sxVector3d force;
			Vector3 forceV3;
			f32 slideAngVel;

			if (checkMessage(BIKEACTION_LEFT_TURN))
			{
				slideAngVel = 3.5f;
				f32 slideAngVelApply = filterSAV.applyToOneSample(slideAngVel);

				sxVector3d newAngVelUp = terrainUpVector * (slideAngVelApply);
				sxVector3d newAngVelForward = terrainForwardVector * projAngVelForward;
				sxVector3d newAngVelRight = terrainRightVector * projAngVelRight;
				sxVector3d newAngVel = newAngVelUp + newAngVelForward + newAngVelRight;
				angVelV3 = SXV_TO_V3(newAngVel);
				chassis->setAngularVelocity(angVelV3);
	
			}
			else if (checkMessage(BIKEACTION_RIGHT_TURN))
			{
				slideAngVel = -3.5f;
				f32 slideAngVelApply = filterSAV.applyToOneSample(slideAngVel);

				sxVector3d newAngVelUp = terrainUpVector * (slideAngVelApply);
				sxVector3d newAngVelForward = terrainForwardVector * projAngVelForward;
				sxVector3d newAngVelRight = terrainRightVector * projAngVelRight;
				sxVector3d newAngVel = newAngVelUp + newAngVelForward + newAngVelRight;
				angVelV3 = SXV_TO_V3(newAngVel);
				chassis->setAngularVelocity(angVelV3);

			}
		}
		collidedCounter++;

	
		if (CGlobalTime::getGlobalTime() - collisionStartTime > 3.0f)
		{
			collisionObjectCounter = 0;
			collisionObject = (CModelObject*)NULL;
		}
	}
}



void HavokBike::collisionHandler( CModelObject* collidedWith, const Havok::CollisionEvent& ce )
{
	HavokBike* bike = dynamic_cast<HavokBike*>(collidedWith);
	CGameOptions* gameOptions = CSxGame::getGameOptions();	
//	CRiderModel* rider = dynamic_cast<CRiderModel*>(collidedWith);

	if (crashState == RIDING_STATE)
	{
		if (collidedWith && collidedCounter > COLLIDED_COUNTER_NO_COLLISION_FRAME)
		{
			if (bike)
			{
				collisionType = OTHER_BIKE_COLLISION;

				bool sameBike = TRUE;
//				if (collisionObjectPos.x != collidedWith->getPosition().x ||
//					collisionObjectPos.y != collidedWith->getPosition().y ||
//					collisionObjectPos.z != collidedWith->getPosition().z)
//					sameBike = FALSE;
				if(collidedWith != collisionObject || !sameBike)
				{
					collisionObjectCounter = 1;
					collisionObject = collidedWith;
					collisionObjectPos = collidedWith->getPosition();
				}
				else
				{
//					collisionObjectCounter++;
				}
				collisionStartTime = CGlobalTime::getGlobalTime();
				collided = true;




				collisionNormal = V3_TO_SXV(ce.m_normal);
				collisionNormal.normalize();

				f32 bikeToBikeAngle = forwardDir * bike->getForwardVector();


				f32 forwardVel = forwardDir * linearVelocity;
				f32 upVel = upDir * linearVelocity;
				f32 rightVel = rightDir * linearVelocity;

				f32 otherBikeForwardVel = forwardDir * bike->getLinearVelocity();
				f32 otherBikeUpVel = upDir * bike->getLinearVelocity();
				f32 otherBikeRightVel = rightDir * bike->getLinearVelocity();


				f32 deltaForwardVel = forwardVel - otherBikeForwardVel;
				f32 deltaUpVel = upVel - otherBikeUpVel;
				f32 deltaRightVel = rightVel - otherBikeRightVel;


				sxVector3d otherBikePos = bike->getPosition();
				sxVector3d thisBikePos = getPosition();
				sxVector3d posVector = otherBikePos - thisBikePos;
				f32 dist = 	thisBikePos.distance(&otherBikePos);

				posVector.normalize();
				bool inFront;
				if (posVector * forwardDir < 0.0f)
					inFront = true;
				else 
					inFront = false;

				f32 inFrontScale;
				if (!inFront)
					inFrontScale = 1.0f - MIN(1.0f, (dist / bikeLength));
				else 
					inFrontScale = 1.0f;

				f32 leanStabilityScale = 1.0f - MIN(1.0f, fabs(bikeRoll / 30.0f));

				if (deltaForwardVel > leanStabilityScale * (TO_METERS_PER_SEC(30.0f) + (inFrontScale * TO_METERS_PER_SEC(20.0f))) )
				{
					willCrashFlag = true;
				}
				else if (deltaRightVel > leanStabilityScale * (TO_METERS_PER_SEC(30.0f) + (inFrontScale * TO_METERS_PER_SEC(20.0f))) )
				{
					willCrashFlag = true;
				}
				else if (deltaUpVel > leanStabilityScale * (TO_METERS_PER_SEC(30.0f) + (inFrontScale * TO_METERS_PER_SEC(20.0f))) )
				{
					willCrashFlag = true;
				}
				else if (thisBikePos.y - otherBikePos.y > 50.0f)
				{
					willCrashFlag = true;
				}
				else if (collisionNormal * upDir < -0.5f)
				{
					willCrashFlag = true;
				}
				else if (fabs(bikeRoll) > 30.0f)
				{
					willCrashFlag = true;
				}



//				else if (upDir * collisionNormal < -0.25f) //-- on the bottom
//				{
//					willCrashFlag = true;
//				}
/*
				else if (bikeToBikeAngle < 0.75f && bikeToBikeAngle > 0.5f)
				{
					if (linearVelocityMag > TO_METERS_PER_SEC(60.0f))
						willCrashFlag = true;
				}
				else if (bikeToBikeAngle < 0.5f && bikeToBikeAngle > 0.0f)
				{
					if (linearVelocityMag > TO_METERS_PER_SEC(45.0f))
						willCrashFlag = true;
				}
				else if (bikeToBikeAngle < 0.0f)
				{
					if (linearVelocityMag > TO_METERS_PER_SEC(20.0f))
						willCrashFlag = true;
				}
				else
				{
					sxVector3d otherBikePos = bike->getPosition();
					sxVector3d thisBikePos = getPosition();
					sxVector3d posVector = otherBikePos - thisBikePos;
					f32 dist = 	thisBikePos.distance(&otherBikePos);

					posVector.normalize();
					bool inFront;
					if (posVector * forwardDir < 0.0f)
						inFront = true;
					else 
						inFront = false;


					if (!inFront && dist > (0.85f * bikeLength) && linearVelocityMag > TO_METERS_PER_SEC(30.0f))
					{
						willCrashFlag = true;
					}

				}
*/
				changeInVelCounter = 10;

/*
				if(collisionObjectCounter >= 5)
				{
					if( gameOptions->getGameOptions( GAMEOPTION_BIKE_TO_BIKE_CRASHES_ON ) )	
					{
						collided = false;
						startCrash(OTHER_BIKE_CRASH);
					}
				}
*/

			}
			else
			{
				collisionType = OBJECT_COLLISION;

				bool samePos = TRUE;
				if (collisionObjectPos.x != collidedWith->getPosition().x ||
					collisionObjectPos.y != collidedWith->getPosition().y ||
					collisionObjectPos.z != collidedWith->getPosition().z)
					samePos = FALSE;
				if(collidedWith != collisionObject || !samePos)
				{
					collisionObjectCounter = 1;
					collisionObject = collidedWith;
					collisionObjectPos = collidedWith->getPosition();
				}
				else
				{
					collisionObjectCounter++;
				}
				collisionStartTime = CGlobalTime::getGlobalTime();
		

				collided = true;
				changeInVelCounter = 10;
				if(collisionObjectCounter >= 5)
				{
					if( gameOptions->getGameOptions( GAMEOPTION_OBJECT_CRASHES_ON ) )	
					{
						collided = false;
						startCrash(OBJECT_CRASH);
					}
				}
			}
		}
	}
}





void HavokBike::crash()
{
	CModel* model = CModel::getInstance();
	Toolkit* toolkit = model->getToolkit();

//	if (!isCPU)
//	{
//		CCamera* camera = dynamic_cast<CCamera*>(model->getObjectByID(getCameraId()));
//		ASSERT(camera != NULL);
//		camera->SetMode(CAM_MODE_CRASH);
//	}




/*
	crashBike->setPinned(false);
	crashBike->setActive(true);

	crashBike->setLinearVelocity(chassis->getLinearVelocity());
	crashBike->setAngularVelocity(chassis->getAngularVelocity());
*/

//	pinBike();









	wheelInfo[FW].setSpinVelocity(0.0f);
	wheelInfo[BW].setSpinVelocity(0.0f);


//	setCenterOfMass(sxVector3d(0,-COM_Y,-COM_Z));
/*
	frontWheel->setPinned(false);
	frontWheel->setActive(true);
	backWheel->setPinned(false);
	backWheel->setActive(true);
*/


/*
	if( isBikeCPU() == TRUE )
	{
		frontWheel->getPrimitive(0)->setCollisionMask(COLLISION_MASK_CHASSIS_AI);
		backWheel->getPrimitive(0)->setCollisionMask(COLLISION_MASK_CHASSIS_AI);
	}
	else
	{
		frontWheel->getPrimitive(0)->setCollisionMask(COLLISION_MASK_CHASSIS_PLAYER);
		backWheel->getPrimitive(0)->setCollisionMask(COLLISION_MASK_CHASSIS_PLAYER);
	}
*/

/*
	toolkit->m_defaultCollisionDetector->enableCollisionGroups(COLLISION_GROUP_HEIGHTFIELD,COLLISION_GROUP_WHEELS);	// heightfield & wheels
	toolkit->m_defaultCollisionDetector->enableCollisionGroups(COLLISION_GROUP_WHEELS,COLLISION_GROUP_WHEELS);	// wheels & wheels
	toolkit->m_defaultCollisionDetector->enableCollisionGroups(COLLISION_GROUP_WHEELS,COLLISION_GROUP_CHASSIS_AI);	// chassis & wheels
	toolkit->m_defaultCollisionDetector->enableCollisionGroups(COLLISION_GROUP_WHEELS,COLLISION_GROUP_CHASSIS_PLAYER);	// chassis & wheels
	toolkit->m_defaultCollisionDetector->enableCollisionGroups(COLLISION_GROUP_STATIC_OBJECTS,COLLISION_GROUP_WHEELS);// wheels & objects
	toolkit->m_defaultCollisionDetector->enableCollisionGroups(COLLISION_GROUP_DYNAMIC_OBJECTS,COLLISION_GROUP_WHEELS);// wheels & objects
	toolkit->m_defaultCollisionDetector->enableCollisionGroups(COLLISION_GROUP_SPLINE_OBJECTS,COLLISION_GROUP_WHEELS);// wheels & objects
*/

	frontWheel->setPinned(true);
	frontWheel->setPinned(false);
	backWheel->setPinned(true);
	backWheel->setPinned(false);


	rc->removeAction(csolver);


	((CBikeSuspension*)m_suspension)->setWheelStrength(0, 10.0f);
	((CBikeSuspension*)m_suspension)->setWheelStrength(1, 10.0f);
	((CBikeSuspension*)m_suspension)->setWheelDampingRelaxation(0, 10.0f);
	((CBikeSuspension*)m_suspension)->setWheelDampingRelaxation(1, 10.0f);
	((CBikeSuspension*)m_suspension)->setWheelDampingCompression(0, 10.0f);
	((CBikeSuspension*)m_suspension)->setWheelDampingCompression(1, 10.0f);

	((CBikeAerodynamics*)m_aerodynamics)->setExtraGravity(Vector3(0.0f, 20.0f, 0.0f));
	lockSuspension = TRUE;
}

void HavokBike::recover()
{
	CModel* model = CModel::getInstance();

	
//	setCenterOfMass(sxVector3d(0,COM_Y,COM_Z));
	
	leanPercent=0;

//	crashBike->setPinned(true);
//	crashBike->setActive(false);
/*
	frontWheel->setPinned(true);
	frontWheel->setActive(false);
	backWheel->setPinned(true);
	backWheel->setActive(false);
*/
//	frontWheel->getPrimitive(0)->setCollisionMask(COLLISION_MASK_WHEELS);
//	backWheel->getPrimitive(0)->setCollisionMask(COLLISION_MASK_WHEELS);
//	Toolkit* toolkit = model->getToolkit();
/*
	toolkit->m_defaultCollisionDetector->disableCollisionGroups(COLLISION_GROUP_HEIGHTFIELD,COLLISION_GROUP_WHEELS);	// heightfield & wheels
	toolkit->m_defaultCollisionDetector->disableCollisionGroups(COLLISION_GROUP_WHEELS,COLLISION_GROUP_WHEELS);	// wheels & wheels
	toolkit->m_defaultCollisionDetector->disableCollisionGroups(COLLISION_GROUP_WHEELS,COLLISION_GROUP_CHASSIS_AI);	// chassis & wheels
	toolkit->m_defaultCollisionDetector->disableCollisionGroups(COLLISION_GROUP_WHEELS,COLLISION_GROUP_CHASSIS_PLAYER);	// chassis & wheels
	toolkit->m_defaultCollisionDetector->disableCollisionGroups(COLLISION_GROUP_STATIC_OBJECTS,COLLISION_GROUP_WHEELS);// wheels & objects
	toolkit->m_defaultCollisionDetector->disableCollisionGroups(COLLISION_GROUP_DYNAMIC_OBJECTS,COLLISION_GROUP_WHEELS);// wheels & objects
	toolkit->m_defaultCollisionDetector->disableCollisionGroups(COLLISION_GROUP_SPLINE_OBJECTS,COLLISION_GROUP_WHEELS);// wheels & objects
*/

	rc->addAction(csolver);

	constraint->setAngle(DEG_TO_RAD(0));
	((CBikeAerodynamics*)m_aerodynamics)->setExtraGravity(Vector3(0.0f, 0.0f, 0.0f));
	lockSuspension = FALSE;

	chassis->setFriction(0.5f);
}






void HavokBike::dampenCrashVelocity(RigidBody* rb)
{
	sxVector3d angDampening;

	Vector3 angVelV3 = rb->getAngularVelocity();
	sxVector3d angVel = V3_TO_SXV(angVelV3);
	
	f32 projAngVelUp = angVel * upDir;
	f32 projAngVelForward = angVel * forwardDir;
	f32 projAngVelRight = angVel * rightDir;

	f32 dampeningScale = getSlopePoint(0.0f, 1.0f, SECS_TO_FRAMES(2.0f), 1.0f, SECS_TO_FRAMES(5.0f), 0.85f, crashCounter);

	//-- in air
	if (!wheelInfo[FW].getIsInContact() && !wheelInfo[BW].getIsInContact())
	{
		angDampening = sxVector3d(	1.0f * dampeningScale, 
									1.0f * dampeningScale, 
									1.0f * dampeningScale);
	} 
	//-- back on ground & front in air
	else
	{
		angDampening = sxVector3d(	1.0f * dampeningScale, 
									1.0f * dampeningScale, 
									1.0f * dampeningScale);
	}



	angVel = (upDir * projAngVelUp * angDampening.x) + 
			 (forwardDir * projAngVelForward * angDampening.y) + 
			 (rightDir * projAngVelRight * angDampening.z);


	angVelV3 = SXV_TO_V3(angVel);
	rb->setAngularVelocity(angVelV3);




	rb->setFriction(getSlopePoint(0.0f, 1.0f, SECS_TO_FRAMES(2.0f), 1.0f, SECS_TO_FRAMES(5.0f), 0.25f, crashCounter));





	sxVector3d lVel;
	Vector3 lVelV3;
	f32 velMag;


	const f32 minCrashSpeed = getSlopePoint(0.0f, TO_METERS_PER_SEC(25.0f), SECS_TO_FRAMES(0.125f), TO_METERS_PER_SEC(12.0f), SECS_TO_FRAMES(1.0f), TO_METERS_PER_SEC(0.0f), crashCounter);

	lVelV3 = rb->getLinearVelocity();
	lVel = V3_TO_SXV(lVelV3);
	velMag = lVel.magnitude();

	if (velMag < minCrashSpeed)
	{
		lVelV3 = lVelV3 * minCrashSpeed / velMag;
		rb->setLinearVelocity(lVelV3);
	}

	


	const f32 maxCrashSpeed = getSlopePoint(0.0f, TO_METERS_PER_SEC(2500.0f), SECS_TO_FRAMES(0.125f), TO_METERS_PER_SEC(500.0f), SECS_TO_FRAMES(1.0f), TO_METERS_PER_SEC(30.0f), crashCounter);

	lVelV3 = rb->getLinearVelocity();
	lVel = V3_TO_SXV(lVelV3);
	velMag = lVel.magnitude();

	if (velMag > maxCrashSpeed)
	{
		lVelV3 = lVelV3 * maxCrashSpeed / velMag;
		rb->setLinearVelocity(lVelV3);
	}



	const f32 maxUpSpeed = TO_METERS_PER_SEC(15.0f);//getSlopePoint(0.0f, TO_METERS_PER_SEC(2500.0f), SECS_TO_FRAMES(0.125f), TO_METERS_PER_SEC(500.0f), SECS_TO_FRAMES(1.0f), TO_METERS_PER_SEC(30.0f), crashCounter);

	lVelV3 = rb->getLinearVelocity();
	lVel = V3_TO_SXV(lVelV3);
	velMag = lVel * sxVector3d(0, -1, 0);

	if (velMag > maxUpSpeed)
	{
		lVelV3 = lVelV3 * maxUpSpeed / velMag;
		rb->setLinearVelocity(lVelV3);
	}

	

}





void HavokBike::limitInitialCrashVelocity(RigidBody* rb)
{
	const f32 maxCrashSpeed = TO_METERS_PER_SEC(20.0f);
	const f32 minCrashSpeed = TO_METERS_PER_SEC(10.0f);

	Vector3 lVelV3 = rb->getLinearVelocity();
	sxVector3d lVel = V3_TO_SXV(lVelV3);
	f32 velMag = lVel.magnitude();

	if (velMag > maxCrashSpeed)
	{
		lVelV3 = lVelV3 * maxCrashSpeed / velMag;
		rb->setLinearVelocity(lVelV3);
	}

	if (velMag < minCrashSpeed)
	{
		lVelV3 = lVelV3 * minCrashSpeed / velMag;
		rb->setLinearVelocity(lVelV3);
	}



	Vector3 angVelV3 = rb->getAngularVelocity();
	sxVector3d angVel = V3_TO_SXV(angVelV3);
	f32 angVelMag = angVel.magnitude();
	f32 minCrashAngSpeed = TO_METERS_PER_SEC(10.0f);

	if (angVelMag < minCrashAngSpeed)
	{

		angVelV3 = angVelV3 * minCrashAngSpeed / angVelMag;
		rb->setAngularVelocity(angVelV3);
	}



	f32 velocityScale = MAX(0.35f, 1.0f - MIN(1.0f, linearVelocityMag / TO_METERS_PER_SEC(60.0f) ));
	if (crashType == GROUND_CRASH)
		velocityScale = 1.0f;

	sxVector3d forcePos;
	Vector3 forceV3 = Vector3(0.0f, -1.0f, 0.0f);
	Vector3 forceUpV3 = Vector3(0.0f, -1.0f, 0.0f);


	bool forwardCrash;

	if (crashType == OBJECT_CRASH)
	{
		if (linearVelocityMag > TO_METERS_PER_SEC(30.0f))
			forwardCrash = true;
		else
			forwardCrash = false;
	}
	else
	{
		forwardCrash = true;
	}


	if (forwardCrash)
	{
		forcePos = getPosition() * SX_TO_HAVOK;
	}
	else
	{
		forcePos = (getPosition() + (forwardDir * 200.0f)) * SX_TO_HAVOK;
	}

	if (crashType == GROUND_CRASH)
	{
		if (bikePitch < 0.0f && bikePitch > -45.0f)
		{
			forceV3 = forceV3 * 450000.0f;
			forceUpV3 = forceUpV3 * 500000.0f * velocityScale;
		}
		else
		{
			forceV3 = forceV3 * 250000.0f;
			forceUpV3 = forceUpV3 * 500000.0f * velocityScale;
		}
	}
	else
	{
		forceV3 = forceV3 * 300000.0f;
		forceUpV3 = forceUpV3 * 500000.0f * velocityScale;
	}



	Vector3 forcePosV3 = SXV_TO_V3(forcePos);
//	rb->applyForce(forceUpV3);
//	rb->applyForce(forceV3, forcePosV3);


}



void HavokBike::startCrash(u32 type)
{
	if (type == OBJECT_CRASH)
		postCrashCounter = SECS_TO_FRAMES(4.0f);

	if (crashEnabled && crashState == RIDING_STATE && postCrashCounter >= SECS_TO_FRAMES(4.0f))
	{
//		forceZeroVelocity();
		crashState = START_CRASH_STATE;
		collisionObjectCounter = 0;
		if (type == OBJECT_CRASH)
		{
			crashStartCounter = 0;//10;
//			pinBike();
//			unpinBike();
		}
		else if (type == OTHER_BIKE_CRASH)
		{
			crashStartCounter = 0;//10;
//			pinBike();
//			unpinBike();
		}
		else
		{
			crashStartCounter = 0;
		}

		collisionObject = (CModelObject*)NULL;
		crashType = type;
		willCrashFlag = false;

		inStunt = 0;
	}
}



void HavokBike::crashUpdate()
{
	CModel* model = CModel::getInstance();
	CWorldModel*	worldModel = ((CSXModel*) CModel::getInstance())->getWorldModel();
	CGameOptions* gameOptions = CSxGame::getGameOptions();	
	
	if (crashState != RIDING_STATE)
	{
		if (crashState == START_CRASH_STATE)
		{
			((CBikeDriverInput*)m_input)->setSteeringInput(0.0f);
			((CBikeDriverInput*)m_input)->setBrakeInput(0.0f);
			((CBikeDriverInput*)m_input)->setAcceleratorInput(0.0f);



			crashBike->setPinned(false);
			crashBike->setActive(true);

			crashBike->setLinearVelocity(chassis->getLinearVelocity());
			crashBike->setAngularVelocity(chassis->getAngularVelocity());

			chassis->setPinned(true);
			chassis->setActive(false);

			frontWheel->setPinned(true);
			frontWheel->setActive(false);

			backWheel->setPinned(true);
			backWheel->setActive(false);


			if (!isCPU)
			{
				CCamera* camera = dynamic_cast<CCamera*>(model->getObjectByID(getCameraId()));
				ASSERT(camera != NULL);
				camera->SetMode(CAM_MODE_CRASH);
			}

//			((CBikeSuspension*)m_suspension)->turnOffSuspension();
			
			if (!crashing && (crashType == OBJECT_CRASH || crashType == OTHER_BIKE_CRASH) && crashStartCounter)
			{
/*
				sxVector3d collDir = lastPos - getPosition();
				collDir.y -= 20.0f;
				collDir.normalize();
				sxVector3d pos = getPosition();
//				pos += (-fForwardDir * 20.0f) + (-collisionNormal * 50.0f);
				pos += (collDir * 100.0f);
				setPosition(pos);
				lastPos = pos;
				updateModelWorld();
//				updateHavokFromModelObject();
//				setStartPos(pos, forwardDir);
*/
			}
/*
			if (!crashing && crashType == GROUND_CRASH)
			{
				sxVector3d pos = getPosition();
				if (bikePitch > 0.0f)
					pos += (forwardDir * 20.0f);
				else
					pos += ((-forwardDir) * 10.0f);

				setPosition(pos);
				lastPos = pos;
				updateModelWorld();
//				updateHavokFromModelObject();
			}
*/


//			if (crashStartCounter < 10)
//			{
				crashing = true;
				recovering = false;
//			}




			if (crashStartCounter == 0)
			{

				crashState = CRASH_STATE;
//				limitInitialCrashVelocity(chassis);
				limitInitialCrashVelocity(crashBike);
				crash();
				crashCounter = 0;
//				crashing = true;
//				recovering = false;


				CSplineManager *pSM = CSplineManager::getSplineManager();
				bool spline = false;
				if (pSM->getPoint(getWayPointTime(), CSplineManager::WayPoint, crashPos, crashDir))
					spline = true;


				if (gameOptions->getTrackType() == TRACKTYPE_BAJA || 
					gameOptions->getTrackType() == TRACKTYPE_FREESTYLE || 
					!spline)
				{
					s32 index = safeIndex;
					if (crashType == OBJECT_CRASH)
						index -= 8;
					else
						index -= 2;

					if (index < 0)
						index += NUM_SAFE_INDEXES;


					sxVector3d pos[2];
					bool level = FALSE;
					do
					{

						crashPos = safePos[index];
						crashDir = safeDir[index];

						pos[BW] = crashPos;
						pos[FW] = crashPos + (safeDir[index] * wheelBase);

						f32 y0 = worldModel->getHeightAt(pos[FW].x, pos[FW].z);
						f32 y1 = worldModel->getHeightAt(pos[BW].x, pos[BW].z);
						if (fabs(y0 - y1) > 50.0f || getPosition().distanceSquared(&crashPos) < 250000.0f)
						{
							
							index--;
							if (index < 0)
								index += NUM_SAFE_INDEXES;
						}
						else
						{
							level = TRUE;
						}
					}while(!level);
				}
			}
			else
			{
				crashStartCounter--;
			}

		}

		else if (crashState == CRASH_STATE)
		{
/*
			Vector3 pos;		// Current Position
			Quaternion quat;	// Current Orientation
			crashBike->getDisplayToWorldTransform(pos,quat);
			vehicle->getChassisRigidBody()->setDisplayToWorldTransform(pos,quat);

			//-- set the orientation
			Havok::Quaternion qx90(0.5f*PI,Vector3(1,0,0)); 
		    quat = quat * qx90;
			sxQuaternion q = Q_TO_SXQ(quat);
		    setOrientationQ( q );
		
			// Set the position
			pos *= HAVOK_TO_SX;		
			setPosition(V3_TO_SXV(pos));
*/


//			if (crashCounter == 30)
//			{
//				gameOptions->setGameOptions( GAMEOPTION_WORLD_COLLISIONS_ON );
//				for (int p=0; p< chassis->getNumPrimitives(); p++)
//					chassis->getPrimitive(p)->setPhantom(FALSE);	
//		
//				frontWheel->getPrimitive(0)->setPhantom(FALSE);
//				backWheel->getPrimitive(0)->setPhantom(FALSE);
//			}


			if (crashCounter >= SECS_TO_FRAMES(3.0f))
			{
				crashState = START_RECOVER_STATE;
			}
			else
			{
//				dampenCrashVelocity(chassis);
				dampenCrashVelocity(crashBike);
				setFlag(BIKE_CRASH);
			}
		}
		else if (crashState == START_RECOVER_STATE)
		{
			crashing = false;
			recovering = true;
			pinBike();
			recover();
			forceZeroVelocity();

			throttle = 0.0f;
			throttleUp = 0.0f;
			throttleUpActive = FALSE;
			turnStep = 0.0f;
			brakingScale = 0.0f;
			framesInAir = 0;
			crashFramesInAir = 0.0f;

			turnCounter = 0;
			powerSlideBrakeTurnCounter = 0;
			powerSlideAccelTurnCounter = 0;

			bikeState = BIKESTATE_NO_TURN_NO_ACCEL;


			for(u32 i = 0; i < BIKEACTION_NUM_STATES; i++)
			{
				pressed[i] = 0.0f;
			}

			numExhaustParticles = 0.0f;
			numTrailParticles = 0.0f;
			numMudParticles= 0.0f;
			numFrontDustParticles = 0.0f;
			numBackDustParticles = 0.0f;


			preLoadPressedCounter = 0;
			preLoadWaitDelay = 0;
			preLoadJumpState = PRELOAD_NO_ATTEMPT_PENDING;
			preLoadJump = FALSE;
			preLoadJumpCounter = 0;
			preLoadJumpPending = FALSE;
			preLoadReportState = PRELOAD_AVAILABLE;
	

			for(u32 i = 0; i < BIKESTATE_NUM_STATES; i++)
			{
				bikeStateCounter[i] = -1000;
				bikeStateSuspended[i] = FALSE;
			}
		
			lastFrameBikeState = lastBikeState = bikeState = BIKESTATE_INAIR;
			lastBikeStateCounter = 0;
	

			if (comShiftState != COMSS_NORMAL)
			{
				switch(comShiftState)
				{
				case COMSS_FORWARD:
					setCenterOfMass(sxVector3d(0.0f,-0.35f,0.0f));
					break;
				case COMSS_BACK:
					setCenterOfMass(sxVector3d(0.0f,0.35f,0.0f));
					break;
				case COMSS_LEFT:
					setCenterOfMass(sxVector3d(-0.20f,0.0f,0.0f));
					break;
				case COMSS_RIGHT:
					setCenterOfMass(sxVector3d(0.20f,0.0f,0.0f));
					break;
				}
				comShiftState = COMSS_NORMAL;
			}


			crashBike->setPinned(true);
			crashBike->setActive(false);

//			crashBike->setLinearVelocity(chassis->getLinearVelocity());
//			crashBike->setAngularVelocity(chassis->getAngularVelocity());

//			chassis->setPinned(true);
//			chassis->setActive(false);

//			frontWheel->setPinned(true);
//			frontWheel->setActive(false);

//			backWheel->setPinned(true);
//			backWheel->setActive(false);


			setStartPos(crashPos, crashDir);
			if (!isCPU)
			{
				CCamera* camera = dynamic_cast<CCamera*>(model->getObjectByID(getCameraId()));
				ASSERT(camera != NULL);
				camera->SetMode(CAM_MODE_FOLLOW);
			}
			crashState = RECOVER_STATE;
//			setFlag(BIKE_CRASH);
		}
		else if (crashState == RECOVER_STATE)
		{
			clearFlags();
			setFlag(BIKE_DECEL);
			setFlag(BIKE_ON_GROUND);

			constraint->setAngle(DEG_TO_RAD(0));
//			((CBikeSuspension*)m_suspension)->turnOnSuspension();
			unpinBike();

//			setFlag(BIKE_CRASH);
			if (crashCounter >= SECS_TO_FRAMES(3.5f))
			{
//				((CBikeSuspension*)m_suspension)->turnOnSuspension();
				crashState = RIDING_STATE;
				recovering = false;

			}
		}
		crashCounter++;
	}
	else
	{
		postCrashCounter++;


		if (onGround())
		{
//			if (CGlobalTime::getGlobalTime() > lastSafeTime + (0.016667f*3.0f))
//			{
				s32 lastIndex = safeIndex;
				lastIndex--;
				if (lastIndex < 0)
					lastIndex += NUM_SAFE_INDEXES;

//				sxVector3d dist = getPosition() - safePos[lastIndex];

				if (getPosition().distanceSquared(&safePos[lastIndex]) > 40000.0f)
				{
					safeDir[safeIndex] = forwardDir;
					safePos[safeIndex] = getPosition();
					lastSafeTime = CGlobalTime::getGlobalTime();

					safeIndex++;
					safeIndex %= NUM_SAFE_INDEXES;
				}
//			}
		}
	}
}









