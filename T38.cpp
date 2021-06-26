#define STRICT
#define ORBITER_MODULE

#include "orbitersdk.h"
#include "..\..\Sound\OrbiterSound_SDK\VESSELSOUND_SDK\ShuttlePB_project\OrbiterSoundSDK50.h"

double ISP = 100000.0; // original has listed ISP 5e4, but Wikipedia has range 1800 km. With 5e4, it's closer to 800 km.
double THRUST = 440000.0;
double FUEL = 20000.0;
double MASS = 12000.0;

bool useRealisticValues = false;
const double THRUST_REAL = (12.3e3 + 9.1e3) * 2.0;
const double FUEL_REAL = 5361.0 - 3266.0;
const double ISP_REAL = THRUST / (FUEL_REAL / (1835.0e3 / (1382e3 / 3600.0)));
const double MASS_REAL = 3266;

const DWORD NVTX = 7;
const VECTOR3 TOUCH1 = _V(0, -2.15, 4.013654);
const VECTOR3 TOUCH2 = _V(-1.518829, -2.15, -1.547508);
const VECTOR3 TOUCH3 = _V(1.518829, -2.15, -1.547508);
const VECTOR3 TOUCH4 = _V(0.0, -0.824, 6.852); // snout
const VECTOR3 TOUCH5 = _V(0.0, 2.161, -5.546); // rear fin tip
const VECTOR3 TOUCH6 = _V(-3.704, -0.881, -2.253); // left wing
const VECTOR3 TOUCH7 = _V(3.704, -0.881, -2.253); // right wing
const double STIFF_WHEEL = 1e6;
const double STIFF_FRAME = 1e7;
const double DAMPING = 1e5;
const double MU_WHEEL_FRONT = 1.6;
const double MU_WHEEL_BACK = 3.0;
const double MU_FRAME = 5.0;
const double MU_LONG_WHEEL_FRONT = 0.1;
const double MU_LONG_WHEEL_BACK = 0.2;
const double MU_LONG_FRAME = 5.0;
TOUCHDOWNVTX TOUCH[NVTX] = {
	// pos, stiff, damping, mu, mu long
	{TOUCH1, STIFF_WHEEL, DAMPING, MU_WHEEL_FRONT, MU_LONG_WHEEL_FRONT},
	{TOUCH2, STIFF_WHEEL, DAMPING, MU_WHEEL_BACK, MU_LONG_WHEEL_BACK},
	{TOUCH3, STIFF_WHEEL, DAMPING, MU_WHEEL_BACK, MU_LONG_WHEEL_BACK},
	{TOUCH4, STIFF_FRAME, DAMPING, MU_FRAME, MU_LONG_FRAME},
	{TOUCH5, STIFF_FRAME, DAMPING, MU_FRAME, MU_LONG_FRAME},
	{TOUCH6, STIFF_FRAME, DAMPING, MU_FRAME, MU_LONG_FRAME},
	{TOUCH7, STIFF_FRAME, DAMPING, MU_FRAME, MU_LONG_FRAME},
};

const VECTOR3 TOUCH_NOGEAR_FRONT = _V(0.0, -1.24, 3.5); // near front wheel wheel up
const DWORD NVTX_NOGEAR = 5;
TOUCHDOWNVTX TOUCH_NOGEAR[NVTX] = {
	// pos, stiff, damping, mu, mu long
	{TOUCH_NOGEAR_FRONT, STIFF_WHEEL, DAMPING, MU_FRAME, MU_LONG_FRAME},
	{TOUCH6, STIFF_FRAME, DAMPING, MU_FRAME, MU_LONG_FRAME}, // wings first, because three first define landed status
	{TOUCH7, STIFF_FRAME, DAMPING, MU_FRAME, MU_LONG_FRAME},
	{TOUCH4, STIFF_FRAME, DAMPING, MU_FRAME, MU_LONG_FRAME},
	{TOUCH5, STIFF_FRAME, DAMPING, MU_FRAME, MU_LONG_FRAME},
};


MESHHANDLE MeshCockpit, MeshCockpit2, MeshFrame;
const VECTOR3 COCKPIT_OFFSET = { 0,0.2,4.5 };

class T38 : public VESSEL4 {
public:
	T38(OBJHANDLE hVessel, int flightmodel);
	~T38();
	void clbkSetClassCaps(FILEHANDLE cfg);
	void clbkPostCreation();
	void clbkPreStep(double simt, double simdt, double mjd);
	void clbkPostStep(double simt, double simdt, double mjd);
	int clbkConsumeBufferedKey(DWORD key, bool down, char* kstate);
	void clbkLoadStateEx(FILEHANDLE scn, void* status);
	void clbkSaveState(FILEHANDLE scn);
	bool clbkLoadVC(int id);
	bool clbkVCMouseEvent(int id, int event, VECTOR3& p);
	bool clbkDrawHUD(int mode, const HUDPAINTSPEC* hps, oapi::Sketchpad* skp);

	void DefineAnimations(void);
	void ToggleLight();

	bool PIDSetTargets(char* str);
	bool PIDSetCoeff(char* str, int co);
	double GetDescRate(double vel);
	//bool PIDSetBank(char* str);
	//bool PIDSetSpeed(char* str);

private:
	static void vlift(VESSEL* v, double aoa, double M, double Re, void* context, double* cl, double* cm, double* cd);
	static void hlift(VESSEL* v, double aoa, double M, double Re, void* context, double* cl, double* cm, double* cd);

	THRUSTER_HANDLE thMain;
	PROPELLANT_HANDLE propMain;
	PARTICLESTREAMSPEC contrailMain, condensation;
	PSTREAM_HANDLE condensationStream;
	bool condensationActive = true;
	const double contrailBegin = 0.4549 * ATMD; // Air density for contrail to begin
	const double contrailEnd = 0.2546 * ATMD; // Air density for contrail to end

	double enginePower = 0.0;

	enum ANIMSTATE { RETRACTED , DEPLOYED, RETRACTING, DEPLOYING } 
	GearStatus = DEPLOYED, 
		SpeedbrakeStatus = RETRACTED,
		FlapStatus = RETRACTED;
	double gearProgress = 1.0;
	double speedbrakeProgress = 0.0;
	double flapProgress = 0.0;
	double wheelRotation = 0.0;

	bool lightsOn = false;

	bool allowHighThrust = false;
	bool highThrust = false; // debug, just for fun
	bool holdingPattern = false;
	double PIDintegralAlt = 0.0, PIDpreviousAltError = 0.0;
	double PIDintegralBank = 0.0, PIDpreviousErrorBank = 0.0;
	double PIDintegralSpeed = 0.0, PIDpreviousErrorSpeed = 0.0;
	char PIDAltInfo[256], PIDBankInfo[256], PIDSpeedInfo[256];
	double PIDtargetAlt = 10e3, PIDtargetBank = 45.0 * RAD, PIDtargetSpeed = 250.0;
	bool autoLand = false;
	double PIDtargetDesc = -0.5;

	double PIDA = 5e-2, PIDB = 5e-2, PIDC = 1e-1;

	enum ANIM_ID { AID_ENGINEMAIN_TOP, AID_ENGINEMAIN_BOT, AID_GEAR, AID_FLAPS, AID_BRAKES, AID_LIGHTS, AID_MFD1_PWR, AID_MFD2_PWR, 
		AID_MFD1_SEL, AID_MFD2_SEL, AID_MFD1_MEN, AID_MFD2_MEN,
	AID_MFD1_BT1, AID_MFD1_BT2, AID_MFD1_BT3, AID_MFD1_BT4, AID_MFD1_BT5, AID_MFD1_BT6, AID_MFD1_BT7, AID_MFD1_BT8, AID_MFD1_BT9, AID_MFD1_BT10, AID_MFD1_BT11, AID_MFD1_BT12,
		AID_MFD2_BT1, AID_MFD2_BT2, AID_MFD2_BT3, AID_MFD2_BT4, AID_MFD2_BT5, AID_MFD2_BT6, AID_MFD2_BT7, AID_MFD2_BT8, AID_MFD2_BT9, AID_MFD2_BT10, AID_MFD2_BT11, AID_MFD2_BT12,
	};

	// OrbiterSound
	double enginePitch = 22000;
	int OSid;
	enum OSWAVS { OSENG = 1, OSGEAR, OSFLAP, OSCLICK, OSCLICKOFF };


	UINT anim_wheel;
	UINT anim_gear;
	UINT anim_gearlever;
	UINT anim_rudder;
	UINT anim_elevator;
	UINT anim_laileron;
	UINT anim_raileron;
	UINT anim_speedbrake;
	UINT anim_flaps;
	//UINT anim_hook;
	UINT anim_power;
	UINT anim_dial44;
	UINT anim_wheel0;
	UINT anim_wheel1;
	UINT anim_wheel2;
	UINT anim_wheel3;
	UINT anim_wheel4;
	UINT anim_wheel5;
	UINT anim_fuel;
	UINT anim_turn;
	UINT anim_dial1;
	UINT anim_dial2;
	UINT anim_dial3;
	UINT anim_bank;
	UINT anim_clock_s;
	UINT anim_clock_m;
	UINT anim_clock_h;
	MGROUP_TRANSFORM* wheel1;

	BEACONLIGHTSPEC beacon1;
	BEACONLIGHTSPEC beacon2;

	UINT animactive;
	UINT animactivel;
	UINT animactive2;
	UINT animactive3;
	UINT animactive4;
	UINT animactive5;
	UINT animactive7;
	UINT animactive9;

	//double speedbrake = 0, gear_proc2 = 0;
};

T38::T38(OBJHANDLE hVessel, int flightmodel) : VESSEL4 (hVessel, flightmodel)
{
	MeshFrame = oapiLoadMeshGlobal("K33\\K-T-38-v4");
	MeshCockpit = oapiLoadMeshGlobal("K33\\K-T-38-v4-Cpit");
	MeshCockpit2 = oapiLoadMeshGlobal("K33\\K-T-38-v4-Cpit_n");

	DefineAnimations();

}

T38::~T38()
{
}

void T38::clbkSetClassCaps(FILEHANDLE cfg)
{
	// Read config
	if (oapiReadItem_bool(cfg, "REALISTIC", useRealisticValues)) oapiWriteLog(useRealisticValues ? "T-38 set to realistic" : "T-38 set to unrealistic");
	else oapiWriteLog("T-38 REALISTIC flag not found");
	if (oapiReadItem_bool(cfg, "SPACEBAR", allowHighThrust)) oapiWriteLog(allowHighThrust ? "T-38 spacebar for 10x thrust enabled" : "T-38 spacebar for 10x thrust disabled");
	else oapiWriteLog("T-38 SPACEBAR flag not found");
	if (useRealisticValues)
	{
		MASS = MASS_REAL;
		FUEL = FUEL_REAL;
		THRUST = THRUST_REAL;
		ISP = ISP_REAL;
	}

	SetSize(8);
	SetEmptyMass(MASS);

	SetRotDrag(_V(3.5, 3.5, 1.0));
	SetPMI(_V(15.5, 22.1, 7.7));
	//SetTrimScale(0.05);
	SetCameraOffset(_V(0, 1.433, 2.1));
	SetTouchdownPoints(TOUCH, NVTX);
	SetNosewheelSteering(true);

	/*const double ASPECT_RATIO = 3.75;
	const double WING_AREA = 15.8;
	const double CHORD_LENGTH = 2.356;
	const double TAIL_AREA = 5.481;
	const double TAIL_ASPECT_RATIO = 2.82;*/
	const double CHORD_LENGTH = 20.0;
	const double WING_AREA = 270.0;
	const double ASPECT_RATIO = 2.266;
	const double TAIL_AREA = 50.0;
	const double TAIL_ASPECT_RATIO = 1.5;
	CreateAirfoil3(LIFT_VERTICAL, _V(0, 0, -0.5), vlift, NULL, CHORD_LENGTH, WING_AREA, ASPECT_RATIO);
	CreateAirfoil3(LIFT_HORIZONTAL, _V(0, 0, -4), hlift, NULL, CHORD_LENGTH, TAIL_AREA, TAIL_ASPECT_RATIO);

	CreateControlSurface(AIRCTRL_ELEVATOR, 7, 0.7, _V(0, 0, -17), AIRCTRL_AXIS_XPOS, anim_elevator);
	CreateControlSurface(AIRCTRL_RUDDER, 2, 0.5, _V(0, 2, -15), AIRCTRL_AXIS_YPOS, anim_rudder);
	CreateControlSurface(AIRCTRL_AILERON, 4.8, 0.2, _V(17.8, 0, -17.8), AIRCTRL_AXIS_XPOS, anim_raileron);
	CreateControlSurface(AIRCTRL_AILERON, 4.8, 0.2, _V(-17.8, 0, -17.8), AIRCTRL_AXIS_XNEG, anim_laileron);
	CreateControlSurface(AIRCTRL_ELEVATORTRIM, 5, 0.1, _V(0, 0, -15), AIRCTRL_AXIS_XPOS, anim_elevator);
	CreateControlSurface(AIRCTRL_FLAP, 5, 5, _V(0, 0, -1), AIRCTRL_AXIS_XPOS, anim_flaps);

	CreateVariableDragElement(&speedbrakeProgress, 8.0, _V(0, 0, 0));
	CreateVariableDragElement(&gearProgress, 0.8, _V(0, -1, 0));

	propMain = CreatePropellantResource(FUEL);
	SURFHANDLE tex_main = oapiRegisterExhaustTexture("K-T-38-v4-Exhaust");

	contrailMain = {
		0, 0.3, 165, 19, 0.019, 0.111, 1.11, 1.11, PARTICLESTREAMSPEC::DIFFUSE,
		PARTICLESTREAMSPEC::LVL_PSQRT, 0, 2,
		PARTICLESTREAMSPEC::ATM_PLOG, 1e-4, 1
	};

	thMain = CreateThruster(_V(0, 0, -4.3), _V(0, 0, 1), THRUST, propMain, ISP);
	CreateThrusterGroup(&thMain, 1, THGROUP_MAIN);

	AddExhaust(thMain, 4, 0.4, _V(-0.1870975, -0.4776336, -6.6), _V(0, 0, -1), tex_main);
	AddExhaust(thMain, 4, 0.4, _V(0.1870975, -0.4776336, -6.6), _V(0, 0, -1), tex_main);
	AddExhaustStream(thMain, _V(-0.1870975, -0.4776336, -6.87), &contrailMain);
	AddExhaustStream(thMain, _V(0.1870975, -0.4776336, -6.87), &contrailMain);

	condensation = {
		0, 3.0, 5, 50, 0.1, 60.0, 0.5, 50.0, PARTICLESTREAMSPEC::DIFFUSE,
		PARTICLESTREAMSPEC::LVL_PSQRT, 0, 2,
		PARTICLESTREAMSPEC::ATM_PLOG, contrailBegin, contrailEnd
	};
	condensationStream = AddExhaustStream(thMain, _V(0, -0.4776, -10), &condensation);

	// Strobe lights
	//Blue strobe
	static VECTOR3 beacon1pos = { 0,-1.215817,2.438052 };
	static VECTOR3 beacon1col = { 0.043, 0.161, 0.992 };
	beacon1.shape = (BEACONSHAPE_STAR);
	beacon1.pos = &beacon1pos;
	beacon1.col = &beacon1col;
	beacon1.size = (0.18);
	beacon1.falloff = (0.6);
	beacon1.period = (1.13);
	beacon1.duration = (0.05);
	beacon1.tofs = (0.6);
	beacon1.active = false;
	AddBeacon(&beacon1);

	//Red strobe
	static VECTOR3 beacon2pos = { 0,0.3330124,0.2956965 };
	static VECTOR3 beacon2col = { 0.988, 0.133, 0.133 };
	beacon2.shape = (BEACONSHAPE_STAR);
	beacon2.pos = &beacon2pos;
	beacon2.col = &beacon2col;
	beacon2.size = (0.18);
	beacon2.falloff = (0.6);
	beacon2.period = (1.13);
	beacon2.duration = (0.05);
	beacon2.tofs = (0.6);
	beacon2.active = false;
	AddBeacon(&beacon2);


	AddMesh(MeshFrame);
	SetMeshVisibilityMode(AddMesh(MeshCockpit2, &COCKPIT_OFFSET), MESHVIS_VC);





}

void T38::clbkPostCreation()
{
	OSid = ConnectToOrbiterSoundDLL(GetHandle());
	RequestLoadVesselWave(OSid, OSENG, "Sound\\K-T-38-v4\\K-T-38-v4-Eng.wav", BOTHVIEW_FADED_FAR);
	RequestLoadVesselWave(OSid, OSGEAR, "Sound\\K-T-38-v4\\K-T-38-v4-Gear.wav", BOTHVIEW_FADED_FAR);
	RequestLoadVesselWave(OSid, OSFLAP, "Sound\\K-T-38-v4\\K-T-38-v4-Flaps.wav", BOTHVIEW_FADED_FAR);
	RequestLoadVesselWave(OSid, OSCLICK, "Sound\\K-T-38-v4\\K-T-38-v4-Click.wav", BOTHVIEW_FADED_FAR);
	RequestLoadVesselWave(OSid, OSCLICKOFF, "Sound\\K-T-38-v4\\K-T-38-v4-ClickOff.wav", BOTHVIEW_FADED_FAR);

	SoundOptionOnOff(OSid, PLAYMAINTHRUST, false);
}

void T38::clbkPreStep(double simt, double simdt, double mjd)
{
	const double GearTime = 2.0;
	switch (GearStatus)
	{
	case T38::RETRACTING:
		gearProgress -= simdt / GearTime;
		break;
	case T38::DEPLOYING:
		gearProgress += simdt / GearTime;
		break;
	default:
		break;
	}

	if (gearProgress < 0.0)
	{
		gearProgress = 0.0;
		GearStatus = RETRACTED;
		SetTouchdownPoints(TOUCH_NOGEAR, NVTX_NOGEAR);
	}
	else if (gearProgress > 1.0)
	{
		gearProgress = 1.0;
		GearStatus = DEPLOYED;
		SetTouchdownPoints(TOUCH, NVTX);
	}

	SetAnimation(anim_gear, 1.0 - gearProgress); // seems like gear is flipped in animation definition.
	SetAnimation(anim_gearlever, 1.0 - gearProgress); // Gear cockpit indicators
	if (GroundContact() && GearStatus == DEPLOYED)
	{
		wheelRotation = fmod(wheelRotation + simdt * GetGroundspeed(), 1.0);
		SetAnimation(anim_wheel, wheelRotation);
	}
	else SetAnimation(anim_wheel, 0.0);

	// Flaps
	const double FlapTime = 2.0;
	switch (FlapStatus)
	{
	case T38::RETRACTING:
		flapProgress -= simdt / FlapTime;
		break;
	case T38::DEPLOYING:
		flapProgress += simdt / FlapTime;
		break;
	default:
		break;
	}

	if (flapProgress < 0.0)
	{
		flapProgress = 0.0;
		FlapStatus = RETRACTED;
	}
	else if (flapProgress > 1.0)
	{
		flapProgress = 1.0;
		FlapStatus = DEPLOYED;
	}

	SetControlSurfaceLevel(AIRCTRL_FLAP, flapProgress);

	// Speedbrake
	const double SpeedbrakeTime = 1.0;
	switch (SpeedbrakeStatus)
	{
	case T38::RETRACTING:
		speedbrakeProgress -= simdt / SpeedbrakeTime;
		break;
	case T38::DEPLOYING:
		speedbrakeProgress += simdt / SpeedbrakeTime;
		break;
	default:
		break;
	}

	if (speedbrakeProgress < 0.0)
	{
		speedbrakeProgress = 0.0;
		SpeedbrakeStatus = RETRACTED;
	}
	else if (speedbrakeProgress > 1.0)
	{
		speedbrakeProgress = 1.0;
		SpeedbrakeStatus = DEPLOYED;
	}

	SetAnimation(anim_speedbrake, speedbrakeProgress);	

	// Throttle animation
	enginePower = GetThrusterLevel(thMain);
	SetAnimation(anim_power, enginePower);

	// Altimeter
	double altitude = GetAltitude(ALTMODE_MEANRAD); // pressure altimeter (I assume).
	altitude = max(0, altitude); // at least 0
	SetAnimation(anim_wheel0, fmod(altitude / 100.0, 1.0));
	// Indicators.
	int alt5 = int(altitude / 1e4) * 10000;
	int alt4 = int((altitude - alt5) / 1e3) * 1000;
	int alt3 = int((altitude - alt5 - alt4) / 1e2) * 100;
	int alt2 = int((altitude - alt5 - alt4 - alt3) / 1e1) * 10;
	int alt1 = int(altitude - alt5 - alt4 - alt3 - alt2);
	SetAnimation(anim_wheel1, double(alt1) / 10.0);
	SetAnimation(anim_wheel2, double(alt2) / 100.0);
	SetAnimation(anim_wheel3, double(alt3) / 1000.0);
	SetAnimation(anim_wheel4, double(alt4) / 10000.0);
	SetAnimation(anim_wheel5, double(alt5) / 100000.0);

	// AOA indicator
	double speed = GetAirspeed();
	double aoa = GetAOA();
	SetAnimation(anim_dial1, speed > 0.6 ? aoa * 6.0 : 0);

	// Compass
	double heading;
	oapiGetFocusHeading(&heading);
	SetAnimation(anim_dial2, heading / PI2);

	// Speedometer
	SetAnimation(anim_dial3, max(1.0 - speed / 0.5144 / 600.0, 0)); // 1 knot = 0.5144 m/s

	// Bank
	SetAnimation(anim_bank, (GetBank() + PI) / PI2);

	// Clock
	double seconds = fmod(simt, 60.0);
	double minutes = fmod(simt, 3600.0) / 60.0;
	double hours = fmod(simt, 86400.0 / 2.0) / 3600.0; // 12 hours on a clock, so divide 86400 by 2.
	SetAnimation(anim_clock_s, seconds / 60.0);
	SetAnimation(anim_clock_m, minutes / 60.0);
	SetAnimation(anim_clock_h, hours / 12.0);

	// Fuel
	SetAnimation(anim_fuel, (1.0 - GetPropellantMass(propMain) / GetPropellantMaxMass(propMain)) / 3.0);

	// Simulate forces on head camera
	double headMove = 0.0;
	if (speed > 0.6) headMove = max(min(aoa * 1.2, 1.0 / 3.0), -1.0 / 3.0);
	SetCameraOffset(_V(0, 1.433 + headMove, 2.1));

	// Set engine thrust as function of density
	double density = GetAtmDensity();
	if (density > 0.195) SetThrusterMax0(thMain, highThrust ? THRUST * 10 : THRUST);
	else if (density > 0.145) SetThrusterMax0(thMain, (highThrust ? THRUST * 10 : THRUST) * sqrt((density - 0.145) / (0.195 - 0.145)));
	else SetThrusterMax0(thMain, 0.0);

	double thrust = GetThrusterLevel(thMain);
	if (thrust > 0.0)
	{
		enginePitch = thrust * 6000.0 + 13000.0;
		PlayVesselWave(OSid, OSENG, LOOP, 200, enginePitch);
	}
	else StopVesselWave(OSid, OSENG);
}

void T38::clbkPostStep(double simt, double simdt, double mjd)
{
	// Contrail (defined here in PostStep just because PreStep is populated enough already).
	if (GetAtmDensity() < contrailEnd)
	{
		DelExhaustStream(condensationStream);
		condensationActive = false;
	}
	else if (!condensationActive)
	{
		condensationStream = AddExhaustStream(thMain, _V(0, -0.4776, -10), &condensation);
		condensationActive = true;
	}

	if (holdingPattern)
	{
		if (autoLand)
		{
			if (GroundContact()) PIDtargetSpeed = 0.0; // turn off engine if landed
			else PIDtargetAlt += PIDtargetDesc * simdt; // don't keep on reducing if landed
			//if (GetAltitude() < PIDtargetAlt - 1.0) PIDtargetAlt = GetAltitude();
		}
		
		// Hold altitude (PIDtargetAlt m)
		double proportionalGainConstantAlt = 0.005; // Kp, with high sensitivity if close to ground
		double integralGainConstantAlt = 0.00001; // Ki
		double derivativeGainConstantAlt = 0.01; // Kd
		if (GetAltitude(ALTMODE_GROUND) < 250.0)
		{
			proportionalGainConstantAlt = 0.01;
			integralGainConstantAlt = 0.001;
			derivativeGainConstantAlt = 0.05;
		}
		double PIDerrorAlt = PIDtargetAlt - GetAltitude(); // setpoint - measured_value
		PIDintegralAlt += PIDerrorAlt * simdt; // integral + error * dt
		double PIDderivative = (PIDerrorAlt - PIDpreviousAltError) / simdt;
		double PIDoutput = proportionalGainConstantAlt * PIDerrorAlt + integralGainConstantAlt * PIDintegralAlt + derivativeGainConstantAlt * PIDderivative;
		PIDpreviousAltError = PIDerrorAlt;
		double pitchLvl = PIDoutput;
		if (PIDerrorAlt > 0.0 && GetPitch() > 45 * RAD) pitchLvl = 0.0; // below target and pitching up 45 degrees
		else if (PIDerrorAlt < 0.0 && GetPitch() < max(-45, -GetAltitude() / 1e3 * 10) * RAD) pitchLvl = 0.0; // above target and pitching down 45 degrees
		sprintf(PIDAltInfo, "Alt: %.3f km, P%+.1f m I%+.0f D%+.2f, pitchLvl: %+.3f", GetAltitude() / 1e3, PIDerrorAlt, PIDintegralAlt, PIDderivative, pitchLvl);
		SetControlSurfaceLevel(AIRCTRL_ELEVATOR, pitchLvl);
		
		// Hold bank (45 degrees right)
		const double proportionalGainConstantBank = 1.0; // Kp
		const double integralGainConstantBank = 0.0001; // Ki
		const double derivativeGainConstantBank = 0.01; // Kd
		double PIDerrorBank = PIDtargetBank - GetBank(); // setpoint - measured_value
		PIDintegralBank += PIDerrorBank * simdt; // integral + error * dt
		double PIDderivativeBank = (PIDerrorBank - PIDpreviousErrorBank) / simdt;
		double PIDoutputBank = proportionalGainConstantBank * PIDerrorBank + integralGainConstantBank * PIDintegralBank + derivativeGainConstantBank * PIDderivativeBank;
		PIDpreviousErrorBank = PIDerrorBank;
		double bankLvl = -PIDoutputBank;
		sprintf(PIDBankInfo, "Bank: %.2f\u00B0, P%+.3f\u00B0 I%+.4f D%+.4f, bankLvl: %+.3f", GetBank() * DEG, PIDerrorBank * DEG, PIDintegralBank, PIDderivativeBank, bankLvl);
		SetControlSurfaceLevel(AIRCTRL_AILERON, bankLvl);

		// Hold speed (250 m/s)
		const double proportionalGainConstantSpeed = 1.0; // Kp
		const double integralGainConstantSpeed = 0.01; // Ki
		const double derivativeGainConstantSpeed = 0.0001; // Kd
		double PIDerrorSpeed = PIDtargetSpeed - GetGroundspeed(); // setpoint - measured_value
		PIDintegralSpeed += PIDerrorSpeed * simdt; // integral + error * dt
		double PIDderivativeSpeed = (PIDerrorSpeed - PIDpreviousErrorSpeed) / simdt;
		double PIDoutputSpeed = proportionalGainConstantSpeed * PIDerrorSpeed + integralGainConstantSpeed * PIDintegralSpeed + derivativeGainConstantSpeed * PIDderivativeSpeed;
		PIDpreviousErrorSpeed = PIDerrorSpeed;
		double thrustLvl = PIDoutputSpeed;
		sprintf(PIDSpeedInfo, "Speed: %.1f m/s, P%+.2f m/s I%+.2f D%+.3f, thrustLvl: %.4f", GetGroundspeed(), PIDerrorSpeed, PIDintegralSpeed, PIDderivativeSpeed, thrustLvl);
		SetThrusterGroupLevel(THGROUP_MAIN, thrustLvl);
	}
}



int T38::clbkConsumeBufferedKey(DWORD key, bool down, char* kstate)
{
	if (!down) return 0; // only process keydown events

	bool PIDsetCoeffA(void* id, char* str, void* data);
	bool PIDsetCoeffB(void* id, char* str, void* data);
	bool PIDsetCoeffC(void* id, char* str, void* data);

	if (KEYMOD_CONTROL(kstate) || KEYMOD_ALT(kstate) || KEYMOD_SHIFT(kstate)) return 0; // no ctrl here

	switch (key) {
	case OAPI_KEY_SPACE: // life is slow
		if (allowHighThrust)
		{
			highThrust = !highThrust;
			if (highThrust) SetThrusterMax0(thMain, THRUST * 10.0);
			else SetThrusterMax0(thMain, THRUST);

			return 1;
		}

		return 0;
	case OAPI_KEY_G: // Gear

		if (GroundContact()) return 1; // gear locked

		PlayVesselWave(OSid, OSGEAR, NOLOOP, 255);

		switch (GearStatus)
		{
		case T38::RETRACTED:
		case T38::RETRACTING:
			GearStatus = DEPLOYING;
			return 1;
		case T38::DEPLOYED:
		case T38::DEPLOYING:
			GearStatus = RETRACTING;
			return 1;
		}

		return 0;

	case OAPI_KEY_B: // Speedbrake

		PlayVesselWave(OSid, OSFLAP, NOLOOP, 255);

		switch (SpeedbrakeStatus)
		{
		case T38::RETRACTED:
		case T38::RETRACTING:
			SpeedbrakeStatus = DEPLOYING;
			return 1;
		case T38::DEPLOYED:
		case T38::DEPLOYING:
			SpeedbrakeStatus = RETRACTING;
			return 1;
		}

		return 0;


	case OAPI_KEY_F: // Flaps

		PlayVesselWave(OSid, OSFLAP, NOLOOP, 255);

		switch (FlapStatus)
		{
		case T38::RETRACTED:
		case T38::RETRACTING:
			FlapStatus = DEPLOYING;
			return 1;
		case T38::DEPLOYED:
		case T38::DEPLOYING:
			FlapStatus = RETRACTING;
			return 1;
		}

		return 0;

	case OAPI_KEY_PERIOD: // Light switch

		//ToggleLight();

		if (autoLand && !GroundContact()) PIDtargetDesc -= GetDescRate(PIDtargetDesc);

		return 1;
	case OAPI_KEY_COMMA:

		if (autoLand && !GroundContact()) PIDtargetDesc += GetDescRate(PIDtargetDesc);
		return 1;
	case OAPI_KEY_P: // holding pattern
		if (holdingPattern) // disengaging
		{
			SetControlSurfaceLevel(AIRCTRL_ELEVATOR, 0.0);
			SetControlSurfaceLevel(AIRCTRL_AILERON, 0.0);
			PIDintegralAlt = 0.0;
			PIDintegralBank = 0.0;
			PIDintegralSpeed = 0.0;
			PIDpreviousAltError = 0.0;
			PIDpreviousErrorBank = 0.0;
			PIDpreviousErrorSpeed = 0.0;
		}

		holdingPattern = !holdingPattern;
		autoLand = false;
		return 1;
	case OAPI_KEY_L: // lift-off / landing
		if (GroundContact()) // lift off
		{
			holdingPattern = true;
			PIDtargetAlt = GetAltitude() + 1e3; // go 1 km up from current (landed) altitude.
			PIDtargetBank = 0.0;
			PIDtargetSpeed = 150.0;
			autoLand = false;
		}
		else if (holdingPattern) // must have autopilot activated.
		{
			//PIDintegralAlt = 0.0;
			//PIDpreviousAltError = 0.0;

			PIDtargetAlt = GetAltitude();
			PIDtargetDesc = -0.5;

			autoLand = true;
		}
		return true;
	case OAPI_KEY_O:
		bool PIDsetTargets(void* id, char* str, void* data);
		if (holdingPattern)
		{
			autoLand = false;
			oapiOpenInputBox("Target altitude (km), bank (\u00B0), speed (m/s)", PIDsetTargets, 0, 20, (void*)this);

			return 1;
		}
	}
	return 0;
}

void T38::clbkLoadStateEx(FILEHANDLE scn, void* status)
{
	char* cbuf;

	while (oapiReadScenario_nextline(scn, cbuf))
	{
		if (!_strnicmp(cbuf, "GEAR", 4))
		{
			GearStatus = ANIMSTATE(atoi(cbuf + 4));
			gearProgress = atof(cbuf + 5);

			if (GearStatus == RETRACTED) SetTouchdownPoints(TOUCH_NOGEAR, NVTX_NOGEAR);
		}
		else if (!_strnicmp(cbuf, "BRAKE", 5))
		{
			SpeedbrakeStatus = ANIMSTATE(atoi(cbuf + 5));
			speedbrakeProgress = atof(cbuf + 6);
		}
		else if (!_strnicmp(cbuf, "FLAPS", 5))
		{
			FlapStatus = ANIMSTATE(atoi(cbuf + 5));
			flapProgress = atof(cbuf + 6);
		}
		else if (!_strnicmp(cbuf, "LIGHTS", 6))
		{
			lightsOn = bool(atoi(cbuf + 6));
		}
		else if (!_strnicmp(cbuf, "PID", 3))
		{
			holdingPattern = true;
			PIDSetTargets(cbuf + 3);
		}
		else ParseScenarioLineEx(cbuf, status);
	}
}

void T38::clbkSaveState(FILEHANDLE scn)
{
	VESSEL4::clbkSaveState(scn); // write default parameters (orbital elements etc.)

	char cbuf[256];

	sprintf(cbuf, "%i %.3f", int(GearStatus), gearProgress);
	oapiWriteScenario_string(scn, "GEAR", cbuf);

	sprintf(cbuf, "%i %.3f", int(SpeedbrakeStatus), speedbrakeProgress);
	oapiWriteScenario_string(scn, "BRAKE", cbuf);

	sprintf(cbuf, "%i %.3f", int(FlapStatus), flapProgress);
	oapiWriteScenario_string(scn, "FLAPS", cbuf);

	oapiWriteScenario_int(scn, "LIGHTS", int(lightsOn));

	sprintf(cbuf, "%.3f, %.1f, %.1f", PIDtargetAlt / 1e3, PIDtargetBank * DEG, PIDtargetSpeed);
	if (holdingPattern) oapiWriteScenario_string(scn, "PID", cbuf);

	oapiWriteLogV("A: %e, B: %e, C: %e", PIDA, PIDB, PIDC);
}

bool T38::clbkLoadVC(int id)
{
	SetCameraOffset(_V(0, 1.433, 2.1));

	SetCameraDefaultDirection(_V(0, 0, 1));

	SetCameraRotationRange(RAD * 90, RAD * 90, RAD * 45, RAD * 45);

	static VCHUDSPEC hud_pilot = { 1, 199,{0,1.4,4.1},0.8 };
	static VCMFDSPEC mfds_left = { 1, 196 };
	static VCMFDSPEC mfds_right = { 1, 197 };


	oapiVCRegisterHUD(&hud_pilot);
	oapiVCRegisterMFD(MFD_LEFT, &mfds_left);
	oapiVCRegisterMFD(MFD_RIGHT, &mfds_right);

	VECTOR3 offset = COCKPIT_OFFSET;

	// Mouse Event


	// Throttle lever Top animations
	oapiVCRegisterArea(AID_ENGINEMAIN_TOP, PANEL_REDRAW_ALWAYS, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_LBPRESSED);
	oapiVCSetAreaClickmode_Spherical(AID_ENGINEMAIN_TOP, _V(-1.701009, -1.744822, 0.3680018) + offset, 0.25);

	// Throttle lever Bot animations
	oapiVCRegisterArea(AID_ENGINEMAIN_BOT, PANEL_REDRAW_ALWAYS, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_LBPRESSED);
	oapiVCSetAreaClickmode_Spherical(AID_ENGINEMAIN_BOT, _V(-1.701009, -1.744822, -0.3068204) + offset, 0.25);


	// Gear lever animations
	oapiVCRegisterArea(AID_GEAR, PANEL_REDRAW_ALWAYS, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(AID_GEAR, _V(-1.595257, -0.1138984, 2.696076) + offset, 0.25);


	// Flaps lever animations
	oapiVCRegisterArea(AID_FLAPS, PANEL_REDRAW_ALWAYS, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(AID_FLAPS, _V(-1.561316, -1.000344, 2.696076) + offset, 0.2);


	// AirBrakes lever animations
	oapiVCRegisterArea(AID_BRAKES, PANEL_REDRAW_ALWAYS, PANEL_MOUSE_LBDOWN);
	oapiVCSetAreaClickmode_Spherical(AID_BRAKES, _V(1.754886, -1.029909, 2.696076) + offset, 0.2);

	// Lights Button
	oapiVCRegisterArea(AID_LIGHTS, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCSetAreaClickmode_Spherical(AID_LIGHTS, _V(-1.521535, 0.5728442, 2.696076) + offset, 0.04);


	//MFD Power Buttons
	const double powerButtonRadius = 0.04; // radius of power button on each MFD1
	const double powerButtonRadius2 = 0.027; // radius of power button on each MFD2

	oapiVCRegisterArea(AID_MFD1_PWR, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_PWR, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);

	oapiVCSetAreaClickmode_Spherical(AID_MFD1_PWR, _V(-0.1268893, -0.5734969, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_PWR, _V(0.9234712, -1.324777E-02, 2.6) + offset, powerButtonRadius2);

	//MFD Mode Select Buttons

	oapiVCRegisterArea(AID_MFD1_SEL, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_SEL, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);

	oapiVCSetAreaClickmode_Spherical(AID_MFD1_SEL, _V(4.668726E-04, -0.5734969, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_SEL, _V(1.017748, -1.159377E-02, 2.6) + offset, powerButtonRadius2);



	//MFD Menu Buttons

	oapiVCRegisterArea(AID_MFD1_MEN, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_MEN, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);

	oapiVCSetAreaClickmode_Spherical(AID_MFD1_MEN, _V(0.1377469, -0.5734969, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_MEN, _V(1.103755, -1.159377E-02, 2.6) + offset, powerButtonRadius2);


	//MFD_LEFT Buttons

	oapiVCRegisterArea(AID_MFD1_BT1, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD1_BT2, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD1_BT3, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD1_BT4, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD1_BT5, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD1_BT6, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD1_BT7, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD1_BT8, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD1_BT9, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD1_BT10, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD1_BT11, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD1_BT12, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);

	oapiVCSetAreaClickmode_Spherical(AID_MFD1_BT1, _V(-0.4554956, 0.1192771, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD1_BT2, _V(-0.4554956, 8.460711E-03, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD1_BT3, _V(-0.4554956, -0.1106255, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD1_BT4, _V(-0.4554956, -0.2230959, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD1_BT5, _V(-0.4554956, -0.3388742, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD1_BT6, _V(-0.4554956, -0.4480366, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD1_BT7, _V(0.4326894, 0.1192771, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD1_BT8, _V(0.4326894, 8.460711E-03, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD1_BT9, _V(0.4326894, -0.1106255, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD1_BT10, _V(0.4326894, -0.2230959, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD1_BT11, _V(0.4326894, -0.3388742, 2.6) + offset, powerButtonRadius);
	oapiVCSetAreaClickmode_Spherical(AID_MFD1_BT12, _V(0.4326894, -0.4480366, 2.6) + offset, powerButtonRadius);


	//MFD_RIGHT Buttons
	oapiVCRegisterArea(AID_MFD2_BT1, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_BT2, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_BT3, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_BT4, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_BT5, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_BT6, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_BT7, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_BT8, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_BT9, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_BT10, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_BT11, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);
	oapiVCRegisterArea(AID_MFD2_BT12, PANEL_REDRAW_NEVER, PANEL_MOUSE_LBDOWN | PANEL_MOUSE_ONREPLAY);

	oapiVCSetAreaClickmode_Spherical(AID_MFD2_BT1, _V(0.6308033, 0.545, 2.6) + offset, powerButtonRadius2);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_BT2, _V(0.6308033, 0.4562395, 2.6) + offset, powerButtonRadius2);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_BT3, _V(0.6308033, 0.3636169, 2.6) + offset, powerButtonRadius2);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_BT4, _V(0.6308033, 0.2759562, 2.6) + offset, powerButtonRadius2);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_BT5, _V(0.6308033, 0.1882955, 2.6) + offset, powerButtonRadius2);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_BT6, _V(0.6308033, 0.1006347, 2.6) + offset, powerButtonRadius2);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_BT7, _V(1.380054, 0.545, 2.6) + offset, powerButtonRadius2);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_BT8, _V(1.380054, 0.4562395, 2.6) + offset, powerButtonRadius2);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_BT9, _V(1.380054, 0.3636169, 2.6) + offset, powerButtonRadius2);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_BT10, _V(1.380054, 0.2759562, 2.6) + offset, powerButtonRadius2);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_BT11, _V(1.380054, 0.1882955, 2.6) + offset, powerButtonRadius2);
	oapiVCSetAreaClickmode_Spherical(AID_MFD2_BT12, _V(1.380054, 0.1006347, 2.6) + offset, powerButtonRadius2);

	// End
	return true;
}

bool T38::clbkVCMouseEvent(int id, int event, VECTOR3& p)
{
	double simdt = oapiGetSimStep();

	switch (id) {
		// panel 0 events:

	case AID_ENGINEMAIN_TOP:
	{
		//if (event & PANEL_MOUSE_LBDOWN) { // record which slider to operate
		//if (GetThrusterLevel(thMain) == 0) enginePower = 0;
		enginePower += simdt;
		//sprintf(oapiDebugString(),"mouse %19.19f", mx1 );
		if (enginePower < 0)enginePower = 0;
		if (enginePower > 1)enginePower = 1;
		SetThrusterLevel(thMain, enginePower);


		return 1;
	}

	case AID_ENGINEMAIN_BOT:
	{
		//if (event & PANEL_MOUSE_LBDOWN) { // record which slider to operate
		//if (GetThrusterLevel(thMain) == 0) enginePower = 0;
		enginePower -= simdt;
		//sprintf(oapiDebugString(),"mouse %19.19f", mx1 );
		enginePower = min(1, max(enginePower, 0)); // limit to 0 - 1
		SetThrusterLevel(thMain, enginePower);


		return 1;
	}

	case AID_GEAR:
	{
		//if (event & PANEL_MOUSE_LBDOWN) { // record which slider to operate
		if (GroundContact()) return 1;

		PlayVesselWave(OSid, OSGEAR, NOLOOP, 255);

		switch (GearStatus)
		{
		case T38::RETRACTED:
		case T38::RETRACTING:
			GearStatus = DEPLOYING;
			return 1;
		case T38::DEPLOYED:
		case T38::DEPLOYING:
			GearStatus = RETRACTING;
			return 1;
		}

		return 0;
	}


	case AID_FLAPS:
	{
		//if (event & PANEL_MOUSE_LBDOWN) { // record which slider to operate

		PlayVesselWave(OSid, OSFLAP, NOLOOP, 255);

		switch (FlapStatus)
		{
		case T38::RETRACTED:
		case T38::RETRACTING:
			FlapStatus = DEPLOYING;
			return 1;
		case T38::DEPLOYED:
		case T38::DEPLOYING:
			FlapStatus = RETRACTING;
			return 1;
		}


		return 0;
	}

	case AID_BRAKES:
	{
		PlayVesselWave(OSid, OSFLAP, NOLOOP, 255);

		//if (event & PANEL_MOUSE_LBDOWN) { // record which slider to operate
		switch (SpeedbrakeStatus)
		{
		case T38::RETRACTED:
		case T38::RETRACTING:
			SpeedbrakeStatus = DEPLOYING;
			return 1;
		case T38::DEPLOYED:
		case T38::DEPLOYING:
			SpeedbrakeStatus = RETRACTING;
			return 1;
		}


		return 0;
	}

	case AID_LIGHTS:
		ToggleLight();
		return 1;

	// power buttons

	case AID_MFD1_PWR:
	{

		oapiSendMFDKey(MFD_LEFT, OAPI_KEY_ESCAPE);

		return 1;
	}

	case AID_MFD2_PWR:
	{

		oapiSendMFDKey(MFD_RIGHT, OAPI_KEY_ESCAPE);

		return 1;
	}




	// Mode Select buttons

	case AID_MFD1_SEL:
	{

		oapiSendMFDKey(MFD_LEFT, OAPI_KEY_F1);

		return 1;
	}

	case AID_MFD2_SEL:
	{

		oapiSendMFDKey(MFD_RIGHT, OAPI_KEY_F1);

		return 1;
	}


	// Menu buttons

	case AID_MFD1_MEN:
	{

		oapiSendMFDKey(MFD_LEFT, OAPI_KEY_GRAVE);

		return 1;
	}

	case AID_MFD2_MEN:
	{

		oapiSendMFDKey(MFD_RIGHT, OAPI_KEY_GRAVE);

		return 1;
	}



	// MFD_LEFT buttons

	case AID_MFD1_BT1:
	{

		oapiProcessMFDButton(MFD_LEFT, 0, event);


		return 1;
	}

	case AID_MFD1_BT2:
	{

		oapiProcessMFDButton(MFD_LEFT, 1, event);


		return 1;
	}

	case AID_MFD1_BT3:
	{

		oapiProcessMFDButton(MFD_LEFT, 2, event);


		return 1;
	}

	case AID_MFD1_BT4:
	{

		oapiProcessMFDButton(MFD_LEFT, 3, event);


		return 1;
	}

	case AID_MFD1_BT5:
	{

		oapiProcessMFDButton(MFD_LEFT, 4, event);


		return 1;
	}

	case AID_MFD1_BT6:
	{

		oapiProcessMFDButton(MFD_LEFT, 5, event);


		return 1;
	}

	case AID_MFD1_BT7:
	{

		oapiProcessMFDButton(MFD_LEFT, 6, event);


		return 1;
	}

	case AID_MFD1_BT8:
	{

		oapiProcessMFDButton(MFD_LEFT, 7, event);


		return 1;
	}

	case AID_MFD1_BT9:
	{

		oapiProcessMFDButton(MFD_LEFT, 8, event);


		return 1;
	}

	case AID_MFD1_BT10:
	{

		oapiProcessMFDButton(MFD_LEFT, 9, event);


		return 1;
	}

	case AID_MFD1_BT11:
	{

		oapiProcessMFDButton(MFD_LEFT, 10, event);


		return 1;
	}

	case AID_MFD1_BT12:
	{

		oapiProcessMFDButton(MFD_LEFT, 11, event);


		return 1;
	}

	// MFD_RIGHT buttons

	case AID_MFD2_BT1:
	{

		oapiProcessMFDButton(MFD_RIGHT, 0, event);


		return 1;
	}

	case AID_MFD2_BT2:
	{

		oapiProcessMFDButton(MFD_RIGHT, 1, event);


		return 1;
	}

	case AID_MFD2_BT3:
	{

		oapiProcessMFDButton(MFD_RIGHT, 2, event);


		return 1;
	}

	case AID_MFD2_BT4:
	{

		oapiProcessMFDButton(MFD_RIGHT, 3, event);


		return 1;
	}

	case AID_MFD2_BT5:
	{

		oapiProcessMFDButton(MFD_RIGHT, 4, event);


		return 1;
	}

	case AID_MFD2_BT6:
	{

		oapiProcessMFDButton(MFD_RIGHT, 5, event);


		return 1;
	}

	case AID_MFD2_BT7:
	{

		oapiProcessMFDButton(MFD_RIGHT, 6, event);


		return 1;
	}

	case AID_MFD2_BT8:
	{

		oapiProcessMFDButton(MFD_RIGHT, 7, event);


		return 1;
	}

	case AID_MFD2_BT9:
	{

		oapiProcessMFDButton(MFD_RIGHT, 8, event);


		return 1;
	}

	case AID_MFD2_BT10:
	{

		oapiProcessMFDButton(MFD_RIGHT, 9, event);


		return 1;
	}

	case AID_MFD2_BT11:
	{

		oapiProcessMFDButton(MFD_RIGHT, 10, event);


		return 1;
	}

	case AID_MFD2_BT12:
	{

		oapiProcessMFDButton(MFD_RIGHT, 11, event);


		return 1;
	}





	}

	return 0;
}

bool T38::clbkDrawHUD(int mode, const HUDPAINTSPEC* hps, oapi::Sketchpad* skp)
{
	char cbuf[256];

	skp->SetTextAlign(oapi::Sketchpad::CENTER);

	sprintf(cbuf, "BOOST ACTIVE");
	if (highThrust) skp->Text(hps->W / 2.0, hps->H / 2.0 * 0.2, cbuf, strlen(cbuf));

	const double LandedAlt = 2.5; // camera is 2.5 m over ground when landed, so give this altitude for prediction of landing spot.
	double distanceToLand = -(GetAltitude(ALTMODE_GROUND) - LandedAlt) * PIDtargetSpeed / PIDtargetDesc;
	double timeToLand = -(GetAltitude(ALTMODE_GROUND) - LandedAlt) / PIDtargetDesc;

	sprintf(cbuf, "Holding altitude %.2f km, bank %.0f\u00B0, speed %.0f m/s.", PIDtargetAlt / 1e3, PIDtargetBank * DEG, PIDtargetSpeed);
	if (GetAltitude(ALTMODE_GROUND) < 200.0 && PIDtargetSpeed < 120.0 && PIDtargetDesc < 0.0) sprintf(cbuf, "Holding altitude %.3f km, bank %.0f\u00B0, speed %.0f m/s. (%.1f km)", PIDtargetAlt / 1e3, PIDtargetBank * DEG, PIDtargetSpeed, distanceToLand / 1e3);
	if (autoLand && PIDtargetDesc < -0.05) sprintf(cbuf, "Holding descent %.1f m/s. Touch dist: %.1f km (%.0f s)", PIDtargetDesc, distanceToLand / 1e3, timeToLand);
	else if (autoLand) sprintf(cbuf, "Holding descent %.1f m/s.", PIDtargetDesc);

	if (holdingPattern)
	{
		skp->Text(hps->W / 2.0, hps->H / 2.0 * 0.55, cbuf, strlen(cbuf));
		skp->Text(hps->W / 2.0, hps->H / 2.0 * 0.65, PIDAltInfo, strlen(PIDAltInfo));
		skp->Text(hps->W / 2.0, hps->H / 2.0 * 0.75, PIDBankInfo, strlen(PIDBankInfo));
		skp->Text(hps->W / 2.0, hps->H / 2.0 * 0.85, PIDSpeedInfo, strlen(PIDSpeedInfo));
	}

	return true;
}

void T38::DefineAnimations(void)
{
	ANIMATIONCOMPONENT_HANDLE parent1;


	// Front Wheel
	static UINT WGrp1[1] = { 187 };

	wheel1 = new MGROUP_ROTATE(0, WGrp1, 1, _V(0, -1.929798, 4.011903), _V(1, 0, 0), (float)(360 * RAD));

	//Turn Front Gear
	static UINT Trn1[4] = { 8,10,11,12 };

	static MGROUP_ROTATE turn1(0, Trn1, 4, _V(0, -1.929798, 4.009077), _V(0, 1, 0), (float)(90 * RAD));

	//Rear left Wheel
	static UINT WGrp3[1] = { 188 };

	static MGROUP_ROTATE wheel2(0, WGrp3, 1, _V(0, -1.89037, -1.545275), _V(1, 0, 0), (float)(360 * RAD));

	//Rear Right wheel
	static UINT WGrp4[1] = { 189 };

	static MGROUP_ROTATE wheel3(0, WGrp4, 1, _V(0, -1.89037, -1.545275), _V(1, 0, 0), (float)(360 * RAD));

	anim_turn = CreateAnimation(0.5);
	anim_wheel = CreateAnimation(0);

	parent1 = AddAnimationComponent(anim_turn, 0, 1, &turn1);
	AddAnimationComponent(anim_wheel, 0, 1, wheel1, parent1);
	AddAnimationComponent(anim_wheel, 0, 1, &wheel2);
	AddAnimationComponent(anim_wheel, 0, 1, &wheel3);

	// Gear anims
	//Front Gear
	static UINT GGrp1[6] = { 8,9,10,11,12,187 };

	static MGROUP_ROTATE gear1(0, GGrp1, 6, _V(0, -1.118236, 3.964859), _V(1, 0, 0), (float)(-111 * RAD));


	//Rear Gear Left
	static UINT GGrp2[3] = { 16,17,189 };

	static MGROUP_ROTATE gear2(0, GGrp2, 3, _V(-1.470682, -0.9093096, -1.54335), _V(0, 0, 1), (float)(91 * RAD));

	//Rear Gear Right
	static UINT GGrp3[3] = { 14,15,188 };

	static MGROUP_ROTATE gear3(0, GGrp3, 3, _V(1.470682, -0.9093096, -1.54335), _V(0, 0, 1), (float)(-91 * RAD));

	//Front Gear Cover mid
	static UINT GGrp4[1] = { 119 };

	static MGROUP_ROTATE gear4(0, GGrp4, 1, _V(0, -1.094748, 3.949046), _V(1, 0, 0), (float)(-123 * RAD));


	//Front Gear Cover Right
	static UINT GGrp5[1] = { 120 };

	static MGROUP_ROTATE gear5(0, GGrp5, 1, _V(0.1278255, -1.089871, 4.75985), _V(0, 0, 1), (float)(-90 * RAD));


	//Rear Cover Left1
	static UINT GGrp12[2] = { 132,133 };

	static MGROUP_ROTATE gear12(0, GGrp12, 2, _V(-1.470682, -0.9093096, -1.54335), _V(0, 0, 1), (float)(99 * RAD));

	//Rear Cover Right1
	static UINT GGrp13[2] = { 130,131 };

	static MGROUP_ROTATE gear13(0, GGrp13, 2, _V(1.470682, -0.9093096, -1.54335), _V(0, 0, 1), (float)(-99 * RAD));

	//Rear Cover Left2
	static UINT GGrp14[1] = { 121 };

	static MGROUP_ROTATE gear14(0, GGrp14, 1, _V(-0.182842, -0.9707013, -1.510372), _V(0, 0, 1), (float)(-90 * RAD));

	//Rear Cover Right2
	static UINT GGrp15[1] = { 122 };

	static MGROUP_ROTATE gear15(0, GGrp15, 1, _V(0.182842, -0.9707013, -1.510372), _V(0, 0, 1), (float)(90 * RAD));


	//Rear Gear Left2
	static UINT GGrp22[2] = { 17,189 };

	static MGROUP_ROTATE gear22(0, GGrp22, 2, _V(-1.470682, -0.9093096, -1.54335), _V(0, 0, 1), (float)(4.35 * RAD));

	//Rear Gear Right2
	static UINT GGrp23[2] = { 15,188 };

	static MGROUP_ROTATE gear23(0, GGrp23, 2, _V(1.470682, -0.9093096, -1.54335), _V(0, 0, 1), (float)(-4.35 * RAD));

	//Rear Gear Left3
	static UINT GGrp24[1] = { 18 };

	static MGROUP_ROTATE gear24(0, GGrp24, 1, _V(-0.6571674, -0.92709, -1.54335), _V(0, 0, 1), (float)(-45 * RAD));

	//Rear Gear Right2
	static UINT GGrp25[1] = { 13 };

	static MGROUP_ROTATE gear25(0, GGrp25, 1, _V(0.6571674, -0.92709, -1.54335), _V(0, 0, 1), (float)(45 * RAD));

	//Rear Gear Left4
	static UINT GGrp26[1] = { 18 };

	static MGROUP_TRANSLATE gear26(0, GGrp26, 1, _V(0.3, 0, 0));

	//Rear Gear Right2
	static UINT GGrp27[1] = { 13 };

	static MGROUP_TRANSLATE gear27(0, GGrp27, 1, _V(-0.3, 0, 0));




	anim_gear = CreateAnimation(0);

	AddAnimationComponent(anim_gear, 0, 0.5, &gear1);
	AddAnimationComponent(anim_gear, 0, 0.4, &gear2);
	AddAnimationComponent(anim_gear, 0, 0.4, &gear3);
	AddAnimationComponent(anim_gear, 0.1, 0.5, &gear4);

	AddAnimationComponent(anim_gear, 0.5, 1, &gear5);



	AddAnimationComponent(anim_gear, 0.1, 0.5, &gear12);
	AddAnimationComponent(anim_gear, 0.1, 0.5, &gear13);
	AddAnimationComponent(anim_gear, 0.5, 1, &gear14);
	AddAnimationComponent(anim_gear, 0.5, 1, &gear15);

	AddAnimationComponent(anim_gear, 0.4, 0.5, &gear22);
	AddAnimationComponent(anim_gear, 0.4, 0.5, &gear23);

	AddAnimationComponent(anim_gear, 0.2, 0.4, &gear24);
	AddAnimationComponent(anim_gear, 0.2, 0.4, &gear25);
	AddAnimationComponent(anim_gear, 0, 0.2, &gear26);
	AddAnimationComponent(anim_gear, 0, 0.2, &gear27);


	//Rudder
	static UINT RGrp1[2] = { 182,185 };

	static MGROUP_ROTATE rudderl(0, RGrp1, 2, _V(0, 0.8505662, -5.413699), _V(0, 1, 0), (float)(-20 * RAD));

	//Rudder Pedals left
	static UINT RGrp3[2] = { 170,171 };

	static MGROUP_ROTATE rudderlp(1, RGrp3, 2, _V(0, -0.9629048, 3.608708), _V(1, 0, 0), (float)(16 * RAD));




	//Rudder Padals right
	static UINT RGrp4[2] = { 172,173 };

	static MGROUP_ROTATE rudderrp(1, RGrp4, 2, _V(0, -0.9629048, 3.608708), _V(1, 0, 0), (float)(-16 * RAD));



	anim_rudder = CreateAnimation(0.5);

	AddAnimationComponent(anim_rudder, 0, 1, &rudderl);

	AddAnimationComponent(anim_rudder, 0, 1, &rudderlp);
	AddAnimationComponent(anim_rudder, 0, 1, &rudderrp);


	//Elevator
	static UINT EGrp1[2] = { 191,206 };

	static MGROUP_ROTATE elevator(0, EGrp1, 2, _V(0, -0.8380946, -5.047291), _V(1, 0, 0), (float)(25 * RAD));


	anim_elevator = CreateAnimation(0.5);

	AddAnimationComponent(anim_elevator, 0, 1, &elevator);




	//Left Aleron
	static UINT LAGrp1[2] = { 199,200 };

	static MGROUP_ROTATE laileron(0, LAGrp1, 2, _V(-2.49599, -0.8865513, -2.35824), _V(1, 0, -0.1), (float)(-33 * RAD));


	anim_laileron = CreateAnimation(0.5);

	AddAnimationComponent(anim_laileron, 0, 1, &laileron);


	//Right Aleron
	static UINT RAGrp1[2] = { 203,204 };

	static MGROUP_ROTATE raileron(0, RAGrp1, 2, _V(2.49599, -0.8865513, -2.35824), _V(1, 0, 0.1), (float)(33 * RAD));


	anim_raileron = CreateAnimation(0.5);

	AddAnimationComponent(anim_raileron, 0, 1, &raileron);


	//Speedbrake
	static UINT SBGrp1[6] = { 19,20,21,22,23,24 };

	static MGROUP_ROTATE speedbrake1(0, SBGrp1, 6, _V(0, -1.020337, -0.635115), _V(1, 0, 0), (float)(-45 * RAD));

	//Brake Lever
	static UINT SBGrp2[3] = { 2,3,4 };

	static MGROUP_ROTATE speedbrake2(1, SBGrp2, 3, _V(0, -1.058364, 2.747205), _V(1, 0, 0), (float)(-45 * RAD));




	anim_speedbrake = CreateAnimation(0);

	AddAnimationComponent(anim_speedbrake, 0, 1, &speedbrake1);
	AddAnimationComponent(anim_speedbrake, 0, 1, &speedbrake2);





	//Flaps animation

	//Left Flap
	static UINT FLGrp1[2] = { 197,198 };

	static MGROUP_ROTATE flapsl(0, FLGrp1, 2, _V(-1.34617, -0.88278, -2.294914), _V(1, 0, 0), (float)(-33 * RAD));


	//Right Flap
	static UINT FRGrp1[2] = { 201,202 };

	static MGROUP_ROTATE flapsr(0, FRGrp1, 2, _V(1.34617, -0.88278, -2.294914), _V(1, 0, 0), (float)(-33 * RAD));

	//Flap Dial
	static UINT FLGrp3[1] = { 46 };

	static MGROUP_ROTATE flaps3(1, FLGrp3, 1, _V(-0.9318672, -0.750552, 0), _V(0, 0, 1), (float)(145 * RAD));

	//Flap Lever
	static UINT FLGrp4[2] = { 48,49 };

	static MGROUP_ROTATE flaps4(1, FLGrp4, 2, _V(0, -1.058364, 2.747205), _V(1, 0, 0), (float)(-95 * RAD));


	anim_flaps = CreateAnimation(0.5);

	AddAnimationComponent(anim_flaps, 0, 1, &flapsl);
	AddAnimationComponent(anim_flaps, 0, 1, &flapsr);
	AddAnimationComponent(anim_flaps, 0, 1, &flaps3);
	AddAnimationComponent(anim_flaps, 0, 1, &flaps4);


	// Gear Lever Animation

	//Gear Lever in Cockpit
	static UINT Glev0[2] = { 57,58 };

	static MGROUP_ROTATE gearlev(1, Glev0, 2, _V(0, -0.1082646, 2.791955), _V(1, 0, 0), (float)(45 * RAD));

	static UINT Glev2[3] = { 59,60,61 };

	static MGROUP_TRANSLATE gearlev2(1, Glev2, 3, _V(0, 0, -0.02));

	anim_gearlever = CreateAnimation(0);

	AddAnimationComponent(anim_gearlever, 0, 1, &gearlev);
	AddAnimationComponent(anim_gearlever, 0, 1, &gearlev2);


	// Throttle Animation

	static UINT Pow0[2] = { 188,189 };

	static MGROUP_TRANSLATE powg(1, Pow0, 2, _V(0, 0, 0.38));

	anim_power = CreateAnimation(0);

	AddAnimationComponent(anim_power, 0, 1, &powg);


	// Altimeter Animations

	// Dial
	static UINT wWGrp0[1] = { 6 };

	static MGROUP_ROTATE wwheel0(1, wWGrp0, 1, _V(-0.9318853, 0.5609121, 0), _V(0, 0, 1), (float)(-360 * RAD));


	anim_wheel0 = CreateAnimation(0);

	AddAnimationComponent(anim_wheel0, 0, 1, &wwheel0);

	// 1st dial from Left
	static UINT wWGrp1[10] = { 110,111,112,113,114,115,116,117,118,119 };

	static MGROUP_ROTATE wwheel1(1, wWGrp1, 10, _V(0, 0.6143472, 2.75), _V(1, 0, 0), (float)(360 * RAD));


	anim_wheel5 = CreateAnimation(0);

	AddAnimationComponent(anim_wheel5, 0, 1, &wwheel1);

	// 2nd dial from Left
	static UINT wWGrp2[10] = { 120,121,122,123,124,125,126,127,128,129 };

	static MGROUP_ROTATE wwheel2(1, wWGrp2, 10, _V(0, 0.6143472, 2.75), _V(1, 0, 0), (float)(360 * RAD));


	anim_wheel4 = CreateAnimation(0);

	AddAnimationComponent(anim_wheel4, 0, 1, &wwheel2);

	// 3rd dial from Left
	static UINT wWGrp3[10] = { 80,81,82,83,84,85,86,87,88,89 };

	static MGROUP_ROTATE wwheel3(1, wWGrp3, 10, _V(0, 0.6143472, 2.74), _V(1, 0, 0), (float)(360 * RAD));


	anim_wheel3 = CreateAnimation(0);

	AddAnimationComponent(anim_wheel3, 0, 1, &wwheel3);

	// 4th dial from Left
	static UINT wWGrp4[10] = { 90,91,92,93,94,95,96,97,98,99 };

	static MGROUP_ROTATE wwheel4(1, wWGrp4, 10, _V(0, 0.6143472, 2.74), _V(1, 0, 0), (float)(360 * RAD));


	anim_wheel2 = CreateAnimation(0);

	AddAnimationComponent(anim_wheel2, 0, 1, &wwheel4);

	// 5th dial from Left
	static UINT wWGrp5[10] = { 100,101,102,103,104,105,106,107,108,109 };

	static MGROUP_ROTATE wwheel5(1, wWGrp5, 10, _V(0, 0.6143472, 2.74), _V(1, 0, 0), (float)(360 * RAD));


	anim_wheel1 = CreateAnimation(0);

	AddAnimationComponent(anim_wheel1, 0, 1, &wwheel5);



	// AOA Animations

	// Dial1
	static UINT dGrp0[1] = { 1 };

	static MGROUP_ROTATE dial1(1, dGrp0, 1, _V(-5.916421E-05, -1.890585, 0), _V(0, 0, 1), (float)(311 * RAD));


	anim_dial1 = CreateAnimation(0);

	AddAnimationComponent(anim_dial1, 0, 1, &dial1);


	// Direction Dial Animations

   // Dial2
	static UINT d2Grp0[1] = { 134 };

	static MGROUP_ROTATE dial2(1, d2Grp0, 1, _V(2.603407E-03, -1.143012, 0), _V(0, 0, 1), (float)(360 * RAD));

	//Compass
	static UINT d2Grp2[4] = { 63,64,65,66 };

	static MGROUP_ROTATE dial22(1, d2Grp2, 4, _V(1.159894, 2.018126, 0.7345342), _V(0, 1, 0), (float)(-360 * RAD));


	anim_dial2 = CreateAnimation(0);

	AddAnimationComponent(anim_dial2, 0, 1, &dial2);
	AddAnimationComponent(anim_dial2, 0, 1, &dial22);


	// Speed Dial Animations

	// Dial3
	static UINT d3Grp0[2] = { 185,186 };

	static MGROUP_ROTATE dial3(1, d3Grp0, 2, _V(-0.9266605, -0.159134, 0), _V(0, 0, 1), (float)(360 * RAD));


	anim_dial3 = CreateAnimation(0);

	AddAnimationComponent(anim_dial3, 0, 1, &dial3);


	// Bank Guage Animations

	static UINT BAGrp0[2] = { 7,191 };

	static MGROUP_ROTATE bank(1, BAGrp0, 2, _V(1.010653, -0.3959423, 0), _V(0, 0, 1), (float)(-360 * RAD));



	anim_bank = CreateAnimation(0.5);

	AddAnimationComponent(anim_bank, 0, 1, &bank);




	// Clock

	// Second Hand
	static UINT dGrp7[1] = { 178 };

	static MGROUP_ROTATE dial7(1, dGrp7, 1, _V(1.563198, -0.4318777, 0), _V(0, 0, 1), (float)(-360 * RAD));


	anim_clock_s = CreateAnimation(0);

	AddAnimationComponent(anim_clock_s, 0, 1, &dial7);

	// Minute Hand
	static UINT dGrp8[1] = { 169 };

	static MGROUP_ROTATE dial8(1, dGrp8, 1, _V(1.563198, -0.4318777, 0), _V(0, 0, 1), (float)(-360 * RAD));


	anim_clock_m = CreateAnimation(0);

	AddAnimationComponent(anim_clock_m, 0, 1, &dial8);

	// Hour Hand
	static UINT dGrp9[1] = { 62 };

	static MGROUP_ROTATE dial9(1, dGrp9, 1, _V(1.563198, -0.4318777, 0), _V(0, 0, 1), (float)(-360 * RAD));


	anim_clock_h = CreateAnimation(0);

	AddAnimationComponent(anim_clock_h, 0, 1, &dial9);


	// Fuel Guage Animation

	static UINT Ful0[1] = { 51 };

	static MGROUP_ROTATE fulg(1, Ful0, 1, _V(1.028024, -1.003252, 0), _V(0, 0, 1), (float)(-360 * RAD));


	anim_fuel = CreateAnimation(0);

	AddAnimationComponent(anim_fuel, 0, 1, &fulg);
}

void T38::ToggleLight()
{
	lightsOn = !lightsOn;

	if (lightsOn) PlayVesselWave(OSid, OSCLICK, NOLOOP, 255);
	else PlayVesselWave(OSid, OSCLICKOFF, NOLOOP, 255);

	beacon1.active = lightsOn;
	beacon2.active = lightsOn;

	//SetAnimation(anim_power, 0.0);

	//SetMeshVisibilityMode(InsertMesh(lightsOn ? MeshCockpit2 : MeshCockpit, 1, &COCKPIT_OFFSET), MESHVIS_VC);
}

bool PIDsetTargets(void* id, char* str, void* data)
{
	return ((T38*)data)->PIDSetTargets(str);
}

bool T38::PIDSetTargets(char* str)
{
	char* commaPos;
	commaPos = strchr(str, ',');
	double previousAlt = PIDtargetAlt;
	PIDtargetAlt = atof(str) * 1e3;
	// Orbiter crashes if trying to GetSurfaceElevation at sim start.
	if (oapiGetSimTime() != 0.0 && previousAlt > GetSurfaceElevation() + 200.0 && PIDtargetAlt > GetSurfaceElevation() + 200.0) PIDintegralAlt = 0.0, PIDpreviousAltError = 0.0; // more than 20 % change -> reset. If not, keep old, to not get sudden change potentially close to the ground.

	if (commaPos != NULL)
	{
		str = str + int(commaPos - str + 1);
		PIDtargetBank = atof(str) * RAD;
		PIDintegralBank = 0.0, PIDpreviousErrorBank = 0.0;
		
		commaPos = strchr(str, ',');
		if (commaPos != NULL)
		{
			str = str + int(commaPos - str + 1);
			PIDtargetSpeed = atof(str);
			PIDintegralSpeed = 0.0, PIDpreviousErrorBank = 0.0;
		}
	}

	return true;
}

bool PIDsetCoeffA(void* id, char* str, void* data)
{
	return ((T38*)data)->PIDSetCoeff(str, 0);
}
bool PIDsetCoeffB(void* id, char* str, void* data)
{
	return ((T38*)data)->PIDSetCoeff(str, 1);
}
bool PIDsetCoeffC(void* id, char* str, void* data)
{
	return ((T38*)data)->PIDSetCoeff(str, 2);
}

bool T38::PIDSetCoeff(char* str, int co)
{
	double val = atof(str);
	
	if (co == 0) PIDA = val;
	else if (co == 1) PIDB = val;
	else PIDC = val;

	return true;
}

double T38::GetDescRate(double vel)
{
	vel = abs(vel);
	if (10.0 < vel) return 1.0;
	else if (2.0 < vel) return 0.5;
	else return 0.1;
}

void T38::vlift(VESSEL* v, double aoa, double M, double Re, void* context, double* cl, double* cm, double* cd)
{
	static const double step = RAD * 15.0;
	static const double istep = 1.0 / step;
	static const int nabsc = 25;
	static const double CL[nabsc] = { 0.1, 0.17, 0.2, 0.2, 0.17, 0.1, 0, -0.11, -0.24, -0.38,  -0.5,  -0.5, -0.02, 0.6355,    0.63,   0.46, 0.28, 0.13, 0.0, -0.16, -0.26, -0.29, -0.24, -0.1, 0.1 };
	static const double CM[nabsc] = { 0,    0,   0,   0,    0,   0, 0,     0,    0,0.002,0.004, 0.0025,0.0012,      0,-0.0012,-0.0007,    0,    0,   0,     0,     0,     0,     0,    0,   0 };
	// lift and moment coefficients from -180 to 180 in 15 degree steps.
	// This uses a documented lift slope of 0.0437/deg, everything else is rather ad-hoc

	aoa += PI;
	int idx = max(0, min(23, (int)(aoa * istep)));
	double d = aoa * istep - idx;
	*cl = CL[idx] + (CL[idx + 1] - CL[idx]) * d;
	*cm = CM[idx] + (CM[idx + 1] - CM[idx]) * d;
	
	*cd = 0.06 + oapiGetInducedDrag(*cl, 2.266, 0.6);
	if (useRealisticValues) *cd = (0.0128 + 0.2 * *cl * *cl) / 2.0;
}

void T38::hlift(VESSEL* v, double beta, double M, double Re, void* context, double* cl, double* cm, double* cd)
{
	static const double step = RAD * 22.5;
	static const double istep = 1.0 / step;
	static const int nabsc = 17;
	static const double CL[nabsc] = { 0, 0.2, 0.3, 0.2, 0, -0.2, -0.3, -0.2, 0, 0.2, 0.3, 0.2, 0, -0.2, -0.3, -0.2, 0 };

	beta += PI;
	int idx = max(0, min(15, (int)(beta * istep)));
	double d = beta * istep - idx;
	*cl = CL[idx] + (CL[idx + 1] - CL[idx]) * d;
	*cm = 0.0;
	*cd = 0.02 + oapiGetInducedDrag(*cl, 1.5, 0.6);
	if (useRealisticValues) *cd = (0.0128 + 0.2 * *cl * *cl) / 2.0;
}



// --------------------------------------------------------------
// Vessel initialisation
// --------------------------------------------------------------
DLLCLBK VESSEL* ovcInit(OBJHANDLE hvessel, int flightmodel)
{
	return new T38(hvessel, flightmodel);
}

// --------------------------------------------------------------
// Vessel cleanup
// --------------------------------------------------------------
DLLCLBK void ovcExit(VESSEL* vessel)
{
	if (vessel) delete (T38*)vessel;
}
