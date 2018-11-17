/*** VisSim Automatic C Code Generator Version 5.0e ***/
/*  Output for C:\afc_working\jucas\engine_deck\engine_f\test_2.vsm at Thu Apr 22 12:11:39 2004 */


#include "math.h"
#include "../include/cgen.h"
#include "../include/cgendll.h"

extern void __stdcall engine_deck (	float GPLA,			//! input, Power Lever Angle (deg)
									float GMACH,		//! input, Flight Mach No.
									float GALT,			//! input, Altitude, ft (geopotential).
									float GPAM,			//! input, Ambient pressure, psia.
									float GTAM,			//! input, Ambient temperature, °R.
									float GHPX,			//! input, Horsepower Extraction
									float GWBLCD,		//! input, Compressor Discharge Bleed, lbm/sec
									float GAJ,			//! input, Physical Nozzle Area (ft2)
									float XIBNGC,		//! input, Assume that the ECS bleed flow is a constant <input> lbm/sec corrected.
									float *UNPF,		//! output, FN, Net Thrust (lbf)
									float *W2A,			//! output, WAT,	Total Fan Inlet Physical Airflow (lbm/sec)
									float *W2AR,		//! output, WAT2,	Total Fan Inlet Corrected Airflow (lbm/sec)
									float *WFTP,		//! output, WFT,	Total Engine Fuel Flow, (lbm/hr)
									float *T6R,			//! output, TT6H,	Core Side Mixing Plane Temperature (°F)
									float *T16,			//! output, TT6C,	Duct Side Mixing Plane Temperature (°F)

									float *P16,			//! output, PT6C,	Duct Side Mixing Plane Pressure (psia)

									float *T6AR,		//! output, TT6M,	Mixing Plane Mixed Mass Average Temperature (°F)
									float *ZPAMB,		//! output, Pamb,	Ambient Pressure (psia)
									float *ZTAMBR,		//! output, Tamb,	Ambient Temperature (degR)
									float *TTBLI,		//! output, TTBLI,	Interstage Bleed Total Temperature (degF)
									float *TTB9,		//! output, TTBLC,	HPC Discharge Bleed Temperature (degF)
									float *PTBLI,		//! output, PTBLI,	Interstage Bleed Total Pressure (psia)
									float *PTB9,		//! output, PTBLC,	HPC Discharge Bleed Pressure (psia)
									float *T7,			//! output, TT9,	Nozzle Exit Total Temperature (degF)
									float *TS9,			//! output, TS9,	Nozzle Exit Static Temperature (degF)
									float *P7,			//! output, PT9,	Nozzle Exit Total Pressure (psia)
									float *PS8,			//! output, PS9,	Nozzle Exit Static Pressure (psia)
									float *RHO9,		//! output, RHO9,	Gas flow Density at Station 9 (lbm/ft^3)
									float *XM9,			//! output, XM9,	Nozzle Discharge Mach Number (n/a)
									float *V9,			//! output, V9,		Nozzle Discharge Velocity (ft/s)
									float *T125,		//! output, TT2.5C,	Fan Exit Duct Side Temperature (degF)
									float *P125 );		//! output, PT2.5C,	Fan Exit Duct Side Total Pressure (psia)


extern CGDOUBLE Zed;

extern CGDOUBLE GPLA;
extern CGDOUBLE GMACH;
extern CGDOUBLE GALT;
extern CGDOUBLE GPAM;
extern CGDOUBLE GTAM;
extern CGDOUBLE GHPX;
extern CGDOUBLE GWBLCD;
extern CGDOUBLE GAJ;
extern CGDOUBLE XIBNGC;
static CGDOUBLE UNPF;
static CGDOUBLE W2A;
static CGDOUBLE W2AR;
static CGDOUBLE WFTP;
static CGDOUBLE T6R;
static CGDOUBLE T16;

static CGDOUBLE P16;

static CGDOUBLE T6AR;
static CGDOUBLE ZPAMB;
static CGDOUBLE ZTAMBR;
static CGDOUBLE TTBLI;
static CGDOUBLE TTB9;
static CGDOUBLE PTBLI;
static CGDOUBLE PTB9;
static CGDOUBLE T7;
static CGDOUBLE TS9;
static CGDOUBLE P7;
static CGDOUBLE PS8;
static CGDOUBLE P125;
static CGDOUBLE T125;
static CGDOUBLE V9;
static CGDOUBLE XM9;
static CGDOUBLE RHO9;
DLL_SIG_DECL(9,23)
DLL_FUNC_DEF(cgMain_test_2)
DLL_EVENT_DEF(cgMain_test_2,"Test_2",test_2)
static void cgMain_test_2Deriv();
static ARG_DESCR outArgInfo28[]={
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
};
static ARG_DESCR inArgInfo28[]={
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
  { T_DOUBLE,0,0,0},
};
static SIM_STATE tSim={0, 0.05, 10,0,0.05,0,0,0,0,0,0,0,0
,outArgInfo28, inArgInfo28,9,23,0,0,0, cgMain_test_2Deriv,0,0,0,0,0,0,0};
SIM_STATE *sim=&tSim;
static unsigned long _lastTickCount;

void cgMain_test_2Deriv()
{
  CGDOUBLE _GPLA_28;
  CGDOUBLE _UNPF_28;
  CGDOUBLE _GMACH_28;
  CGDOUBLE _W2A_28;
  CGDOUBLE _W2AR_28;
  CGDOUBLE _GALT_28;
  CGDOUBLE _WFTP_28;
  CGDOUBLE _GPAM_28;
  CGDOUBLE _T6R_28;
  CGDOUBLE _GTAM_28;
  CGDOUBLE _T16_28;

  CGDOUBLE _P16_28;
  
  CGDOUBLE _GHPX_28;
  CGDOUBLE _T6AR_28;
  CGDOUBLE _GWBLCD_28;
  CGDOUBLE _ZPAMB_28;
  CGDOUBLE _GAJ_28;
  CGDOUBLE _ZTAMBR_28;
  CGDOUBLE _XIBNGC_28;
  CGDOUBLE _TTBLI_28;
  CGDOUBLE _TTB9_28;
  CGDOUBLE _PTBLI_28;
  CGDOUBLE _PTB9_28;
  CGDOUBLE _T7_28;
  CGDOUBLE _TS9_28;
  CGDOUBLE _P7_28;
  CGDOUBLE _PS8_28;
  CGDOUBLE _RHO9_28;
  CGDOUBLE _XM9_28;
  CGDOUBLE _V9_28;
  CGDOUBLE _T125_28;
  CGDOUBLE _P125_28;



		float GPLA;			//! input, Power Lever Angle (deg)
		float GMACH;		//! input, Flight Mach No.
		float GALT;			//! input, Altitude, ft (geopotential).
		float GPAM;			//! input, Ambient pressure, psia.
		float GTAM;			//! input, Ambient temperature, °R.
		float GHPX;			//! input, Horsepower Extraction
		float GWBLCD;		//! input, Compressor Discharge Bleed, lbm/sec
		float GAJ;			//! input, Physical Nozzle Area (ft2)
		float XIBNGC;		//! input, Assume that the ECS bleed flow is a constant <input> lbm/sec corrected.
		float UNPF;			//! output, FN,		Net Thrust (lbf)
		float W2A;			//! output, WAT,	Total Fan Inlet Physical Airflow (lbm/sec)
		float W2AR;			//! output, WAT2,	Total Fan Inlet Corrected Airflow (lbm/sec)
		float WFTP;			//! output, WFT,	Total Engine Fuel Flow, (lbm/hr)
		float T6R;			//! output, TT6H,	Core Side Mixing Plane Temperature (°F)
		float T16;			//! output, TT6C,	Duct Side Mixing Plane Temperature (°F)

		float P16;			//! output, PT6C,	Duct Side Mixing Plane Pressure (psia)
		
		float T6AR;			//! output, TT6M,	Mixing Plane Mixed Mass Average Temperature (°F)
		float ZPAMB;		//! output, Pamb,	Ambient Pressure (psia)
		float ZTAMBR;		//! output, Tamb,	Ambient Temperature (degR)
		float TTBLI;		//! output, TTBLI,	Interstage Bleed Total Temperature (degF)
		float TTB9;			//! output, TTBLC,	HPC Discharge Bleed Temperature (degF)
		float PTBLI;		//! output, PTBLI,	Interstage Bleed Total Pressure (psia)
		float PTB9;			//! output, PTBLC,	HPC Discharge Bleed Pressure (psia)
		float T7;			//! output, TT9,	Nozzle Exit Total Temperature (degF)
		float TS9;			//! output, TS9,	Nozzle Exit Static Temperature (degF)
		float P7;			//! output, PT9,	Nozzle Exit Total Pressure (psia)
		float PS8;			//! output, PS9,	Nozzle Exit Static Pressure (psia)
		float RHO9;			//! output, RHO9,	Gas flow Density at Station 9 (lbm/ft^3)
		float XM9;			//! output, XM9,	Nozzle Discharge Mach Number (n/a)
		float V9;			//! output, V9,		Nozzle Discharge Velocity (ft/s)
		float T125;			//! output, TT2.5C,	Fan Exit Duct Side Temperature (degF)
		float P125;			//! output, PT2.5C,	Fan Exit Duct Side Total Pressure (psia)




  /* Test_2 */
  _GPLA_28 =  sim->inSig[0] ;
  _GMACH_28 =  sim->inSig[1] ;
  _GALT_28 =  sim->inSig[2] ;
  _GPAM_28 =  sim->inSig[3] ;
  _GTAM_28 =  sim->inSig[4] ;
  _GHPX_28 =  sim->inSig[5] ;
  _GWBLCD_28 =  sim->inSig[6] ;
  _GAJ_28 =  sim->inSig[7] ;
  _XIBNGC_28 =  sim->inSig[8] ;


  GPLA		=  _GPLA_28 ;
  GMACH		=  _GMACH_28 ;
  GALT		=  _GALT_28 ;
  GPAM		=  _GPAM_28 ;
  GTAM		=  _GTAM_28 ;
  GHPX		=  _GHPX_28 ;
  GWBLCD	=  _GWBLCD_28 ;
  GAJ		=  _GAJ_28 ;
  XIBNGC	=  _XIBNGC_28 ;


//  _UNPF_28 = ( _GPLA_28 *1.00000000000000);
//  _W2A_28 = ( _GMACH_28 *2.00000000000000);
//  _WFTP_28 = ( _GALT_28 *3.00000000000000);
//  _T6R_28 = ( _GPAM_28 *4.00000000000000);
//  _T16_28 = ( _GTAM_28 *5.00000000000000);
//  _T6AR_28 = ( _GHPX_28 *6.00000000000000);
//  _ZPAMB_28 = ( _GWBLCD_28 *7.00000000000000);
//  _ZTAMBR_28 = ( _GAJ_28 *8.00000000000000);
//  _TTBLI_28 = ( _XIBNGC_28 *9.00000000000000);
//  _TTB9_28 = ( _XIBNGC_28 *10.0000000000000);
//  _PTBLI_28 = ( _XIBNGC_28 *11.0000000000000);
//  _PTB9_28 = ( _XIBNGC_28 *12.0000000000000);
//  _T7_28 = ( _XIBNGC_28 *13.0000000000000);
//  _TS9_28 = ( _XIBNGC_28 *14.0000000000000);
//  _P7_28 = ( _XIBNGC_28 *15.0000000000000);
//  _PS8_28 = ( _XIBNGC_28 *16.0000000000000);
//  _RHO9_28 = ( _XIBNGC_28 *17.0000000000000);
//  _XM9_28 = ( _XIBNGC_28 *18.0000000000000);
//  _V9_28 = ( _XIBNGC_28 *19.0000000000000);
//  _T125_28 = ( _XIBNGC_28 *20.0000000000000);
//  _P125_28 = ( _XIBNGC_28 *21.0000000000000);



//		GPLA		= 20.0;		//! input, Power Lever Angle (deg)
//		GMACH		= 0.0;		//! input, Flight Mach No.
//		GALT		= 0.0;		//! input, Altitude, ft (geopotential).
//		GPAM		= 14.70;	//! input, Ambient pressure, psia.
//		GTAM		= 580.0;	//! input, Ambient temperature, °R.
//		GHPX		= 75.0;		//! input, Horsepower Extraction
//		GWBLCD		= 0.0;		//! input, Compressor Discharge Bleed, lbm/sec
//		GAJ			= 2.9;		//! input, Physical Nozzle Area (ft2)
//		XIBNGC		= 8.0;		//! input, Assume that the ECS bleed flow is a constant <input> lbm/sec corrected.



		engine_deck (	GPLA,			//! input, Power Lever Angle (deg)
						GMACH,			//! input, Flight Mach No.
						GALT,			//! input, Altitude, ft (geopotential).
						GPAM,			//! input, Ambient pressure, psia.
						GTAM,			//! input, Ambient temperature, °R.
						GHPX,			//! input, Horsepower Extraction
						GWBLCD,			//! input, Compressor Discharge Bleed, lbm/sec
						GAJ,			//! input, Physical Nozzle Area (ft2)
						XIBNGC,			//! input, Assume that the ECS bleed flow is a constant <input> lbm/sec corrected.
						&UNPF,			//! output, FN, Net Thrust (lbf)
						&W2A,			//! output, WAT,	Total Fan Inlet Physical Airflow (lbm/sec)
						&W2AR,			//! output, WAT2,	Total Fan Inlet Corrected Airflow (lbm/sec)
						&WFTP,			//! output, WFT,	Total Engine Fuel Flow, (lbm/hr)
						&T6R,			//! output, TT6H,	Core Side Mixing Plane Temperature (°F)
						&T16,			//! output, TT6C,	Duct Side Mixing Plane Temperature (°F)

						&P16,			//! output, PT6C,	Duct Side Mixing Plane Pressure (psia)

						&T6AR,			//! output, TT6M,	Mixing Plane Mixed Mass Average Temperature (°F)
						&ZPAMB,			//! output, Pamb,	Ambient Pressure (psia)
						&ZTAMBR,		//! output, Tamb,	Ambient Temperature (degR)
						&TTBLI,			//! output, TTBLI,	Interstage Bleed Total Temperature (degF)
						&TTB9,			//! output, TTBLC,	HPC Discharge Bleed Temperature (degF)
						&PTBLI,			//! output, PTBLI,	Interstage Bleed Total Pressure (psia)
						&PTB9,			//! output, PTBLC,	HPC Discharge Bleed Pressure (psia)
						&T7,			//! output, TT9,	Nozzle Exit Total Temperature (degF)
						&TS9,			//! output, TS9,	Nozzle Exit Static Temperature (degF)
						&P7,			//! output, PT9,	Nozzle Exit Total Pressure (psia)
						&PS8,			//! output, PS9,	Nozzle Exit Static Pressure (psia)
						&RHO9,			//! output, RHO9,	Gas flow Density at Station 9 (lbm/ft^3)
						&XM9,			//! output, XM9,	Nozzle Discharge Mach Number (n/a)
						&V9,			//! output, V9,		Nozzle Discharge Velocity (ft/s)
						&T125,			//! output, TT2.5C,	Fan Exit Duct Side Temperature (degF)
						&P125 );		//! output, PT2.5C,	Fan Exit Duct Side Total Pressure (psia)





  _UNPF_28		=  UNPF ;
  _W2A_28		=  W2A ;
  _W2AR_28		=  W2AR ;
  _WFTP_28		=  WFTP ;
  _T6R_28		=  T6R ;
  _T16_28		=  T16 ;

  _P16_28		=  P16 ;
  
  _T6AR_28		=  T6AR ;
  _ZPAMB_28		=  ZPAMB ;
  _ZTAMBR_28	=  ZTAMBR ;
  _TTBLI_28		=  TTBLI ;
  _TTB9_28		=  TTB9 ;
  _PTBLI_28		=  PTBLI ;
  _PTB9_28		=  PTB9 ;
  _T7_28		=  T7 ;
  _TS9_28		=  TS9 ;
  _P7_28		=  P7 ;
  _PS8_28		=  PS8 ;
  _RHO9_28		=  RHO9 ;
  _XM9_28		=  XM9 ;
  _V9_28		=  V9 ;
  _T125_28		=  T125 ;
  _P125_28		=  P125 ;




  sim->outSig[0] =  _UNPF_28 ;
  sim->outSig[1] =  _W2A_28 ;
  sim->outSig[2] =  _W2AR_28 ;
  sim->outSig[3] =  _WFTP_28 ;
  sim->outSig[4] =  _T6R_28 ;
  sim->outSig[5] =  _T16_28 ;

  sim->outSig[6] =  _P16_28 ;
  
  sim->outSig[7] =  _T6AR_28 ;
  sim->outSig[8] =  _ZPAMB_28 ;
  sim->outSig[9] =  _ZTAMBR_28 ;
  sim->outSig[10] =  _TTBLI_28 ;
  sim->outSig[11] =  _TTB9_28 ;
  sim->outSig[12] =  _PTBLI_28 ;
  sim->outSig[13] =  _PTB9_28 ;
  sim->outSig[14] =  _T7_28 ;
  sim->outSig[15] =  _TS9_28 ;
  sim->outSig[16] =  _P7_28 ;
  sim->outSig[17] =  _PS8_28 ;
  sim->outSig[18] =  _RHO9_28 ;
  sim->outSig[19] =  _XM9_28 ;
  sim->outSig[20] =  _V9_28 ;
  sim->outSig[21] =  _T125_28 ;
  sim->outSig[22] =  _P125_28 ;

  _lastTickCount = sim->tickCount;

}

EXPORT32 void PASCAL EXPORT cgMain_test_2SS(SIM_STATE *pSim, long *runCount)
{
  cgDllSimInit( pSim, &tSim, *runCount);
}

EXPORT32 void PASCAL EXPORT cgMain_test_2SE(SIM_STATE *pSim, long *runCount)
{
}
