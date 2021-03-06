#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H

#include <mach/mt_typedefs.h>

/* ============================================================*/
/* define*/
/* ============================================================*/
#define BAT_NTC_10 1
#define BAT_NTC_47 0

#if (BAT_NTC_10 == 1)
#ifndef VENDOR_EDIT /* OPPO 2016-01-30 sjc Modify for charging */
#define RBAT_PULL_UP_R	16900
#else
#define RBAT_PULL_UP_R	16000
#endif //VENDOR_EDIT
#endif

#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R	61900
#endif

#define RBAT_PULL_UP_VOLT	1800



/* ============================================================*/
/* ENUM*/
/* ============================================================*/

/* ============================================================*/
/* structure*/
/* ============================================================*/

/* ============================================================*/
/* typedef*/
/* ============================================================*/
typedef struct _BATTERY_PROFILE_STRUCT
{
	kal_int32 percentage;
	kal_int32 voltage;
} BATTERY_PROFILE_STRUCT, *BATTERY_PROFILE_STRUCT_P;

typedef struct _R_PROFILE_STRUCT
{
	kal_int32 resistance; /* Ohm*/
	kal_int32 voltage;
} R_PROFILE_STRUCT, *R_PROFILE_STRUCT_P;

typedef enum
{
	T1_0C,
	T2_25C,
	T3_50C
} PROFILE_TEMPERATURE;

/* ============================================================*/
/* External Variables*/
/* ============================================================*/

/* ============================================================*/
/* External function*/
/* ============================================================*/

/* ============================================================*/
/* <DOD, Battery_Voltage> Table*/
/* ============================================================*/
#if (BAT_NTC_10 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
		{-20, 68237},
		{-15, 53650},
		{-10, 42506},
		{ -5, 33892},
		{  0, 27219},
		{  5, 22021},
		{ 10, 17926},
		{ 15, 14674},
		{ 20, 12081},
		{ 25, 10000},
		{ 30, 8315},
		{ 35, 6948},
		{ 40, 5834},
		{ 45, 4917},
		{ 50, 4161},
		{ 55, 3535},
		{ 60, 3014}
	};
#endif

#if (BAT_NTC_47 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
		{-20, 483954},
		{-15, 360850},
		{-10, 271697},
		{ -5, 206463},
		{  0, 158214},
		{  5, 122259},
		{ 10, 95227},
		{ 15, 74730},
		{ 20, 59065},
		{ 25, 47000},
		{ 30, 37643},
		{ 35, 30334},
		{ 40, 24591},
		{ 45, 20048},
		{ 50, 16433},
		{ 55, 13539},
		{ 60, 11210}
	};
#endif

/* T0 -10C*/
BATTERY_PROFILE_STRUCT battery_profile_t0[] = {
#ifdef VENDOR_EDIT /* OPPO 2016-01-30 sjc Modify for charging */
	 {0, 4323},
	 {2, 4299},
	 {4, 4277},
	 {6, 4257},
	 {8, 4236},
	 {11, 4216},
	 {13, 4193},
	 {15, 4168},
	 {17, 4140},
	 {19, 4103},
	 {21, 4070},
	 {23, 4041},
	 {25, 4009},
	 {27, 3980},
	 {29, 3958},
	 {32, 3940},
	 {34, 3926},
	 {36, 3916},
	 {38, 3905},
	 {40, 3894},
	 {42, 3883},
	 {44, 3873},
	 {46, 3864},
	 {48, 3854},
	 {51, 3847},
	 {53, 3838},
	 {55, 3830},
	 {57, 3823},
	 {59, 3816},
	 {61, 3809},
	 {63, 3802},
	 {65, 3796},
	 {67, 3790},
	 {69, 3783},
	 {72, 3777},
	 {74, 3769},
	 {76, 3762},
	 {78, 3754},
	 {80, 3744},
	 {82, 3734},
	 {84, 3723},
	 {86, 3714},
	 {88, 3708},
	 {90, 3703},
	 {93, 3695},
	 {95, 3672},
	 {97, 3611},
	 {99, 3498},
	 {100, 3388},
	 {100, 3319},
	 {100, 3275},
	 {100, 3246},
	 {100, 3225},
	 {100, 3209},
	 {100, 3201},
	 {100, 3194},
	 {100, 3187},
	 {100, 3181},
	 {100, 3180},
	 {100, 3175},
	 {100, 3172},
	 {100, 3166},
	 {100, 3162},
	 {100, 3159},
	 {100, 3155},
	 {100, 3153},
	 {100, 3151},
	 {100, 3148},
	 {100, 3148},
	//{339, 3168},
	//{339, 3162},
	//{339, 3157},
	//{339, 3151},
	//{339, 3143},
	//{339, 3133},
	//{339, 3125},
	//{339, 3125},
	//{339, 3125},
	//{339, 3125},
	//{339, 3125},
	//{339, 3125},
	//{339, 3125},
#endif //VENDOR_EDIT
};      
        
/* T1 0C */
BATTERY_PROFILE_STRUCT battery_profile_t1[] = {
#ifdef VENDOR_EDIT /* OPPO 2016-01-30 sjc Modify for charging */
	 {0, 4340},
	 {2, 4316},
	 {4, 4285},
	 {5, 4224},
	 {7, 4196},
	 {9, 4176},
	 {11, 4157},
	 {12, 4139},
	 {14, 4122},
	 {16, 4106},
	 {18, 4091},
	 {20, 4082},
	 {21, 4067},
	 {23, 4038},
	 {25, 4008},
	 {27, 3988},
	 {28, 3974},
	 {30, 3963},
	 {32, 3956},
	 {34, 3947},
	 {35, 3933},
	 {37, 3916},
	 {39, 3900},
	 {41, 3884},
	 {43, 3871},
	 {44, 3860},
	 {46, 3850},
	 {48, 3841},
	 {50, 3833},
	 {51, 3826},
	 {53, 3820},
	 {55, 3813},
	 {57, 3807},
	 {59, 3802},
	 {60, 3797},
	 {62, 3793},
	 {64, 3788},
	 {66, 3785},
	 {67, 3782},
	 {69, 3781},
	 {71, 3778},
	 {73, 3775},
	 {74, 3771},
	 {76, 3766},
	 {78, 3760},
	 {80, 3754},
	 {82, 3746},
	 {83, 3737},
	 {85, 3728},
	 {87, 3715},
	 {89, 3704},
	 {90, 3700},
	 {92, 3695},
	 {94, 3690},
	 {96, 3672},
	 {98, 3603},
	 {99, 3483},
	 {100, 3314},
	 {100, 3210},
	 {100, 3141},
	 {100, 3107},
	 {100, 3090},
	 {100, 3073},
	 {100, 3067},
	 {100, 3056},
	 {100, 3044},
	 {100, 3035},
	 {100, 3030},
	 {100, 3030},
	//{235, 3047},
	//{235, 3041},
	//{235, 3032},
	//{235, 3029},
	//{235, 3018},
	//{235, 3008},
	//{235, 3002},
	//{235, 3003},
	//{235, 2995},
	//{235, 2985},
	//{235, 2983},
	//{235, 2979},
	//{235, 2967},
#endif //VENDOR_EDIT
};

/* T2 25C*/
BATTERY_PROFILE_STRUCT battery_profile_t2[] = {
#ifdef VENDOR_EDIT /* OPPO 2016-01-30 sjc Modify for charging */
	 {0, 4344},
	 {2, 4317},
	 {3, 4294},
	 {5, 4274},
	 {7, 4254},
	 {8, 4235},
	 {10, 4216},
	 {12, 4198},
	 {13, 4180},
	 {15, 4162},
	 {16, 4144},
	 {18, 4127},
	 {20, 4109},
	 {21, 4093},
	 {23, 4077},
	 {25, 4066},
	 {26, 4051},
	 {28, 4027},
	 {30, 4005},
	 {31, 3993},
	 {33, 3984},
	 {35, 3972},
	 {36, 3959},
	 {38, 3946},
	 {40, 3933},
	 {41, 3919},
	 {43, 3904},
	 {44, 3887},
	 {46, 3872},
	 {48, 3859},
	 {49, 3849},
	 {51, 3840},
	 {53, 3833},
	 {54, 3825},
	 {56, 3818},
	 {58, 3812},
	 {59, 3806},
	 {61, 3801},
	 {63, 3796},
	 {64, 3791},
	 {66, 3788},
	 {68, 3783},
	 {69, 3779},
	 {71, 3775},
	 {72, 3771},
	 {74, 3765},
	 {76, 3759},
	 {77, 3754},
	 {79, 3748},
	 {81, 3742},
	 {82, 3736},
	 {84, 3727},
	 {86, 3717},
	 {87, 3708},
	 {89, 3694},
	 {91, 3691},
	 {92, 3688},
	 {94, 3685},
	 {95, 3677},
	 {97, 3624},
	 {99, 3530},
	 {100, 3361},
	 {100, 3139},
	 {100, 3091},
	 {100, 3058},
	 {100, 3032},
	 {100, 3019},
	 {100, 3008},
	 {100, 3008},
	//{111, 2998},
	//{111, 2987},
	//{111, 2978},
	//{111, 2971},
	//{111, 2963},
	//{111, 2958},
	//{111, 2956},
	//{111, 2953},
	//{111, 2948},
	//{111, 2940},
	//{111, 2934},
	//{111, 2929},
	//{111, 2926},
#endif //VENDOR_EDIT
};

/* T3 50C*/
BATTERY_PROFILE_STRUCT battery_profile_t3[] = {
#ifdef VENDOR_EDIT /* OPPO 2016-01-30 sjc Modify for charging */
	 {0, 4347},
	 {2, 4321},
	 {3, 4297},
	 {5, 4275},
	 {7, 4255},
	 {8, 4235},
	 {10, 4217},
	 {12, 4198},
	 {13, 4179},
	 {15, 4161},
	 {16, 4143},
	 {18, 4125},
	 {20, 4108},
	 {21, 4092},
	 {23, 4075},
	 {25, 4059},
	 {26, 4043},
	 {28, 4028},
	 {30, 4013},
	 {31, 3998},
	 {33, 3985},
	 {35, 3972},
	 {36, 3959},
	 {38, 3946},
	 {40, 3933},
	 {41, 3921},
	 {43, 3907},
	 {45, 3888},
	 {46, 3871},
	 {48, 3858},
	 {49, 3848},
	 {51, 3840},
	 {53, 3832},
	 {54, 3824},
	 {56, 3818},
	 {58, 3810},
	 {59, 3804},
	 {61, 3798},
	 {63, 3793},
	 {64, 3789},
	 {66, 3785},
	 {68, 3780},
	 {69, 3775},
	 {71, 3771},
	 {73, 3760},
	 {74, 3749},
	 {76, 3743},
	 {77, 3735},
	 {79, 3729},
	 {81, 3724},
	 {82, 3718},
	 {84, 3710},
	 {86, 3699},
	 {87, 3690},
	 {89, 3677},
	 {91, 3675},
	 {92, 3673},
	 {94, 3670},
	 {96, 3658},
	 {97, 3600},
	 {99, 3506},
	 {100, 3339},
	 {100, 3061},
	 {100, 2985},
	 {100, 2955},
	 {100, 2932},
	 {100, 2920},
	 {100, 2912},
	 {100, 2912},
	//{110, 2913},
	//{110, 2903},
	//{110, 2899},
	//{110, 2889},
	//{110, 2889},
	//{110, 2889},
	//{110, 2889},
	//{110, 2889},
	//{110, 2889},
	//{110, 2889},
	//{110, 2889},
	//{110, 2889},
	//{110, 2889},
#endif //VENDOR_EDIT
};

/* battery profile for actual temperature. The size should be the same as T1, T2 and T3*/
BATTERY_PROFILE_STRUCT battery_profile_temperature[] = {
#ifdef VENDOR_EDIT /* OPPO 2016-01-30 sjc Modify for charging */
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0}
#endif //VENDOR_EDIT
};

/* ============================================================*/
/* <Rbat, Battery_Voltage> Table*/
/* ============================================================*/
/* T0 -10C*/
R_PROFILE_STRUCT r_profile_t0[] = {
#ifdef VENDOR_EDIT /* OPPO 2016-01-30 sjc Modify for charging */
	 {580, 4323},
	 {580, 4299},
	 {595, 4277},
	 {613, 4257},
	 {620, 4236},
	 {630, 4216},
	 {635, 4193},
	 {653, 4168},
	 {693, 4140},
	 {783, 4103},
	 {918, 4070},
	 {968, 4041},
	 {975, 4009},
	 {970, 3980},
	 {973, 3958},
	 {973, 3940},
	 {975, 3926},
	 {978, 3916},
	 {978, 3905},
	 {980, 3894},
	 {985, 3883},
	 {985, 3873},
	 {1000, 3864},
	 {998, 3854},
	 {1010, 3847},
	 {1018, 3838},
	 {1023, 3830},
	 {1033, 3823},
	 {1048, 3816},
	 {1058, 3809},
	 {1065, 3802},
	 {1075, 3796},
	 {1088, 3790},
	 {1098, 3783},
	 {1113, 3777},
	 {1120, 3769},
	 {1138, 3762},
	 {1150, 3754},
	 {1168, 3744},
	 {1185, 3734},
	 {1200, 3723},
	 {1220, 3714},
	 {1248, 3708},
	 {1290, 3703},
	 {1338, 3695},
	 {1390, 3672},
	 {1443, 3611},
	 {1523, 3498},
	 {1470, 3388},
	 {1298, 3319},
	 {1190, 3275},
	 {1118, 3246},
	 {1078, 3225},
	 {1043, 3209},
	 {1018, 3201},
	 {993, 3194},
	 {975, 3187},
	 {975, 3181},
	 {953, 3180},
	 {963, 3175},
	 {945, 3172},
	 {960, 3166},
	 {943, 3162},
	 {935, 3159},
	 {950, 3155},
	 {940, 3153},
	 {943, 3151},
	 {945, 3148},
	 {978, 3148},
	//{960, 3168},
	//{940, 3162},
	//{895, 3157},
	//{890, 3151},
	//{863, 3143},
	//{925, 3133},
	//{893, 3125},
	//{893, 3125},
	//{893, 3125},
	//{893, 3125},
	//{893, 3125},
	//{893, 3125},
	//{893, 3125}
#endif //VENDOR_EDIT
};

/* T1 0C*/
R_PROFILE_STRUCT r_profile_t1[] = {
#ifdef VENDOR_EDIT /* OPPO 2016-01-30 sjc Modify for charging */
	 {258, 4340},
	 {258, 4316},
	 {253, 4285},
	 {290, 4224},
	 {390, 4196},
	 {395, 4176},
	 {400, 4157},
	 {405, 4139},
	 {410, 4122},
	 {418, 4106},
	 {420, 4091},
	 {438, 4082},
	 {450, 4067},
	 {440, 4038},
	 {433, 4008},
	 {438, 3988},
	 {440, 3974},
	 {443, 3963},
	 {453, 3956},
	 {455, 3947},
	 {450, 3933},
	 {440, 3916},
	 {433, 3900},
	 {425, 3884},
	 {423, 3871},
	 {425, 3860},
	 {428, 3850},
	 {430, 3841},
	 {433, 3833},
	 {440, 3826},
	 {443, 3820},
	 {450, 3813},
	 {458, 3807},
	 {465, 3802},
	 {473, 3797},
	 {480, 3793},
	 {485, 3788},
	 {493, 3785},
	 {500, 3782},
	 {513, 3781},
	 {520, 3778},
	 {528, 3775},
	 {533, 3771},
	 {543, 3766},
	 {548, 3760},
	 {563, 3754},
	 {573, 3746},
	 {585, 3737},
	 {605, 3728},
	 {618, 3715},
	 {640, 3704},
	 {675, 3700},
	 {718, 3695},
	 {780, 3690},
	 {865, 3672},
	 {945, 3603},
	 {1105, 3483},
	 {1290, 3314},
	 {1040, 3210},
	 {870, 3141},
	 {790, 3107},
	 {738, 3090},
	 {718, 3073},
	 {683, 3067},
	 {673, 3056},
	 {658, 3044},
	 {673, 3035},
	 {643, 3030},
	 {585, 3030},
	//{685, 3047},
	//{650, 3041},
	//{683, 3032},
	//{615, 3029},
	//{643, 3018},
	//{660, 3008},
	//{635, 3002},
	//{530, 3003},
	//{640, 2995},
	//{595, 2985},
	//{535, 2983},
	//{463, 2979},
	//{600, 2967}
#endif //VENDOR_EDIT
};

/* T2 25C*/
R_PROFILE_STRUCT r_profile_t2[] = {
#ifdef VENDOR_EDIT /* OPPO 2016-01-30 sjc Modify for charging */
	 {128, 4344},
	 {128, 4317},
	 {125, 4294},
	 {128, 4274},
	 {128, 4254},
	 {128, 4235},
	 {128, 4216},
	 {130, 4198},
	 {133, 4180},
	 {133, 4162},
	 {133, 4144},
	 {138, 4127},
	 {135, 4109},
	 {140, 4093},
	 {143, 4077},
	 {150, 4066},
	 {153, 4051},
	 {148, 4027},
	 {150, 4005},
	 {158, 3993},
	 {163, 3984},
	 {160, 3972},
	 {163, 3959},
	 {163, 3946},
	 {165, 3933},
	 {160, 3919},
	 {155, 3904},
	 {143, 3887},
	 {133, 3872},
	 {125, 3859},
	 {125, 3849},
	 {125, 3840},
	 {128, 3833},
	 {125, 3825},
	 {125, 3818},
	 {128, 3812},
	 {130, 3806},
	 {130, 3801},
	 {133, 3796},
	 {133, 3791},
	 {138, 3788},
	 {138, 3783},
	 {138, 3779},
	 {138, 3775},
	 {138, 3771},
	 {133, 3765},
	 {128, 3759},
	 {130, 3754},
	 {130, 3748},
	 {130, 3742},
	 {133, 3736},
	 {133, 3727},
	 {130, 3717},
	 {135, 3708},
	 {128, 3694},
	 {133, 3691},
	 {135, 3688},
	 {145, 3685},
	 {158, 3677},
	 {153, 3624},
	 {170, 3530},
	 {213, 3361},
	 {868, 3139},
	 {763, 3091},
	 {688, 3058},
	 {668, 3032},
	 {580, 3019},
	 {563, 3008},
	 {538, 3008},
	//{558, 2998},
	//{538, 2987},
	//{515, 2978},
	//{485, 2971},
	//{458, 2963},
	//{455, 2958},
	//{403, 2956},
	//{398, 2953},
	//{373, 2948},
	//{373, 2940},
	//{408, 2934},
	//{420, 2929},
	//{335, 2926}
#endif //VENDOR_EDIT
};

/* T3 50C*/
R_PROFILE_STRUCT r_profile_t3[] = {
#ifdef VENDOR_EDIT /* OPPO 2016-01-30 sjc Modify for charging */
	 {98, 4347},
	 {98, 4321},
	 {95, 4297},
	 {95, 4275},
	 {93, 4255},
	 {93, 4235},
	 {98, 4217},
	 {98, 4198},
	 {95, 4179},
	 {95, 4161},
	 {98, 4143},
	 {95, 4125},
	 {98, 4108},
	 {103, 4092},
	 {100, 4075},
	 {103, 4059},
	 {103, 4043},
	 {105, 4028},
	 {105, 4013},
	 {105, 3998},
	 {108, 3985},
	 {113, 3972},
	 {113, 3959},
	 {115, 3946},
	 {118, 3933},
	 {123, 3921},
	 {123, 3907},
	 {110, 3888},
	 {103, 3871},
	 {98, 3858},
	 {98, 3848},
	 {98, 3840},
	 {98, 3832},
	 {98, 3824},
	 {100, 3818},
	 {98, 3810},
	 {98, 3804},
	 {98, 3798},
	 {100, 3793},
	 {105, 3789},
	 {108, 3785},
	 {110, 3780},
	 {108, 3775},
	 {113, 3771},
	 {103, 3760},
	 {98, 3749},
	 {100, 3743},
	 {95, 3735},
	 {98, 3729},
	 {98, 3724},
	 {98, 3718},
	 {98, 3710},
	 {95, 3699},
	 {100, 3690},
	 {95, 3677},
	 {98, 3675},
	 {103, 3673},
	 {105, 3670},
	 {110, 3658},
	 {105, 3600},
	 {118, 3506},
	 {143, 3339},
	 {668, 3061},
	 {483, 2985},
	 {398, 2955},
	 {370, 2932},
	 {335, 2920},
	 {295, 2912},
	 {300, 2912},
	//{303, 2913},
	//{308, 2903},
	//{255, 2899},
	//{273, 2889},
	//{273, 2889},
	//{273, 2889},
	//{273, 2889},
	//{273, 2889},
	//{273, 2889},
	//{273, 2889},
	//{273, 2889},
	//{273, 2889},
	//{273, 2889}
#endif //VENDOR_EDIT
};

/* r-table profile for actual temperature. The size should be the same as T1, T2 and T3*/
R_PROFILE_STRUCT r_profile_temperature[] = {
#ifdef VENDOR_EDIT /* OPPO 2016-01-30 sjc Modify for charging */
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0},
	//{0, 0}
#endif //VENDOR_EDIT
};

/* ============================================================*/
/* function prototype*/
/* ============================================================*/
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUCT_P fgauge_get_profile(kal_uint32 temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUCT_P fgauge_get_profile_r_table(kal_uint32 temperature);

#endif

