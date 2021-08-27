/* 
 *
 * yixue.ge add for oppo project
 *
 *
 */
#ifndef _OPPO_PROJECT_H_
#define _OPPO_PROJECT_H_

enum{
	HW_VERSION__UNKNOWN,
	HW_VERSION__10, 	//OPPO_15111_DVT
	HW_VERSION__11, 	//OPPO_15111
	HW_VERSION__12, 	//OPPO_15112
	HW_VERSION__13, 	//OPPO_15113
	HW_VERSION__14, 	//OPPO_15114
	HW_VERSION__15, 	
	HW_VERSION__16, 	
};


enum{
	RF_VERSION__UNKNOWN,
	RF_VERSION__11,		
	RF_VERSION__12,		
	RF_VERSION__13,
	RF_VERSION__21,		
	RF_VERSION__22,		
	RF_VERSION__23,
	RF_VERSION__31,		
	RF_VERSION__32,		
	RF_VERSION__33,
};


#define GET_PCB_VERSION() (get_PCB_Version())
#define GET_PCB_VERSION_STRING() (get_PCB_Version_String())

#define GET_MODEM_VERSION() (get_Modem_Version())
#define GET_OPERATOR_VERSION() (get_Operator_Version())



enum OPPO_PROJECT {
	OPPO_UNKOWN = 0,
	OPPO_15005 = 15005,
	OPPO_15009 = 15009,
	OPPO_15011 = 15011,
	OPPO_15015 = 15015,
	OPPO_15021 = 15021,
	OPPO_15057 = 15057,
	OPPO_15018 = 15018,
	OPPO_15022 = 15022,
	OPPO_15025 = 15025,
	OPPO_15029 = 15029,
	OPPO_15037 = 15037,
	OPPO_15035 = 15035,
	OPPO_15043 = 15043,
	OPPO_15111_DVT = 15110,
	OPPO_15111 = 15111,
	OPPO_15112 = 15112,
	OPPO_15113 = 15113,
	OPPO_15114 = 15114,
	OPPO_15127 = 15127,
	OPPO_15129 = 15129,
	OPPO_15130 = 15130,
	OPPO_15131 = 15131,
	OPPO_15133 = 15133,
	OPPO_15134 = 15134,
	OPPO_16031 = 16031,
	OPPO_16021 = 16021,
	OPPO_16023 = 16023,
	OPPO_16024 = 16024,
	OPPO_16034 = 16034,
};

enum OPPO_OPERATOR {
	OPERATOR_UNKOWN 			= 0,
	OPERATOR_OPEN_MARKET 		= 1,
	OPERATOR_CHINA_MOBILE 		= 2,
	OPERATOR_CHINA_UNICOM 		= 3,
	OPERATOR_CHINA_TELECOM 		= 4,
	OPERATOR_FOREIGN 			= 5,
	OPERATOR_FOREIGN_WCDMA	  	= 6,	
	OPERATOR_FOREIGN_RESERVED   = 7,
	OPERATOR_ALL_TELECOM_CARRIER = 8,
	OPERATOR_ALL_MOBILE_CARRIER = 9,
	OPERATOR_ALL_PROJECT_16031 = 10,  
	OPERATOR_ALL_PROJECT_16034 = 11,
};

enum{
	SMALLBOARD_VERSION__0,
	SMALLBOARD_VERSION__1,
	SMALLBOARD_VERSION__2,
	SMALLBOARD_VERSION__3,
	SMALLBOARD_VERSION__4,
	SMALLBOARD_VERSION__5,
	SMALLBOARD_VERSION__6,
	SMALLBOARD_VERSION__UNKNOWN = 100,
};

typedef enum OPPO_PROJECT OPPO_PROJECT;

typedef struct
{
	unsigned int	nProject;
	unsigned int	nModem;
	unsigned int	nOperator;
	unsigned int	nPCBVersion;
} ProjectInfoCDTType;

//unsigned int init_project_version(void);
unsigned int get_project(void);
unsigned int is_project(OPPO_PROJECT project );
unsigned int get_PCB_Version(void);
unsigned int get_Modem_Version(void);
unsigned int get_Operator_Version(void);

#endif