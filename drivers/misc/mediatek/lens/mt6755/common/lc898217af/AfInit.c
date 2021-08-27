//********************************************************************************
//
//		<< LC898217 Evaluation Soft >>
//		Program Name	: AfIni.c
//		Design			: Y.Yamada
//		History			: First edition						2015.07.03 Y.Tashita
//********************************************************************************
//**************************
//	Include Header File		
//**************************
#include	"Af.h"

//********************************************************************************
// Function Name 	: CheckCver
// Retun Value		: SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Check CVER register Function
// History			: First edition 						2015.07.04 Y.Tashita
//********************************************************************************
unsigned char CheckCver( void )
{
	unsigned char UcLsiVer;

	RegReadA( CVER, &UcLsiVer );

	return( UcLsiVer == 0x71 ) ? SUCCESS : FAILURE ;
}

//********************************************************************************
// Function Name 	: WakeUpCheck
// Retun Value		: unsigned char UcWakeUpSts
// Argment Value	: Void
// Explanation		: WakeUp Check for autodownload
// History		: First edition 		     2015.07.11 John.Jeong
//********************************************************************************
unsigned char	WakeUpCheck(unsigned char UcRescailMode)
{
	unsigned char	UcStatus, UcReadDat ;
	unsigned short	i ;
	
	//I2C communication check if necessary	
	UcStatus	= CheckCver() ;

	if( UcStatus != FAILURE ) {
		for( i = 0 ; i < 3000 ; i++  ) {
			RegReadA( FUNCRSLT1, &UcReadDat ) ;
			if(UcRescailMode == RESCAILING){
				if( UcReadDat == 0x01 )             //Rescailing Mode
					break ;
			}
			else
			{
				if( !UcReadDat )
					break ;			
			}
		}

		if( i == 3000 ) {
			UcStatus	= DOWNLOAD_ERROR ;
		}
	}
		
	RegWriteA( PINC, 0x02 ) ;
	RegWriteA( TESTC, 0x20 ) ;

	
	return( UcStatus ) ;
}

//********************************************************************************
// Function Name 	: WitTim
// Retun Value		: NON
// Argment Value	: Wait Time(ms)
// Explanation		: Timer Wait Function
// History			: First edition 						2015.07.04 Y.Tashita
//********************************************************************************
void	WitTim( unsigned short	UsWitTim )
{
	unsigned long	UlLopIdx, UlWitCyc ;

	UlWitCyc	= ( unsigned long )( ( float )UsWitTim / NOP_TIME / ( float )12 ) ;

	for( UlLopIdx = 0 ; UlLopIdx < UlWitCyc ; UlLopIdx++ )
	{
		;
	}
}


//********************************************************************************
// Function Name 	: RescailEn
// Retun Value		: Void
// Argment Value	: Void
// Explanation		: Rescail Enable
// History		: First edition 		    2015.07.11 John.Jeong
//******************************************************************************** //FUNCRUN2
void RescailEn( void )
{
	RegWriteA(FUNCRUN2	, 0x02 );  //0xA1 02h, After Calibration Data Writing, Rescailing Run 
	WitTim(1) ;//WAIT	1ms
}

//********************************************************************************
// Function Name 	: Init
// Retun Value		: unsigned char
// Argment Value	: NON
// Explanation		: Calibration and Initialize
// History		: First edition 		2015.07.13 John Jeong
//********************************************************************************
unsigned char lc898217_Init( void )
{
	unsigned char	UcSts ;
	//unsigned char	UcDownloadMode = AUTO_DOWNLOAD;	
	//unsigned char	UcMode = RESCAILING ;
	
	//After Power On
	msleep(8); //Wait 8ms, Make sure VDD Stabile,
		   //This is one example of time. Please adjust wait time to fit your system.
		   //And change WitTim() to your system delay function.
	 
	UcSts = WakeUpCheck(RESCAILING);
		if (UcSts != SUCCESS)
			return FAILURE;	

	RescailEn(); //A1h 02h setting
		
	return SUCCESS;	
}

//********************************************************************************
// Function Name 	: Stmv217
// Retun Value		: Stepmove Parameter
// Argment Value	: Stepmove Parameter, Target Position
// Explanation		: Stpmove Function
// History		: First edition 		    2015.07.08 John.Jeong
//********************************************************************************
#define TIME_OUT 1000
#define TIMEOUT_ERROR 10

unsigned char Stmv217( unsigned char DH, unsigned char DL )
{
	unsigned char	UcConvCheck;
	unsigned char	UcSetTime;
	unsigned char	UcCount;	
	
	//AF Operation
	RegWriteA(TARGETH	, DH );  //0x84, DH 0x0X
	RegWriteA(TARGETL	, DL );  //0x85, DL 0xXX
	WitTim(5) ;//WAIT	5ms
	UcCount = 0;
	do
	{
		RegReadA(SRVSTATE1, &UcConvCheck); 	//0xB0
		UcConvCheck = UcConvCheck & 0x80;  	//bit7 == 0 or not
		RegReadA(TGTCNVTIM, &UcSetTime); 	//0xB2 Settle Time check Settle Time Calculate =
							// ( B2h Read Value x 0.171mS ) - 2.731mS Condition:
							//EEPROM 55h(THD1) = 0x20
							//82h:bit2-0( LOCCNVCNT ) = 100b
		WitTim(2) ;//WAIT 2ms
		UcCount++;
		if(UcCount == TIME_OUT)	{
			return TIMEOUT_ERROR;
		}
	}while(UcConvCheck); 	
	
	return SUCCESS;
}

//********************************************************************************
// Function Name 	: SetPosition
// Retun Value		: NON
// Argment Value	: unsigned char UcPosition
// Explanation		: AF Operation function Range 1023 to 0
// History		: First edition 		2015.07.13 John Jeong
//********************************************************************************
unsigned char SetPosition(unsigned short UsPosition)
{
    unsigned char UcPosH;
    unsigned char UcPosL;
    unsigned char UcAFSts;
    //unsigned char UcMaxPosition = 1023;  

    UsPosition = 1023 - UsPosition; //Only SI1330C, SI1331C

    UcPosH = (unsigned char)UsPosition >> 8;
    UcPosL = (unsigned char)UsPosition & 0x00FF;    

    UcAFSts = Stmv217( UcPosH, UcPosL );   
    if (UcAFSts != SUCCESS)
	return FAILURE;	

    return  UcAFSts; 
}
