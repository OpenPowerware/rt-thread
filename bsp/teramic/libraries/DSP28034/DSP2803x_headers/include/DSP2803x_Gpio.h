

#ifndef DSP2803x_GPIO_H
#define DSP2803x_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------
// GPIO A control register bit definitions */
struct GPACTRL_BITS {        // bits   description
   Uint32 QUALPRD0:8;        // 7:0    Qual period
   Uint32 QUALPRD1:8;        // 15:8   Qual period
   Uint32 QUALPRD2:8;        // 23:16  Qual period
   Uint32 QUALPRD3:8;        // 31:24  Qual period
};

union GPACTRL_REG {
   Uint32              all;
   struct GPACTRL_BITS bit;
};

//----------------------------------------------------
// GPIO B control register bit definitions */
struct GPBCTRL_BITS {        // bits   description
   Uint32 QUALPRD0:8;        // 7:0    Qual period
   Uint32 QUALPRD1:8;		 // 15:8    Qual period
   Uint32 rsvd2:16;          // 31:16  reserved
};

union GPBCTRL_REG {
   Uint32              all;
   struct GPBCTRL_BITS bit;
};

//----------------------------------------------------
// GPIO Qual/MUX select register bit definitions */
struct GPA1_BITS {            // bits   description
   Uint32 GPIO0:2;            // 1:0    GPIO0
   Uint32 GPIO1:2;            // 3:2    GPIO1
   Uint32 GPIO2:2;            // 5:4    GPIO2
   Uint32 GPIO3:2;            // 7:6    GPIO3
   Uint32 GPIO4:2;            // 9:8    GPIO4
   Uint32 GPIO5:2;            // 11:10  GPIO5
   Uint32 GPIO6:2;            // 13:12  GPIO6
   Uint32 GPIO7:2;            // 15:14  GPIO7
   Uint32 GPIO8:2;			  // 17:16  GPIO8
   Uint32 GPIO9:2;		      // 19:18  GPIO9
   Uint32 GPIO10:2;			  // 21:20  GPIO10
   Uint32 GPIO11:2;			  // 23:22  GPIO11
   Uint32 GPIO12:2;			  // 25:24  GPIO12
   Uint32 GPIO13:2;			  // 27:26  GPIO13
   Uint32 GPIO14:2;			  // 29:28  GPIO14
   Uint32 GPIO15:2;			  // 31:30  GPIO15
};

struct GPA2_BITS {            // bits   description
   Uint32 GPIO16:2;           // 1:0      GPIO16
   Uint32 GPIO17:2;           // 3:2      GPIO17
   Uint32 GPIO18:2;           // 5:4      GPIO18
   Uint32 GPIO19:2;           // 7:6      GPIO19
   Uint32 GPIO20:2;			  // 9:8      GPIO20
   Uint32 GPIO21:2;			  // 11:10    GPIO21
   Uint32 GPIO22:2;			  // 13:12    GPIO22
   Uint32 GPIO23:2;			  // 15:14    GPIO23
   Uint32 GPIO24:2;			  // 17:16    GPIO24
   Uint32 GPIO25:2;			  // 19:18    GPIO25
   Uint32 GPIO26:2;			  // 21:20    GPIO26
   Uint32 GPIO27:2;			  // 23:22    GPIO27
   Uint32 GPIO28:2;			  // 25:24    GPIO28
   Uint32 GPIO29:2;			  // 27:26    GPIO29
   Uint32 GPIO30:2;			  // 29:28    GPIO30
   Uint32 GPIO31:2;			  // 31:30    GPIO31
};

struct GPB1_BITS {            // bits   description
   Uint32 GPIO32:2;           // 1:0    GPIO32
   Uint32 GPIO33:2;           // 3:2    GPIO33
   Uint32 GPIO34:2;           // 5:4    GPIO34
   Uint32 GPIO35:2;           // 7:6    GPIO35
   Uint32 GPIO36:2;           // 9:8    GPIO36
   Uint32 GPIO37:2;           // 11:10  GPIO37
   Uint32 GPIO38:2;           // 13:12  GPIO38
   Uint32 GPIO39:2;           // 15:14  GPIO39
   Uint32 GPIO40:2;           // 17:16  GPIO40
   Uint32 GPIO41:2;           // 19:18  GPIO41
   Uint32 GPIO42:2;           // 21:20  GPIO42
   Uint32 GPIO43:2;           // 23:22  GPIO43
   Uint32 GPIO44:2;           // 25:24  GPIO44
   Uint32 rsvd1:6;            // 31:26  reserved
};

struct AIO_BITS {            // bits   description
   Uint32 rsvd1:2;           // 1:0    reserved
   Uint32 rsvd2:2;           // 3:2    reserved
   Uint32 AIO2:2;            // 5:4    AIO2
   Uint32 rsvd3:2;           // 7:6    reserved
   Uint32 AIO4:2;            // 9:8    AIO4
   Uint32 rsvd4:2;           // 11:10  reserved
   Uint32 AIO6:2;            // 13:12  AIO6
   Uint32 rsvd5:2;           // 15:14  reserved
   Uint32 rsvd6:2;           // 17:16  reserved
   Uint32 rsvd7:2;           // 19:18  reserved
   Uint32 AIO10:2;           // 21:20  AIO10
   Uint32 rsvd8:2;           // 23:22  reserved
   Uint32 AIO12:2;           // 25:24  AIO12
   Uint32 rsvd9:2;           // 27:26  reserved
   Uint32 AIO14:2;           // 29:28  AIO14
   Uint32 rsvd10:2;          // 31:30  reserved
};

union GPA1_REG {
   Uint32              all;
   struct GPA1_BITS    bit;
};


struct GPAQ1_BITS  
{
	Uint32 GPIO0:2; 		   // 1:0	 GPIO0
	Uint32 GPIO1:2; 		   // 3:2	 GPIO1
	Uint32 GPIO2:2; 		   // 5:4	 GPIO2
	Uint32 GPIO3:2; 		   // 7:6	 GPIO3
	Uint32 GPIO4:2; 		   // 9:8	 GPIO4
	Uint32 GPIO5:2; 		   // 11:10  GPIO5
	Uint32 GPIO6:2; 		   // 13:12  GPIO6
	Uint32 GPIO7:2; 		   // 15:14  GPIO7
	Uint32 GPIO8:2; 		   // 17:16  GPIO8
	Uint32 GPIO9:2; 		   // 19:18  GPIO9	
	Uint32 GPIO10:2;   		   // 21:20  GPIO10
	Uint32 GPIO11:2; 		   // 23:22  GPIO11
	Uint32 GPIO12:2; 		   // 25:24  GPIO12
	Uint32 GPIO13:2; 		   // 27:26  GPIO13
	Uint32 GPIO14:2; 		   // 29:28  GPIO14
	Uint32 GPIO15:2; 		   // 31:30  GPIO15
};


union GPAQ1_REG{
	Uint32				all;
	struct GPAQ1_BITS    bit;

};


struct GPAQ2_BITS  
{
	Uint32 GPIO16:2; 		   // 1:0	 GPIO16
	Uint32 GPIO17:2; 		   // 3:2	 GPIO17
	Uint32 GPIO18:2; 		   // 5:4	 GPIO18
	Uint32 GPIO19:2; 		   // 7:6	 GPIO19
	Uint32 GPIO20:2; 		   // 9:8	 GPIO20
	Uint32 GPIO21:2; 		   // 11:10  GPIO21
	Uint32 GPIO22:2; 		   // 13:12  GPIO22
	Uint32 GPIO23:2; 		   // 15:14  GPIO23
	Uint32 GPIO24:2; 		   // 17:16  GPIO24
	Uint32 GPIO25:2; 		   // 19:18  GPIO25
	Uint32 GPIO26:2; 		   // 21:20  GPIO26
	Uint32 GPIO27:2; 		   // 23:22  GPIO27
	Uint32 GPIO28:2; 		   // 25:24  GPIO28
	Uint32 GPIO29:2; 		   // 27:26  GPIO29
	Uint32 GPIO30:2; 		   // 29:28  GPIO30
	Uint32 GPIO31:2;		   // 31:30  GPIO31
};


union GPAQ2_REG{
	Uint32				all;
	struct GPAQ2_BITS    bit;

};


union GPA2_REG {
   Uint32              all;
   struct GPA2_BITS    bit;
};

union GPB1_REG {
   Uint32              all;
   struct GPB1_BITS    bit;
};

struct GPBQ1_BITS
{
   Uint32 GPIO32:2;           // 1:0    GPIO32
   Uint32 GPIO33:2;           // 3:2    GPIO33
   Uint32 GPIO34:2;           // 5:4    GPIO34
   Uint32 GPIO35:2;           // 7:6    GPIO35
   Uint32 GPIO36:2;           // 9:8    GPIO36
   Uint32 GPIO37:2;           // 11:10  GPIO37
   Uint32 GPIO38:2;           // 13:12  GPIO38
   Uint32 GPIO39:2;           // 15:14  GPIO39
   Uint32 GPIO40:2;           // 17:16  GPIO40
   Uint32 GPIO41:2;           // 19:18  GPIO41
   Uint32 GPIO42:2;           // 21:20  GPIO42
   Uint32 GPIO43:2;           // 23:22  GPIO43
   Uint32 GPIO44:2;           // 25:24  GPIO44
   Uint32 rsvd:6;             // 31:26  reserved
};

union  GPBQ1_REG
{
	Uint32 all;
	struct GPBQ1_BITS    bit;
};

union AIO_REG {
   Uint32              all;
   struct AIO_BITS     bit;
};

//----------------------------------------------------
// GPIO DIR/TOGGLE/SET/CLEAR register bit definitions */
struct GPADAT_BITS {          // bits   description
   Uint32 GPIO0:1;            // 0      GPIO0
   Uint32 GPIO1:1;            // 1      GPIO1
   Uint32 GPIO2:1;            // 2      GPIO2
   Uint32 GPIO3:1;            // 3      GPIO3
   Uint32 GPIO4:1;            // 4      GPIO4
   Uint32 GPIO5:1;            // 5      GPIO5
   Uint32 GPIO6:1;            // 6      GPIO6
   Uint32 GPIO7:1;            // 7      GPIO7
   Uint32 GPIO8:1;            // 8      GPIO8
   Uint32 GPIO9:1;            // 9      GPIO9
   Uint32 GPIO10:1;           // 10     GPIO10
   Uint32 GPIO11:1;           // 11     GPIO11
   Uint32 GPIO12:1;           // 12     GPIO12
   Uint32 GPIO13:1;           // 13     GPIO13
   Uint32 GPIO14:1;           // 14     GPIO14
   Uint32 GPIO15:1;           // 15     GPIO15
   Uint32 GPIO16:1;           // 16     GPIO16
   Uint32 GPIO17:1;           // 17     GPIO17
   Uint32 GPIO18:1;           // 18     GPIO18
   Uint32 GPIO19:1;           // 19     GPIO19
   Uint32 GPIO20:1;           // 20     GPIO20
   Uint32 GPIO21:1;           // 21     GPIO21
   Uint32 GPIO22:1;           // 22     GPIO22
   Uint32 GPIO23:1;           // 23     GPIO23
   Uint32 GPIO24:1;           // 24     GPIO24
   Uint32 GPIO25:1;           // 25     GPIO25
   Uint32 GPIO26:1;           // 26     GPIO26
   Uint32 GPIO27:1;           // 27     GPIO27
   Uint32 GPIO28:1;           // 28     GPIO28
   Uint32 GPIO29:1;           // 29     GPIO29
   Uint32 GPIO30:1;           // 30     GPIO30
   Uint32 GPIO31:1;           // 31     GPIO31
};

struct GPBDAT_BITS {          // bits   description
   Uint32 GPIO32:1;           // 0      GPIO32
   Uint32 GPIO33:1;           // 1      GPIO33
   Uint32 GPIO34:1;           // 2      GPIO34
   Uint32 GPIO35:1;           // 3      GPIO35
   Uint32 GPIO36:1;           // 4      GPIO36
   Uint32 GPIO37:1;           // 5      GPIO37
   Uint32 GPIO38:1;           // 6      GPIO38
   Uint32 GPIO39:1;           // 7      GPIO39
   Uint32 GPIO40:1;           // 8      GPIO40
   Uint32 GPIO41:1;           // 9      GPIO41
   Uint32 GPIO42:1;           // 10     GPIO42
   Uint32 GPIO43:1;           // 11     GPIO43
   Uint32 GPIO44:1;           // 12     GPIO44
   Uint32 rsvd1:3;            // 15:13  reserved
   Uint32 rsvd2:16;           // 31:16  reserved
};

struct AIODAT_BITS {         // bits   description
   Uint32 rsvd1:1;           // 0      reserved
   Uint32 rsvd2:1;           // 1      reserved
   Uint32 AIO2:1;            // 2      AIO2
   Uint32 rsvd3:1;           // 3      reserved
   Uint32 AIO4:1;            // 4      AIO4
   Uint32 rsvd4:1;           // 5      reserved
   Uint32 AIO6:1;            // 6      AIO6
   Uint32 rsvd5:1;           // 7      reserved
   Uint32 rsvd6:1;           // 8      reserved
   Uint32 rsvd7:1;           // 9      reserved
   Uint32 AIO10:1;           // 10     AIO10
   Uint32 rsvd8:1;           // 11     reserved
   Uint32 AIO12:1;           // 12     AIO12
   Uint32 rsvd9:1;           // 13     reserved
   Uint32 AIO14:1;           // 14     AIO14
   Uint32 rsvd10:1;          // 15     reserved
   Uint32 rsvd11:16;         // 31:16  reserved
};

union GPADAT_REG {
   Uint32              all;
   struct GPADAT_BITS  bit;
};

union GPBDAT_REG {
   Uint32              all;
   struct GPBDAT_BITS  bit;
};

union AIODAT_REG {
   Uint32			   all;
   struct AIODAT_BITS  bit;
};

//----------------------------------------------------------------
// GPIO XINT1/XINT2/XNMI/XCLKIN select register bit definitions */
struct GPIOXINT_BITS {        // bits   description
    Uint32 GPIOSEL:5;         // 4:0    Select GPIO interrupt input source
    Uint32 rsvd1:11;          // 15:5   reserved
    Uint32 rsvd2:16;          // 31:16  reserved
};

union GPIOXINT_REG {
   Uint32                all;
   struct GPIOXINT_BITS  bit;
};




/****************************************************************




*******************************************************************/


struct GPIO_CTRL_REGS {

	union  GPA1_REG	   GPAMUX1;   // GPIO A Mux 1 Register (GPIO0 to 15)   00
	union  GPA2_REG	   GPAMUX2;   // GPIO A Mux 2 Register (GPIO16 to 31)	04 
	union  GPB1_REG	   GPBMUX1;   // GPIO B Mux 1 Register (GPIO32 to 38)	08
	union  AIO_REG	   AIOMUX1;   // Analog IO Mux 1 Register (AIO0 to 15)	0c
	
	union  GPACTRL_REG  GPACTRL;   // GPIO A Control Register (GPIO0 to 31)		10
	union  GPBCTRL_REG  GPBCTRL;   // GPIO B Control Register (GPIO32 to 38)    14	   		

	union  GPAQ1_REG	   GPAQSEL1;  // GPIO A Qualifier Select 1 Register (GPIO0 to 15)		18
	union  GPAQ2_REG	   GPAQSEL2;  // GPIO A Qualifier Select 2 Register (GPIO16 to 31)		1C

	union  GPBQ1_REG	  	GPBQSEL1;  // GPIO B Qualifier Select 1 Register (GPIO32 to 38)		20	

	union  GPADAT_REG   GPADIR;	  // GPIO A Direction Register (GPIO0 to 31)			24		   
	union  GPBDAT_REG   GPBDIR;	  // GPIO B Direction Register (GPIO32 to 38)	   28
	union  AIODAT_REG   AIODIR;	  // Analog IO Direction Register (AIO0 to 15)			2c	   		   
	union  GPADAT_REG   GPAPUD;	  // GPIO A Pull Up Disable Register (GPIO0 to 31)		30
	union  GPBDAT_REG   GPBPUD;	  // GPIO B Pull Up Disable Register (GPIO32 to 38)		34

};

struct GPIO_DATA_REGS {

	union  GPADAT_REG       GPADAT;       // GPIO Data Register (GPIO0 to 31)			38
	union  GPBDAT_REG       GPBDAT;       // GPIO Data Register (GPIO32 to 38)			3C
	union  AIODAT_REG       AIODAT;       // Analog IO Data Register (AIO0-15)			40
	union  GPADAT_REG		GPASET; 	  // GPIO Data Set Register (GPIO0 to 31)		44
	union  GPADAT_REG       GPACLEAR;     // GPIO Data Clear Register (GPIO0 to 31)		48
	union  GPADAT_REG       GPATOGGLE;    // GPIO Data Toggle Register (GPIO0 to 31)	4C
	union  GPBDAT_REG       GPBSET;       // GPIO Data Set Register (GPIO32 to 38)		50
	union  GPBDAT_REG       GPBCLEAR;     // GPIO Data Clear Register (GPIO32 to 38)		54
   	union  GPBDAT_REG       GPBTOGGLE;    // GPIO Data Toggle Register (GPIO32 to 38)		58
   	union  AIODAT_REG       AIOSET;       // Analog IO Data Set Register (AIO0-15)			5C
   	union  AIODAT_REG       AIOCLEAR;     // Analog IO Data Clear Register (AIO0-15)		60
   	union  AIODAT_REG       AIOTOGGLE;    // Analog IO Data Toggle Register (AIO0-15)		64
	
};

struct GPIO_INT_REGS {
	 union  GPIOXINT_REG	   GPIOXINT1SEL; // XINT1 GPIO Input Selection			68
	 union  GPIOXINT_REG	   GPIOXINT2SEL; // XINT2 GPIO Input Selection			6C
	 union  GPIOXINT_REG	   GPIOXINT3SEL; // XINT3 GPIO Input Selection			70
	 union  GPADAT_REG	  	   GPIOLPMSEL;	 // Low power modes GPIO input select	74
};

	
#ifdef __cplusplus
}

#endif /* extern "C" */



extern volatile struct GPIO_CTRL_REGS *const P_GpioCtrlRegs;
extern volatile struct GPIO_CTRL_REGS GpioCtrlRegs;


extern volatile struct GPIO_DATA_REGS *const P_GpioDataRegs;
extern volatile struct GPIO_DATA_REGS GpioDataRegs;


extern volatile struct GPIO_INT_REGS *const P_GpioIntRegs;
extern volatile struct GPIO_INT_REGS GpioIntRegs;





#endif  

//===========================================================================
// End of file.
//===========================================================================
