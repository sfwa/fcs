/* Parse the parameter spec */
%{
#define YYERROR_VERBOSE
extern char *yytext;
%}
%token LBRACE RBRACE SECTION EQU VALUE STRING BOOT_MODE PARAM_INDEX OPTIONS 
%token MULTI_I2C_ID MY_I2C_ID CORE_FREQ_MHZ I2C_CLK_FREQ_KHZ 
%token EXE_FILE PCI_PARMS NEXT_DEV_ADDR NEXT_DEV_ADDR_EXT ADDRESS_DELAY SWPLL
%token DEV_ADDR_EXT DEV_ADDR LAYOUT ALIGN PAD LENGTH PAD_FILE_ID
%token SWPLL_PREDIV SWPLL_MULT SWPLL_POSTDIV SWPLL_FLAGS
%token ADDR_WIDTH N_PINS MODE C2T_DELAY BUS_FREQ_MHZ BUS_FREQ_KHZ CSEL
%%

promspec   : segment
		   | promspec segment
 		   ;

segment	   : bootParams
		   | layout
		   | pad
		   ;

bootParams : SECTION LBRACE assigns RBRACE
			 { section (); }
		   | PCI_PARMS EQU STRING
		     { setPciParams (yytext); }
		   ;

layout     : LAYOUT LBRACE assigns RBRACE
			 { setLayout ();  }
		   ;

pad		   : PAD LBRACE assigns RBRACE
		     { setPad ();  }
		   ;


assigns    : assign
		   | assigns assign
		   ;

assign     : keyword EQU VALUE
			 { assignKeyVal ($1, $3); }
		   | keyword EQU STRING
		     { assignKeyStr ($1, yytext); }
		   ;

keyword    : BOOT_MODE		     {  $$=$1;  }
		   | PARAM_INDEX		 {  $$=$1;  }
		   | OPTIONS			 {  $$=$1;  }
		   | MULTI_I2C_ID	     {  $$=$1;  }
		   | MY_I2C_ID			 {  $$=$1;  }
		   | CORE_FREQ_MHZ		 {  $$=$1;  }
		   | I2C_CLK_FREQ_KHZ    {  $$=$1;  }
		   | EXE_FILE			 {  $$=$1;  }
		   | NEXT_DEV_ADDR       {  $$=$1;  }
		   | NEXT_DEV_ADDR_EXT   {  $$=$1;  }
		   | DEV_ADDR_EXT        {  $$=$1;  }
		   | ADDRESS_DELAY       {  $$=$1;  }
		   | SWPLL				 {  $$=$1;  }
		   | DEV_ADDR			 {  $$=$1;  }
		   | ALIGN				 {  $$=$1;  }
		   | LENGTH              {  $$=$1;  }
		   | PAD_FILE_ID         {  $$=$1;  }
		   | SWPLL_PREDIV		 {  $$=$1;  }
		   | SWPLL_MULT			 {  $$=$1;  }
		   | SWPLL_POSTDIV		 {  $$=$1;  }
		   | SWPLL_FLAGS		 {  $$=$1;  }
		   | ADDR_WIDTH          {  $$=$1;  }
		   | N_PINS				 {  $$=$1;  }
		   | MODE                {  $$=$1;  }
		   | C2T_DELAY           {  $$=$1;  }
		   | BUS_FREQ_MHZ        {  $$=$1;  }
		   | BUS_FREQ_KHZ        {  $$=$1;  }
		   | CSEL				 {  $$=$1;  }
		   ;

%%


