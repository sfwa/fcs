/* Lexical analyzer for the eeprom boot builder */
%{
#include "rparse.tab.h"
extern int line;
%}
%%

"{" return (LBRACE);	
"}" return (RBRACE);   
"=" return (EQU);	    

section           { yylval = SECTION;           rBegin(SECTION); return (SECTION);           }
layout			  { yylval = LAYOUT;			rBegin(LAYOUT);  return (LAYOUT);			 }
pad				  { yylval = PAD;				rBegin(PAD);     return (PAD);				 }
boot_mode         { yylval = BOOT_MODE;                          return (BOOT_MODE);         }
param_index       { yylval = PARAM_INDEX;                        return (PARAM_INDEX);       }
options           { yylval = OPTIONS;                            return (OPTIONS);           }
multi_i2c_id      { yylval = MULTI_I2C_ID;                       return (MULTI_I2C_ID);      }
my_i2c_id         { yylval = MY_I2C_ID;                          return (MY_I2C_ID);         }
core_freq_mhz     { yylval = CORE_FREQ_MHZ;                      return (CORE_FREQ_MHZ);     }
i2c_clk_freq_khz  { yylval = I2C_CLK_FREQ_KHZ;                   return (I2C_CLK_FREQ_KHZ);  }
exe_file          { yylval = EXE_FILE;                           return (EXE_FILE);          }
pci_parm		  { yylval = PCI_PARMS;                          return (PCI_PARMS);	     }
dev_addr_ext      { yylval = DEV_ADDR_EXT;                       return (DEV_ADDR_EXT);		 }
dev_addr		  { yylval = DEV_ADDR;			                 return (DEV_ADDR);		     }
next_dev_addr     { yylval = NEXT_DEV_ADDR;                      return (NEXT_DEV_ADDR);     }
next_dev_addr_ext { yylval = NEXT_DEV_ADDR_EXT;                  return (NEXT_DEV_ADDR_EXT); }
address_delay     { yylval = ADDRESS_DELAY;                      return (ADDRESS_DELAY);     }
sw_pll			  { yylval = SWPLL;				                 return (SWPLL);			 }
align			  { yylval = ALIGN;							     return (ALIGN);             }
len				  { yylval = LENGTH;							 return (LENGTH);			 }
pad_file_id       { yylval = PAD_FILE_ID;						 return (PAD_FILE_ID);		 }
sw_pll_prediv	  { yylval = SWPLL_PREDIV;						 return (SWPLL_PREDIV);      }
sw_pll_mult	      { yylval = SWPLL_MULT;						 return (SWPLL_MULT);        }
sw_pll_postdiv    { yylval = SWPLL_POSTDIV;     				 return (SWPLL_POSTDIV);     }
sw_pll_flags      { yylval = SWPLL_FLAGS;						 return (SWPLL_FLAGS);       }
addr_width        { yylval = ADDR_WIDTH;						 return (ADDR_WIDTH);        }
n_pins            { yylval = N_PINS;							 return (N_PINS);			 }
mode              { yylval = MODE;								 return (MODE);				 }
c2t_delay		  { yylval = C2T_DELAY;							 return (C2T_DELAY);		 }
bus_freq_mhz	  { yylval = BUS_FREQ_MHZ;					     return (BUS_FREQ_MHZ);	     }
bus_freq_khz      { yylval = BUS_FREQ_KHZ;						 return (BUS_FREQ_KHZ);		 }
csel			  { yylval = CSEL;								 return (CSEL);				 }


[0-9]+ 		 { yylval = atoi(yytext); return (VALUE);  }
0x[0-9a-f]+ { sscanf (&yytext[2], "%x", &yylval); return (VALUE);  }

\".*\"		return (STRING);

;.*\n       line++ ;
[ \t]+		;
[\n]		line++ ;

%%




