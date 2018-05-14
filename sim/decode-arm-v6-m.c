/*
  Copyright (c) 1999-2008, Phillip Stanley-Marbell (author)
	Copyright (c) 2018, Andreas Theodosiou and Kyriacos Bagdates (author)

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
  copyright notice, this list of conditions and the following
  disclaimer.

	*	Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials
  provided with the distribution.

	*	Neither the name of the author nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior written
  permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include "instr-arm-v6-m.h"
#include "endian-arm-v6-m.h"
#include "sf.h"

int count_ones(unsigned value)
{
  unsigned count;
  for (count = 0; value != 0; count++; value &= value-1);
  return count;
}

void
arm_decode(Engine *E, ushort instr, ARMPipestage *stage)
{
	/*								*/
	/*	Must declare volatile to guarantee cookie cutting 	*/
	/*	(won't work w/o it, e.g., on SunOS 5.8 w/ gcc 2.95.3)	*/
	/*	Should stop using bitfields anyway :{P			*/
	/*								*/
	volatile decode_instr 	*tmp;


	stage->instr = instr;


	/*								*/
	/*	Fill fptr with (void *)nop by default so we dont have	*/
	/*	to check for valid fptrs ever, when using decode cache	*/
	/*	and, e.g., instr is '0'.				*/
	/*								*/
	stage->format = INSTR_HINT;
	stage->cycles = 1;
	stage->fptr = (void *)arm_nop;


	tmp = (void *)&instr;

	switch (tmp->dibit_14)
    {
		case B00:
      {
        switch (tmp->dibit_12)
          {
          case B00:
            {
              switch (tmp->unibit_11)
                {
                case B0:
                  {
                    stage->format = INSTR_SHIFT;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_lsli;
                    stage->op = OP_LSLI;
                  }
                case B1:
                  {
                    stage->format = INSTR_SHIFT;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_lsri;
                    stage->op = OP_LSRI;
                  }
                }
            }
          case B01:
            {
              switch (tmp->unibit_11)
                {
                case B0:
                  {
                    stage->format = INSTR_SHIFT;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_asri;
                    stage->op = OP_ASRI;
                  }
                case B1:
                  {
                    switch (tmp->dibit_9)
                      {
                      case B00:
                        {
                          stage->format = INSTR_SHIFT;
                          stage->cycles = 1;
                          stage->fptr = (void *)arm_add;
                          stage->op = OP_ADD;
                        }
                      case B01:
                        {
                          stage->format = INSTR_SHIFT;
                          stage->cycles = 1;
                          stage->fptr = (void *)arm_sub;
                          stage->op = OP_SUB;
                        }
                      case B10:
                        {
                          stage->format = INSTR_SHIFT;
                          stage->cycles = 1;
                          stage->fptr = (void *)arm_addi;
                          stage->op = OP_ADDI;
                        }
                      case B11:
                        {
                          stage->format = INSTR_SHIFT;
                          stage->cycles = 1;
                          stage->fptr = (void *)arm_subi;
                          stage->op = OP_SUBI;
                        }
                      }
                  }
                }
            }
          case B10:
            {
              switch (tmp->unibit_11)
                {
                case B0:
                  {
                    stage->format = INSTR_SHIFT;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_movi;
                    stage->op = OP_MOVI;
                  }
                case B1:
                  {
                    stage->format = INSTR_SHIFT;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_cmpi;
                    stage->op = OP_CMPI;
                  }
                }
            }
          case B11:
            {
              switch(tmp->unibit_11)
                {
                case B0:
                  {
                    stage->format = INSTR_SHIFT;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_addi;
                    stage->op = OP_ADDI;
                  }
                case B1:
                  {
                    stage->format = INSTR_SHIFT;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_subi;
                    stage->op = OP_SUBI;
                  }
                }
            }
          }
      }

    case B01:
      {
        switch(tmp->dibit_12)
          {
          case B00:
            {
              switch(tmp->unibit_11)
                {
                case B0:
                  {
                    switch(tmp->dibit_9)
                      {
                      case B00:
                        {
                          switch(tmp->unibit_8)
                            {
                            case B0:
                              {
                                switch(tmp->dibit_6)
                                  {
                                  case B00:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_and;
                                      stage->op = OP_AND;
                                    }
                                  case B01:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_eor;
                                      stage->op = OP_eor;
                                    }
                                  case B10:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_lsl;
                                      stage->op = OP_lsl;
                                    }
                                  case B11:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_lsr;
                                      stage->op = OP_lsr;
                                    }
                                  }
                              }
                            case B1:
                              {
                                switch(tmp->dibit_6)
                                  {
                                  case B00:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_asr;
                                      stage->op = OP_ASR;
                                    }
                                  case B01:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_adc;
                                      stage->op = OP_ADC;
                                    }
                                  case B10:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_sbc;
                                      stage->op = OP_SBC;
                                    }
                                  case B11:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_ror;
                                      stage->op = OP_ROR;
                                    }
                                  }
                              }
                            }
                        }
                      case B01:
                        {
                          switch(tmp->unibit_8)
                            {
                            case B0:
                              {
                                switch(tmp->dibit_6)
                                  {
                                  case B00:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_tst;
                                      stage->op = OP_TST;
                                    }
                                  case B01:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_rsb;
                                      stage->op = OP_RSB;
                                    }
                                  case B10:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_cmp;
                                      stage->op = OP_CMP;
                                    }
                                  case B11:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_cmn;
                                      stage->op = OP_CMN;
                                    }
                                  }
                              }
                            case B1:
                              {
                                switch(tmp->dibit_6)
                                  {
                                  case B00:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_orr;
                                      stage->op = OP_ORR;
                                    }
                                  case B01:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1; /* or 32 depends on multipler implementation */
                                      stage->fptr = (void *)arm_MUL;
                                      stage->op = OP_MUL;
                                    }
                                  case B10:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_bic;
                                      stage->op = OP_BIC;
                                    }
                                  case B11:
                                    {
                                      stage->format = INSTR_DATA;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_MVN;
                                      stage->op = OP_MVN;
                                    }
                                  }
                              }
                            }
                        }
                      case B10:
                        {
                          switch(tmp->unibit_8)
                            {
                              case B0:
                                {
                                  stage->format = INSTR_SPECIAL;
                                  stage->fptr = (void *)arm_add;
                                  stage->op = OP_ADD;
                                  if ((tmp->dibit_6 >= 2) && (tmp->tribit_0 == 7))
                                      stage->cycles = 2;
                                  else stage->cycles = 1;
                                }
                            case B1:
                              {
                                switch(tmp->dibit_6)
                                  {
                                  case B00:
                                    {break;}
                                  default:
                                    {
                                      stage->format = INSTR_SPECIAL;
                                      stage->cycles = 1;
                                      stage->fptr = (void*)arm_cmp;
                                      stage->op = OP_CMP;
                                    }
                                  }
                              }
                            }
                        }
                      case B11:
                        {
                          switch(tmp->unibit_8)
                            {
                            case B0:
                              {
                                stage->format = INSTR_SPECIAL;
                                stage->fptr = (void*)arm_mov;
                                stage->op = OP_MOV;
                                if ((tmp->dibit_6 >= 2) && (tmp->tribit_0 == 7))
                                  stage->cycles = 2;
                                else stage->cycles = 1;
                              }
                            case B1:
                              {
                                if (tmp->dibit_6 >= 2)
                                  {
                                  stage->format = INSTR_SPECIAL;
                                  stage->cycles = 2;
                                  stage->fptr = (void*)arm_blx;
                                  stage->op = OP_BLX;
                                  }
                                else
                                  {
                                    stage->format = INSTR_SPECIAL;
                                    stage->cycles = 2;
                                    stage->fptr = (void*)arm_bx;
                                    stage->op = OP_BX;
                                  }
                              }
                            }
                        }
                      }
                  }
                case B1:
                  {
                    stage->format = INSTR_SPECIAL;
                    stage->cycles = 1; /* single cycle I/O port */
                    stage->fptr = (void*)arm_ldrl;
                    stage->op = OP_LDRL;
                  }
                }
            }
          case B01:
            {
              switch(tmp->unibit_11)
                {
                case B0:
                  {
                    switch(tmp->dibit_9)
                      {
                      case B00:
                        {
                          stage->format = INSTR_LOAD_STORE;
                          stage->cycles = 1;
                          stage->fptr = (void*)arm_str;
                          stage->op = OP_STR;
                        }
                      case B01:
                        {
                          stage->format = INSTR_LOAD_STORE;
                          stage->cycles = 1;
                          stage->fptr = (void*)arm_strh;
                          stage->op = OP_STRH;
                        }
                      case B10:
                        {
                          stage->format = INSTR_LOAD_STORE;
                          stage->cycles = 1;
                          stage->fptr = (void*)arm_strb;
                          stage->op = OP_STRB;
                        }
                      case B11:
                        {
                          stage->format = INSTR_LOAD_STORE;
                          stage->cycles = 1;
                          stage->fptr = (void*)arm_ldrsb;
                          stage->op = OP_LDRSB;
                        }
                      }
                  }
                case B1:
                  {
                    switch(tmp->dibit_9)
                      {
                      case B00:
                        {
                          stage->format = INSTR_LOAD_STORE;
                          stage->cycles = 1;
                          stage->fptr = (void*)arm_ldr;
                          stage->op = OP_LDR;
                        }
                      case B01:
                        {
                          stage->format = INSTR_LOAD_STORE;
                          stage->cycles = 1;
                          stage->fptr = (void*)arm_ldrh;
                          stage->op = OP_LDRH;
                        }
                      case B10:
                        {
                          stage->format = INSTR_LOAD_STORE;
                          stage->cycles = 1;
                          stage->fptr = (void*)arm_ldrb;
                          stage->op = OP_LDRB;
                        }
                      case B11:
                        {
                          stage->format = INSTR_LOAD_STORE;
                          stage->cycles = 1;
                          stage->fptr = (void*)arm_ldrsh;
                          stage->op = OP_LDRSH;
                        }
                      }
                  }
                }
            }
          case B10:
            {
              switch(tmp->unibit_11)
                {
                B0:
                  {
                    stage->format = INSTR_LOAD_STORE;
                    stage->cycles = 1;
                    stage->fptr = (void*)arm_stri;
                    stage->op = OP_STRI;
                  }
                B1:
                  {
                    stage->format = INSTR_LOAD_STORE;
                    stage->cycles = 1;
                    stage->fptr = (void*)arm_ldri;
                    stage->op = OP_LDRI;
                  }
                }
            }
          case B11:
            {
              switch(tmp->unibit_11)
                {
                case B0:
                  {
                    stage->format = INSTR_LOAD_STORE;
                    stage->cycles = 1;
                    stage->fptr = (void*)arm_strbi;
                    stage->op = OP_STRBI;
                  }
                case B1:
                  {
                  stage->format = INSTR_LOAD_STORE;
                  stage->cycles = 1;
                  stage->fptr = (void*)arm_ldrbi;
                  stage->op = OP_LDRBI;
                  }
                }
            }
          }
      }

		case B10:
      {
        switch(tmp->dibit_12)
          {
          case B00:
            {
              switch(tmp->unibit_11)
                {
                case B0:
                  {
                    stage->format = INSTR_LOAD_STORE;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_strhi;
                    stage->op = OP_STRHI;
                  }
                case B1:
                  {
                    stage->format = INSTR_LOAD_STORE;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_ldrhi;
                    stage->op = OP_LDRHI;
                  }
                }
            }
          case B01:
            {

              switch(tmp->unibit_11)
                {
                case B0:
                  {
                    stage->format = INSTR_LOAD_STORE;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_strhi;
                    stage->op = OP_STRHI;
                  }
                case B1:
                  {
                    stage->format = INSTR_LOAD_STORE;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_ldrhi;
                    stage->op = OP_LDRHI;
                  }
                }
            }
          case B10:
            {
              switch(tmp->unibit_11)
                {
                case B0:
                  {
                    stage->format = INSTR_ADR;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_adr;
                    stage->op = OP_ADR;
                  }
                case B1:
                  {
                    stage->format = INSTR_ADDSPI;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_addspi;
                    stage->op = OP_ADDSPI;
                  }
                }
            }
          case B11:
            {
              switch(tmp->unibit_11)
                {
                case B0:
                  {
                    switch(tmp->dibit_9)
                      {
                      case B00:
                        {
                          if (tmp->dibit_6 >= 2)
                            {
                              stage->format = INSTR_MISC;
                              stage->cycles = 1;
                              stage->fptr = (void *)arm_subspi;
                              stage->op = OP_SUBSPI;
                            }
                          else
                            {
                              stage->format = INSTR_MISC;
                              stage->cycles = 1;
                              stage->fptr = (void *)arm_addspi;
                              stage->op = OP_ADDSPI;
                            }
                        }
                      case B01:
                        {
                          switch(tmp->dibit_6)
                            {
                            case B00:
                              {
                                stage->format = INSTR_MISC;
                                stage->cycles = 1;
                                stage->fptr = (void *)arm_sxth;
                                stage->op = OP_SXTH;
                              }
                            case B01:
                              {
                                stage->format = INSTR_MISC;
                                stage->cycles = 1;
                                stage->fptr = (void *)arm_sxtb;
                                stage->op = OP_SXTB;
                              }
                            case B10:
                              {
                                stage->format = INSTR_MISC;
                                stage->cycles = 1;
                                stage->fptr = (void *)arm_uxth;
                                stage->op = OP_UXTH;
                              }
                            case B11:
                              {
                                stage->format = INSTR_MISC;
                                stage->cycles = 1;
                                stage->fptr = (void *)arm_uxtb;
                                stage->op = OP_UXTB;
                              }
                            }
                        }
                      case B10:
                        {
                          /* number of cycles varies depending on the number of registers */
                          instr_lsmp *tmp_2 = (instr_lsmp *)tmp;
                          unsigned regs = tmp_2->regs;
                          stage->format = INSTR_MISC;
                          stage->cycles = count_ones(regs);
                          stage->fptr = (void *)arm_push;
                          stage->op = OP_PUSH;
                        }
                      case B11:
                        {
                          switch(tmp->unibit_8)
                            {
                            case B0:
                            {
                              switch(tmp->dibit_6)
                                {
                                case B00:
                                  {}
                                case B01:
                                  {
                                    switch(tmp->unibit_5)
                                      {
                                      case B1:
                                        {
                                          stage->format = INSTR_MISC;
                                          stage->cycles = 1;
                                          stage->fptr = (void *)arm_sps;
                                          stage->op = OP_SPS;
                                        }
                                      case B0:
                                        {break;}
                                      }
                                  }
                                case B10:
                                  {}
                                case B11:
                                  {}
                                }
                            }
                            case B1:
                              {}
                        }
                      }
                  }
                case B1:
                  {
                    switch(tmp->dibit_9)
                      {
                      case B00:
                        {break;}
                      case B01:
                        {
                          switch(tmp->dibit_6)
                            {
                            case B00:
                              {
                                stage->format = INSTR_MISC;
                                stage->cycles = 1;
                                stage->fptr = (void *)arm_rev;
                                stage->op = OP_REV;
                              }
                            case B01:
                              {
                                stage->format = INSTR_MISC;
                                stage->cycles = 1;
                                stage->fptr = (void *)arm_rev16;
                                stage->op = OP_REV16;
                              }
                            case B10:
                              {break;}
                            case B11:
                              {
                                stage->format = INSTR_MISC;
                                stage->cycles = 1;
                                stage->fptr = (void *)arm_revsh;
                                stage->op = OP_REVSH;
                              }
                            }
                        }
                      case B10:
                        {
                          /* clock cycle varies depending on list of registers */
                          instr_lsmp *tmp_2 = (instr_lsmp *)tmp;
                          unsigned regs = tmp_2->regs;
                          stage->format = INSTR_MISC;
                          stage->cycles = count_ones(regs);
                          stage->fptr = (void *)arm_pop;
                          stage->op = OP_POP;
                        }
                      case B11:
                        {
                          switch(tmp->unibit_8)
                            {
                            case B0:
                              {
                                /* cycle count depends on processor and debug configuration */
                                stage->format = INSTR_MISC;
                                stage->cycles = 1;
                                stage->fptr = (void *)arm_bkpt;
                                stage->op = OP_BKPT;
                              }
                            case B1:
                              {
                                switch(tmp->dibit_6)
                                  {
                                  case B00:
                                    {
                                      switch(tmp->unibit_5)
                                        {
                                        case B0:
                                          {
                                            switch(tmp->unibit_4)
                                              {
                                              case B0:
                                                {
                                                  stage->format = INSTR_MISC;
                                                  stage->cycles = 1;
                                                  stage->fptr = (void *)arm_nop;
                                                  stage->op = OP_NOP;
                                                }
                                              case B1:
                                                {
                                                stage->format = INSTR_MISC;
                                                stage->cycles = 1;
                                                stage->fptr = (void *)arm_yield;
                                                stage->op = OP_YIELD;
                                                }
                                          }
                                        }
                                        case B1:
                                          {
                                            switch(tmp->unibit_4)
                                              {
                                              case B0:
                                                {
                                                  stage->format = INSTR_MISC;
                                                  stage->cycles = 2;
                                                  stage->fptr = (void *)arm_wfe;
                                                  stage->op = OP_WFE;
                                                }
                                              case B1:
                                                {
                                                  stage->format = INSTR_MISC;
                                                  stage->cycles = 1;
                                                  stage->fptr = (void *)arm_wfi;
                                                  stage->op = OP_WFI;
                                                }
                                              }
                                          }
                                        }
                                  }
                                  case B01:
                                    {
                                      stage->format = INSTR_MISC;
                                      stage->cycles = 1;
                                      stage->fptr = (void *)arm_sev;
                                      stage->op = OP_SEV;
                                    }
                              }
                            }
                        }
                      }
                  }
                }
            }
          }
      }
          }
      }

    case B11:
      {
        switch(tmp->dibit_12)
          {
          case B00:
            {
              switch(tmp->unibit_11)
                {
                case B0:
                  {
                    /* clock cycles depend on register list */
                    instr_lsmp *tmp_2 = (instr_lsmp *)tmp;
                    unsigned regs = tmp_2->regs;
                    stage->format = INSTR_LSMP;
                    stage->cycles = count_ones(regs);
                    stage->fptr = (void *)arm_stm;
                    stage->op = OP_STM;
                  }
                case B1:
                  {
                    /* clock cycles depend on register list */
                    instr_lsmp *tmp_2 = (instr_lsmp *)tmp;
                    unsigned regs = tmp_2->regs;
                    stage->format = INSTR_LSMP;
                    stage->cycles = count_ones(regs);
                    stage->fptr = (void *)arm_ldm;
                    stage->op = OP_LDM;
                  }
                }
            }
          case B01:
            {
              if (tmp->dibit_9 == 3)
                {
                  switch(tmp->unibit_8)
                    {
                    case B0:
                      {
                        stage->format = INSTR_CBRANCH;
                        stage->cycles = 1;
                        stage->fptr = (void *)arm_udf;
                        stage->op = OP_UDF;
                      }
                    case B1:
                      {
                        /* cycle count depends on processor and debug configuration */
                        stage->format = INSTR_CBRANCH;
                        stage->cycles = 1;
                        stage->fptr = (void *)arm_cvc;
                        stage->op = OP_CVC;
                      }
                    }
                }
              else
                {
                  stage->format = INSTR_CBRANCH;
                  stage->cycles = 2;
                  stage->fptr = (void *)arm_b;
                  stage->op = OP_B;
                }
            }
          case B10:
            {
              stage->format = INSTR_UBRANCH;
              stage->cycles = 2;
              stage->fptr = (void *)arm_ubranch;
              stage->op = OP_UBRANCH;
            }
          case B11:
            {
              switch(tmp->unibit_11)
                {
                case B0:
                  {
                    stage->format = INSTR_BL_HI;
                    stage->cycles = 2;
                    stage->fptr = (void *)arm_bl_hi;
                    stage->op = OP_BL_HI;
                  }
                case B1:
                  {
                    stage->format = INSTR_BL_LO;
                    stage->cycles = 1;
                    stage->fptr = (void *)arm_bl_lo;
                    stage->op = OP_BL_lo;
                  }
                }
            }
          }
      }

    default:
      {
        mprint(E, NULL, siminfo, "\nUnknown instruction seen in superHdecode()!\n\n");
        mexit(E, "See above messages", -1);

        break;
      }
    }
}


	return;
}
