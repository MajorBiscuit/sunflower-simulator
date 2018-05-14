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

enum
{
	OP_ADC,
  OP_ADD,
	OP_ADDI,
	OP_ADDSPI,
  OP_ADDSPR,
  OP_ADR,
	OP_AND,
	OP_ASR,
  OP_ASRI,
	OP_ANDM,
	OP_B,
	OP_BIC,
  OP_BKPT,
  OP_BL_LO,
  OP_BL_HI,
  OP_BLX,
  OP_BX,
  OP_CMN,
  OP_CMP,
  OP_CMPI,
  OP_CPS,
  OP_EOR,
  OP_LDR,
  OP_LDRSP,
  OP_LDRI,
  OP_LDRL,
  OP_LDRB,
  OP_LDRBI,
  OP_LDRH,
  OP_LDRHI,
  OP_LDRSB,
  OP_LDRSH,
  OP_LSL,
  OP_LSLI,
  OP_LSR,
  OP_LSRI,
  OP_MOV,
  OP_MOVI,
  OP_MUL,
  OP_NOP,
  OP_ORR,
  OP_POP,
  OP_PUSH,
  OP_REV,
  OP_REV16,
  OP_REVSH,
  OP_ROR,
  OP_RSBI,
  OP_SBC,
  OP_SEV,
  OP_STM,
  OP_STRI,
  OP_STR,
  OP_STRSP,
  OP_STRB,
  OP_STRBI,
  OP_STRH,
  OP_STRHI,
  OP_SUBI,
  OP_SUB,
  OP_SUBSPI,
  OP_SVC,
  OP_SXTB,
  OP_SXTH,
  OP_TST,
  OP_UDF,
  OP_UXTB,
  OP_UXTH,
  OP_WFE,
  OP_WFI,
  OP_YIELD,

	OP_MAX,
};
