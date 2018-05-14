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

/*									*/
/*	Implementation of the operations performed by the instrs.	*/
/*									*/
/*	References : ARMv6-M Architecture Reference Manual				*/
/*									*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "sf.h"
#include "mmu-hitachi-sh.h"
#include "instr-arm-v6-m.h"
#include "endian-arm-v6-m.h"
#include "mextern.h"

/*									*/
/*   ADD (Add Binary): Arithmetic Instruction				*/
/*									*/
/*   Format 		      Abstract	         Code 		     Cycle 	*/
/*   -------------------------------------------------------	*/
/*   ADDS Rd,Rn,Rm     Rm+Rn -> Rd 	                    1 */
/*   ADD  Rdn,Rm       Rm+Rdn -> Rdn                    1 */
/*   ADDS Rd,Rn,#imm3  Rn+imm3 -> Rd 	                  1 */
/*	 ADDS Rdn,#imm8    Rdn+imm8 -> Rdn							    1 */
/*   ADD  Rdm,SP,Rdm   Rdm+SP -> Rdm                    1 */
/*   ADD  SP,Rm        Rm+SP -> SP                      1 */
/*   ADD  Rd,SP,#imm8  SP+imm8 -> Rd                    1 */
/*   ADD  SP,SP,#imm7  SP+imm7 -> SP                    1 */
/*   Description: Adds general register Rn data to Rm data, and		*/
/*   stores the result in Rd. 8-bit immediate data can be added 	*/
/*   instead of Rm data. Since the 8-bit immediate data is sign-	*/
/*   extended to 32 bits, this instruction can add and subtract 	*/
/*   immediate data.							*/
/*									*/

unsigned carry_out(unsigned carry_in, ulong m, ulong n)
{
  ulong tmp0, tmp1, tmp2;
  unsigned carry = 0;

	tmp1 = n + m;
	tmp0 = n;
  tmp2 = tmp1 + carry_in;
	if (tmp0 > tmp1)
    carry = 1;
	else
    carry = 0;
  if (tmp1 > tmp2)
    carry = 1;
  return carry;
}

unsigned overflow(unsigned carry_in, long m, long n)
{
  unsigned overflow_flag = 0;
  long result = m + n + carry_in;
  long long result_ll = (long long)(m + n + carry_in);
  long long sum = (long long)result;
  if (result_ll == sum)
    overflow_flag = 0;
  else overflow_flag = 1;
  return overflow_flag;
}

void
arm_adc(Engine *E, State *S, ulong d, ulong m)
{
  ulong Rd = reg_read(E, S, d);
  ulong Rm = reg_read(E, S, m);
  ulong sum = Rd + Rm + S->arm->SR.C;
  unsigned n = 0;
  if (sum & (1 << 31))
    n = 1;
  S->arm->SR.N = n;
  S->arm->SR.Z = (sum == 0);
  S->arm->SR.C = carry_out(S->arm->SR.C, Rd, Rm);
  S->arm->SR.V = overflow(S->arm->SR.C, Rd, Rm);
  return;
}

void
arm_add(Engine *E, State *S, ulong d, ulong m, ulong n)
{
  ulong Rn = reg_read(E, S, n);
  ulong Rm = reg_read(E, S, m);
  ulong sum = Rm + Rn;
  reg_set(E, S, d, sum);
  if (d = 15)
    {
      ulong tmp_0 = 0xFFFFFFFF - 1;
      ulong sum_15 = sum & tmp_0;
      reg_set(E, S, d, sum_15);
    }
  else
    {
      reg_set(E, S, d, sum);
      unsigned n = 0;
      if (sum & (1 << 31))
        n = 1;
      S->arm->SR.N = n;
      S->arm->SR.Z = (sum == 0);
      S->arm->SR.C = carry_out(0, Rm, Rn);
      S->arm->SR.V = overflow(0, Rm, Rn);
    }
	return;
}

void
arm_addi(Engine *E, State *S, ulong d, ulong imm, ulong n)
{
  ulong Rn = reg_read(E, S, n);
  ulong sum = Rn + imm;
  reg_set(E, S, d, sum);
  unsigned n = 0;
  if (sum & (1 << 31)) {
    n = 1;
  }
  S->arm->SR.N = n;
  S->arm->SR.Z = (sum==0);
  S->arm->SR.C = carry_out(0, Rn, imm);
  S->arm->SR.V = overflow(0, Rn, imm);
  return;
}

void arm_addspi(Engine *E, State *S, ulong d, ulong imm)
{
  ulong R_SP = reg_read(E, S, 13);
  reg_set(E, s, d, R_SP + (imm * 4));
  return;
}

void arm_addspr(Engine *E, State *S, ulong d, ulong m)
{
  ulong Rm = reg_read(E, S, m);
  ulong Rd = reg_read(E, S, d);
  /* ulong R_SP = reg_read(E, S, 13); */
  ulong sum = Rm + Rd;
  if (d = 15)
    {
      ulong tmp_0 = 0xFFFFFFFF - 1;
      ulong sum_15 = sum & tmp_0;
      reg_set(E, S, d, sum_15);
    }
  else
      reg_set(E, S, d, sum);
  return;
}

void arm_addr(Engine *E, State *S, ulong d, ulong imm)
{
  ulong R_PC = reg_read(E, S, 15);
  ulong LSB_reset = 0xFFFFFFFF - 3;
  R_PC = R_PC & LSB_reset;
  ulong sum = R_PC + (imm * 4);
  reg_set(E, S, d, sum);
  return;
}

void arm_and(Engine *E, State *S, ulong d, ulong m)
{
  ulong Rm = reg_read(E, S, m);
  ulong Rd = reg_read(E, S, d);
  ulong result = Rm & Rd;
  unsigned n = 0;
  if (result & (1 << 31))
    n = 1;
  S->arm->SR.N = n;
  S->arm->SR.Z = (result == 0);
  reg_set(E, S, d, result);
  return;
}

void arm_asr(Engine *E, State *S, ulong d, ulong m)
{
  ulong Rm = reg_read(E, S, m);
  ulong Rd = reg_read(E, S, d);
  /* As per the DecodeImmShift('10', imm5) function of the ARMv6-M Architecture Reference Manual the immediate is given by the 8 LSB of Rm */
  ulong shift = Rm & 0x000000FF;
  /* mask used to determine the sign of the 2s complement number */
  ulong s_mask = 0x80000000;
  ulong Rd_shifted = 0;
  if (Rd & s_mask == 0)
    {
      /* if positive, do logical shift */
      Rd_shifted = Rd >> shift;
    }
  else {
    ulong one_mask = 0;
    int i;
    for (i = 0; i < (32 - shift); i++)
      {
        one_mask += 1 << i;
      }
    /* set the bits that represent the sign, reset the bits that hold the shifted value */
    ulong one_mask = 0xFFFFFFFF - one_mask;
    Rd_shifted = Rd >> shift;
    Rd_shifted = Rd_shifted | one_mask;
  }
  reg_set(E, S, d, Rd_shifted);
  if (result & (1 << 31))
    n = 1;
  S->arm->SR.N = n;
  S->arm->SR.Z = (result == 0);
  /* the carry bit is the last bit to be shifted to the right */
  ulong c_mask = 1 << shift;
  unsigned carry = ((Rd & c_mask) != 0);
  S->arm->SR.C = carry;
}


void arm_asri(Engine *E, State *S, ulong d, ulong m, ulong imm)
{
  ulong Rm = reg_read(E, S, m);
  ulong shift;
  /* As per the DecodeImmShift('10', imm5) function of the ARMv6-M Architecture Reference Manual when the immediate is 0 the instruction shifts to the right by 32 bits. */
  if (imm == 0)
    shift = 32;
  else
    shift = imm;
  /* mask used to determine the sign of the 2s complement number */
  ulong s_mask = 0x80000000;
  ulong Rm_shifted = 0;
  if (Rm & s_mask == 0)
    {
      /* if positive, do logical shift */
      Rm_shifted = Rm >> shift;
    }
  else {
    ulong one_mask = 0;
    int i;
    for (i = 0; i < (32 - shift); i++)
      {
        one_mask += 1 << i;
      }
    /* set the bits that represent the sign, reset the bits that hold the shifted value */
    ulong one_mask = 0xFFFFFFFF - one_mask;
    Rm_shifted = Rm >> shift;
    Rm_shifted = Rm_shifted | one_mask;
  }
  reg_set(E, S, d, Rm_shifted);
  if (result & (1 << 31))
    n = 1;
  S->arm->SR.N = n;
  S->arm->SR.Z = (result == 0);
  /* the carry bit is the last bit to be shifted to the right */
  ulong c_mask = 1 << shift;
  unsigned carry = ((Rm & c_mask) != 0);
  S->arm->SR.C = carry;
  return;
}

void arm_b(Engine *E, State *S, long imm)
{
  imm *= 2;
  ulong R_PC = reg_read(E, S, 15);
  ulong result = (R_PC + imm) & 0xFFFFFFFE;
  reg_set(E, S, 15, result);
  return;
}

/* First 16 bytes of bl instruction */
void arm_bl_lo(Engine *E, State *S, ulong imm)
{
  S->arm->bl_halfword = imm;
  return;
}

void arm_bl_hi(Engine *E, State *S, ulong imm_10, unsigned S)
{
  /* J1 and J2 are 1-bit variables according to the armV6 T2 manual. However, in our case the assembler always produces instructions with J1 and J2 set to 1 */
  unsigned J1, J2 = 1;
  unsigned I1 = ~(J1 ^ S);
  unsigned I2 = ~(J2 ^ S);
  /* Fetch imm_11 from first bl narrow instruction */
  ulong imm_11 = S->arm->bl_halfword;
  ulong imm_32 = imm_11 + (imm_10 << 11) + (I2 << 21) + (I1 << 22) + (S << 23);
  imm_32 = imm_32 << 1;
  if (S == 0x0001)
    {
      imm_32 = imm_32 | 0xFF000000;
    }
  ulong next_address = reg_read(E, S, 15);
  reg_set(E, S, 14, (next_address | 1));
  ulong result = (next_address + imm_32) & 0xFFFFFFFE;
  reg_set(E, S, 15, result);
  return;
}

void arm_blx(Engine *E, State *S, ulong m)
{
  ulong next_address = reg_read(E, S, 15) - 2;
  /* armV6-M ISA manual instructs us to set the first bit of the next isntruction address */
  reg_set(E, S, 14, (next_address | 1));
  ulong Rm = reg_read(E, S, m);
  /* branch with link and exchange peforms a branch to the address in Rm, copies the address of the next instruction to the LR and changes the instruction set.
     If bit[0] of Rm is 0 the processor changes to, or remains in, ARM state.
     If bit[0] of Rm is 1, the processor changes to, or remains in, Thumb state.
     armV6-M only supports only supports Thumb execution, hence setting the T flag of the  Execution Program Status Register to 0 triggers a Hard Fault.*/
  S->arm->SR.T = Rm & 0x00000001;
  ulong result = Rm & 0xFFFFFFFE;
  reg_set(E, S, 15, result);
}

void arm_bx(Engine *E, State *S, ulong m)
{
  ulong Rm = reg_read(E, S, m);
  S->arm->SR.T = Rm & 0x00000001;
  ulong result = Rm & 0xFFFFFFFE;
  /* TODO */
  /* armV6-M regerence manual specifies that BX can be used for an exception return. An exception return occurs when the processor is in Handler Mode and BX loads a value of 0xFXXXXXXX into the PC.
   Pseudocode
   if (CurrentMode == Mode_Handler & Rm[31:28] == 0b1111 )
       ExceptionReturn(Rm[27:0])*/
  reg_set(E, S, 15, result);
  return;
}
