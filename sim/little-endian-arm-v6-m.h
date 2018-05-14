/*
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

typedef struct
{
  unsigned  imm:8;
  unsigned  dst:3;
  unsigned  code:5;
} instr_adr;

typedef struct
{
  unsigned  imm:8;
  unsigned  dst:3;
  unsigned  code:5;
} instr_addspi;

typedef struct
{
	unsigned	dst:3;
  unsigned  operand_a:3;
  unsigned  operand_b:2;
  unsigned  operand_c:1;
  unsigned  func_b:2;
  unsigned  func_a:3;
  unsigned  code:2;
} instr_shift;

typedef struct
{
	unsigned 	dst:3;
  unsigned  operand:3;
	unsigned	func:4;
	unsigned	code:6;
} instr_data;

typedef struct
{
  unsigned 	dst:3;
	unsigned	operand:4;
  unsigned  dn:1;
  unsigned  func:2;
	unsigned	code:6;
} instr_special_data;

typedef struct
{
	unsigned 	source:3;
  unsigned  base:3;
  unsigned  offset_a:2;
  unsigned  offset_b:1;
  unsigned  dst:2;
	unsigned	code_b:1;
	unsigned	code_a:4;
} instr_load_store;

typedef struct
{
	unsigned 	dst:3;
  unsigned  src:3;
  unsigned  middle_6:1;
  unsigned  middle_8:1;
  unsigned  middle_9:1;
	unsigned	func:3;
	unsigned  code:4;
} instr_misc;

typedef struct
{
	unsigned	operands:8;
	unsigned	func:4;
	unsigned	code:4;
} instr_cbranch;

typedef struct
{
  unsigned  imm:11;
  unsigned  code:5;
} instr_ubranch;

typedef struct
{
  unsigned imm:10;
  unsigned S:1;
  unsigned code:5;
} instr_bl_hi;

typedef struct
{
  unsigned imm:11;
  unsigned J2:1;
  unsigned code_2:1;
  unsigned J1:1;
  unsigned code_1:2;
} instr_bl_lo;

typedef struct
{
  unsigned  imm:8;
  unsigned  dst:3;
  unsigned  code:5;
} instr_ldr;

typedef struct
{
  unsigned  regs:8;
  unsigned  source:3;
  unsigned  code:5;
} instr_lsmp;

typedef struct
{
  unsigned tribit_0:3;
  unsigned unibit_3:1;
  unsigned unibit_4:1;
	unsigned unibit_5:1;
  unsigned dibit_6:2;
  unsigned unibit_8:1;
  unsigned dibit_9:2;
	unsigned unibit_11:1;
	unsigned dibit_12:2;
	unsigned dibit_14:2;
} decode_instr;
