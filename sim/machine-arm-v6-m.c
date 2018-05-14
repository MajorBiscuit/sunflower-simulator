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
#include <string.h>
#include <math.h>
#include <float.h>
#include "sf.h"
#include "mextern.h"
#include "ilpa.h"


static tuck void	drain_pipeline(Engine *E, State *S);
static tuck int		interruptible(State *S);


tuck int
interruptible(State *S)
{
	//TODO: this is not quite right, since we should be
	//	able to take intrs even if bus is busy
	if (S->arm->B->pbuslock && (S->arm->B->pbuslocker != S->NODE_ID))
	{
		return 0;
	}

	switch (S->arm->P.ID.op)
	{
		case OP_BF:
		case OP_BT:
		case OP_BRA:
		case OP_BSR:
		case OP_JMP:
		case OP_JSR:
		case OP_RTS:
		case OP_RTE:
		case OP_TRAPA:
		case OP_BFS:
		case OP_BTS:
		case OP_BRAF:
		case OP_BSRF:
		{
			return 0;
		}
	}

	switch (S->arm->P.IF.op)
	{
		case OP_BF:
		case OP_BT:
		case OP_BRA:
		case OP_BSR:
		case OP_JMP:
		case OP_JSR:
		case OP_RTS:
		case OP_RTE:
		case OP_TRAPA:
		case OP_BFS:
		case OP_BTS:
		case OP_BRAF:
		case OP_BSRF:
		{
			return 0;
		}
	}

	return 1;
}

tuck void
drain_pipeline(Engine *E, State *S)
{
	/*							*/
	/*	Note: this will continue trying to drain	*/
	/*	pipeline even if simulator is off, since it	*/
	/*	is called from (possibly) spawned scheduler	*/
	/*	thread. TODO: we can't halt this now because	*/
	/*	we'll lose state of how we got here: Once we	*/
	/*	implement per-CPU setjmp state-saving for 	*/
	/*	scheduler, that should be easier.		*/
	/*							*/
	while (	(S->arm->P.IF.instr != 0x9) ||
		(S->arm->P.ID.instr != 0x9) ||
		(S->arm->P.EX.instr != 0x9) ||
		(S->arm->P.MA.instr != 0x9) ||
		(S->arm->P.WB.instr != 0x9))
	{
		/*	Call ARMstep, with flag set to drain pipe	*/
		ARMstep(E, S, 1);
	}

	return;
}

void
ARMsettimerintrdelay(Engine *E, State *S, int delay)
{
	S->arm->TIMER_INTR_DELAY = delay * 1E-6;
}

tuck int
ARMcheck_nic_intr(Engine *E, State *S)
{
	return ARM_check_nic_intr_macro(S);
}

tuck int
ARMcheck_batt_intr(Engine *E, State *S)
{
	return ((((S->BATT) &&
			((Batt *)S->BATT)->battery_remaining <=
			((Batt *)S->BATT)->battery_capacity*S->battery_alert_frac) ||
			(S->ENABLE_TOO_MANY_FAULTS && (S->nfaults >= S->faultthreshold))) &&
		(S->arm->SR.BL == 0) &&
		(S->arm->SR.IMASK < BATT_LOW_FIXED_INTRLEVEL) &&
		!((int)S->ICLK % 60));
}

void
ARMdumpregs(Engine *E, State *S)
{
	int i;

	for (i = 0; i < 16; i++)
	{
		mprint(E, S, nodeinfo, "R%-2d\t\t", i);
		mbitprint(E, S, 32, S->arm->R[i]);
		mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->R[i]);
	}

	return;
}

void
ARMdumpsysregs(Engine *E, State *S)
{
	int i;
	long tmp;

	for (i = 0; i < 8; i++)
	{
		mprint(E, S, nodeinfo, "%-7s%d\t", "R_BANK_", i);
		mbitprint(E, S, 32, S->arm->R_BANK[i]);
		mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->R_BANK[i]);
	}

	memmove(&tmp, &S->arm->SR, sizeof(tmp));
	mprint(E, S, nodeinfo, "%-8s\t", "SR");
	mbitprint(E, S, 32, tmp);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", tmp);

	memmove(&tmp, &S->arm->SSR, sizeof(tmp));
	mprint(E, S, nodeinfo, "%-8s\t", "SSR");
	mbitprint(E, S, 32, tmp);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", tmp);

	mprint(E, S, nodeinfo, "%-8s\t", "GBR");
	mbitprint(E, S, 32, S->arm->GBR);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->GBR);

	mprint(E, S, nodeinfo, "%-8s\t", "MACH");
	mbitprint(E, S, 32, S->arm->MACH);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->MACH);

	mprint(E, S, nodeinfo, "%-8s\t", "MACL");
	mbitprint(E, S, 32, S->arm->MACL);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->MACL);

	mprint(E, S, nodeinfo, "%-8s\t", "PR");
	mbitprint(E, S, 32, S->arm->PR);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->PR);

	mprint(E, S, nodeinfo, "%-8s\t", "VBR");
	mbitprint(E, S, 32, S->arm->VBR);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->VBR);

	mprint(E, S, nodeinfo, "%-8s\t", "PC");
	mbitprint(E, S, 32, S->PC);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->PC);

	mprint(E, S, nodeinfo, "%-8s\t", "SPC");
	mbitprint(E, S, 32, S->arm->SPC);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->SPC);

	mprint(E, S, nodeinfo, "%-8s\t", "TTB");
	mbitprint(E, S, 32, S->arm->TTB);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->TTB);

	mprint(E, S, nodeinfo, "%-8s\t", "TEA");
	mbitprint(E, S, 32, S->arm->TEA);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->TEA);

	mprint(E, S, nodeinfo, "%-8s\t", "MMUCR");
	mbitprint(E, S, 32, S->arm->MMUCR);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->MMUCR);

	mprint(E, S, nodeinfo, "%-8s\t", "PTEH");
	mbitprint(E, S, 32, S->arm->PTEH);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->PTEH);

	mprint(E, S, nodeinfo, "%-8s\t", "PTEL");
	mbitprint(E, S, 32, S->arm->PTEL);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->PTEL);

	mprint(E, S, nodeinfo, "%-8s\t", "TRA");
	mbitprint(E, S, 32, S->arm->TRA);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->TRA);

	mprint(E, S, nodeinfo, "%-8s\t", "EXPEVT");
	mbitprint(E, S, 32, S->arm->EXPEVT);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->EXPEVT);

	mprint(E, S, nodeinfo, "%-8s\t", "INTEVT");
	mbitprint(E, S, 32, S->arm->INTEVT);
	mprint(E, S, nodeinfo, "  [0x%08lx]\n", S->arm->INTEVT);

	mprint(E, S, nodeinfo, "SLEEP = [%s]\n", (S->sleep == 1 ? "YES" : "NO"));

	return;
}

void
ARMfatalaction(Engine *E, State *S)
{
	ARMdumptlb(E, S);
	mprint(E, S, nodeinfo, "FATAL (node %d): P.EX=[%s]\n",\
			S->NODE_ID, opstrs[S->arm->P.EX.op]);

	return;
}

tuck void
ARMstallaction(Engine *E, State *S, ulong addr, int type, int latency)
{
	/*	PAU may change VDD	*/
	if (SF_PAU_DEFINED)
	{
		pau_feed(E, S, type, addr);
	}

	/*								*/
	/*	Stall fetch unit on next access or instr in EX		*/
	/*	the stall actually occurs when in MA, since we've	*/
	/*	completed the EX wait before we get executed.		*/
	/*								*/
	if (S->arm->mem_access_type == MEM_ACCESS_IFETCH)
	{
		S->arm->P.fetch_stall_cycles += latency;
	}
	else
	{
		S->arm->P.EX.cycles += latency;
	}

	/*								*/
	/*	TODO: This will have to change when we implement	*/
	/*	setjmp idea for simulating memory stalls		*/
	/*								*/
	//So that accesses to main mem that are not mapped objects do not lock bus
	//if (latency == 0)
	//{
	//	return;
	//}

	S->arm->B->pbuslock = 1;
	S->arm->B->pbuslocker = S->NODE_ID;
	S->arm->B->pbuslock_type = type;
	S->arm->B->pbuslock_addr = addr;

	return;
}


tuck int
ARMtake_timer_intr(Engine *E, State *S)
{
	if (!interruptible(S))
	{
		return -1;
	}

	/*									*/
	/*	Timer interrupts should be in terms of absolute time, not	*/
	/*	clock cycles, so that even w/ volt/freq scaling, we will	*/
	/*	still get timer interrupts at the same time spacing		*/
	/*									*/
	if ((S->arm->ENABLE_CLK_INTR) && (S->ICLK) &&
		(S->arm->SR.BL == 0) &&
		(S->arm->SR.IMASK < TIMER_FIXED_INTRLEVEL)
	  )
	{
		if (S->step == ARMfaststep)
		{
			S->arm->SPC = S->PC;
		}
		else
		{
			drain_pipeline(E, S);

			/*						*/
			/*	Must do this after drain because 	*/
			/*	executed instruction during drain 	*/
			/*	might set PC (e.g. a BRA)		*/
			/*						*/
			S->arm->SPC = S->PC;
		}

		S->arm->SSR = S->arm->SR;
		S->arm->SR.BL = 1;
		S->arm->SR.MD = 1;
		S->arm->SR.RB = 1;
		S->arm->INTEVT = TMU0_TUNI0_EXCP_CODE;
		S->PC = S->arm->VBR + 0x600;
		S->sleep = 0;
	}

	return 0;
}

tuck void
ARMtake_exception(Engine *E, State *S)
{
	enum		{ABORTED_AND_RETRIED, ABORTED, COMPLETED, INVALID_HANDLING};
	Interrupt	*intr;
	int		handling = INVALID_HANDLING;


	intr = (Interrupt *)pic_intr_dequeue(E, S, S->arm->excpQ);
	if (intr == NULL)
	{
		sfatal(E, S,
		"We supposedly had an exception, but nothing was queued!");
	}

	S->arm->SSR = S->arm->SR;
	S->arm->SR.BL = 1;
	S->arm->SR.MD = 1;
	S->arm->SR.RB = 1;
	S->sleep = 0;

	switch (intr->type)
	{
		case H_UDI_RESET_EXCP:
		{
			S->PC = RESET_VECTOR_ADDR;
			S->arm->EXPEVT = H_UDI_RESET_EXCP_CODE;
			handling = ABORTED;
			break;
		}
		case POWER_ON_RESET_EXCP:
		{
			S->PC = RESET_VECTOR_ADDR;
			S->arm->EXPEVT = POWER_ON_RESET_EXCP_CODE;
			handling = ABORTED;
			break;
		}
		case MANUAL_RESET_EXCP:
		{
			S->PC = RESET_VECTOR_ADDR;
			S->arm->EXPEVT = MANUAL_RESET_EXCP_CODE;
			handling = ABORTED;
			break;
		}
		case TLB_LOAD_MISS_EXCP:
		{
			S->PC = S->arm->VBR + TLB_MISS_OFFSET;
			S->arm->EXPEVT = TLB_LOAD_MISS_EXCP_CODE;
			handling = ABORTED_AND_RETRIED;
			break;
		}
		case TLB_STORE_MISS_EXCP:
		{
			S->PC = S->arm->VBR + TLB_MISS_OFFSET;
			S->arm->EXPEVT = TLB_STORE_MISS_EXCP_CODE;
			handling = ABORTED_AND_RETRIED;
			break;
		}
		case TLB_LOAD_INVALID_EXCP:
		{
			S->PC = S->arm->VBR + TLB_INVALID_OFFSET;
			S->arm->EXPEVT = TLB_LOAD_INVALID_EXCP_CODE;
			handling = ABORTED_AND_RETRIED;
			break;
		}
		case TLB_STORE_INVALID_EXCP:
		{
			S->PC = S->arm->VBR + TLB_INVALID_OFFSET;
			S->arm->EXPEVT = TLB_STORE_INVALID_EXCP_CODE;
			handling = ABORTED_AND_RETRIED;
			break;
		}
		case TLB_INIT_PAGEWRITE_EXCP:
		{
			S->PC = S->arm->VBR + TLB_INIT_PAGEWRITE_OFFSET;
			S->arm->EXPEVT = TLB_INIT_PAGEWRITE_EXCP_CODE;
			handling = ABORTED_AND_RETRIED;
			break;
		}
		case TLB_LOAD_PROTECT_EXCP:
		{
			S->PC = S->arm->VBR + TLB_PROTECT_OFFSET;
			S->arm->EXPEVT = TLB_LOAD_PROTECT_EXCP_CODE;
			handling = ABORTED_AND_RETRIED;
			break;
		}
		case TLB_STORE_PROTECT_EXCP:
		{
			S->PC = S->arm->VBR + TLB_PROTECT_OFFSET;
			S->arm->EXPEVT = TLB_STORE_PROTECT_EXCP_CODE;
			handling = ABORTED_AND_RETRIED;
			break;
		}
		case CPU_LOAD_ADDRERR_EXCP:
		{
			/*	Doesn't make a difference; Self documenting.	*/
			if (intr->misc == MEM_ACCESS_IFETCH)
			{
				S->PC = S->arm->VBR + CPU_INSTR_ADDRERR_OFFSET;
			}
			else
			{
				S->PC = S->arm->VBR + CPU_ADDRERR_OFFSET;
			}

			S->arm->EXPEVT = CPU_LOAD_ADDRERR_EXCP_CODE;
			handling = ABORTED_AND_RETRIED;
			break;
		}
		case CPU_STORE_ADDRERR_EXCP:
		{
			S->PC = S->arm->VBR + CPU_ADDRERR_OFFSET;
			S->arm->EXPEVT = CPU_STORE_ADDRERR_EXCP_CODE;
			handling = ABORTED_AND_RETRIED;
			break;
		}
		case TRAPA_EXCP_CODE:
		{
			S->PC = S->arm->VBR + TRAPA_OFFSET;
			S->arm->EXPEVT = TRAPA_EXCP_CODE;
			handling = COMPLETED;
			break;
		}
		case ILLEGAL_INSTR_EXCP:
		{
			S->PC = S->arm->VBR + ILLEGAL_INSTR_OFFSET;
			S->arm->EXPEVT = ILLEGAL_INSTR_EXCP_CODE;
			handling = ABORTED_AND_RETRIED;
			break;
		}
		case ILLEGAL_SLOT_INSTR_EXCP:
		{
			S->PC = S->arm->VBR + ILLEGAL_SLOT_INSTR_OFFSET;
			S->arm->EXPEVT = ILLEGAL_SLOT_INSTR_EXCP_CODE;
			handling = ABORTED_AND_RETRIED;
			break;
		}
		case USER_BKPOINT_TRAP_EXCP:
		{
			S->PC = S->arm->VBR + USER_BKPOINT_OFFSET;
			S->arm->EXPEVT = USER_BKPOINT_TRAP_EXCP_CODE;
			handling = COMPLETED;
			break;
		}
		case DMA_ADDRERR_EXCP:
		{
			S->PC = S->arm->VBR + DMA_ADDRERR_OFFSET;
			S->arm->EXPEVT = DMA_ADDRERR_EXCP_CODE;
			handling = COMPLETED;
			break;
		}

		default:
		{
			sfatal(E, S, "Unknown/invalid exception code");
		}
	}

	if (handling == ABORTED)
	{
		/*	Current instruction is aborted		*/
		ARMpipeflush(S);
	}
	else if (handling == ABORTED_AND_RETRIED)
	{
		ARMpipeflush(S);
		S->arm->SPC = intr->value;
	}
	else if (handling == COMPLETED)
	{
		/*	Current instruction is completed	*/
		if (S->step == ARMfaststep)
		{
			S->arm->SPC = S->PC;
		}
		else
		{
			drain_pipeline(E, S);
			S->arm->SPC = S->PC;
		}
	}

	mfree(E, intr, "Interrupt *interrupt in machine-hitachi-sh.c");

	return;

}

tuck int
ARMtake_nic_intr(Engine *E, State *S)
{
	Interrupt	*interrupt;

	if (!interruptible(S))
	{
		return -1;
	}

	/* 								*/
	/*	(PC is incremented at end of step() in pipeline.c)	*/
	/*	We need to re-exec instruction which is currently in	*/
	/*	ID when we RTE, so save that kids PC! (then do a 	*/
	/*	ifidflush()). If we are faststeping, then S->PC is 	*/
	/*	the next instr that we would place into EX and exec.	*/
	/*								*/
	/*	i.e., at this point, we are yet to execute instruction	*/
	/*	@PC (fastep/step below) so rather than setting SPC 	*/
	/*	to PC+2, we set it to PC, so that RTE executes the	*/
	/*	instr after then one after we caught the interrupt.	*/
	/*								*/
	if (S->step == ARMfaststep)
	{
		S->arm->SPC = S->PC;
	}
	else
	{
		drain_pipeline(E, S);

		/*						*/
		/*	Must do this after drain because 	*/
		/*	executed instruction during drain 	*/
		/*	might set PC (e.g. a BRA)		*/
		/*						*/
		S->arm->SPC = S->PC;
	}

	S->arm->SSR = S->arm->SR;
	S->arm->SR.BL = 1;
	S->arm->SR.MD = 1;
	S->arm->SR.RB = 1;

	S->PC = S->arm->VBR + 0x600;
	S->sleep = 0;

	interrupt = (Interrupt *)pic_intr_dequeue(E, S, S->arm->nicintrQ);
	if (interrupt == NULL)
	{
		sfatal(E, S,
		"We supposedly had an interrupt, but nothing was queued!");
	}

	/*								*/
	/*	The value field, which in the case of NIC, specifies 	*/
	/*	which  interface the interrupt was generated by. The	*/
	/*	apps. interpret exception codes offset from base as	*/
	/*	the interface number (well, you know what i mean...)	*/
	/*								*/
	switch (interrupt->type)
	{
		case NIC_RXOK_INTR:
		{
			S->arm->INTEVT = (NIC_RX_EXCP_CODE + interrupt->value);
			break;
		}

		case NIC_TXOK_INTR:
		{
			S->arm->INTEVT = (NIC_TX_EXCP_CODE + interrupt->value);
			break;
		}

		case NIC_ADDRERR_INTR:
		{
			S->arm->INTEVT = (NIC_ADDR_EXCP_CODE + interrupt->value);
			break;
		}

		case NIC_FRAMEERR_INTR:
		{
			S->arm->INTEVT = (NIC_FRAME_EXCP_CODE + interrupt->value);
			break;
		}

		case NIC_COLLSERR_INTR:
		{
			S->arm->INTEVT = (NIC_COLLS_EXCP_CODE + interrupt->value);
			break;
		}

		case NIC_CSENSEERR_INTR:
		{
			S->arm->INTEVT = (NIC_CSENSE_EXCP_CODE + interrupt->value);
			break;
		}

		case NIC_RXOVRRUNERR_INTR:
		{
			S->arm->INTEVT = (NIC_RXOVRRUN_EXCP_CODE + interrupt->value);
			break;
		}

		case NIC_RXUNDRRUNERR_INTR:
		{
			S->arm->INTEVT = (NIC_RXUNDRRUN_EXCP_CODE + interrupt->value);
			break;
		}

		case NIC_TXOVRRUNERR_INTR:
		{
			S->arm->INTEVT = (NIC_TXOVRRUN_EXCP_CODE + interrupt->value);
			break;
		}

		case NIC_TXUNDRRUNERR_INTR:
		{
			S->arm->INTEVT = (NIC_TXUNDRRUN_EXCP_CODE + interrupt->value);
			break;
		}

		case NIC_CSUMERR_INTR:
		{
			S->arm->INTEVT = (NIC_CSUM_EXCP_CODE + interrupt->value);
			break;
		}

		default:
		{
			mprint(E, S, nodeinfo, "Received interrupt type [%d]\n",
				interrupt->type);

			sfatal(E, S, "Unknown interrupt type!");
		}
	}
	mfree(E, interrupt, "Interrupt *interrupt in machine-hitachi-sh.c");

	return 0;
}

tuck int
ARMtake_batt_intr(Engine *E, State *S)
{
	if (!interruptible(S))
	{
		return -1;
	}

	if (S->step == ARMfaststep)
	{
		S->arm->SPC = S->PC;
	}
	else
	{
		drain_pipeline(E, S);

		/*						*/
		/*	Must do this after drain because 	*/
		/*	executed instruction during drain 	*/
		/*	might set PC (e.g. a BRA)		*/
		/*						*/
		S->arm->SPC = S->PC;
	}

	S->arm->SSR = S->arm->SR;
	S->arm->SR.BL = 1;
	S->arm->SR.MD = 1;
	S->arm->SR.RB = 1;
	S->arm->INTEVT = BATT_LOW_EXCP_CODE;
	S->PC = S->arm->VBR + 0x600;

	return 0;
}

void
ARMresetcpu(Engine *E, State *S)
{
	int	i;


	ARMpipeflush(S);


	S->MEMSIZE = DEFLT_MEMSIZE;
	S->MEMBASE = SUPERH_MEMBASE;
	S->MEMEND = S->MEMBASE + S->MEMSIZE;

	S->mem_r_latency = DEFAULT_MEMREAD_LATENCY;
	S->mem_w_latency = DEFAULT_MEMWRITE_LATENCY;


	memset(&S->arm->P, 0, sizeof(SuperHPipe));
	memset(&S->energyinfo, 0, sizeof(EnergyInfo));
	memset(&S->arm->R, 0, sizeof(ulong)*16);
	memset(&S->arm->R_BANK, 0, sizeof(ulong)*8);
	memset(&S->arm->SR, 0, sizeof(SuperHSREG));
	memset(&S->arm->SSR, 0, sizeof(SuperHSREG));
	memset(S->MEM, 0, S->MEMSIZE);
	memset(S->arm->B, 0, sizeof(SuperHBuses));

	/*								*/
	/*	The only the ratio of size:blocksize and assoc are	*/
	/*	significant when Cache struct is used for modeling TLB	*/
	/*								*/
	ARMtlb_init(E, S, 128, 1, 4);

	S->arm->GBR = 0;
	S->arm->VBR = SUPERH_MEMBASE;
	S->arm->MACH = 0;
	S->arm->MACL = 0;
	S->PC = SUPERH_MEMBASE;
	S->arm->PR = 0;
	S->arm->SPC = 0;
	S->pcstackheight = 0;
	S->fpstackheight = 0;


	S->TIME = E->globaltimepsec;
	S->arm->TIMER_LASTACTIVATE = 0.0;
	S->arm->TIMER_INTR_DELAY = 1E-3;
	S->dyncnt = 0;
	S->nfetched = 0;
	S->CLK = 0;
	S->ICLK = 0;
	S->cmdbuf_nbytes = 0;

	S->CYCLETIME = SUPERH_ORIG_CYCLE;
	S->VDD = SUPERH_ORIG_VDD;
	S->SVDD = 0.0;
	S->LOWVDD = S->VDD/2.0;


/*
	TODO:	set these using the power_scale routines. power_scale* should update these
		(there, not here). This also takes care of the setvdd/setfreq via sf.y
		adjustments needed.

	S->mem_r_latency
	S->mem_w_latency
	S->flash_r_latency
	S->flash_w_latency
*/

	S->voltscale_alpha = 2.0;
//BUG?
	S->voltscale_K = SUPERH_ORIG_VDD * SUPERH_ORIG_CYCLE;
	S->voltscale_Vt = 0.0;

	S->Cycletrans = 0;
	S->arm->mem_access_type = 0;

	S->arm->PTEL = 0;
	S->arm->PTEH = 0;
	S->arm->TTB = 0;
	S->arm->TEA = 0;
	S->arm->MMUCR = 0;

	S->arm->TRA = 0;
	S->arm->EXPEVT = 0;
	S->arm->INTEVT = 0;

	S->arm->ICR = 0;
	S->arm->ICRA = 0;
	S->arm->ICRB = 0;

	S->runnable = 0;
	S->sleep = 0;
	S->ustart = 0;
	S->ufinish = 0;
	S->startclk = 0;
	S->finishclk = 0;

	S->step = ARMstep;
	S->pipelined = 1;
	S->pipeshow = 0;

	S->arm->txok_intrenable_flag		= 0;
	S->arm->rxok_intrenable_flag		= 1;
	S->arm->addrerr_intrenable_flag	= 1;
	S->arm->frameerr_intrenable_flag	= 1;
	S->arm->collserr_intrenable_flag	= 1;
	S->arm->csenseerr_intrenable_flag	= 1;
	S->arm->rxovrrunerr_intrenable_flag	= 1;
	S->arm->txovrrunerr_intrenable_flag	= 1;
	S->arm->rxundrrunerr_intrenable_flag	= 1;
	S->arm->txundrrunerr_intrenable_flag	= 1;
	S->arm->csumerr_intrenable_flag	= 1;

	fault_setnodepfun(E, S, "urnd");

	if (SF_PAU_DEFINED)
	{
		pau_init(E, S, S->arm->npau);
		mprint(E, S, nodeinfo,
			"Done with pauinit, for %d PAU entries...\n",
			S->arm->npau);
	}

	for (i = OP_ADD; i <= OP_XTRCT; i++)
	{
		double reading = (R0000[i].reading1 + R0000[i].reading2)/2;

		/*							*/
		/*	Scaled current, I2 = (I1*V2*t1)/(V1*t2);	*/
		/*							*/
		S->scaledcurrents[i] =
			((reading*S->VDD*SUPERH_ORIG_CYCLE)/(SUPERH_READINGS_VDD*S->CYCLETIME))*1E-3;

		S->arm->opncycles[i] = R0000[i].ncycles;
	}

	/*	Since we've reset VDD, need to update this	*/
	E->mincycpsec = PICOSEC_MAX;
	E->maxcycpsec = 0;
	for (i = 0; i < E->nnodes; i++)
	{
		E->mincycpsec = min(E->mincycpsec, E->sp[i]->CYCLETIME);
		E->maxcycpsec = max(E->maxcycpsec, E->sp[i]->CYCLETIME);
	}

	return;
}


State *
ARMnewstate(Engine *E, double xloc, double yloc, double zloc, char *trajfilename)
{
	int	i;
	State 	*S;
	char 	*logfilename;


	S = (State *)mcalloc(E, 1, sizeof(State), "(State *)S");
	if (S == NULL)
	{
		mexit(E, "Failed to allocate memory for State *S.", -1);
	}

	S->arm = (SuperHState *)mcalloc(E, 1, sizeof(SuperHState), "S->arm");
	if (S->arm == NULL)
	{
		mexit(E, "Failed to allocate memory for S->arm.", -1);
	}

	S->MEM = (uchar *)mcalloc(E, 1, DEFLT_MEMSIZE, "(uchar *)S->MEM");
	if (S->MEM == NULL)
	{
		mexit(E, "Failed to allocate memory for S->MEM.", -1);
	}

	S->arm->B = (SuperHBuses *)mcalloc(E, 1, sizeof(SuperHBuses), "(SuperHBuses *)S->arm->B");
	if (S->arm->B == NULL)
	{
		mexit(E, "Failed to allocate memory for S->arm->B.", -1);
	}


	S->N = (Numa *)mcalloc(E, 1, sizeof(Numa), "(Numa *)S->N");
	if (S->N == NULL)
	{
		mexit(E, "Failed to allocate memory for S->N.", -1);
	}
	S->N->count = 0;

	/*	Actual entries are allocated when a region is installed		*/
	S->N->regions = (Numaregion **)mcalloc(E, MAX_NUMA_REGIONS,
		sizeof(Numaregion*), "(Numaregion **)S->N->regions");
	if (S->N->regions == NULL)
	{
		mexit(E, "Failed to allocate memory for S->N->regions.", -1);
	}


	S->Nstack = (Numa *)mcalloc(E, 1, sizeof(Numa), "(Numa *)S->Nstack");
	if (S->Nstack == NULL)
	{
		mexit(E, "Failed to allocate memory for S->Nstack.", -1);
	}
	S->Nstack->count = 0;

	/*	Actual entries are allocated when a region is installed		*/
	S->Nstack->regions = (Numaregion **)mcalloc(E, MAX_NUMA_REGIONS,
		sizeof(Numaregion*), "(Numaregion **)S->Nstack->regions");
	if (S->Nstack->regions == NULL)
	{
		mexit(E, "Failed to allocate memory for S->Nstack->regions.", -1);
	}


	S->RT = (Regtraces *)mcalloc(E, 1, sizeof(Regtraces), "(Regtraces *)S->RT");
	if (S->RT == NULL)
	{
		mexit(E, "Failed to allocate memory for S->RT.", -1);
	}
	S->RT->count = 0;

	/*	Actual entries are allocated when a region is installed		*/
	S->RT->regvts = (Regvt **)mcalloc(E, MAX_REG_TRACERS,
		sizeof(Regvt*), "(Regvt **)S->RT->regvts");
	if (S->RT->regvts == NULL)
	{
		mexit(E, "Failed to allocate memory for S->RT->regvts.", -1);
	}


	if (SF_SIMLOG)
	{
		logfilename = (char *)mcalloc(E, 1, MAX_NAMELEN*sizeof(char),
			"logfilename in machine-hitachi-sh.c");
		if (logfilename == NULL)
		{
                	mexit(E, "Failed to allocate memory for logfilename.", -1);
        	}

		msnprint(logfilename, MAX_NAMELEN, "simlog.node%d", E->nnodes);

		S->logfd = mcreate(logfilename, M_OWRITE|M_OTRUNCATE);
		mfree(E, logfilename, "char * logfilename in machine-hitachi-sh.c");

		if (S->logfd < 0)
		{
			mexit(E, "Could not open logfile for writing.", -1);
		}
	}

	E->cp = S;
	E->sp[E->nnodes] = S;
	mprint(E, NULL, siminfo, "New node created with node ID %d\n", E->nnodes);

	/*	Update the min cycle time	*/
	E->mincycpsec = PICOSEC_MAX;
	E->maxcycpsec = 0;
	for (i = 0; i < E->nnodes; i++)
	{
		E->mincycpsec = min(E->mincycpsec, E->sp[i]->CYCLETIME);
		E->maxcycpsec = max(E->maxcycpsec, E->sp[i]->CYCLETIME);
	}

	S->dumpregs = ARMdumpregs;
	S->dumpsysregs = ARMdumpsysregs;
	S->fatalaction = ARMfatalaction;
	S->stallaction = ARMstallaction;
	S->settimerintrdelay = ARMsettimerintrdelay;

	S->take_nic_intr = ARMtake_nic_intr;
	S->take_timer_intr = ARMtake_timer_intr;
	S->take_batt_intr = ARMtake_batt_intr;
	S->check_batt_intr = ARMcheck_batt_intr;
	S->check_nic_intr = ARMcheck_nic_intr;

	S->cache_init = ARMcache_init;
	S->resetcpu = ARMresetcpu;
	S->step = ARMstep;
	S->cyclestep = ARMstep;
	S->faststep = ARMfaststep;
	S->dumppipe = ARMdumppipe;
	S->pipeflush = ARMpipeflush;

	/*	Most of the device registers are SH7708 specific	*/
	S->devreadbyte = dev7708readbyte;
	S->devreadword = dev7708readword;
	S->devreadlong = dev7708readlong;
	S->devwritebyte = dev7708writebyte;
	S->devwriteword = dev7708writeword;
	S->devwritelong = dev7708writelong;
	S->split = ARMsplit;
	S->vmtranslate = ARMvmtranslate;
	S->dumptlb = ARMdumptlb;
	S->cache_deactivate = ARMcache_deactivate;
	S->cache_printstats = ARMcache_printstats;

	S->writebyte = ARMwritebyte;

	S->xloc = xloc;
	S->yloc = yloc;
	S->zloc = zloc;

	if (trajfilename != NULL)
	{
		S->trajfilename = (char *)mcalloc(E, 1, strlen(trajfilename)+1, "S->trajfilename in "SF_FILE_MACRO);
		if (S->trajfilename == nil)
		{
			mexit(E, "mcalloc failed for S->trajfilename in "SF_FILE_MACRO, -1);
		}
		strcpy(S->trajfilename, trajfilename);
	}

	S->NODE_ID = E->baseid + E->nnodes;


	/*	Must know correct number of nodes in resetcpu()		*/
	E->nnodes++;
	S->resetcpu(E, S);

	S->arm->nicintrQ = (InterruptQ *)mcalloc(E, 1, sizeof(InterruptQ),
		"InterruptQ *nicintrQ in ARMnewstate()");
	if (S->arm->nicintrQ == NULL)
	{
		mexit(E, "Failed to allocate memory for InterruptQ *nicintrQ in ARMnewstate().", -1);
	}

	S->arm->nicintrQ->hd = (Interrupt *)mcalloc(E, 1, sizeof(Interrupt),
		"Interrupt *S->arm->nicintrQ->hd in ARMnewstate()");
	S->arm->nicintrQ->tl = (Interrupt *)mcalloc(E, 1, sizeof(Interrupt),
		"Interrupt *S->arm->nicintrQ->tl in ARMnewstate()");
	if (S->arm->nicintrQ->hd == NULL || S->arm->nicintrQ->tl == NULL)
	{
		mexit(E, "Failed to allocate memory for S->arm->nicintrQ->hd | S->arm->nicintrQ->tl.", -1);
	}

	S->arm->excpQ = (InterruptQ *)mcalloc(E, 1, sizeof(InterruptQ),
		"InterruptQ *excpQ in ARMnewstate()");
	if (S->arm->excpQ == NULL)
	{
		mexit(E, "Failed to allocate memory for InterruptQ *excpQ in ARMnewstate().", -1);
	}

	S->arm->excpQ->hd = (Interrupt *)mcalloc(E, 1, sizeof(Interrupt),
		"Interrupt *S->arm->excpQ->hd in ARMnewstate()");
	S->arm->excpQ->tl = (Interrupt *)mcalloc(E, 1, sizeof(Interrupt),
		"Interrupt *S->arm->excpQ->tl in ARMnewstate()");
	if (S->arm->excpQ->hd == NULL || S->arm->excpQ->tl == NULL)
	{
		mexit(E, "Failed to allocate memory for S->arm->excpQ->hd | S->arm->excpQ->tl.", -1);
	}


	return S;
}

void
ARMsplit(Engine *E, State *S, ulong startpc, ulong stackptr, ulong argaddr, char *idstr)
{
	int		i;
	Numaregion	*tmp;


	/*								*/
	/*	Split the current CPU into two, sharing the same mem	*/
	/*								*/
	State	*N = ARMnewstate(E, S->xloc, S->yloc, S->zloc, S->trajfilename);

	mprint(E, NULL, siminfo,
		"Splitting node %d to new node %d:\n\t\tstartpc @ 0x" UHLONGFMT
		", stack @ 0x" UHLONGFMT ", arg @ 0x" UHLONGFMT "\n\n",
		S->NODE_ID, N->NODE_ID, startpc, stackptr, argaddr);

	mfree(E, N->MEM, "N->MEM in ARMsplit");
	mfree(E, N->arm->B, "N->arm->B in ARMsplit");

	N->MEM = S->MEM;
	N->arm->B = S->arm->B;

	/*								*/
	/*	Make a copy of the entire Numaregion queue, and reset 	*/
	/*	all the counters on the copy to 0. This enables us to	*/
	/*	inherit all created numaregions, but still maintain 	*/
	/*	individual counters.					*/
	/*								*/
	for (i = 0; i < S->N->count; i++)
	{
		tmp = (Numaregion *) mcalloc(E, 1, sizeof(Numaregion),
			"N->N->regions entry in machine-hitachi-sh.c");
		if (tmp == NULL)
		{
			merror(E, "mcalloc failed for N->N->regions entry in machine-hitachi-sh.c");
			return;
		}

		N->N->regions[i] = tmp;
		memcpy(N->N->regions[i], S->N->regions[i], sizeof(Numaregion));

		N->N->regions[i]->nreads = 0;
		N->N->regions[i]->nwrites = 0;
	}
	N->N->count = S->N->count;

	for (i = 0; i < S->Nstack->count; i++)
	{
		tmp = (Numaregion *) mcalloc(E, 1, sizeof(Numaregion),
			"N->Nstack->regions entry in machine-hitachi-sh.c");
		if (tmp == NULL)
		{
			merror(E, "mcalloc failed for N->Nstack->regions entry in machine-hitachi-sh.c");
			return;
		}

		N->Nstack->regions[i] = tmp;
		memcpy(N->Nstack->regions[i], S->Nstack->regions[i], sizeof(Numaregion));

		N->Nstack->regions[i]->nreads = 0;
		N->Nstack->regions[i]->nwrites = 0;
	}
	N->Nstack->count = S->Nstack->count;


	N->MEMBASE = S->MEMBASE;
	N->MEMEND = S->MEMEND;
	N->MEMSIZE = S->MEMSIZE;

	N->arm->R[12] = stackptr;
	N->arm->R[4] = argaddr;

	/*								*/
	/*	    Inherit the machine status (e.g., priv) too		*/
	/*	TODO: could memcpy the whole State *, but I haven't	*/
	/*	yet finalized how much heterogeneity in spawned 	*/
	/*	processors will exist					*/
	/*								*/
	N->arm->SR = S->arm->SR;
	N->arm->ENABLE_CLK_INTR = S->arm->ENABLE_CLK_INTR;
	N->arm->TIMER_INTR_DELAY = S->arm->TIMER_INTR_DELAY;

	strncpy(N->idstr, idstr, MAX_NAMELEN);
	N->PC = startpc;
	N->runnable = 1;
	E->cp = N;

	return;
}
