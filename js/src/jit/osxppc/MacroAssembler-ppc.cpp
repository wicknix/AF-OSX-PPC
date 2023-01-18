/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/*

   IonPower (C)2015 Contributors to TenFourFox. All rights reserved.
 
   Authors: Cameron Kaiser <classilla@floodgap.com>
   with thanks to Ben Stuhl and David Kilbridge 
   and the authors of the ARM and MIPS ports
 
 */

#include "jit/osxppc/MacroAssembler-ppc.h"

#include "mozilla/DebugOnly.h"
#include "mozilla/MathAlgorithms.h"

#include "jit/Bailouts.h"
#include "jit/BaselineFrame.h"
#include "jit/BaselineRegisters.h"
#include "jit/JitFrames.h"
#include "jit/MoveEmitter.h"

using namespace js;
using namespace jit;

using mozilla::Abs;

static const int32_t PAYLOAD_OFFSET = NUNBOX32_PAYLOAD_OFFSET;
static const int32_t TAG_OFFSET = NUNBOX32_TYPE_OFFSET;

void
MacroAssemblerPPC::convertBoolToInt32(Register src, Register dest)
{
	ispew("convertBoolToInt32(reg, reg)");
	
    // Although native PPC boolean is word-size, preserve Ion's expectations
    // by masking off the higher-order bits.
    andi_rc(dest, src, 0xff);
}

void
MacroAssemblerPPC::convertInt32ToDouble(Register src, FloatRegister dest)
{
	ispew("convertInt32ToDouble(reg, reg)");
	MOZ_ASSERT(src != tempRegister);
	
	// No GPR<->FPR moves! Use the library routine.
	dangerous_convertInt32ToDouble(src, dest,
		(dest == fpTempRegister) ? fpConversionRegister : // Baseline
			fpTempRegister);
}

void
MacroAssemblerPPC::convertInt32ToDouble(const Address &addr, FloatRegister dest)
{
	// XXX: This ass-U-mes that the address is pointing to a jsval,
	// which means that we need to convert the 32-bit word at addr+4
	// because type is at addr+0 in our superior big-endian world.
	// In all current uses the address is always 4(r1), so assert that
	// in case Mozilla one day ever fixes it, or I fix it as part of
	// something else.
	ispew("[[ convertInt32ToDouble(adr, fpr) (ASSUMING ADR IS JSVAL)");
	lwz(addressTempRegister, addr.base, addr.offset+4);
	MOZ_ASSERT(addr.offset == 4 && addr.base == stackPointerRegister);

	dangerous_convertInt32ToDouble(addressTempRegister, dest,
		(dest == fpTempRegister) ? fpConversionRegister : // Baseline
			fpTempRegister);
	ispew("   convertInt32ToDouble(adr, fpr) ]]");
}

void
MacroAssemblerPPC::convertUInt32ToDouble(Register src, FloatRegister dest)
{
	// This is almost the same as convertInt, except the zero constant
	// is 0x4330 0000 0000 0000 (not 8000), and we don't xoris the sign.
	// See also OPPCC chapter 8 p.157.
	ispew("[[ convertUInt32ToDouble(reg, fpr)");
	
	// It's possible to call this for temp registers, so use spares.
	FloatRegister fpTemp = (dest == fpTempRegister) ? fpConversionRegister
		: fpTempRegister;
	Register temp = (src == tempRegister) ? addressTempRegister 
		: tempRegister;
		
	// Build temporary frame with zero constant.
	x_lis(temp, 0x4330);
	stwu(temp, stackPointerRegister, -8); // cracked on G5
	x_lis(temp, 0x0000); // not 8000
	
	// (2nd G5 dispatch group)
	stw(src, stackPointerRegister, 4);
	// Don't flip integer value's sign; use as integer component directly.
	// stack now has 0x4330 0000 <src>
#ifdef _PPC970_
	// G5 and POWER4+ do better if the lfd and the stws aren't in the
	// same dispatch group.
	x_nop();
	x_nop();
	x_nop();
#endif

	// (3rd G5 dispatch group)
	// Load intermediate float.
	lfd(dest, stackPointerRegister, 0);
#ifdef _PPC970_
	x_nop();
	x_nop();
	x_nop();
#endif

	// (4th G5 dispatch group)
	stw(temp, stackPointerRegister, 4);
	// stack now has 0x4330 0000 0000 0000
#ifdef _PPC970_
	x_nop();
	x_nop();
	x_nop();
#endif

	// (Final G5 dispatch group)
	// Load zero float and normalize with a subtraction operation.
	lfd(fpTemp, stackPointerRegister, 0);
	fsub(dest, dest, fpTemp);
	// Destroy temporary frame.
	addi(stackPointerRegister, stackPointerRegister, 8);
	
	ispew("   convertUInt32ToDouble(reg, fpr) ]]");
}

void
MacroAssemblerPPC::convertUInt32ToFloat32(Register src, FloatRegister dest)
{
	// Since PPC FPRs are both float and double, just call the other routine.
	convertUInt32ToDouble(src, dest);
}

void
MacroAssemblerPPC::convertDoubleToFloat32(FloatRegister src, FloatRegister dest)
{
	ispew("convertDoubleToFloat32(fpr, fpr)");
	frsp(dest, src);
}

// Truncate src to 32 bits and move to dest. If not representable in 32
// bits, branch to the failure label.
void
MacroAssemblerPPC::branchTruncateDouble(FloatRegister src, Register dest,
                                         Label *fail)
{
	ispew("[[ branchTruncateDouble(fpreg, reg, l)");
	MOZ_ASSERT(src != fpTempRegister);
	MOZ_ASSERT(dest != tempRegister);
	
	// Again, no GPR<->FPR moves! So, back to the stack.
	// Turn into a fixed-point integer (i.e., truncate).
	//
	// TODO: Is fctiwz_rc with FPSCR exceptions faster than checking constants??
	// See below. 
	// XXX: It looks like it is: see FPSCR VXCVI and the CodeGenerator.

	fctiwz(fpTempRegister, src);
	// Stuff in a temporary frame (cracked on G5).
	stfdu(fpTempRegister, stackPointerRegister, -8);
	// Load this constant here (see below); this spaces the dispatch
	// group out and can be parallel. Even if the stfdu leads the dispatch
	// group, it's cracked, so two slots, and this load must use two, and
	// the lwz will not go in the branch slot.
	x_li32(tempRegister, 0x7fffffff);
	// Pull out the lower 32 bits. This is the result.
	lwz(dest, stackPointerRegister, 4);
	
	// We don't care if a truncation occurred. In fact, all we care is
	// that the integer result fits in 32 bits. Fortunately, fctiwz will
	// tip us off: if src > 2^31-1, then dest becomes 0x7fffffff, the
	// largest 32-bit positive integer. If src < -2^31, then dest becomes
	// 0x80000000, the largest 32-bit negative integer. So we just test
	// for those two values. If either value is found, fail-branch. Use
	// unsigned compares, since these are logical values.
	
	// (tempRegister was loaded already)
	cmplw(dest, tempRegister);
	// Destroy the temporary frame before testing, since we might branch.
	// The cmplw will execute in parallel for "free."
	addi(stackPointerRegister, stackPointerRegister, 8);
	bc(Equal, fail);
	
	x_li32(tempRegister, 0x80000000); // sign extends!
	cmplw(dest, tempRegister);
	bc(Equal, fail);
	ispew("  branchTruncateDouble(fpreg, reg, l) ]]");
}

// Convert src to an integer and move to dest. If the result is not representable
// as an integer (i.e., not integral, or out of range), branch.
void
MacroAssemblerPPC::convertDoubleToInt32(FloatRegister src, Register dest,
                                         Label *fail, bool negativeZeroCheck)
{
	ispew("[[ convertDoubleToInt32(fpr, reg, l, bool)");
	MOZ_ASSERT(dest != tempRegister);
	
	// Turn into a fixed-point integer (i.e., truncate). Set FX for any
	// exception (inexact or bad conversion), which becomes CR1+LT with
	// fctiwz_rc. FI is not sticky, so we need not whack it, but XX is.
	// On G5, all of these instructions are separate dispatch groups.
	// Worse still, mtfsb* requires FPSCR to be serialized, so clear as
	// few bits as possible. mtfsfi wouldn't be any more efficient here.
	mtfsb0(6);  // whack XX
	mtfsb0(23); // whack VXCVI
	mtfsb0(0);  // then whack summary FX
	
	fp2int(src, dest, false);
	
	// Test and branch if inexact (i.e., if "less than").
	bc(LessThan, fail, cr1);
	
	// If negativeZeroCheck is true, we need to also branch to the
	// failure label if the result is -0 (if false, we don't care).
        // fctiwz. will merrily convert -0 because, well, it's zero!
	if (negativeZeroCheck) {
		ispew("<< checking for negativeZero >>");
		
		// If it's not zero, don't bother.
		and__rc(tempRegister, dest, dest);
		BufferOffset nonZero = _bc(0, NonZero, cr0, LikelyB);
		
		// FP negative zero is 0x8000 0000 0000 0000 in the IEEE 754
		// standard, so test the upper 32 bits by extracting it on the
		// stack (similar to our various breakDouble iteractions). We have
		// no constant to compare against, so this is the best option.
		stfdu(src, stackPointerRegister, -8);
#ifdef _PPC970_
		x_nop();
		x_nop(); // stfdu is cracked
#endif
		lwz(tempRegister, stackPointerRegister, 0); // upper 32 bits
		and__rc(addressTempRegister, tempRegister, tempRegister);
		addi(stackPointerRegister, stackPointerRegister, 8);
		bc(NonZero, fail);
		
		bindSS(nonZero);
	}
	ispew("  convertDoubleToInt32(fpr, reg, l, bool) ]]");
}

void
MacroAssemblerPPC::convertFloat32ToInt32(FloatRegister src, Register dest,
                                          Label *fail, bool negativeZeroCheck)
{
	convertDoubleToInt32(src, dest, fail, negativeZeroCheck);
}

void
MacroAssemblerPPC::convertFloat32ToDouble(FloatRegister src, FloatRegister dest)
{
	// Nothing to do, except if they are not the same.
	ispew("convertFloat32ToDouble(fpr, fpr)");
	if (src != dest)
		fmr(dest, src);
}

void
MacroAssemblerPPC::branchTruncateFloat32(FloatRegister src, Register dest,
                                          Label *fail)
{
	branchTruncateDouble(src, dest, fail);
}

void
MacroAssemblerPPC::convertInt32ToFloat32(Register src, FloatRegister dest)
{
	convertInt32ToDouble(src, dest);
}

void
MacroAssemblerPPC::convertInt32ToFloat32(const Address &src, FloatRegister dest)
{
	convertInt32ToDouble(src, dest);
}

void
MacroAssemblerPPC::addDouble(FloatRegister src, FloatRegister dest)
{
	ispew("addDouble(fpr, fpr)");
    fadd(dest, dest, src);
}

void
MacroAssemblerPPC::subDouble(FloatRegister src, FloatRegister dest)
{
	ispew("subDouble(fpr, fpr)");
    fsub(dest, dest, src); // T = A-B (not like subf)
}

void
MacroAssemblerPPC::mulDouble(FloatRegister src, FloatRegister dest)
{
	ispew("mulDouble(fpr, fpr)");
    fmul(dest, dest, src);
}

void
MacroAssemblerPPC::divDouble(FloatRegister src, FloatRegister dest)
{
	ispew("divDouble(fpr, fpr)");
    fdiv(dest, dest, src);
}

void
MacroAssemblerPPC::negateDouble(FloatRegister reg)
{
	ispew("negateDouble(fpr)");
    fneg(reg, reg);
}

void
MacroAssemblerPPC::inc64(AbsoluteAddress dest)
{
	ispew("[[ inc64(aadr)");
	// Get the absolute address, increment its second word (big endian!),
	// and then the FIRST word if needed.
	// TODO: G5 should kick ass here, but dest might not be aligned.
	// Consider using aligned doubleword instructions if dest is aligned
	// to doubleword (ld/addi/nop*/std would work). Right now this is only
	// used infrequently in one place.

	// (First G5 dispatch group.) Load effective address.
	x_li32(addressTempRegister, (uint32_t)dest.addr); // 2 inst
	
	// Get the LSB and increment it, setting or clearing carry.
	lwz(tempRegister, addressTempRegister, 4);
	addic(tempRegister, tempRegister, 1);
	
	// (Second G5 dispatch group.) Store it.
	stw(tempRegister, addressTempRegister, 4);
#ifdef _PPC970_
	// Keep the stw and the following lwz in separate groups.
	x_nop();
	x_nop();
	x_nop();
#endif

	// (Third G5 dispatch group.) Load MSB and increment it with carry
	// by adding zero. This way we don't need to branch!
	lwz(tempRegister, addressTempRegister, 0);
	addze(tempRegister, tempRegister);
#ifdef _PPC970_
	// Force the next stw into another dispatch group.
	x_nop();
	x_nop();
#endif

	// (Final G5 dispatch group.) Store it; done.
	stw(tempRegister, addressTempRegister, 0);
	ispew("   inc64(aadr) ]]");
}

void
MacroAssemblerPPC::ma_li32(Register dest, ImmGCPtr ptr)
{
	m_buffer.ensureSpace(8); // make sure this doesn't get broken up
    writeDataRelocation(ptr);
    ma_li32Patchable(dest, Imm32(uintptr_t(ptr.value)));
}

void
MacroAssemblerPPC::ma_li32(Register dest, AbsoluteLabel *label)
{
    MOZ_ASSERT(!label->bound());
    // Thread the patch list through the unpatched address word in the
    // instruction stream.
    BufferOffset bo = m_buffer.nextOffset();
    ma_li32Patchable(dest, Imm32(label->prev()));
    label->setPrev(bo.getOffset());
}

void
MacroAssemblerPPC::ma_li32(Register dest, Imm32 imm)
{
	x_li32(dest, imm.value);
}

void
MacroAssemblerPPC::ma_li32Patchable(Register dest, Imm32 imm)
{
    m_buffer.ensureSpace(2 * sizeof(uint32_t));
    x_p_li32(dest, imm.value);
}

void
MacroAssemblerPPC::ma_li32Patchable(Register dest, ImmPtr imm)
{
    return ma_li32Patchable(dest, Imm32(int32_t(imm.value)));
}

// We use these abstracted forms of |and|,|or|,etc. to avoid all that mucking about
// with immediate quantities. rc = set Rc bit (irrelevant for andi_rc).
// Only |and| implements the Rc bit, because it's the only one where it matters
// to Ion conditionals. Raw instructions required for the remainder. Sorry. :P
void
MacroAssemblerPPC::ma_and(Register rd, Register rs, bool rc)
{
    if (rc)
    	and__rc(rd, rd, rs);
    else
    	and_(rd, rd, rs);
}

void
MacroAssemblerPPC::ma_and(Register rd, Register rs, Register rt, bool rc)
{
	if (rc)
		and__rc(rd, rs, rt);
	else
		and_(rd, rs, rt);
}

void
MacroAssemblerPPC::ma_and(Register rd, Imm32 imm)
{
    ma_and(rd, rd, imm);
}

void
MacroAssemblerPPC::ma_and(Register rd, Register rs, Imm32 imm)
{
    if (Imm16::IsInUnsignedRange(imm.value)) {
        andi_rc(rd, rs, imm.value);
    } else if (!(imm.value & 0x0000FFFF)) {
    	andis_rc(rd, rs, (imm.value >> 16));
    } else {
    	MOZ_ASSERT(rs != tempRegister);
        x_li32(tempRegister, imm.value);
        and__rc(rd, rs, tempRegister);
    }
}

void
MacroAssemblerPPC::ma_or(Register rd, Register rs)
{
    or_(rd, rd, rs);
}

void
MacroAssemblerPPC::ma_or(Register rd, Register rs, Register rt)
{
    or_(rd, rs, rt);
}

void
MacroAssemblerPPC::ma_or(Register rd, Imm32 imm)
{
    ma_or(rd, rd, imm);
}

void
MacroAssemblerPPC::ma_or(Register rd, Register rs, Imm32 imm)
{
    if (Imm16::IsInUnsignedRange(imm.value)) {
        ori(rd, rs, imm.value);
    } else if (!(imm.value && 0x0000FFFF)) {
    	oris(rd, rs, (imm.value >> 16));
    } else {
    	MOZ_ASSERT(rs != tempRegister);
        x_li32(tempRegister, imm.value);
        or_(rd, rs, tempRegister);
    }
}

void
MacroAssemblerPPC::ma_xor(Register rd, Register rs)
{
    xor_(rd, rd, rs);
}

void
MacroAssemblerPPC::ma_xor(Register rd, Register rs, Register rt)
{
    xor_(rd, rs, rt);
}

void
MacroAssemblerPPC::ma_xor(Register rd, Imm32 imm)
{
    ma_xor(rd, rd, imm);
}

void
MacroAssemblerPPC::ma_xor(Register rd, Register rs, Imm32 imm)
{
    if (Imm16::IsInUnsignedRange(imm.value)) {
        xori(rd, rs, imm.value);
    } else if (!(imm.value && 0x0000FFFF)) {
    	xoris(rd, rs, (imm.value >> 16));
    } else {
    	MOZ_ASSERT(rs != tempRegister);
        x_li32(tempRegister, imm.value);
        xor_(rd, rs, tempRegister);
    }
}

// Arithmetic.

void
MacroAssemblerPPC::ma_addu(Register rd, Register rs, Imm32 imm)
{
	MOZ_ASSERT(rs != tempRegister); // it would be li, then!
    if (Imm16::IsInSignedRange(imm.value)) {
        addi(rd, rs, imm.value);
    } else if (!(imm.value && 0x0000FFFF)) {
    	addis(rd, rs, (imm.value >> 16));
    } else {
        x_li32(tempRegister, imm.value);
        add(rd, rs, tempRegister);
    }
}

void
MacroAssemblerPPC::ma_addu(Register rd, Register rs)
{
    add(rd, rd, rs);
}

void
MacroAssemblerPPC::ma_addu(Register rd, Imm32 imm)
{
    ma_addu(rd, rd, imm);
}

// If rs + rt overflows, go to the failure label.
void
MacroAssemblerPPC::ma_addTestOverflow(Register rd, Register rs, Register rt, Label *overflow)
{
	addo(rd, rs, rt);
	bc(Overflow, overflow);
}

void
MacroAssemblerPPC::ma_addTestOverflow(Register rd, Register rs, Imm32 imm, Label *overflow)
{
	MOZ_ASSERT(rs != tempRegister);
	x_li32(tempRegister, imm.value);
	ma_addTestOverflow(rd, rs, tempRegister, overflow);
}

// Subtract.
// MIPS operand order is T = A - B, whereas in PPC it's T = B - A.
// Keep MIPS order here for simplicity at the MacroAssembler level.
void
MacroAssemblerPPC::ma_subu(Register rd, Register rs, Register rt)
{
    subf(rd, rt, rs); //as_subu(rd, rs, rt);
}

void
MacroAssemblerPPC::ma_subu(Register rd, Register rs, Imm32 imm)
{
	MOZ_ASSERT(rs != tempRegister);
    if (Imm16::IsInSignedRange(-imm.value)) {
        addi(rd, rs, -imm.value);
    } else {
        x_li32(tempRegister, imm.value);
        subf(rd, tempRegister, rs); //as_subu(rd, rs, ScratchRegister);
    }
}

void
MacroAssemblerPPC::ma_subu(Register rd, Imm32 imm)
{
    ma_subu(rd, rd, imm);
}

void
MacroAssemblerPPC::ma_subTestOverflow(Register rd, Register rs, Register rt, Label *overflow)
{
    subfo(rd, rt, rs); //ma_subu(rd, rs, rt);
    bc(Overflow, overflow);
}

void
MacroAssemblerPPC::ma_subTestOverflow(Register rd, Register rs, Imm32 imm, Label *overflow)
{
    if (imm.value != INT32_MIN) {
        ma_addTestOverflow(rd, rs, Imm32(-imm.value), overflow);
    } else {
    	MOZ_ASSERT(rs != tempRegister);
        x_li32(tempRegister, imm.value);
        ma_subTestOverflow(rd, rs, tempRegister, overflow);
    }
}

void
MacroAssemblerPPC::ma_mult(Register rs, Imm32 imm)
{
	if (Imm16::IsInSignedRange(imm.value)) {
		if (rs != tempRegister &&
				(imm.value == 3 || imm.value == 5 || imm.value == 10 || imm.value == 12)) {
			// Ensure rs is a different register for *3 and *5.
			// This can save an entire cycle.
			// (Hey, on a G3, that could really mean something!)
			x_mr(tempRegister, rs);
			x_sr_mulli(rs, tempRegister, imm.value);
		} else {
			x_sr_mulli(rs, rs, imm.value);
		}
	} else {
		MOZ_ASSERT(rs != tempRegister);
		x_li32(tempRegister, imm.value);
		mullw(rs, rs, tempRegister);
	}
}

void
MacroAssemblerPPC::ma_mul_branch_overflow(Register rd, Register rs, Register rt, Label *overflow)
{
	mullwo(rd, rs, rt);
	bc(Overflow, overflow);
}

void
MacroAssemblerPPC::ma_mul_branch_overflow(Register rd, Register rs, Imm32 imm, Label *overflow)
{
    x_li32(tempRegister, imm.value);
    ma_mul_branch_overflow(rd, rs, tempRegister, overflow);
}

void
MacroAssemblerPPC::ma_div_branch_overflow(Register rd, Register rs, Register rt, Label *overflow)
{
	// Save the original multiplier just in case.
	Register orreg = rt;
	Register prreg = rs;
	if (rd == rt) {
		MOZ_ASSERT(rt != addressTempRegister);
		x_mr(addressTempRegister, rt);
		orreg = addressTempRegister;
	}
	if (rd == rs) {
		MOZ_ASSERT(rs != tempRegister);
		x_mr(tempRegister, rs);
		prreg = tempRegister;
	}
	// MIPS: LO = rs/rt
	divwo(rd, rs, rt);
	bc(Overflow, overflow);
	// If the remainder is non-zero, branch to overflow as well.
	MOZ_ASSERT(rs != tempRegister);
	MOZ_ASSERT(rd != tempRegister);
	// This must be 32-bit representable or the divwo would have already failed.
	mullw(addressTempRegister, rd, orreg);
	subf_rc(tempRegister, addressTempRegister, prreg);
	bc(NonZero, overflow);
}

void
MacroAssemblerPPC::ma_div_branch_overflow(Register rd, Register rs, Imm32 imm, Label *overflow)
{
    x_li32(tempRegister, imm.value);
    ma_div_branch_overflow(rd, rs, tempRegister, overflow);
}

// Memory.

void
MacroAssemblerPPC::ma_load(Register dest, Address address,
                            LoadStoreSize size, LoadStoreExtension extension, bool swapped)
{
    bool indexed = false;

    // All byte-swap instructions are zero extension and halfword- or word-sized.
    // Warning: does not cover 64-bit byte swapping instructions, if ever implemented.
    MOZ_ASSERT_IF(swapped, ((size == SizeHalfWord && ZeroExtend == extension) || size == SizeWord));

    if (!Imm16::IsInSignedRange(address.offset) || swapped) { // All byte-swap instructions are indexed.
    	// Need indexed instructions. Put the offset into temp.
    	MOZ_ASSERT(address.base != tempRegister);
        x_li32(tempRegister, address.offset);
        indexed = true;
    }

    switch (size) {
      case SizeByte:
      	if (indexed)
      		lbzx(dest, address.base, tempRegister);
      	else
      		lbz(dest, address.base, address.offset);
        if (ZeroExtend != extension)
        	extsb(dest, dest);
        break;
      case SizeHalfWord:
      	// We can use lha for sign extension and save an instruction.
      	if (indexed && swapped) // always ZeroExtend
      		lhbrx(dest, address.base, tempRegister);
      	else if (indexed && extension == ZeroExtend) // indexed, not swapped, zero extension
      		lhzx(dest, address.base, tempRegister);
      	else if (indexed) // indexed, not swapped, sign extension
      		lhax(dest, address.base, tempRegister);
      	else if (extension == ZeroExtend) // not indexed, not swapped, zero extension
      		lhz(dest, address.base, address.offset);
      	else // not swapped, not indexed, not zero extension
      		lha(dest, address.base, address.offset);
        break;
      case SizeWord:
      	if (indexed && swapped)
      		lwbrx(dest, address.base, tempRegister);
      	else if (indexed)
      		lwzx(dest, address.base, tempRegister);
      	else
      		lwz(dest, address.base, address.offset);
        break;
      default:
        MOZ_CRASH("Invalid argument for ma_load");
    }
}

void
MacroAssemblerPPC::ma_load(Register dest, const BaseIndex &src,
                            LoadStoreSize size, LoadStoreExtension extension, bool swapped)
{
    computeScaledAddress(src, addressTempRegister);
    ma_load(dest, Address(addressTempRegister, src.offset), size, extension, swapped);
}

void
MacroAssemblerPPC::ma_store(Register data, Address address, LoadStoreSize size,
                             LoadStoreExtension extension)
{
    // data can be tempRegister sometimes.
    Register scratch = (data == tempRegister) ? addressTempRegister : tempRegister;
    bool indexed = false;

    if (!Imm16::IsInSignedRange(address.offset)) {
    	// Need indexed instructions. Put the offset into a temp register.
    	MOZ_ASSERT(address.base != scratch);
        x_li32(scratch, address.offset);
        indexed = true;
    }

    switch (size) {
      case SizeByte:
      	if (indexed)
      		stbx(data, address.base, scratch);
      	else
        	stb(data, address.base, address.offset);
        break;
      case SizeHalfWord:
      	if (indexed)
      		sthx(data, address.base, scratch);
      	else
        	sth(data, address.base, address.offset);
        break;
      case SizeWord:
      	if (indexed)
      		stwx(data, address.base, scratch);
      	else
        	stw(data, address.base, address.offset);
        break;
      default:
        MOZ_CRASH("Invalid argument for ma_store");
    }
}

void
MacroAssemblerPPC::ma_store(Register data, const BaseIndex &dest,
                             LoadStoreSize size, LoadStoreExtension extension)
{
    computeScaledAddress(dest, addressTempRegister);
    ma_store(data, Address(addressTempRegister, dest.offset), size, extension);
}

void
MacroAssemblerPPC::ma_store(Imm32 imm, const BaseIndex &dest,
                             LoadStoreSize size, LoadStoreExtension extension)
{
    computeEffectiveAddress(dest, addressTempRegister);
    x_li32(tempRegister, imm.value);
    // We can use addressTempRegister as the base because we know the offset
    // will fit (so ma_store won't need it as another temporary register).
    ma_store(tempRegister, Address(addressTempRegister, 0), size, extension);
}

void
MacroAssemblerPPC::computeScaledAddress(const BaseIndex &address, Register dest)
{
    int32_t shift = Imm32::ShiftOf(address.scale).value;
    if (shift) {
    	MOZ_ASSERT(address.index != tempRegister);
    	MOZ_ASSERT(address.base != tempRegister);
        x_slwi(tempRegister, address.index, shift);
        add(dest, address.base, tempRegister);
    } else {
        add(dest, address.base, address.index);
    }
}

// Shortcuts for when we know we're transferring 32 bits of data.
// Held over from MIPS to make the code translation smoother, but mostly
// extraneous.
void
MacroAssemblerPPC::ma_lw(Register data, Address address)
{
    ma_load(data, address, SizeWord);
}

void
MacroAssemblerPPC::ma_sw(Register data, Address address)
{
    ma_store(data, address, SizeWord);
}

void
MacroAssemblerPPC::ma_sw(Imm32 imm, Address address)
{
    MOZ_ASSERT(address.base != tempRegister);
    x_li32(tempRegister, imm.value);

    if (Imm16::IsInSignedRange(address.offset)) {
        stw(tempRegister, address.base, address.offset);
    } else {
        MOZ_ASSERT(address.base != addressTempRegister);

        x_li32(addressTempRegister, address.offset);
        stwx(tempRegister, address.base, addressTempRegister);
    }
}

void
MacroAssemblerPPC::ma_sw(Register data, BaseIndex &address)
{
    ma_store(data, address, SizeWord);
}

// These are the cheater pops and pushes that handle "fake LR."
void
MacroAssemblerPPC::ma_pop(Register r)
{
	Register rr = (r == lr) ? tempRegister : r;
	lwz(rr, stackPointerRegister, 0);
	if (r == lr) {
		// Naughty!
		x_mtlr(rr); // new dispatch group on G5
	}
	// XXX. Not needed if r is sp?
	addi(stackPointerRegister, stackPointerRegister, 4);
}

void
MacroAssemblerPPC::ma_push(Register r)
{
	Register rr = r;
	if (r == lr) {
		// Naughty!
		x_mflr(tempRegister); // new dispatch group on G5
		rr = tempRegister;
	}
	stwu(rr, stackPointerRegister, -4); // cracked on G5
}

BufferOffset
MacroAssemblerPPC::_b(int32_t off, BranchAddressType bat, LinkBit lb)
{
	return Assembler::b(off, bat, lb);
}
BufferOffset
MacroAssemblerPPC::_bc(int16_t off, Assembler::Condition cond, CRegisterID cr, LikelyBit lkb, LinkBit lb)
{
	return Assembler::bc(off, cond, cr, lkb, lb);
}
BufferOffset
MacroAssemblerPPC::_bc(int16_t off, Assembler::DoubleCondition cond, CRegisterID cr, LikelyBit lkb, LinkBit lb)
{
	return Assembler::bc(off, cond, cr, lkb, lb);
}

// IF YOU CHANGE THIS: change PPC_B_STANZA_LENGTH in MacroAssembler-ppc.h
void
MacroAssemblerPPC::b(Label *l, bool is_call)
{
	// For future expansion.
	MOZ_ASSERT(!is_call);
	
	// Our calls are designed so that the return address is always after the bl/bctrl.
	// Thus, we can always have the option of a short call.
	m_buffer.ensureSpace(PPC_B_STANZA_LENGTH + 4);
	
	if (l->bound()) {
		// Label is already bound, emit an appropriate stanza.
		// This means that the code was already generated in this buffer.
		BufferOffset bo = nextOffset();
		int32_t offs = l->offset();
		int32_t moffs = bo.getOffset();
		MOZ_ASSERT(moffs >= offs);
		
		moffs = offs - (moffs + PPC_B_STANZA_LENGTH - 4);
		if (JOffImm26::IsInRange(moffs)) {
			// Emit a four instruction short jump.
			x_nop();
			x_nop();
			x_nop();
			Assembler::b(moffs, RelativeBranch, (is_call) ? LinkB : DontLinkB);
		} else {
			// Emit a four instruction long jump.
			addLongJump(nextOffset());
            // The offset could be short, so make it patchable.
			x_p_li32(tempRegister, offs);
			x_mtctr(r0);
			bctr((is_call) ? LinkB : DontLinkB);
		}
		MOZ_ASSERT((nextOffset().getOffset() - bo.getOffset()) == PPC_B_STANZA_LENGTH);
		return;
	}

	// Second word holds a pointer to the next branch in label's chain.
	uint32_t nextInChain = l->used() ? l->offset() : LabelBase::INVALID_OFFSET;

	BufferOffset bo = x_trap();
#if DEBUG
	JitSpew(JitSpew_Codegen, "-------- --- %08x <<< b offset", nextInChain);
#endif
	writeInst(nextInChain);
	l->use(bo.getOffset());
    x_nop(); // Spacer
    
    Assembler::b(4, RelativeBranch, (is_call) ? LinkB : DontLinkB); // Will be patched by bind()
    MOZ_ASSERT((nextOffset().getOffset() - bo.getOffset()) == PPC_B_STANZA_LENGTH);
}
void
MacroAssemblerPPC::bl(Label *l) // OMIGOSH I'M LAZY
{
	b(l, true);
}

// IF YOU CHANGE THIS: change PPC_BC_STANZA_LENGTH in MacroAssembler-ppc.h
void
MacroAssemblerPPC::bc(uint32_t rawcond, Label *l)
{
	m_buffer.ensureSpace(PPC_BC_STANZA_LENGTH + 4); // paranoia strikes deep in the heartland
	
	if (l->bound()) {
		// Label is already bound, emit an appropriate stanza.
		// This means that the code was already generated in this buffer.
		BufferOffset bo = nextOffset();
		int32_t offs = l->offset();
		int32_t moffs = bo.getOffset();
		MOZ_ASSERT(moffs >= offs);
		
		moffs = offs - (moffs + PPC_BC_STANZA_LENGTH - 4);
		if (BOffImm16::IsInSignedRange(moffs)) {
			// Emit a five instruction short conditional.
			x_nop();
			x_nop();
			x_nop();
			x_nop();
			Assembler::bc(moffs, rawcond);
		} else {
			// Emit a five instruction long conditional, inverting the bit sense for the first branch.
			MOZ_ASSERT(!(rawcond & 1)); // Always and similar conditions not yet handled
			rawcond ^= 8; // flip 00x00 (see OPPCC p.372)
			
			Assembler::bc(PPC_BC_STANZA_LENGTH, rawcond);
			addLongJump(nextOffset());
			// The offset could be "short," so make it patchable.
			x_p_li32(tempRegister, offs);
			x_mtctr(tempRegister);
			bctr();
		}
		MOZ_ASSERT((nextOffset().getOffset() - bo.getOffset()) == PPC_BC_STANZA_LENGTH);
		return;
	}

	// Second word holds a pointer to the next branch in label's chain.
	uint32_t nextInChain = l->used() ? l->offset() : LabelBase::INVALID_OFFSET;

	BufferOffset bo = x_trap();
#if DEBUG
	JitSpew(JitSpew_Codegen, "-------- --- %08x <<< bc %04x offset", nextInChain, rawcond);
#endif
	writeInst(nextInChain);
	l->use(bo.getOffset());
	
	// Leave sufficient space in case this gets patched long. We need two more nops.
    x_nop();
    x_nop();
    
	// Finally, emit a dummy bc with the right bits set so that bind() will fix it later.
    Assembler::bc(4, rawcond);
    MOZ_ASSERT((nextOffset().getOffset() - bo.getOffset()) == PPC_BC_STANZA_LENGTH);
}
void
MacroAssemblerPPC::bc(Condition cond, Label *l, CRegisterID cr)
{
	bc(computeConditionCode(cond, cr), l);
}
void
MacroAssemblerPPC::bc(DoubleCondition cond, Label *l, CRegisterID cr)
{
	bc(computeConditionCode(cond, cr), l);
}

// Floating point instructions.
void
MacroAssemblerPPC::ma_lis(FloatRegister dest, float value)
{
	// Fugly.
    uint32_t fasi = mozilla::BitwiseCast<uint32_t>(value);

    x_li32(tempRegister, fasi); // probably two instructions
    // Remember, no GPR<->FPR moves, so we have to put this on the stack.
    stwu(tempRegister, stackPointerRegister, -4); // cracked
#if _PPC970_
	// Keep the stwu and the lfs separate in case the stwu ends up leading a dispatch group.
	x_nop();
	x_nop();
#endif
	lfs(dest, stackPointerRegister, 0);
	addi(stackPointerRegister, stackPointerRegister, 4);
}

void
MacroAssemblerPPC::ma_lid(FloatRegister dest, double value)
{
    struct DoubleStruct {
        uint32_t hi;
        uint32_t lo;
    } ;
    DoubleStruct intStruct = mozilla::BitwiseCast<DoubleStruct>(value);
    
    // Like ma_lis, but with two pieces. Push lo, then hi (endianness!).
    addi(stackPointerRegister, stackPointerRegister, -8);
    x_li32(tempRegister, intStruct.hi);
    x_li32(addressTempRegister, intStruct.lo);
    stw(tempRegister, stackPointerRegister, 0);
    stw(addressTempRegister, stackPointerRegister, 4);
#if _PPC970_
	// Hard to say where they will end up, so assume the worst on instruction timing.
	x_nop();
	x_nop();
	x_nop();
#endif
	lfd(dest, stackPointerRegister, 0);
	addi(stackPointerRegister, stackPointerRegister, 8);
}

void
MacroAssemblerPPC::ma_liNegZero(FloatRegister dest)
{
	ma_lid(dest, -0.0);
}

void
MacroAssemblerPPC::ma_mv(FloatRegister src, ValueOperand dest)
{
	// Remember: big endian!
	stfdu(src, stackPointerRegister, -8); // cracked on G5
#if _PPC970_
	x_nop();
	x_nop();
#endif
	lwz(dest.typeReg(), stackPointerRegister, 0);
	lwz(dest.payloadReg(), stackPointerRegister, 4);
	addi(stackPointerRegister, stackPointerRegister, 8);
}

void
MacroAssemblerPPC::ma_mv(ValueOperand src, FloatRegister dest)
{
	addi(stackPointerRegister, stackPointerRegister, -8);
	// Endianness!
	stw(src.typeReg(), stackPointerRegister, 0);
	stw(src.payloadReg(), stackPointerRegister, 4);
#if _PPC970_
	// Hard to say where they will end up, so assume the worst on instruction timing.
	x_nop();
	x_nop();
	x_nop();
#endif
	lfd(dest, stackPointerRegister, 0);
	addi(stackPointerRegister, stackPointerRegister, 8);
}

void
MacroAssemblerPPC::ma_ls(FloatRegister ft, Address address)
{
    if (Imm16::IsInSignedRange(address.offset)) {
        lfs(ft, address.base, address.offset);
    } else {
        MOZ_ASSERT(address.base != addressTempRegister);
        x_li32(addressTempRegister, address.offset);
        lfsx(ft, address.base, addressTempRegister);
    }
}

void
MacroAssemblerPPC::ma_ld(FloatRegister ft, Address address)
{
    if (Imm16::IsInSignedRange(address.offset)) {
        lfd(ft, address.base, address.offset);
    } else {
        MOZ_ASSERT(address.base != addressTempRegister);
        x_li32(addressTempRegister, address.offset);
        lfdx(ft, address.base, addressTempRegister);
    }
}

void
MacroAssemblerPPC::ma_sd(FloatRegister ft, Address address)
{
    if (Imm16::IsInSignedRange(address.offset)) {
        stfd(ft, address.base, address.offset);
    } else {
        MOZ_ASSERT(address.base != tempRegister);
        x_li32(tempRegister, address.offset);
        stfdx(ft, address.base, tempRegister);
    }
}

void
MacroAssemblerPPC::ma_sd(FloatRegister ft, BaseIndex address)
{
    computeScaledAddress(address, addressTempRegister);
    ma_sd(ft, Address(addressTempRegister, address.offset));
}

void
MacroAssemblerPPC::ma_ss(FloatRegister ft, Address address)
{
    if (Imm16::IsInSignedRange(address.offset)) {
        stfs(ft, address.base, address.offset);
    } else {
        MOZ_ASSERT(address.base != tempRegister);
        x_li32(tempRegister, address.offset);
        stfsx(ft, address.base, tempRegister);
    }
}

void
MacroAssemblerPPC::ma_ss(FloatRegister ft, BaseIndex address)
{
    MOZ_ASSERT(address.base != addressTempRegister);
    computeScaledAddress(address, addressTempRegister);
    ma_ss(ft, Address(addressTempRegister, address.offset));
}

void
MacroAssemblerPPC::ma_pop(FloatRegister fs)
{
	// Always double precision
	lfd(fs, stackPointerRegister, 0);
	addi(stackPointerRegister, stackPointerRegister, 8);
}

void
MacroAssemblerPPC::ma_push(FloatRegister fs)
{
	// Always double precision
	stfdu(fs, stackPointerRegister, -8); // cracked on G5
}

Assembler::Condition
MacroAssemblerPPC::ma_cmp(Register lhs, Register rhs, Assembler::Condition c)
{
	// Replaces cmp32.
	// DO NOT CALL computeConditionCode here. We don't have enough context.
	// Work with the raw condition only.
	
	MOZ_ASSERT(!(c & ConditionOnlyXER) && c != Always);
	
	if ((c == Zero || c == NonZero || c == Signed || c == NotSigned) && (lhs == rhs)) {
		// If same register, equality may cause this to pass, so we bit test or explicitly
		// compare to immediate zero for these special cases.
		// If different registers, invariably the rhs is already zero, so we
		// simply allow it to be interpreted as Equal/NotEqual/LessThan etc. as usual.
		// Usually this comes from ma_cmp(reg, 0, c) below.
		cmpwi(lhs, 0);
	} else if (PPC_USE_UNSIGNED_COMPARE(c)) {
		cmplw(lhs, rhs);
	} else {
		cmpw(lhs, rhs);
	}
	// For future expansion.
	return c;
}

// XXX: Consider a version for ImmTag/ImmType that is always unsigned.
Assembler::Condition
MacroAssemblerPPC::ma_cmp(Register lhs, Imm32 rhs, Condition c)
{
	MOZ_ASSERT(!(c & ConditionOnlyXER) && c != Always);
	
	// Because lhs could be either tempRegister or addressTempRegister
	// depending on what's loading it, we have to handle both situations.
	// (Damn r0 restrictions.)
	Register t = (lhs == tempRegister) ? addressTempRegister : tempRegister;
	
	if (PPC_USE_UNSIGNED_COMPARE(c)) {
		if (!Imm16::IsInUnsignedRange(rhs.value)) {	
			x_li32(t, rhs.value);
			return ma_cmp(lhs, t, c);
		}
		cmplwi(lhs, rhs.value);
	} else {
		if (!Imm16::IsInSignedRange(rhs.value)) {
			x_li32(t, rhs.value); // sign extends
			return ma_cmp(lhs, t, c);
		}
		cmpwi(lhs, rhs.value);
	}
	// For future expansion.
	return c;
}

Assembler::Condition
MacroAssemblerPPC::ma_cmp(Register lhs, Address rhs, Assembler::Condition c)
{
	Register t = (lhs == tempRegister) ? addressTempRegister : tempRegister;
	ma_lw(t, rhs);
	return ma_cmp(lhs, t, c);
}
Assembler::Condition
MacroAssemblerPPC::ma_cmp(Register lhs, ImmPtr rhs, Assembler::Condition c)
{
	return ma_cmp(lhs, Imm32((uint32_t)rhs.value), c);
}
Assembler::Condition
MacroAssemblerPPC::ma_cmp(Register lhs, ImmGCPtr rhs, Assembler::Condition c)
{
	// ImmGCPtrs must always be patchable, so optimization is not helpful.
	Register t = (lhs == tempRegister) ? addressTempRegister : tempRegister;
	ma_li32(t, rhs);
	return ma_cmp(lhs, t, c);
}
Assembler::Condition
MacroAssemblerPPC::ma_cmp(Address lhs, Imm32 rhs, Assembler::Condition c)
{
	ma_lw(addressTempRegister, lhs);
	return ma_cmp(addressTempRegister, rhs, c);
}
Assembler::Condition
MacroAssemblerPPC::ma_cmp(Address lhs, Register rhs, Assembler::Condition c)
{
	Register t = (rhs == tempRegister) ? addressTempRegister : tempRegister;
	ma_lw(t, lhs);
	return ma_cmp(t, rhs, c);
}
Assembler::Condition
MacroAssemblerPPC::ma_cmp(Address lhs, ImmGCPtr rhs, Assembler::Condition c)
{
	ma_lw(addressTempRegister, lhs);
	ma_li32(tempRegister, rhs); // MUST BE PATCHABLE
	return ma_cmp(addressTempRegister, tempRegister, c);
}

Assembler::DoubleCondition
MacroAssemblerPPC::ma_fcmp(FloatRegister lhs, FloatRegister rhs, Assembler::DoubleCondition c)
{
	fcmpu(lhs, rhs);
	// For future expansion.
	return c;
}

// In the below, CR0 is assumed.
void
MacroAssemblerPPC::ma_cr_set_slow(Assembler::Condition c, Register dest)
{
	// If the result of the condition is true, set dest to 1; else
	// set it to zero.
	//
	// General case, which is branchy and sucks.

	x_li(dest, 1);
	BufferOffset end = _bc(0, c);
	x_li(dest, 0);
	
	bindSS(end);
}
void
MacroAssemblerPPC::ma_cr_set_slow(Assembler::DoubleCondition c, Register dest)
{
	// If the result of the condition is true, set dest to 1; else
	// set it to zero.
	// Almost exactly the same, just for doubles.
	//
	// General case, which is branchy and sucks.

	x_li(dest, 1);
	BufferOffset end = _bc(0, c);
	x_li(dest, 0);
	
        bindSS(end);
}
void
MacroAssemblerPPC::ma_cr_set(Assembler::Condition cond, Register dest)
{
	// Fast paths.
	// Extract the relevant bits from CR0 as int.
	if (cond == Assembler::Equal) {
		MFCR0(r0);
		rlwinm(dest, r0, 3, 31, 31); // get CR0[EQ]
	} else if (cond == Assembler::NotEqual) {
		// Same, but inverted with the xori (flip the bit).
		MFCR0(r0);
		rlwinm(r0, r0, 3, 31, 31); // get CR0[EQ]
		xori(dest, r0, 1); // flip sign into the payload reg
	} else if (cond == Assembler::GreaterThan) {
		MFCR0(r0);
		rlwinm(dest, r0, 2, 31, 31); // get CR0[GT]
	} else if (cond == Assembler::LessThanOrEqual) {
		// Inverse (not reverse).
		MFCR0(r0);
		rlwinm(r0, r0, 2, 31, 31); // get CR0[GT]
		xori(dest, r0, 1); // flip sign into the payload reg
	} else if (cond == Assembler::LessThan) {
		MFCR0(r0);
		rlwinm(dest, r0, 1, 31, 31); // get CR0[LT]
	} else if (cond == Assembler::GreaterThanOrEqual) {
		MFCR0(r0);
		rlwinm(r0, r0, 1, 31, 31); // get CR0[LT]
		xori(dest, r0, 1); // flip sign into the payload reg
	} else {
		// Use the branched version to cover other things.
		ma_cr_set_slow(cond, dest);
	}
}
void
MacroAssemblerPPC::ma_cr_set(Assembler::DoubleCondition cond, Register dest)
{
	bool isUnordered;

	// Check for simple ordered/unordered before checking synthetic codes.
	if (cond == Assembler::DoubleUnordered) {
		MFCR0(r0);
		rlwinm(dest, r0, 4, 31, 31); // get CR0[FU]. FU! FUUUUUUUUUU-
	} else if (cond == Assembler::DoubleOrdered) {
		// Same, but with the xori (flip the bit).
		MFCR0(r0);
		rlwinm(r0, r0, 4, 31, 31);
		xori(dest, r0, 1); // flip sign into the payload reg
	} else {
		// This is a synthetic condition code.
		// Extract it into the condition and whether it's "OrUnordered."
		const uint8_t fuBit = crBit(cr0, Assembler::DoubleUnordered);
		const uint8_t condBit = crBit(cr0, cond);
		isUnordered = (cond & Assembler::DoubleUnordered) ? true : false;
		Assembler::DoubleCondition baseDCond = (Assembler::DoubleCondition)((isUnordered) ?
			(cond & ~Assembler::DoubleUnordered) : cond);

		// Fast paths.
		if (baseDCond == Assembler::DoubleEqual) {
			if (isUnordered) cror(condBit, fuBit, condBit);
			MFCR0(r0);
			rlwinm(dest, r0, 3, 31, 31); // get CR0[FE]
		} else if (baseDCond == Assembler::DoubleNotEqual) {
			// Same, but inverted with the xori (flip the bit).
			if (isUnordered)
				// Flip FU and AND it with condBit.
				crandc(condBit, condBit, fuBit);
			MFCR0(r0);
			rlwinm(r0, r0, 3, 31, 31); // get CR0[FE]
			xori(dest, r0, 1); // flip sign into the payload reg
		} else if (baseDCond == Assembler::DoubleGreaterThan) {
			if (isUnordered) cror(condBit, fuBit, condBit);
			MFCR0(r0);
			rlwinm(dest, r0, 2, 31, 31); // get CR0[FG]
		} else if (baseDCond == Assembler::DoubleLessThanOrEqual) {
			// Inverse (not reverse).
			if (isUnordered) crandc(condBit, condBit, fuBit);
			MFCR0(r0);
			rlwinm(r0, r0, 2, 31, 31); // get CR0[FG]
			xori(dest, r0, 1); // flip sign into the payload reg
		} else if (baseDCond == Assembler::DoubleLessThan) {
			if (isUnordered) cror(condBit, fuBit, condBit);
			MFCR0(r0);
			rlwinm(dest, r0, 1, 31, 31); // get CR0[FL]
		} else if (baseDCond == Assembler::DoubleGreaterThanOrEqual) {
			// Inverse (not reverse).
			if (isUnordered) crandc(condBit, condBit, fuBit);
			MFCR0(r0);
			rlwinm(r0, r0, 1, 31, 31); // get CR0[FL]
			xori(dest, r0, 1); // flip sign into the payload reg
		} else {
			// Use the branched version to cover other things.
			ma_cr_set_slow(cond, dest);
		}
	}
}

void
MacroAssemblerPPC::ma_cmp_set(Assembler::Condition cond, Register lhs, Register rhs, Register dest) {
	// Fast paths, PowerPC Compiler Writer's Guide, Appendix D et al. These
	// avoid use of CR, which could be slow (on G5, mfcr is microcoded).
	// Due to possibly constrained register usage, we don't use the optimals,
	// especially since we may be called with at least one temporary register.
	// TODO: Add subfe and subfze to support unsigned Above/Below, though
	// these are probably used a lot less.
	
	ispew("ma_cmp_set(cond, reg, reg, reg)");
	MOZ_ASSERT(lhs != tempRegister);
	MOZ_ASSERT(rhs != tempRegister);
	MOZ_ASSERT(dest != tempRegister);
	
	if (cond == Assembler::Equal) {
		subf(r0, rhs, lhs); // p.141
		cntlzw(r0, r0);
		x_srwi(dest, r0, 5);
	} else if (cond == Assembler::NotEqual) {
		subf(r0, lhs, rhs); // p.58
		subf(dest, rhs, lhs);
		or_(dest, dest, r0);
		rlwinm(dest, dest, 1, 31, 31);
	} else if (cond == Assembler::LessThan) { // SIGNED
		subfc(r0, rhs, lhs); // p.200
		eqv(dest, rhs, lhs);
		x_srwi(r0, dest, 31);
		addze(dest, r0);
		rlwinm(dest, dest, 0, 31, 31);
	} else if (cond == Assembler::GreaterThan) { // SIGNED
		// Reverse (not inverse).
		subfc(r0, lhs, rhs);
		eqv(dest, lhs, rhs);
		x_srwi(r0, dest, 31);
		addze(dest, r0);
		rlwinm(dest, dest, 0, 31, 31);
	} else if (cond == Assembler::LessThanOrEqual) { // SIGNED
		// We need to recruit the emergency temporary register for the next two tests
		// in case r12 is in use.
		x_srwi(r0, lhs, 31); // p.200
		srawi(emergencyTempRegister, rhs, 31);
		subfc(dest, lhs, rhs); // We can clobber dest here.
		adde(dest, emergencyTempRegister, r0);
	} else if (cond == Assembler::GreaterThanOrEqual) { // SIGNED
		// Reverse (not inverse).
		x_srwi(r0, rhs, 31);
		srawi(emergencyTempRegister, lhs, 31);
		subfc(dest, rhs, lhs); // We can clobber dest here.
		adde(dest, emergencyTempRegister, r0);
	} else {
		// Use the branched version as a slow path for any condition.
		ma_cr_set(ma_cmp(lhs, rhs, cond), dest);
	}
}

void
MacroAssemblerPPC::ma_cmp_set(Assembler::Condition cond, Address lhs, ImmPtr rhs, Register dest) {
	MOZ_ASSERT(lhs.base != tempRegister);
	MOZ_ASSERT(lhs.base != addressTempRegister);

	// Damn. Recruit a temporary register and spill it. r3 is good enough.
	stwu(r3, stackPointerRegister, -4);
	ma_lw(r3, lhs);
	x_li32(addressTempRegister, (uint32_t)rhs.value);
	ma_cmp_set(cond, r3, addressTempRegister, dest);
	lwz(r3, stackPointerRegister, 0);
	addi(stackPointerRegister, stackPointerRegister, 4);
}

void
MacroAssemblerPPC::ma_cmp_set(Assembler::Condition cond, Register lhs, Imm32 rhs, Register dest) {
	MOZ_ASSERT(lhs != tempRegister);

	x_li32(addressTempRegister, rhs.value);
	ma_cmp_set(cond, lhs, addressTempRegister, dest);
}
void
MacroAssemblerPPC::ma_cmp_set(Assembler::Condition cond, Register lhs, ImmPtr rhs, Register dest) {
	MOZ_ASSERT(lhs != addressTempRegister);

	x_li32(addressTempRegister, (uint32_t)rhs.value);
	ma_cmp_set(cond, lhs, addressTempRegister, dest);
}
void
MacroAssemblerPPC::ma_cmp_set(Assembler::Condition cond, Register lhs, Address rhs, Register dest) {
	MOZ_ASSERT(lhs != tempRegister);
	MOZ_ASSERT(lhs != addressTempRegister);
	MOZ_ASSERT(rhs.base != tempRegister);
	MOZ_ASSERT(rhs.base != addressTempRegister);

	ma_lw(addressTempRegister, rhs);
	ma_cmp_set(cond, lhs, addressTempRegister, dest);
}

void
MacroAssemblerPPC::ma_cmp_set(Assembler::DoubleCondition cond, FloatRegister lhs, FloatRegister rhs, Register dest) {
	ma_cr_set(ma_fcmp(lhs, rhs, cond), dest);
}

bool
MacroAssemblerPPCCompat::buildFakeExitFrame(Register scratch, uint32_t *offset)
{
	ispew("[[ buildFakeExitFrame(reg, offs)");
    mozilla::DebugOnly<uint32_t> initialDepth = framePushed();

    CodeLabel cl;
    ma_li32(scratch, cl.dest());

    uint32_t descriptor = MakeFrameDescriptor(framePushed(), JitFrame_IonJS);
    Push(Imm32(descriptor));
    Push(scratch);

    bind(cl.src());
    *offset = currentOffset();

    MOZ_ASSERT(framePushed() == initialDepth + ExitFrameLayout::Size());
    ispew("   buildFakeExitFrame(reg, offs) ]]");
    return addCodeLabel(cl);
}

bool
MacroAssemblerPPCCompat::buildOOLFakeExitFrame(void *fakeReturnAddr)
{
	ispew("[[ buildOOLFakeExitFrame(void *)");
    uint32_t descriptor = MakeFrameDescriptor(framePushed(), JitFrame_IonJS);

    Push(Imm32(descriptor)); // descriptor_
    Push(ImmPtr(fakeReturnAddr));

	ispew("   buildOOLFakeExitFrame(void *) ]]");
    return true;
}

void
MacroAssemblerPPCCompat::callWithExitFrame(Label *target)
{
	ispew("[[ callWithExitFrame(l)");
    uint32_t descriptor = MakeFrameDescriptor(framePushed(), JitFrame_IonJS);
    Push(Imm32(descriptor)); // descriptor

    ma_callJitHalfPush(target);
    ispew("   callWithExitFrame(l) ]]");
}

void
MacroAssemblerPPCCompat::callWithExitFrame(JitCode *target)
{
	ispew("[[ callWithExitFrame(jitcode)");
    uint32_t descriptor = MakeFrameDescriptor(framePushed(), JitFrame_IonJS);
    Push(Imm32(descriptor)); // descriptor

    addPendingJump(m_buffer.nextOffset(), ImmPtr(target->raw()), Relocation::JITCODE);
    ma_li32Patchable(addressTempRegister, ImmPtr(target->raw()));
    ma_callJitHalfPush(addressTempRegister);
    ispew("   callWithExitFrame(jitcode) ]]");
}

void
MacroAssemblerPPCCompat::callWithExitFrame(JitCode *target, Register dynStack)
{
	ispew("[[ callWithExitFrame(jitcode, reg)");
    ma_addu(dynStack, dynStack, Imm32(framePushed()));
    makeFrameDescriptor(dynStack, JitFrame_IonJS);
    Push(dynStack); // descriptor

    addPendingJump(m_buffer.nextOffset(), ImmPtr(target->raw()), Relocation::JITCODE);
    ma_li32Patchable(addressTempRegister, ImmPtr(target->raw()));
    ma_callJitHalfPush(addressTempRegister);
    ispew("   callWithExitFrame(jitcode, reg) ]]");
}

void
MacroAssemblerPPCCompat::callJit(Register callee)
{
	ma_callJit(callee);
}
void
MacroAssemblerPPCCompat::callJitFromAsmJS(Register callee) // XXX
{
    ma_callJitNoPush(callee);

    // The Ion ABI has the callee pop the return address off the stack.
    // The asm.js caller assumes that the call leaves sp unchanged, so bump
    // the stack.
    subPtr(Imm32(sizeof(void*)), stackPointerRegister);
}

void
MacroAssemblerPPCCompat::reserveStack(uint32_t amount)
{
    if (amount)
        ma_subu(stackPointerRegister, stackPointerRegister, Imm32(amount));
    adjustFrame(amount);
}

void
MacroAssemblerPPCCompat::freeStack(uint32_t amount)
{
    MOZ_ASSERT(amount <= framePushed_);
    if (amount)
        ma_addu(stackPointerRegister, stackPointerRegister, Imm32(amount));
    adjustFrame(-amount);
}

void
MacroAssemblerPPCCompat::freeStack(Register amount)
{
    add(stackPointerRegister, stackPointerRegister, amount);
}

void
MacroAssembler::PushRegsInMask(RegisterSet set, FloatRegisterSet simdSet)
{
    MOZ_ASSERT(!SupportsSimd() && simdSet.size() == 0);
    
	// TODO: We don't need two reserveStacks.
	int32_t diffF = set.fpus().size() * sizeof(double);
	int32_t diffG = set.gprs().size() * sizeof(intptr_t);
	reserveStack(diffG);
	for (GeneralRegisterBackwardIterator iter(set.gprs()); iter.more(); iter++) {
	    diffG -= sizeof(intptr_t);
	    if (*iter == lr) { // "Fake LR"
	    	x_mflr(tempRegister);
	    	stw(tempRegister, stackPointerRegister, diffG);
	    } else
	    storePtr(*iter, Address(StackPointer, diffG));
	}
	MOZ_ASSERT(diffG == 0);
	reserveStack(diffF);
	for (FloatRegisterBackwardIterator iter(set.fpus()); iter.more(); iter++) {
	    diffF -= sizeof(double);
	    storeDouble(*iter, Address(StackPointer, diffF));
	}
	MOZ_ASSERT(diffF == 0);
}

void
MacroAssembler::PopRegsInMaskIgnore(RegisterSet set, RegisterSet ignore, FloatRegisterSet simdSet)
{
    MOZ_ASSERT(!SupportsSimd() && simdSet.size() == 0);
    
	int32_t diffG = set.gprs().size() * sizeof(intptr_t);
	int32_t diffF = set.fpus().size() * sizeof(double);
	const int32_t reservedG = diffG;
	const int32_t reservedF = diffF;

	// TODO: We don't need two freeStacks.
	{
	    for (FloatRegisterBackwardIterator iter(set.fpus()); iter.more(); iter++) {
	        diffF -= sizeof(double);
	        if (!ignore.has(*iter))
	            loadDouble(Address(StackPointer, diffF), *iter);
	    }
	    freeStack(reservedF);
	}
	MOZ_ASSERT(diffF == 0);
	{
	    for (GeneralRegisterBackwardIterator iter(set.gprs()); iter.more(); iter++) {
	        diffG -= sizeof(intptr_t);
	        if (!ignore.has(*iter)) {
	        	if (*iter == lr) { // Fake "LR"
	        		lwz(tempRegister, stackPointerRegister, diffG);
	        		x_mtlr(tempRegister);
	        	} else
	            loadPtr(Address(StackPointer, diffG), *iter);
	        }
	    }
	    freeStack(reservedG);
	}
	MOZ_ASSERT(diffG == 0);
}

void
MacroAssemblerPPCCompat::add32(Register src, Register dest)
{
	ispew("add32(reg, reg)");
    add(dest, dest, src);
}

void
MacroAssemblerPPCCompat::add32(Imm32 imm, Register dest)
{
	ispew("add32(imm, reg)");
    ma_addu(dest, dest, imm);
}

void
MacroAssemblerPPCCompat::add32(Imm32 imm, const Address &dest)
{
	ispew("[[ add32(imm, adr)");
    load32(dest, addressTempRegister);
    ma_addu(addressTempRegister, imm);
    store32(addressTempRegister, dest);
    ispew("   add32(imm, adr) ]]");
}

void
MacroAssemblerPPCCompat::sub32(Imm32 imm, Register dest)
{
	ispew("sub32(imm, reg)");
    ma_subu(dest, dest, imm);
}

void
MacroAssemblerPPCCompat::sub32(Register src, Register dest)
{
	ispew("sub32(reg, reg)");
    ma_subu(dest, dest, src);
}

void
MacroAssemblerPPCCompat::addPtr(Register src, Register dest)
{
	ispew("addPtr(reg, reg");
    ma_addu(dest, src);
}

void
MacroAssemblerPPCCompat::addPtr(const Address &src, Register dest)
{
	ispew("[[ addPtr(adr, reg)");
    loadPtr(src, tempRegister);
    ma_addu(dest, tempRegister);
    ispew("   addPtr(adr, reg)");
}

void
MacroAssemblerPPCCompat::subPtr(Register src, Register dest)
{
	ispew("subPtr(reg, reg)");
    ma_subu(dest, dest, src);
}

void
MacroAssemblerPPCCompat::not32(Register reg)
{
	ispew("not32(reg)");
	// OPPCC appendix A p.540
    nor(reg, reg, reg);
}

// Logical operations
void
MacroAssemblerPPCCompat::and32(Register src, Register dest)
{
	ispew("and32(reg, reg)");
    ma_and(dest, dest, src);
}

void
MacroAssemblerPPCCompat::and32(Imm32 imm, Register dest)
{
	ispew("and32(imm, reg)");
    ma_and(dest, imm);
}

void
MacroAssemblerPPCCompat::and32(Imm32 imm, const Address &dest)
{
	ispew("[[ and32(imm, adr)");
    load32(dest, addressTempRegister);
    ma_and(addressTempRegister, imm);
    store32(addressTempRegister, dest);
    ispew("   and32(imm, adr) ]]");
}

void
MacroAssemblerPPCCompat::and32(const Address &src, Register dest)
{
	ispew("[[ and32(adr, reg)");
    load32(src, tempRegister);
    ma_and(dest, tempRegister);
    ispew("   and32(adr, reg) ]]");
}

void
MacroAssemblerPPCCompat::or32(Imm32 imm, Register dest)
{
	ispew("or32(imm, reg)");
    ma_or(dest, imm);
}


void
MacroAssemblerPPCCompat::or32(Imm32 imm, const Address &dest)
{
	ispew("[[ or32(imm, adr)");
    load32(dest, addressTempRegister);
    ma_or(addressTempRegister, imm);
    store32(addressTempRegister, dest);
	ispew("   or32(imm, adr) ]]");
}

void
MacroAssemblerPPCCompat::xor32(Imm32 imm, Register dest)
{
	ispew("xor32(imm, reg)");
    ma_xor(dest, imm);
}

void
MacroAssemblerPPCCompat::xorPtr(Imm32 imm, Register dest)
{
	ispew("xorPtr(imm, reg)");
    ma_xor(dest, imm);
}

void
MacroAssemblerPPCCompat::xorPtr(Register src, Register dest)
{
	ispew("xorPtr(reg, reg)");
    ma_xor(dest, src);
}

void
MacroAssemblerPPCCompat::orPtr(Imm32 imm, Register dest)
{
	ispew("orPtr(imm, reg)");
    ma_or(dest, imm);
}

void
MacroAssemblerPPCCompat::orPtr(Register src, Register dest)
{
	ispew("orPtr(reg, reg)");
    ma_or(dest, src);
}

void
MacroAssemblerPPCCompat::andPtr(Imm32 imm, Register dest)
{
	ispew("andPtr(imm, reg)");
    ma_and(dest, imm);
}

void
MacroAssemblerPPCCompat::andPtr(Register src, Register dest)
{
	ispew("andPtr(reg, reg)");
    ma_and(dest, src);
}

void
MacroAssemblerPPCCompat::move32(Imm32 imm, Register dest)
{
	ispew("move32(imm, reg)");
    ma_li32(dest, imm);
}

void
MacroAssemblerPPCCompat::move32(Register src, Register dest)
{
	ispew("move32(reg, reg)");
    x_mr(dest, src);
}

void
MacroAssemblerPPCCompat::movePtr(Register src, Register dest)
{
	ispew("movePtr(reg, reg)");
    x_mr(dest, src);
}
void
MacroAssemblerPPCCompat::movePtr(ImmWord imm, Register dest)
{
	ispew("movePtr(immw, reg)");
    ma_li32(dest, Imm32(imm.value));
}

void
MacroAssemblerPPCCompat::movePtr(ImmGCPtr imm, Register dest)
{
	ispew("movePtr(immgcptr, reg)");
    ma_li32(dest, imm);
}

void
MacroAssemblerPPCCompat::movePtr(ImmMaybeNurseryPtr imm, Register dest)
{
	ispew("movePtr(nursery, reg)");
    movePtr(noteMaybeNurseryPtr(imm), dest);
}
void
MacroAssemblerPPCCompat::movePtr(ImmPtr imm, Register dest)
{
	ispew("movePtr(immptr, reg)");
    movePtr(ImmWord(uintptr_t(imm.value)), dest);
}
void
MacroAssemblerPPCCompat::movePtr(AsmJSImmPtr imm, Register dest)
{
	ispew("movePtr(asmjsimmptr, reg)");
    append(AsmJSAbsoluteLink(CodeOffsetLabel(nextOffset().getOffset()), imm.kind()));
    ma_li32Patchable(dest, Imm32(-1));
}

void
MacroAssemblerPPCCompat::load8ZeroExtend(const Address &address, Register dest)
{
	ispew("load8ZeroExtend(adr, reg)");
    ma_load(dest, address, SizeByte, ZeroExtend);
}

void
MacroAssemblerPPCCompat::load8ZeroExtend(const BaseIndex &src, Register dest)
{
	ispew("load8ZeroExtend(bi, reg)");
    ma_load(dest, src, SizeByte, ZeroExtend);
}

void
MacroAssemblerPPCCompat::load8SignExtend(const Address &address, Register dest)
{
	ispew("load8SignExtend(adr, reg)");
    ma_load(dest, address, SizeByte, SignExtend);
}

void
MacroAssemblerPPCCompat::load8SignExtend(const BaseIndex &src, Register dest)
{
	ispew("load8SignExtend(bi, reg)");
    ma_load(dest, src, SizeByte, SignExtend);
}

void
MacroAssemblerPPCCompat::load16ZeroExtend(const Address &address, Register dest)
{
	ispew("load16ZeroExtend(adr, reg)");
    ma_load(dest, address, SizeHalfWord, ZeroExtend);
}

void
MacroAssemblerPPCCompat::load16ZeroExtend(const BaseIndex &src, Register dest)
{
	ispew("load16ZeroExtend(bi, reg)");
    ma_load(dest, src, SizeHalfWord, ZeroExtend);
}

void
MacroAssemblerPPCCompat::load16SignExtend(const Address &address, Register dest)
{
	ispew("load16SignExtend(adr, reg)");
    ma_load(dest, address, SizeHalfWord, SignExtend);
}

void
MacroAssemblerPPCCompat::load16SignExtend(const BaseIndex &src, Register dest)
{
	ispew("load16SignExtend(bi, reg)");
    ma_load(dest, src, SizeHalfWord, SignExtend);
}

void
MacroAssemblerPPCCompat::load32(const Address &address, Register dest)
{
	ispew("load32(adr, reg)");
    ma_lw(dest, address);
}

void
MacroAssemblerPPCCompat::load32(const BaseIndex &address, Register dest)
{
	ispew("load32(bi, reg)");
    ma_load(dest, address, SizeWord);
}

void
MacroAssemblerPPCCompat::load32(AbsoluteAddress address, Register dest)
{
	ispew("load32(aadr, reg");
    x_li32(addressTempRegister, (uint32_t)address.addr);
    lwz(dest, addressTempRegister, 0);
}

void
MacroAssemblerPPCCompat::loadPtr(const Address &address, Register dest)
{
	ispew("loadPtr(adr, reg)");
    ma_lw(dest, address);
}

void
MacroAssemblerPPCCompat::loadPtr(const BaseIndex &src, Register dest)
{
	ispew("loadPtr(bi, reg)");
    load32(src, dest);
}

void
MacroAssemblerPPCCompat::loadPtr(AbsoluteAddress address, Register dest)
{
	ispew("loadPtr(aadr, reg");
	load32(address, dest);
}

void
MacroAssemblerPPCCompat::loadPtr(AsmJSAbsoluteAddress address, Register dest)
{
	ispew("loadPtr(asmjsaadr, reg)");
    movePtr(AsmJSImmPtr(address.kind()), tempRegister);
    loadPtr(Address(tempRegister, 0x0), dest);
}

void
MacroAssemblerPPCCompat::loadPrivate(const Address &address, Register dest)
{
	ispew("loadPrivate(adr, reg)");
	MOZ_ASSERT(PAYLOAD_OFFSET == 4); // wtf if not
    ma_lw(dest, Address(address.base, address.offset + PAYLOAD_OFFSET));
}

void
MacroAssemblerPPCCompat::loadDouble(const Address &address, FloatRegister dest)
{
	ispew("loadDouble(adr, fpr)");
    ma_ld(dest, address);
}

void
MacroAssemblerPPCCompat::loadDouble(const BaseIndex &src, FloatRegister dest)
{
	ispew("loadDouble(bi, fpr)");
    computeScaledAddress(src, addressTempRegister);
    ma_ld(dest, Address(addressTempRegister, src.offset));
}

void
MacroAssemblerPPCCompat::loadFloatAsDouble(const Address &address, FloatRegister dest)
{
	ispew("loadFloatAsDouble(adr, fpr)");
    ma_ls(dest, address);
    // The FPU automatically converts to double precision for us.
}

void
MacroAssemblerPPCCompat::loadFloatAsDouble(const BaseIndex &src, FloatRegister dest)
{
	ispew("loadFloat32(bi, fpr)");
    loadFloat32(src, dest);
	// The FPU automatically converts to double precision for us.
}

void
MacroAssemblerPPCCompat::loadFloat32(const Address &address, FloatRegister dest)
{
	ispew("loadFloat32(adr, fpr)");
    ma_ls(dest, address);
}

void
MacroAssemblerPPCCompat::loadFloat32(const BaseIndex &src, FloatRegister dest)
{
	ispew("loadFloat32(bi, fpr)");
    computeScaledAddress(src, addressTempRegister);
    ma_ls(dest, Address(addressTempRegister, src.offset));
}

void
MacroAssemblerPPCCompat::store8(Imm32 imm, const Address &address)
{
	ispew("store8(imm, adr)");
    ma_li32(tempRegister, imm);
    ma_store(tempRegister, address, SizeByte);
}

void
MacroAssemblerPPCCompat::store8(Register src, const Address &address)
{
	ispew("store8(reg, adr)");
    ma_store(src, address, SizeByte);
}

void
MacroAssemblerPPCCompat::store8(Imm32 imm, const BaseIndex &dest)
{
	ispew("store8(imm, bi)");
    ma_store(imm, dest, SizeByte);
}

void
MacroAssemblerPPCCompat::store8(Register src, const BaseIndex &dest)
{
	ispew("store8(reg, bi)");
    ma_store(src, dest, SizeByte);
}

void
MacroAssemblerPPCCompat::store16(Imm32 imm, const Address &address)
{
	ispew("store16(imm, adr)");
    ma_li32(tempRegister, imm);
    ma_store(tempRegister, address, SizeHalfWord);
}

void
MacroAssemblerPPCCompat::store16(Register src, const Address &address)
{
	ispew("store16(reg, adr)");
    ma_store(src, address, SizeHalfWord);
}

void
MacroAssemblerPPCCompat::store16(Imm32 imm, const BaseIndex &dest)
{
	ispew("store16(imm, bi)");
    ma_store(imm, dest, SizeHalfWord);
}

void
MacroAssemblerPPCCompat::store16(Register src, const BaseIndex &address)
{
	ispew("store16(reg, bi)");
    ma_store(src, address, SizeHalfWord);
}

void
MacroAssemblerPPCCompat::store32(Register src, AbsoluteAddress address)
{
	ispew("storePtr(reg, aadr)");
    storePtr(src, address);
}

void
MacroAssemblerPPCCompat::store32(Register src, const Address &address)
{
	ispew("store32(reg, adr");
    storePtr(src, address);
}

void
MacroAssemblerPPCCompat::store32(Imm32 src, const Address &address)
{
	ispew("store32(imm, adr)");
    move32(src, tempRegister);
    storePtr(tempRegister, address);
}

void
MacroAssemblerPPCCompat::store32(Imm32 imm, const BaseIndex &dest)
{
	ispew("store32(imm, bi)");
    ma_store(imm, dest, SizeWord);
}

void
MacroAssemblerPPCCompat::store32(Register src, const BaseIndex &dest)
{
	ispew("store32(reg, bi)");
    ma_store(src, dest, SizeWord);
}

template <typename T>
void
MacroAssemblerPPCCompat::storePtr(ImmWord imm, T address)
{
	ispew("storePtr(immw, T)");
    ma_li32(tempRegister, Imm32(imm.value));
    ma_sw(tempRegister, address);
}

template void MacroAssemblerPPCCompat::storePtr<Address>(ImmWord imm, Address address);
template void MacroAssemblerPPCCompat::storePtr<BaseIndex>(ImmWord imm, BaseIndex address);

template <typename T>
void
MacroAssemblerPPCCompat::storePtr(ImmPtr imm, T address)
{
	ispew("storePtr(immptr, T)");
    storePtr(ImmWord(uintptr_t(imm.value)), address);
}

template void MacroAssemblerPPCCompat::storePtr<Address>(ImmPtr imm, Address address);
template void MacroAssemblerPPCCompat::storePtr<BaseIndex>(ImmPtr imm, BaseIndex address);

template <typename T>
void
MacroAssemblerPPCCompat::storePtr(ImmGCPtr imm, T address)
{
	ispew("storePtr(immgcptr, T)");
    ma_li32(tempRegister, imm);
    ma_sw(tempRegister, address);
}

template void MacroAssemblerPPCCompat::storePtr<Address>(ImmGCPtr imm, Address address);
template void MacroAssemblerPPCCompat::storePtr<BaseIndex>(ImmGCPtr imm, BaseIndex address);

void
MacroAssemblerPPCCompat::storePtr(Register src, const Address &address)
{
	ispew("storePtr(reg, adr)");
    ma_sw(src, address);
}

void
MacroAssemblerPPCCompat::storePtr(Register src, const BaseIndex &address)
{
	ispew("storePtr(reg, bi)");
    ma_store(src, address, SizeWord);
}

void
MacroAssemblerPPCCompat::storePtr(Register src, AbsoluteAddress dest)
{
	ispew("storePtr(reg, aadr)");
    MOZ_ASSERT(src != addressTempRegister);
    x_li32(addressTempRegister, (uint32_t)dest.addr);
    stw(src, addressTempRegister, 0);
}

void
MacroAssemblerPPCCompat::clampIntToUint8(Register srcdest)
{
	ispew("[[ clampIntToUint8(reg)");
	// If < 0, return 0; if > 255, return 255; else return same.
	//
	// Exploit this property for a branchless version:
	// max(a,0) = (a + abs(a)) / 2 (do this first)
	// min(a,255) = (a + 255 - abs(a-255)) / 2
	// Use srcdest as temporary work area.

	// Save the original value first.
	x_mr(tempRegister, srcdest);

	// Next, compute max(original,0), starting with abs(original).
	// Absolute value routine from PowerPC Compiler Writer's Guide, p.50.
	srawi(srcdest, tempRegister, 31);
	xor_(addressTempRegister, srcdest, tempRegister);
	subf(srcdest, srcdest, addressTempRegister);
	// Finally, add original value and divide by 2. Leave in srcdest.
	add(srcdest, srcdest, tempRegister);
	srawi(srcdest, srcdest, 1);

	// Now for min(srcdest, 255). First, compute abs(a-255) into adrTemp.
	// We can clobber tempRegister safely now since we don't need the
	// original value anymore, but we need to push srcdest since we need
	// three working registers for the integer absolute value.
	//
	x_li32(addressTempRegister, 255);
	stwu(srcdest, stackPointerRegister, -4); // cracked on G5, don't care
	subf(addressTempRegister, addressTempRegister, srcdest); // T=B-A
	// Okay to clobber dest now ...
	srawi(srcdest, addressTempRegister, 31);
	xor_(tempRegister, srcdest, addressTempRegister);
	subf(addressTempRegister, srcdest, tempRegister);
	// Now 255 - addressTempRegister. Get srcdest back between instructions.
	x_li32(tempRegister, 255);
	// (There were adequate instructions between this lwz and the stwu to
	// keep them in separate G5 branch groups, so no nops are needed.)
	lwz(srcdest, stackPointerRegister, 0);
	subf(addressTempRegister, addressTempRegister, tempRegister); // T=B-A
	// Add srcdest back to addressTempRegister, and divide by 2.
	addi(stackPointerRegister, stackPointerRegister, 4);
	add(srcdest, srcdest, addressTempRegister);
	srawi(srcdest, srcdest, 1);

	// Ta-daaa!
	ispew("   clampIntToUint8(reg) ]]");
}

// XXX: THIS CAN BE MAJORLY IMPROVED.
// Note: this function clobbers the input register.
void
MacroAssembler::clampDoubleToUint8(FloatRegister input, Register output)
{
	ispew("[[ clampDoubleToUint8(fpr, reg)");
    MOZ_ASSERT(input != fpTempRegister);
    Label positive, done;

    // <= 0 or NaN --> 0
    zeroDouble(fpTempRegister);
    branchDouble(DoubleGreaterThan, input, fpTempRegister, &positive);
    {
        move32(Imm32(0), output);
        jump(&done);
    }

    bind(&positive);

    // Add 0.5 and truncate.
    loadConstantDouble(0.5, fpTempRegister);
    addDouble(fpTempRegister, input);

    Label outOfRange;

    branchTruncateDouble(input, output, &outOfRange);
    branch32(Assembler::Above, output, Imm32(255), &outOfRange);
    {
        // Check if we had a tie.
        convertInt32ToDouble(output, fpTempRegister);
        branchDouble(DoubleNotEqual, input, fpTempRegister, &done);

        // It was a tie. Mask out the ones bit to get an even value.
        // See also js_TypedArray_uint8_clamp_double.
        and32(Imm32(~1), output);
        jump(&done);
    }

    // > 255 --> 255
    bind(&outOfRange);
    {
        move32(Imm32(255), output);
    }

    bind(&done);
    ispew("   clampDoubleToUint8(fpr, reg) ]]");
}

void
MacroAssemblerPPCCompat::subPtr(Imm32 imm, const Register dest)
{
	ispew("subPtr(imm, reg)");
    ma_subu(dest, dest, imm);
}

void
MacroAssemblerPPCCompat::subPtr(const Address &addr, const Register dest)
{
	ispew("subPtr(adr, reg)");
    loadPtr(addr, tempRegister);
    subPtr(tempRegister, dest);
}

void
MacroAssemblerPPCCompat::subPtr(Register src, const Address &dest)
{
	ispew("subPtr(reg, adr)");
    loadPtr(dest, addressTempRegister);
    subPtr(src, addressTempRegister);
    storePtr(addressTempRegister, dest);
}

void
MacroAssemblerPPCCompat::addPtr(Imm32 imm, const Register dest)
{
	ispew("addPtr(imm, reg)");
    ma_addu(dest, imm);
}

void
MacroAssemblerPPCCompat::addPtr(Imm32 imm, const Address &dest)
{
	ispew("[[ addPtr(imm, adr)");
    loadPtr(dest, addressTempRegister);
    addPtr(imm, addressTempRegister);
    storePtr(addressTempRegister, dest);
    ispew("   addPtr(imm, adr) ]]");
}

void
MacroAssemblerPPCCompat::branchDouble(DoubleCondition cond, FloatRegister lhs,
                                       FloatRegister rhs, Label *label)
{
	ispew("branchDouble(cond, fpr, fpr, l)");
	ma_fcmp(lhs, rhs, cond); // CR0
	bc(cond, label);
}

void
MacroAssemblerPPCCompat::branchFloat(DoubleCondition cond, FloatRegister lhs,
                                      FloatRegister rhs, Label *label)
{
	ispew("branchFloat(cond, fpr, fpr, l)");
	branchDouble(cond, lhs, rhs, label);
}

// higher level tag testing code
Operand
MacroAssemblerPPC::ToPayload(Operand base)
{
	MOZ_ASSERT(PAYLOAD_OFFSET == 4); // big endian!!!
    return Operand(Register::FromCode(base.base()), base.disp() + PAYLOAD_OFFSET);
}

Operand
MacroAssemblerPPC::ToType(Operand base)
{
	MOZ_ASSERT(TAG_OFFSET == 0); // big endian!!!
    return Operand(Register::FromCode(base.base()), base.disp() + TAG_OFFSET);
}

void
MacroAssemblerPPCCompat::branchTestGCThing(Condition cond, const Address &address, Label *label)
{
	ispew("branchTestGCThing(cond, adr, l)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    Condition newc = (cond == Equal) ? AboveOrEqual : Below;
    extractTag(address, addressTempRegister);
    bc(ma_cmp(addressTempRegister, ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET), newc), label);
}
void
MacroAssemblerPPCCompat::branchTestGCThing(Condition cond, const BaseIndex &src, Label *label)
{
	ispew("branchTestGCThing(cond, bi, l)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    Condition newc = (cond == Equal) ? AboveOrEqual : Below;
    extractTag(src, addressTempRegister);
    bc(ma_cmp(addressTempRegister, ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET), newc), label);
}

void
MacroAssemblerPPCCompat::branchTestPrimitive(Condition cond, const ValueOperand &value,
                                              Label *label)
{
    branchTestPrimitive(cond, value.typeReg(), label);
}
void
MacroAssemblerPPCCompat::branchTestPrimitive(Condition cond, Register tag, Label *label)
{
	ispew("branchTestPrimitive(cond, reg, l)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    Condition newc = (cond == Equal) ? Below : AboveOrEqual;
    bc(ma_cmp(tag, ImmTag(JSVAL_UPPER_EXCL_TAG_OF_PRIMITIVE_SET), newc), label);
}

void
MacroAssemblerPPCCompat::branchTestInt32(Condition cond, const ValueOperand &value, Label *label)
{
	ispew("branchTestInt32(cond, vo, l)");
    branchTestInt32(cond, value.typeReg(), label);
}
void
MacroAssemblerPPCCompat::branchTestInt32(Condition cond, Register tag, Label *label)
{
	ispew("branchTestInt32(cond, reg, l)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    bc(ma_cmp(tag, ImmType(JSVAL_TYPE_INT32), (cond)), label);
}
void
MacroAssemblerPPCCompat::branchTestInt32(Condition cond, const Address &address, Label *label)
{
	ispew("branchTestInt32(cond, adr, l)");
    extractTag(address, addressTempRegister);
    branchTestInt32(cond, addressTempRegister, label);
}
void
MacroAssemblerPPCCompat::branchTestInt32(Condition cond, const BaseIndex &src, Label *label)
{
	ispew("branchTestInt32(cond, bi, l)");
    extractTag(src, addressTempRegister);
    branchTestInt32(cond, addressTempRegister, label);
}

void
MacroAssemblerPPCCompat::branchTestBoolean(Condition cond, const ValueOperand &value,
                                             Label *label)
{
	branchTestBoolean(cond, value.typeReg(), label);
}
void
MacroAssemblerPPCCompat::branchTestBoolean(Condition cond, Register tag, Label *label)
{
    ispew("branchTestBoolean(cond, reg, l)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    bc(ma_cmp(tag, ImmType(JSVAL_TYPE_BOOLEAN), (cond)), label);
}
void
MacroAssemblerPPCCompat::branchTestBoolean(Condition cond, const BaseIndex &src, Label *label)
{
	ispew("branchTestBoolean(cond, bi, l)");
    extractTag(src, addressTempRegister);
    branchTestBoolean(cond, addressTempRegister, label);
}

void
MacroAssemblerPPCCompat::branchTestDouble(Condition cond, const ValueOperand &value, Label *label)
{
	branchTestDouble(cond, value.typeReg(), label);
}
void
MacroAssemblerPPCCompat::branchTestDouble(Condition cond, Register tag, Label *label)
{
	ispew("branchTestDouble(cond, reg, l)");
    MOZ_ASSERT(cond == Assembler::Equal || cond == NotEqual);
    Condition actual = (cond == Equal) ? Below : AboveOrEqual;
    bc(ma_cmp(tag, ImmTag(JSVAL_TAG_CLEAR), actual), label);
}
void
MacroAssemblerPPCCompat::branchTestDouble(Condition cond, const Address &address, Label *label)
{
	ispew("branchTestDouble(cond, adr, l)");
    extractTag(address, addressTempRegister);
    branchTestDouble(cond, addressTempRegister, label);
}
void
MacroAssemblerPPCCompat::branchTestDouble(Condition cond, const BaseIndex &src, Label *label)
{
	ispew("branchTestDouble(cond, bi, l)");
    extractTag(src, addressTempRegister);
    branchTestDouble(cond, addressTempRegister, label);
}

void
MacroAssemblerPPCCompat::branchTestNull(Condition cond, const ValueOperand &value, Label *label)
{
	branchTestNull(cond, value.typeReg(), label);
}
void
MacroAssemblerPPCCompat::branchTestNull(Condition cond, Register tag, Label *label)
{
	ispew("branchTestNull(cond, reg, l)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    bc(ma_cmp(tag, ImmTag(JSVAL_TAG_NULL), (cond)), label);
}
void
MacroAssemblerPPCCompat::branchTestNull(Condition cond, const Address &src, Label *label)
{
	ispew("branchTestNull(cond, adr, l)");
    extractTag(src, addressTempRegister);
    branchTestNull(cond, addressTempRegister, label);
}
void
MacroAssemblerPPCCompat::branchTestNull(Condition cond, const BaseIndex &src, Label *label)
{
	ispew("branchTestNull(cond, bi, l)");
    extractTag(src, addressTempRegister);
    branchTestNull(cond, addressTempRegister, label);
}
void
MacroAssemblerPPCCompat::testNullSet(Condition cond, const ValueOperand &value, Register dest)
{
	ispew("testNullSet(cond, vo, reg)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    ma_cr_set(ma_cmp(value.typeReg(), ImmType(JSVAL_TYPE_NULL), (cond)), dest);
}

void
MacroAssemblerPPCCompat::branchTestObject(Condition cond, const ValueOperand &value, Label *label)
{
    branchTestObject(cond, value.typeReg(), label);
}
void
MacroAssemblerPPCCompat::branchTestObject(Condition cond, Register tag, Label *label)
{
	ispew("branchTestObject(cond, reg, l)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    bc(ma_cmp(tag, ImmTag(JSVAL_TAG_OBJECT), (cond)), label);
}
void
MacroAssemblerPPCCompat::branchTestObject(Condition cond, const BaseIndex &src, Label *label)
{
	ispew("branchTestObject(cond, bi, l)");
    extractTag(src, addressTempRegister);
    branchTestObject(cond, addressTempRegister, label);
}
void
MacroAssemblerPPCCompat::testObjectSet(Condition cond, const ValueOperand &value, Register dest)
{
	ispew("testObjectSet(cond, vo, reg)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    ma_cr_set(ma_cmp(value.typeReg(), ImmType(JSVAL_TYPE_OBJECT), (cond)), dest);
}

void
MacroAssemblerPPCCompat::branchTestString(Condition cond, const ValueOperand &value, Label *label)
{
    branchTestString(cond, value.typeReg(), label);
}
void
MacroAssemblerPPCCompat::branchTestString(Condition cond, Register tag, Label *label)
{
	ispew("branchTestString(cond, reg, l)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    bc(ma_cmp(tag, ImmTag(JSVAL_TAG_STRING), (cond)), label);
}
void
MacroAssemblerPPCCompat::branchTestString(Condition cond, const BaseIndex &src, Label *label)
{
	ispew("branchTestString(cond, bi, l)");
    extractTag(src, addressTempRegister);
    branchTestString(cond, addressTempRegister, label);
}

void
MacroAssemblerPPCCompat::branchTestSymbol(Condition cond, const ValueOperand &value, Label *label)
{
    branchTestSymbol(cond, value.typeReg(), label);
}
void
MacroAssemblerPPCCompat::branchTestSymbol(Condition cond, const Register &tag, Label *label)
{
	ispew("branchTestSymbol(cond, reg, l)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    bc(ma_cmp(tag, ImmTag(JSVAL_TAG_SYMBOL), cond), label);
}
void
MacroAssemblerPPCCompat::branchTestSymbol(Condition cond, const BaseIndex &src, Label *label)
{
	ispew("branchTestSymbol(cond, bi, l)");
    extractTag(src, addressTempRegister);
    branchTestSymbol(cond, addressTempRegister, label);
}

void
MacroAssemblerPPCCompat::branchTestUndefined(Condition cond, const ValueOperand &value,
                                              Label *label)
{
    branchTestUndefined(cond, value.typeReg(), label);
}
void
MacroAssemblerPPCCompat::branchTestUndefined(Condition cond, Register tag, Label *label)
{
	ispew("branchTestUndefined(cond, reg, l)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    bc(ma_cmp(tag, ImmTag(JSVAL_TAG_UNDEFINED), (cond)), label);
}
void
MacroAssemblerPPCCompat::branchTestUndefined(Condition cond, const BaseIndex &src, Label *label)
{
	ispew("branchTestUndefined(cond, bi, l)");
    extractTag(src, addressTempRegister);
    branchTestUndefined(cond, addressTempRegister, label);
}
void
MacroAssemblerPPCCompat::branchTestUndefined(Condition cond, const Address &address, Label *label)
{
	ispew("branchTestUndefined(cond, adr, l)");
    extractTag(address, addressTempRegister);
    branchTestUndefined(cond, addressTempRegister, label);
}
void
MacroAssemblerPPCCompat::testUndefinedSet(Condition cond, const ValueOperand &value, Register dest)
{
	ispew("testUndefinedSet(cond, vo, reg)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    ma_cmp_set((cond), value.typeReg(), ImmType(JSVAL_TYPE_UNDEFINED), dest);
}

void
MacroAssemblerPPCCompat::branchTestNumber(Condition cond, const ValueOperand &value, Label *label)
{
    branchTestNumber(cond, value.typeReg(), label);
}
void
MacroAssemblerPPCCompat::branchTestNumber(Condition cond, Register tag, Label *label)
{
	ispew("branchTestNumber(cond, reg, l)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    bc(ma_cmp(tag, ImmTag(JSVAL_UPPER_INCL_TAG_OF_NUMBER_SET),
         cond == Equal ? BelowOrEqual : Above), label);
}

void
MacroAssemblerPPCCompat::branchTestMagic(Condition cond, const ValueOperand &value, Label *label)
{
    branchTestMagic(cond, value.typeReg(), label);
}
void
MacroAssemblerPPCCompat::branchTestMagic(Condition cond, Register tag, Label *label)
{
	ispew("branchTestMagic(cond, reg, l)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    bc(ma_cmp(tag, ImmTag(JSVAL_TAG_MAGIC), (cond)), label);
}
void
MacroAssemblerPPCCompat::branchTestMagic(Condition cond, const Address &address, Label *label)
{
	ispew("branchTestMagic(cond, adr, l)");
    extractTag(address, addressTempRegister);
    branchTestMagic(cond, addressTempRegister, label);
}
void
MacroAssemblerPPCCompat::branchTestMagic(Condition cond, const BaseIndex &src, Label *label)
{
	ispew("branchTestMagic(cond, bi, l)");
    extractTag(src, addressTempRegister);
    branchTestMagic(cond, addressTempRegister, label);
}

void
MacroAssemblerPPCCompat::branchTestValue(Condition cond, const ValueOperand &value,
                                          const Value &v, Label *label)
{
	ispew("branchTestValue(cond, vo, v, l)");
    moveData(v, tempRegister);
    // Unsigned comparisons??
	cmpw(value.payloadReg(), tempRegister);

    if (cond == Equal) {
        BufferOffset done = _bc(0, NotEqual);
        bc(ma_cmp(value.typeReg(), Imm32(getType(v)), Equal), label); 

        bindSS(done);
    } else {
        MOZ_ASSERT(cond == NotEqual);
        bc(NotEqual, label);
        bc(ma_cmp(value.typeReg(), Imm32(getType(v)), NotEqual), label);
    }
}
void
MacroAssemblerPPCCompat::branchTestValue(Condition cond, const Address &valaddr,
                                          const ValueOperand &value, Label *label)
{
	ispew("branchTestValue(cond, adr, vo, l)");
    MOZ_ASSERT(cond == Equal || cond == NotEqual);

    // Test tag.
    ma_lw(tempRegister, Address(valaddr.base, valaddr.offset + TAG_OFFSET));
    branchPtr(cond, tempRegister, value.typeReg(), label);

    // Test payload.
    ma_lw(tempRegister, Address(valaddr.base, valaddr.offset + PAYLOAD_OFFSET));
    branchPtr(cond, tempRegister, value.payloadReg(), label);
}

// Unboxing.
void
MacroAssemblerPPCCompat::unboxNonDouble(const ValueOperand &operand, Register dest)
{
	ispew("unboxNonDouble(vo, reg)");
    if (operand.payloadReg() != dest)
        x_mr(dest, operand.payloadReg());
}

void
MacroAssemblerPPCCompat::unboxNonDouble(const Address &src, Register dest)
{
	ispew("unboxNonDouble(adr, reg)");
    ma_lw(dest, Address(src.base, src.offset + PAYLOAD_OFFSET));
}

void
MacroAssemblerPPCCompat::unboxInt32(const ValueOperand &operand, Register dest)
{
	ispew("unboxInt32(vo, reg)");
	if (operand.payloadReg() != dest)
    	x_mr(dest, operand.payloadReg());
}

void
MacroAssemblerPPCCompat::unboxInt32(const Address &src, Register dest)
{
	ispew("unboxInt32(adr, reg)");
    ma_lw(dest, Address(src.base, src.offset + PAYLOAD_OFFSET));
}

void
MacroAssemblerPPCCompat::unboxBoolean(const ValueOperand &operand, Register dest)
{
	ispew("unboxBoolean(vo, reg)");
	if (operand.payloadReg() != dest)
    	x_mr(dest, operand.payloadReg());
}

void
MacroAssemblerPPCCompat::unboxBoolean(const Address &src, Register dest)
{
	ispew("unboxBoolean(adr, reg)");
    ma_lw(dest, Address(src.base, src.offset + PAYLOAD_OFFSET));
}

void
MacroAssemblerPPCCompat::unboxDouble(const ValueOperand &operand, FloatRegister dest)
{
	ispew("[[ unboxDouble(vo, fpr)");
	
	// ENDIAN!
	stwu(operand.payloadReg(), stackPointerRegister, -4);
	stwu(operand.typeReg(), stackPointerRegister, -4); // both cracked
#if _PPC970_
	x_nop();
	x_nop();
#endif
	lfd(dest, stackPointerRegister, 0);
	addi(stackPointerRegister, stackPointerRegister, 8);
	
	ispew("   unboxDouble(vo, fpr) ]]");
}

void
MacroAssemblerPPCCompat::unboxDouble(const Address &src, FloatRegister dest)
{
	ispew("[[ unboxDouble(adr, fpr)");
    ma_lw(tempRegister, Address(src.base, src.offset + PAYLOAD_OFFSET));
    stwu(tempRegister, stackPointerRegister, -4);
    // We have to add another 4 here -- we just pushed!
    ma_lw(tempRegister, Address(src.base, src.offset + TAG_OFFSET + 4));
    stwu(tempRegister, stackPointerRegister, -4);
#if _PPC970_
	x_nop();
	x_nop();
#endif
	lfd(dest, stackPointerRegister, 0);
	addi(stackPointerRegister, stackPointerRegister, 8);
	
	ispew("   unboxDouble(adr, fpr) ]]");
}

void
MacroAssemblerPPCCompat::unboxString(const ValueOperand &operand, Register dest)
{
	ispew("unboxString(vo, reg)");
    x_mr(dest, operand.payloadReg());
}

void
MacroAssemblerPPCCompat::unboxString(const Address &src, Register dest)
{
	ispew("unboxString(adr, reg)");
    ma_lw(dest, Address(src.base, src.offset + PAYLOAD_OFFSET));
}

void
MacroAssemblerPPCCompat::unboxObject(const ValueOperand &src, Register dest)
{
	ispew("unboxObject(vo, reg)");
    x_mr(dest, src.payloadReg());
}

void
MacroAssemblerPPCCompat::unboxObject(const Address &src, Register dest)
{
	ispew("unboxObject(adr, reg)");
    ma_lw(dest, Address(src.base, src.offset + PAYLOAD_OFFSET));
}

void
MacroAssemblerPPCCompat::unboxObject(const BaseIndex &bi, Register dest)
{
	ispew("unboxObject(bi, reg)");
	
	MOZ_ASSERT(bi.base != addressTempRegister);
	computeScaledAddress(bi, addressTempRegister);
	ma_lw(dest, Address(addressTempRegister, PAYLOAD_OFFSET));
}

// XXX: OPTIMIZE
void
MacroAssemblerPPCCompat::unboxValue(const ValueOperand &src, AnyRegister dest)
{
	ispew("[[ unboxValue(vo, anyreg)");
    if (dest.isFloat()) {
        Label notInt32, end;
        
        branchTestInt32(Assembler::NotEqual, src, &notInt32);
        convertInt32ToDouble(src.payloadReg(), dest.fpu());
        b(&end);
        
        bind(&notInt32);
        unboxDouble(src, dest.fpu());
        bind(&end);
    } else if (src.payloadReg() != dest.gpr()) {
        x_mr(dest.gpr(), src.payloadReg());
    }
    ispew("   unboxValue(vo, anyreg) ]]");
}

void
MacroAssemblerPPCCompat::unboxPrivate(const ValueOperand &src, Register dest)
{
	ispew("unboxPrivate(vo, reg)");
    x_mr(dest, src.payloadReg());
}

void
MacroAssemblerPPCCompat::boxDouble(FloatRegister src, const ValueOperand &dest)
{
	ispew("boxDouble(fpr, vo)");
	ma_mv(src, dest);
}

void
MacroAssemblerPPCCompat::boxNonDouble(JSValueType type, Register src,
                                       const ValueOperand &dest)
{
	ispew("boxNonDouble(jsvaluetype, reg, vo)");
    if (src != dest.payloadReg())
        x_mr(dest.payloadReg(), src);
    ma_li32(dest.typeReg(), ImmType(type));
}

void
MacroAssemblerPPCCompat::boolValueToDouble(const ValueOperand &operand, FloatRegister dest)
{
	ispew("[[ boolValueToDouble(vo, fpr)");
    convertBoolToInt32(operand.payloadReg(), addressTempRegister);
    convertInt32ToDouble(addressTempRegister, dest);
    ispew("   boolValueToDouble(vo, fpr) ]]");
}

void
MacroAssemblerPPCCompat::int32ValueToDouble(const ValueOperand &operand,
                                             FloatRegister dest)
{
    convertInt32ToDouble(operand.payloadReg(), dest);
}

void
MacroAssemblerPPCCompat::boolValueToFloat32(const ValueOperand &operand,
                                             FloatRegister dest)
{
	ispew("[[ boolValueToFloat32(vo, fpr)");
    convertBoolToInt32(addressTempRegister, operand.payloadReg());
    convertInt32ToFloat32(addressTempRegister, dest);
    ispew("   boolValueToFloat32(vo, fpr) ]]");
}

void
MacroAssemblerPPCCompat::int32ValueToFloat32(const ValueOperand &operand,
                                              FloatRegister dest)
{
    convertInt32ToFloat32(operand.payloadReg(), dest);
}

void
MacroAssemblerPPCCompat::loadConstantFloat32(float f, FloatRegister dest)
{
	ispew("loadConstantFloat32(float, fpr)");
    ma_lis(dest, f);
}

void
MacroAssemblerPPCCompat::loadInt32OrDouble(const Address &src, FloatRegister dest)
{
	ispew("[[ loadInt32OrDouble(adr, fpr)");
    Label notInt32, end;
    
    // If it's an int, convert it to double.
    ma_lw(tempRegister, Address(src.base, src.offset + TAG_OFFSET));
    branchTestInt32(Assembler::NotEqual, tempRegister, &notInt32);
    ma_lw(tempRegister, Address(src.base, src.offset + PAYLOAD_OFFSET));
    // XXX: tempRegister is heavily used by the conversion routines, so this is simpler for now.
    x_mr(addressTempRegister, tempRegister);
    convertInt32ToDouble(addressTempRegister, dest);
    b(&end);

    // Not an int, so just load as double.
    bind(&notInt32);
    ma_ld(dest, src);
    bind(&end);
    ispew("   loadInt32OrDouble(adr, fpr) ]]");
}

void
MacroAssemblerPPCCompat::loadInt32OrDouble(Register base, Register index,
                                            FloatRegister dest, int32_t shift)
{
	ispew("[[ loadInt32OrDouble(reg, reg, fpr, imm)");
    Label notInt32, end;

    // If it's an int, convert it to double.
    computeScaledAddress(BaseIndex(base, index, ShiftToScale(shift)), addressTempRegister);
    // Since we only have one scratch, we need to stomp over it with the tag.
    load32(Address(addressTempRegister, TAG_OFFSET), tempRegister);
    branchTestInt32(Assembler::NotEqual, tempRegister, &notInt32);

    computeScaledAddress(BaseIndex(base, index, ShiftToScale(shift)), addressTempRegister);
    load32(Address(addressTempRegister, PAYLOAD_OFFSET), tempRegister);
    // XXX: tempRegister is heavily used by the conversion routines, so this is simpler for now.
    x_mr(addressTempRegister, tempRegister);
    convertInt32ToDouble(addressTempRegister, dest);
    b(&end);

    // Not an int, so just load as double.
    bind(&notInt32);
    // First, recompute the offset that had been stored in the scratch register
    // since the scratch register was overwritten loading in the type.
    computeScaledAddress(BaseIndex(base, index, ShiftToScale(shift)), addressTempRegister);
    loadDouble(Address(addressTempRegister, 0), dest);
    bind(&end);
    ispew("   loadInt32OrDouble(reg, reg, fpr, imm) ]]");
}

void
MacroAssemblerPPCCompat::loadConstantDouble(double dp, FloatRegister dest)
{
	ispew("loadConstantDouble(double, fpr)");
    ma_lid(dest, dp);
}

void
MacroAssemblerPPCCompat::branchTestInt32Truthy(bool b, const ValueOperand &value, Label *label)
{
	ispew("branchTestInt32Truthy(bool, vo, l)");
    and__rc(tempRegister, value.payloadReg(), value.payloadReg());
    bc(b ? NonZero : Zero, label);
}

void
MacroAssemblerPPCCompat::branchTestStringTruthy(bool b, const ValueOperand &value, Label *label)
{
	ispew("branchTestStringTruthy(bool, vo, l)");
    Register string = value.payloadReg();
    ma_lw(tempRegister, Address(string, JSString::offsetOfLength()));
    cmpwi(tempRegister, 0);
    bc(b ? NotEqual : Equal, label);
}

void
MacroAssemblerPPCCompat::branchTestDoubleTruthy(bool b, FloatRegister value, Label *label)
{
	ispew("branchTestDoubleTruthy(bool, fpr, l)");
    MOZ_ASSERT(value != fpTempRegister);
    ma_lid(fpTempRegister, 0.0);
    DoubleCondition cond = b ? DoubleNotEqual : DoubleEqualOrUnordered;
    fcmpu(fpTempRegister, value);
    bc(cond, label);
}

void
MacroAssemblerPPCCompat::branchTestBooleanTruthy(bool b, const ValueOperand &operand,
                                                  Label *label)
{
    branchTestInt32Truthy(b, operand, label); // Same type on PPC.
}

Register
MacroAssemblerPPCCompat::extractObject(const Address &address, Register scratch)
{
    ma_lw(scratch, Address(address.base, address.offset + PAYLOAD_OFFSET));
    return scratch;
}

Register
MacroAssemblerPPCCompat::extractTag(const Address &address, Register scratch)
{
    ma_lw(scratch, Address(address.base, address.offset + TAG_OFFSET));
    return scratch;
}

Register
MacroAssemblerPPCCompat::extractTag(const BaseIndex &address, Register scratch)
{
    computeScaledAddress(address, scratch);
    return extractTag(Address(scratch, address.offset), scratch);
}


uint32_t
MacroAssemblerPPCCompat::getType(const Value &val)
{
    jsval_layout jv = JSVAL_TO_IMPL(val);
    return jv.s.tag;
}

// ENDIAN!
void
MacroAssemblerPPCCompat::loadUnboxedValue(Address address, MIRType type, AnyRegister dest)
{
    ispew("loadUnboxedValue(adr, mirtype, anyreg)");

    if (dest.isFloat())
        // loadInt32OrDouble already knows to correct for endianness.
        loadInt32OrDouble(address, dest.fpu());
    else
        // Correct the address to include the payload offset.
        ma_lw(dest.gpr(), Address(address.base, address.offset + PAYLOAD_OFFSET));
}

// ENDIAN!
void
MacroAssemblerPPCCompat::loadUnboxedValue(BaseIndex address, MIRType type, AnyRegister dest)
{
	ispew("loadUnboxedValue(bi, mirtype, anyreg)");

    if (dest.isFloat())
        // loadInt32OrDouble already knows to correct for endianness.
        loadInt32OrDouble(address.base, address.index, dest.fpu(), address.scale);
    else {
        // Correct the base index to include the payload offset.
        MOZ_ASSERT(address.base != addressTempRegister);

        computeScaledAddress(address, addressTempRegister);
    	ma_lw(dest.gpr(), Address(addressTempRegister, PAYLOAD_OFFSET));
    }
}

// ENDIAN!
template <typename T>
void
MacroAssemblerPPCCompat::storeUnboxedValue(ConstantOrRegister value, MIRType valueType, const T &dest,
                                            MIRType slotType)
{
	ispew("[[ storeUnboxedValue(cor, mirtype, T, mirtype)");
    if (valueType == MIRType_Double) {
        storeDouble(value.reg().typedReg().fpu(), dest);
        return;
    }

    // Store the type tag if needed.
    if (valueType != slotType)
        storeTypeTag(ImmType(ValueTypeFromMIRType(valueType)), dest);

    // Store the payload. storePayload uses the corrected offset.
    if (value.constant())
        storePayload(value.value(), dest);
    else
        storePayload(value.reg().typedReg().gpr(), dest);
    ispew("   storeUnboxedValue(cor, mirtype, T, mirtype) ]]");
}

template void
MacroAssemblerPPCCompat::storeUnboxedValue(ConstantOrRegister value, MIRType valueType, const Address &dest,
                                            MIRType slotType);

template void
MacroAssemblerPPCCompat::storeUnboxedValue(ConstantOrRegister value, MIRType valueType, const BaseIndex &dest,
                                            MIRType slotType);

void
MacroAssemblerPPCCompat::moveData(const Value &val, Register data)
{
	ispew("moveData(v, reg)");
    jsval_layout jv = JSVAL_TO_IMPL(val);
    if (val.isMarkable())
        ma_li32(data, ImmGCPtr(reinterpret_cast<gc::Cell *>(val.toGCThing())));
    else
        ma_li32(data, Imm32(jv.s.payload.i32));
}

void
MacroAssemblerPPCCompat::moveValue(const Value &val, Register type, Register data)
{
	ispew("moveValue(v, reg, reg)");
    MOZ_ASSERT(type != data);
    ma_li32(type, Imm32(getType(val)));
    moveData(val, data);
}
void
MacroAssemblerPPCCompat::moveValue(const Value &val, const ValueOperand &dest)
{
    moveValue(val, dest.typeReg(), dest.payloadReg());
}

// See also jit::PatchBackedge.
CodeOffsetJump
MacroAssemblerPPCCompat::backedgeJump(RepatchLabel *label)
{
	ispew("[[ backedgeJump(rl)");
    // Only one branch per label.
    MOZ_ASSERT(!label->used());
    uint32_t dest = label->bound() ? label->offset() : LabelBase::INVALID_OFFSET;
    BufferOffset bo = nextOffset();
    label->use(bo.getOffset());

    // Backedges are short jumps when bound, but can become long when patched.
    m_buffer.ensureSpace(40); // paranoia
    if (label->bound()) {
        int32_t offset = label->offset() - bo.getOffset();
        MOZ_ASSERT(JOffImm26::IsInRange(offset));
        Assembler::b(offset);
    } else {
        // Jump to the first stanza by default to jump to the loop header.
        // PatchBackedge may be able to help us later.
        Assembler::b(4);
    }
    
    // Stanza 1: loop header.
    x_p_li32(tempRegister, dest);
    x_mtctr(tempRegister);
    bctr();
    // Stanza 2: interrupt loop target (AsmJS only).
    x_p_li32(tempRegister, dest);
    x_mtctr(tempRegister);
    bctr();
    ispew("   backedgeJump(rl) ]]");
    return CodeOffsetJump(bo.getOffset());
}

// XXX: Not fully implemented (see CodeGeneratorShared::jumpToBlock()).
CodeOffsetJump
MacroAssemblerPPCCompat::jumpWithPatch(RepatchLabel *label, Condition cond)
{
	// Since these are patchable, no sense in optimizing them.
	ispew("[[ jumpWithPatch(rl, c)");
    // Only one branch per label.
    MOZ_ASSERT(!label->used());
    MOZ_ASSERT(cond == Always); // Not tested (but should work).
    
    // Put some extra space in, just in case bcctr() generates additional instructions.
    m_buffer.ensureSpace(((cond == Always) ? 5 : 8) * sizeof(uint32_t));
    uint32_t dest = label->bound() ? label->offset() : LabelBase::INVALID_OFFSET;

    BufferOffset bo = nextOffset();
    label->use(bo.getOffset());
    addLongJump(bo);
    x_p_li32(tempRegister, dest);
    x_mtctr(tempRegister);
    
    // If always, use bctr(). If not, use bcctr() with the appropriate condition.
    // Assume the comparison already occurred.
    if (cond == Always) {
    	bctr();
    } else {
    	bcctr(cond);
    }
    ispew("   jumpWithPatch(rl, c) ]]");
    return CodeOffsetJump(bo.getOffset());
}

/////////////////////////////////////////////////////////////////
// X86/X64-common/ARM/MIPS/IonPower interface.
/////////////////////////////////////////////////////////////////
void
MacroAssemblerPPCCompat::storeValue(ValueOperand val, Operand dst)
{
	ispew("storeValue(vo, o)");
    storeValue(val, Address(Register::FromCode(dst.base()), dst.disp()));
}

void
MacroAssemblerPPCCompat::storeValue(ValueOperand val, const BaseIndex &dest)
{
	ispew("storeValue(vo, bi)");
    computeScaledAddress(dest, addressTempRegister);
    storeValue(val, Address(addressTempRegister, dest.offset));
}

void
MacroAssemblerPPCCompat::storeValue(JSValueType type, Register reg, BaseIndex dest)
{
	ispew("storeValue(jsvaluetype, reg, bi)");
	MOZ_ASSERT(dest.base != tempRegister);
	MOZ_ASSERT(dest.base != addressTempRegister);
    computeScaledAddress(dest, addressTempRegister);

    int32_t offset = dest.offset;
    if (!Imm16::IsInSignedRange(offset)) {
    	x_li32(tempRegister, offset);
    	add(addressTempRegister, tempRegister, addressTempRegister);
        offset = 0;
    }
    storeValue(type, reg, Address(addressTempRegister, offset));
}

void
MacroAssemblerPPCCompat::storeValue(ValueOperand val, const Address &dest)
{
	ispew("storeValue(vo, adr)");
    ma_sw(val.payloadReg(), Address(dest.base, dest.offset + PAYLOAD_OFFSET));
    ma_sw(val.typeReg(), Address(dest.base, dest.offset + TAG_OFFSET));
}

void
MacroAssemblerPPCCompat::storeValue(JSValueType type, Register reg, Address dest)
{
	ispew("storeValue(jsvaluetype, reg, adr)");
    MOZ_ASSERT(dest.base != tempRegister);

    ma_sw(reg, Address(dest.base, dest.offset + PAYLOAD_OFFSET));
    ma_li32(tempRegister, ImmTag(JSVAL_TYPE_TO_TAG(type)));
    ma_sw(tempRegister, Address(dest.base, dest.offset + TAG_OFFSET));
}

void
MacroAssemblerPPCCompat::storeValue(const Value &val, Address dest)
{
	ispew("storeValue(v, adr)");
    MOZ_ASSERT(dest.base != tempRegister);

    ma_li32(tempRegister, Imm32(getType(val)));
    ma_sw(tempRegister, Address(dest.base, dest.offset + TAG_OFFSET));
    moveData(val, tempRegister);
    ma_sw(tempRegister, Address(dest.base, dest.offset + PAYLOAD_OFFSET));
}

void
MacroAssemblerPPCCompat::storeValue(const Value &val, BaseIndex dest)
{
	ispew("storeValue(v, bi)");
	MOZ_ASSERT(dest.base != tempRegister);
	MOZ_ASSERT(dest.base != addressTempRegister);
	
    computeScaledAddress(dest, addressTempRegister);

    int32_t offset = dest.offset;
    if (!Imm16::IsInSignedRange(offset)) {
        x_li32(tempRegister, offset);
        add(addressTempRegister, tempRegister, addressTempRegister);
        offset = 0;
    }
    storeValue(val, Address(addressTempRegister, offset));
}

void
MacroAssemblerPPCCompat::loadValue(const BaseIndex &addr, ValueOperand val)
{
	ispew("loadValue(bi, vo)");
    computeScaledAddress(addr, addressTempRegister);
    loadValue(Address(addressTempRegister, addr.offset), val);
}

void
MacroAssemblerPPCCompat::loadValue(Address src, ValueOperand val)
{
	ispew("loadValue(adr, vo)");
	
    // Ensure that loading the payload does not erase the pointer to the
    // Value in memory.
    if (src.base != val.payloadReg()) {
        ma_lw(val.payloadReg(), Address(src.base, src.offset + PAYLOAD_OFFSET));
        ma_lw(val.typeReg(), Address(src.base, src.offset + TAG_OFFSET));
    } else {
        ma_lw(val.typeReg(), Address(src.base, src.offset + TAG_OFFSET));
        ma_lw(val.payloadReg(), Address(src.base, src.offset + PAYLOAD_OFFSET));
    }
}

void
MacroAssemblerPPCCompat::tagValue(JSValueType type, Register payload, ValueOperand dest)
{
	ispew("tagValue(jsvaluetype, reg, vo)");
    MOZ_ASSERT(payload != dest.typeReg());
    
    ma_li32(dest.typeReg(), ImmType(type));
    if (payload != dest.payloadReg())
        x_mr(dest.payloadReg(), payload);
}

void
MacroAssemblerPPCCompat::pushValue(ValueOperand val)
{
	ispew("pushValue(vo)");
	
	// Endian! Payload on first.
	stwu(val.payloadReg(), stackPointerRegister, -4);
	stwu(val.typeReg(), stackPointerRegister, -4);
}

void
MacroAssemblerPPCCompat::pushValue(const Address &addr)
{
	ispew("pushValue(adr)");
	
    // Allocate stack slots for type and payload. One for each.
    ma_subu(stackPointerRegister, stackPointerRegister, Imm32(sizeof(Value)));
    // Store type and payload.
    ma_lw(tempRegister, Address(addr.base, addr.offset + TAG_OFFSET));
    ma_sw(tempRegister, Address(stackPointerRegister, TAG_OFFSET));
    ma_lw(tempRegister, Address(addr.base, addr.offset + PAYLOAD_OFFSET));
    ma_sw(tempRegister, Address(stackPointerRegister, PAYLOAD_OFFSET));
}

void
MacroAssemblerPPCCompat::popValue(ValueOperand val)
{
	ispew("popValue(vo)");
	
    // Load payload and type.
    lwz(val.payloadReg(), stackPointerRegister, PAYLOAD_OFFSET);
    lwz(val.typeReg(), stackPointerRegister, TAG_OFFSET);
    // Free stack.
    addi(stackPointerRegister, stackPointerRegister, 8);
}

void
MacroAssemblerPPCCompat::storePayload(const Value &val, Address dest)
{
	ispew("storePayload(v, adr)");
	
    moveData(val, addressTempRegister);
    ma_sw(addressTempRegister, Address(dest.base, dest.offset + PAYLOAD_OFFSET));
}

void
MacroAssemblerPPCCompat::storePayload(Register src, Address dest)
{
	ispew("storePayload(reg, adr");
    ma_sw(src, Address(dest.base, dest.offset + PAYLOAD_OFFSET));
}

void
MacroAssemblerPPCCompat::storePayload(const Value &val, const BaseIndex &dest)
{
	ispew("storePayload(v, bi)");
	MOZ_ASSERT(dest.base != tempRegister);
	MOZ_ASSERT(dest.base != addressTempRegister);
    MOZ_ASSERT(dest.offset == 0);

    computeScaledAddress(dest, addressTempRegister);
    moveData(val, tempRegister);
    stw(tempRegister, addressTempRegister, NUNBOX32_PAYLOAD_OFFSET);
}

void
MacroAssemblerPPCCompat::storePayload(Register src, const BaseIndex &dest)
{
	ispew("storePayload(reg, bi)");
    MOZ_ASSERT(dest.offset == 0);

    computeScaledAddress(dest, addressTempRegister);
    stw(src, addressTempRegister, NUNBOX32_PAYLOAD_OFFSET);
}

void
MacroAssemblerPPCCompat::storeTypeTag(ImmTag tag, Address dest)
{
    ma_li32(tempRegister, tag);
    ma_sw(tempRegister, Address(dest.base, dest.offset + TAG_OFFSET));
}

void
MacroAssemblerPPCCompat::storeTypeTag(ImmTag tag, const BaseIndex &dest)
{
	ispew("storeTypeTag(immt, bi)");
    MOZ_ASSERT(dest.offset == 0);

    computeScaledAddress(dest, addressTempRegister);
    ma_li32(tempRegister, tag);
    stw(tempRegister, addressTempRegister, TAG_OFFSET);
}

/* XXX We should only need (and use) callJit */

void
MacroAssemblerPPC::ma_callJitNoPush(const Register r)
{
	ispew("!!!>>> ma_callJitNoPush <<<!!!");
	ma_callJit(r);
}

void
MacroAssemblerPPC::ma_callJitHalfPush(const Register r)
{
	ispew("!!!>>> ma_callJitHalfPush <<<!!!");
	ma_callJit(r);
}

void
MacroAssemblerPPC::ma_callJit(const Register r)
{
	ispew("ma_callJit(reg)");
	
	// This is highly troublesome on PowerPC because the normal ABI expects
	// the *callee* to save LR, and the instruction set is written with this
	// in mind (MIPS gets around this problem by saving $ra in the delay slot,
	// but that's an ugly hack). Since we don't have a reliable way of writing the
	// prologue of the JitCode we call to do this, we can't use LR for calls
	// to generated code!
	//
	// Instead, we simply compute a return address and store that, and return to
	// that instead, so that we basically act like x86.
	
	CodeLabel returnPoint;
	
	x_mtctr(r); // new dispatch group
	ma_li32(tempRegister, returnPoint.dest()); // push computed return address
	stwu(tempRegister, stackPointerRegister, -4);
	// This is guaranteed to be in a different dispatch group, so no nops needed.
	bctr(DontLinkB);
	
	// Return point is here.
	bind(returnPoint.src());
	if (!addCodeLabel(returnPoint))
		MOZ_CRASH("addCodeLabel failed in callJit");
}

void
MacroAssemblerPPC::ma_callJitHalfPush(Label *label) // XXX probably should change the name.
{
	ispew("!!!>>> ma_callJitHalfPush (label) <<<!!!");
	
	// Same limitation applies.
	CodeLabel returnPoint;
	
	ma_li32(tempRegister, returnPoint.dest()); // push computed return address
	stwu(tempRegister, stackPointerRegister, -4);
	b(label);	
	
	// Return point is here.
	bind(returnPoint.src());
	if (!addCodeLabel(returnPoint))
		MOZ_CRASH("addCodeLabel failed in callJit");
}

/* Although these two functions call fixed addresses, optimizing them is
   not very helpful because they are used quite rarely. */
void
MacroAssemblerPPC::ma_call(ImmPtr dest)
{
	ispew("ma_call(immptr)");
    ma_li32Patchable(tempRegister, dest);
    ma_callJit(tempRegister);
}

void
MacroAssemblerPPC::ma_jump(ImmPtr dest)
{
	ispew("ma_jump(immptr)");
    ma_li32Patchable(tempRegister, dest);
    x_mtctr(tempRegister);
#if _PPC970_
	x_nop();
	x_nop();
	x_nop();
#endif
	bctr(DontLinkB);
}

/* Assume these calls and returns are JitCode, because anything calling a PowerOpen ABI
   routine would go through callWithABI() and blr(). */
   
void
MacroAssemblerPPCCompat::call(const Register r) {
	ispew("call -> callJit");
	ma_callJit(r);
}

void
MacroAssemblerPPCCompat::call(Label *l) {
	ispew("call(l) -> callJit");
	ma_callJitHalfPush(l); // XXX
}

void
MacroAssemblerPPCCompat::retn(Imm32 n)
{
	ispew("retn");
	// The return address is on top of the stack, not LR.
	
	lwz(tempRegister, stackPointerRegister, 0);
/* XXX: is mtlr/blr faster than mtctr/bctr? */
	x_mtctr(tempRegister); // LR is now restored, new dispatch group
	addi(stackPointerRegister, stackPointerRegister, n.value);
#if _PPC970_
	x_nop();
	x_nop();
	x_nop(); // keep it out of the branch slot
#endif
	// Branch.
	bctr(DontLinkB);
}

void
MacroAssemblerPPCCompat::breakpoint()
{
// This is always external and should always be marked. We never call this ourselves.
	ispew("assumeUnreachable");
    x_trap();
}

void
MacroAssemblerPPCCompat::ensureDouble(const ValueOperand &source, FloatRegister dest,
                                       Label *failure)
{
	ispew("ensureDouble(vo, fpr, l)");
	
    Label isDouble, done;
    branchTestDouble(Assembler::Equal, source.typeReg(), &isDouble);
    branchTestInt32(Assembler::NotEqual, source.typeReg(), failure);

    convertInt32ToDouble(source.payloadReg(), dest);
    jump(&done);

    bind(&isDouble);
    unboxDouble(source, dest);

    bind(&done);
}

void
MacroAssemblerPPCCompat::setupABICall(uint32_t args)
{
    MOZ_ASSERT(!inCall_);
    inCall_ = true;
    args_ = args;
    passedGPRs_ = 0;
    passedFPRs_ = 0;
}

// For simplicity, all ABI calls are considered unaligned.
void
MacroAssemblerPPCCompat::setupAlignedABICall(uint32_t args)
{
	MOZ_CRASH("use setupUnalignedABICall");
}

void
MacroAssemblerPPCCompat::setupUnalignedABICall(uint32_t args, Register scratch)
{
    setupABICall(args);
	// The actual stack alignment occurs just prior to the call.
}

void
MacroAssemblerPPCCompat::passABIArg(const MoveOperand &from, MoveOp::Type type)
{
	if (!enoughMemory_) return;
	if (type == MoveOp::FLOAT32 || type == MoveOp::DOUBLE) {
		FloatRegister fpr;
		
		// In PowerOpen, f1-f13 are parameter registers.
		if (++passedFPRs_ > 13) {
			// OMG
			MOZ_CRASH("FPR ABI argument overflow");
		} else {
			fpr = FloatRegister::FromCode((FloatRegister::Code)passedFPRs_);
			// All values are doubles, so skip two GPRs as per ABI.
			passedGPRs_ += 2;
		}
#if(0)
// XXX: This used to be substantially faster but no longer seems to be.
// Left here for future consideration.
		if (!from.isFloatReg()) { // i.e., we need to convert; invariably a VMWrapper
			enoughMemory_ = moveResolver_.addMove(from, MoveOperand(fpr), type);
			return;
		}
		// This is a float register move. We can use a simplification to work around
		// the MoveResolver since the current JIT only ever utilizes one or two arguments
		// and they're always stored in registers. This is substantially faster since it
		// never spills to the stack.
		ispew("fast passABIArg for FPR");
		
		// Don't try to execute moves with f0, or if we already allocated the argreg.
		// This should never occur with current code.
		// XXX: It would be nice to check the moveResolver for FPR moves that may conflict,
		// but it's hard to figure out at what stage we are, and to deal with previous moves.
                // These should assert in release builds as well.
		if (from.floatReg() == fpTempRegister)
                    MOZ_CRASH("from.floatReg() is temp register?!");
		if((int)from.floatReg() < (int)fpr)
                    MOZ_CRASH("moving float arguments in unexpected order?!");
		
		if (from.floatReg() == fpr)
			return; // no-op
		fmr(fpr, from.floatReg());
#else
		if (!from.isFloatReg() // i.e., we need to convert
			|| from.floatReg() != fpr)
				enoughMemory_ = moveResolver_.addMove(from, MoveOperand(fpr), type);
#endif
	} else {
		MOZ_ASSERT(type == MoveOp::GENERAL);
		Register gpr;
		
		// In PowerOpen, r3-r10 are parameter registers.
		if (++passedGPRs_ > 8) {
			// OMG
			MOZ_CRASH("GPR ABI argument overflow");
		} else {
			gpr = Register::FromCode((Register::Code)2+passedGPRs_);
		}
		if (!from.isGeneralReg() // i.e., we need to convert
			|| from.reg() != gpr)
				enoughMemory_ = moveResolver_.addMove(from, MoveOperand(gpr), type);
	}
}

void
MacroAssemblerPPCCompat::passABIArg(Register reg)
{
    passABIArg(MoveOperand(reg), MoveOp::GENERAL);
}

void
MacroAssemblerPPCCompat::passABIArg(FloatRegister freg, MoveOp::Type type)
{
    passABIArg(MoveOperand(freg), type);
}

void
MacroAssemblerPPCCompat::checkStackAlignment()
{
	// This is a no-op for PowerPC; we assume unalignment of the stack
	// right up until we branch.
}

void
MacroAssembler::alignFrameForICArguments(AfterICSaveLive &aic)
{
	// Exists for MIPS. We don't use this.
}

void
MacroAssembler::restoreFrameAlignmentForICArguments(AfterICSaveLive &aic)
{
	// Exists for MIPS. We don't use this.
}

// Non-volatile registers for saving r1 and LR because we assume the
// callee will merrily wax them and we don't really have a linkage area.
static const Register stackSave = r16; // Reserved in Architecture-ppc.h.
static const Register returnSave = r18; // Reserved in Architecture-ppc.h.

void
MacroAssemblerPPCCompat::callWithABIPre(uint32_t *stackAdjust, bool callFromAsmJS)
{
    MOZ_ASSERT(inCall_);
    MOZ_ASSERT(!*stackAdjust); // XXX
    MOZ_ASSERT(!callFromAsmJS); // should NEVER happen

	ispew("-- Resolving movements --");
    // Position all arguments.
    {
        enoughMemory_ = enoughMemory_ && moveResolver_.resolve();
        if (!enoughMemory_)
            return;

        MoveEmitter emitter(*this);
        emitter.emit(moveResolver_);
        emitter.finish();
    }
    ispew("-- Movements resolved --");

	// Forcibly align the stack. (We'll add a dummy stack frame in a moment.)
	andi_rc(tempRegister, stackPointerRegister, 4);
	// Save the old stack pointer so we can get it back after the call without
	// screwing up the ABI-compliant routine's ability to walk stack frames.
	x_mr(stackSave, stackPointerRegister);
	subf(stackPointerRegister, tempRegister, stackPointerRegister);
	andi_rc(tempRegister, stackPointerRegister, 8);
	subf(stackPointerRegister, tempRegister, stackPointerRegister);
	// Pull down 512 bytes for a generous stack frame that should accommodate anything.
	x_subi(stackPointerRegister, stackPointerRegister, 512);
	
	// Store the trampoline SP as the phony backreference. This lets the called routine
	// skip anything Ion or Baseline pushed on the stack (the trampoline saved this for
	// us way back when).
	stw(returnSave, stackPointerRegister, 0);
	// Now that it's saved, put LR there since we have no true linkage area.
	x_mflr(returnSave);
}

void
MacroAssemblerPPCCompat::callWithABIPost(uint32_t stackAdjust, MoveOp::Type result)
{
	// Get back LR.
	x_mtlr(returnSave);
	// Get back the trampoline SP for future calls.
	lwz(returnSave, stackPointerRegister, 0);
	// Get back the old SP prior to the forcible alignment.
	x_mr(stackPointerRegister, stackSave);

    MOZ_ASSERT(inCall_);
    inCall_ = false;
}

void
MacroAssemblerPPCCompat::callWithABI(void *fun, MoveOp::Type result)
{
	uint32_t stackAdjust = 0;
	ispew("vv -- callWithABI(void *, result) -- vv");

    if ((uint32_t)fun & 0xfc000000) { // won't fit in 26 bits
        x_li32(addressTempRegister, (uint32_t)fun);
        x_mtctr(addressTempRegister); // Load CTR here to avoid nops later.
    }
    callWithABIPre(&stackAdjust);
    if ((uint32_t)fun & 0xfc000000) { // won't fit in 26 bits
	bctr(LinkB);	
    } else {
        _b((uint32_t)fun, AbsoluteBranch, LinkB);
    }
    callWithABIPost(stackAdjust, result);

    ispew("^^ -- callWithABI(void *, result) -- ^^");
}

void
MacroAssemblerPPCCompat::callWithABI(AsmJSImmPtr imm, MoveOp::Type result)
{
    uint32_t stackAdjust = 0;
	ispew("vv -- callWithABI(asmjsimmptr, result) -- vv");
	
	// XXX; this'll crash immediately when it calls Pre. Left for snickers.
    callWithABIPre(&stackAdjust, /* callFromAsmJS = */ true);
    x_mtrap(); // call(imm);
    callWithABIPost(stackAdjust, result);
    
    ispew("^^ -- callWithABI(asmjsimmptr, result) -- ^^");
}

void
MacroAssemblerPPCCompat::callWithABI(const Address &fun, MoveOp::Type result)
{
    uint32_t stackAdjust = 0;
	ispew("vv -- callWithABI(adr, result) -- vv");
	
    ma_lw(addressTempRegister, Address(fun.base, fun.offset));
    x_mtctr(addressTempRegister);
    callWithABIPre(&stackAdjust);
	bctr(LinkB);    
    callWithABIPost(stackAdjust, result);
    
    ispew("^^ -- callWithABI(adr, result) -- ^^");
}

void
MacroAssemblerPPCCompat::callWithABI(Register fun, MoveOp::Type result)
{
    uint32_t stackAdjust = 0;
	ispew("vv -- callWithABI(reg, result) -- vv");
	
	x_mtctr(fun);
    callWithABIPre(&stackAdjust);
	bctr(LinkB);
    callWithABIPost(stackAdjust, result);
    
    ispew("^^ -- callWithABI(reg, result) -- ^^");
}

void
MacroAssemblerPPCCompat::handleFailureWithHandlerTail(void *handler)
{
	ispew("vv -- handleFailureWithHandlerTail(void *) -- vv");
	// Call an ABI-compliant handler function, reserving adequate space on the stack
	// for a non-ABI compliant exception frame which we will then analyse.
	
    int size = (sizeof(ResumeFromException) + 16) & ~15;
    x_subi(stackPointerRegister, stackPointerRegister, size);
    // This is going to be our argument, so just grab r3 now and save a MoveEmitter.
    x_mr(r3, stackPointerRegister);

    // Ask for an exception handler.
    setupUnalignedABICall(1, r4);
    passABIArg(r3);
    callWithABI(handler);

    Label entryFrame;
    Label catch_;
    Label finally;
    Label return_;
    Label bailout;

    // Now, analyse the returned frame.
    // Get the type.
    lwz(r3, stackPointerRegister, offsetof(ResumeFromException, kind));
    // XXX: These are always short branches, so we can optimize this much better,
    // even though this code is considered a slow path.
    branch32(Assembler::Equal, r3, Imm32(ResumeFromException::RESUME_ENTRY_FRAME), &entryFrame);
    branch32(Assembler::Equal, r3, Imm32(ResumeFromException::RESUME_CATCH), &catch_);
    branch32(Assembler::Equal, r3, Imm32(ResumeFromException::RESUME_FINALLY), &finally);
    branch32(Assembler::Equal, r3, Imm32(ResumeFromException::RESUME_FORCED_RETURN), &return_);
    branch32(Assembler::Equal, r3, Imm32(ResumeFromException::RESUME_BAILOUT), &bailout);
	x_trap(); // Oops.

	// Entry frame case:
    // No exception handler. Load the error value, load the new stack pointer
    // and return from the entry frame.
    bind(&entryFrame);
    lwz(stackPointerRegister, stackPointerRegister, offsetof(ResumeFromException, stackPointer));
    lwz(r3, stackPointerRegister, 0);
    x_mtctr(r3); // new dispatch group; always JitCode (but could be trampoline)
    moveValue(MagicValue(JS_ION_ERROR), JSReturnOperand);
    addi(stackPointerRegister, stackPointerRegister, 4);
#if _PPC970_
	x_nop(); // expect three instructions above
#endif
	bctr();
	
	// Catch handler case:
    // If we found a catch handler, this must be a baseline frame. Restore
    // state and jump to the catch block.
    bind(&catch_);
    lwz(r3, stackPointerRegister, offsetof(ResumeFromException, target));
    x_mtctr(r3); // force a new dispatch group here for G5
    // Then get the old BaselineFrameReg,
    lwz(BaselineFrameReg, stackPointerRegister, offsetof(ResumeFromException, framePointer));
    // get the old stack pointer,
    lwz(stackPointerRegister, stackPointerRegister, offsetof(ResumeFromException, stackPointer));
#if defined(_PPC970_)
	x_nop();
	x_nop(); // keep the bctr out of the branch slot
#endif
	// and branch.
	bctr();
	
	// Finally block case:
    // If we found a finally block, this must be a baseline frame. Push
    // two values expected by JSOP_RETSUB: BooleanValue(true) and the
    // exception.
    bind(&finally);
    ValueOperand exception = ValueOperand(r4, r5);
    loadValue(Address(sp, offsetof(ResumeFromException, exception)), exception);

	// Then prepare to branch,
	lwz(r3, stackPointerRegister, offsetof(ResumeFromException, target));
    x_mtctr(r3); // force a new dispatch group here for G5
    // get the old BaselineFrameReg,
    lwz(BaselineFrameReg, stackPointerRegister, offsetof(ResumeFromException, framePointer));
    // get the old stack pointer,
    lwz(stackPointerRegister, stackPointerRegister, offsetof(ResumeFromException, stackPointer));
	// and finally push Boolean true and the exception, which will confidently keep the
	// bctr away from the mtctr for G5. TODO: pipeline using multi-push.
	pushValue(BooleanValue(true));
	pushValue(exception);
	// Bye.
	bctr();

	// Last but not least: forced return case.
    // Only used in debug mode. Return BaselineFrame->returnValue() to the
    // caller.
    bind(&return_);
    // Get the old BaselineFrameReg,
    lwz(BaselineFrameReg, stackPointerRegister, offsetof(ResumeFromException, framePointer));
    // get the old stack pointer,
    lwz(stackPointerRegister, stackPointerRegister, offsetof(ResumeFromException, stackPointer));
    // load our return value,
    loadValue(Address(BaselineFrameReg, BaselineFrame::reverseOffsetOfReturnValue()),
              JSReturnOperand);
    // remove the stack frame,
    x_mr(stackPointerRegister, BaselineFrameReg);
    // and jet.
    pop(BaselineFrameReg);
    
    // If profiling is enabled, then update the lastProfilingFrame to refer to the
    // caller frame before returning.
    {
    	Label skipProfilingInstrumentation;
    	
    	// Is the profiler enabled?
    	AbsoluteAddress addressOfEnabled(GetJitContext()->runtime->spsProfiler().addressOfEnabled());
    	branch32(Assembler::Equal, addressOfEnabled, Imm32(0), &skipProfilingInstrumentation);
    		// This is a short branch, but this is a slow path of a slow path.
        profilerExitFrame();
        bind(&skipProfilingInstrumentation);
    }

    ret();

    // If we are bailing out to baseline to handle an exception, jump to
    // the bailout tail stub.
    bind(&bailout);
    lwz(r4, stackPointerRegister, offsetof(ResumeFromException, target));
    x_mtctr(r4); // new dispatch group
    x_li32(r3, BAILOUT_RETURN_OK);
    lwz(r5, stackPointerRegister, offsetof(ResumeFromException, bailoutInfo));
#if defined(_PPC970_)
	x_nop(); // x_li32 is probably only one instruction.
#endif
	// Bye.
	bctr();
	ispew("^^ -- handleFailureWithHandlerTail(); -- ^^");
}

CodeOffsetLabel
MacroAssemblerPPCCompat::toggledJump(Label *label)
{
	ispew("[[ toggledJump(l)");
    m_buffer.ensureSpace(24);
    CodeOffsetLabel ret(nextOffset().getOffset());
    x_nop(); // This will be patched by ToggleToJmp.
    b(label);
    
    // This should always be a branch+4 bytes long, including the nop.
    MOZ_ASSERT(nextOffset().getOffset() - ret.offset() == PPC_B_STANZA_LENGTH + 4);
    ispew("  toggledJump(l) ]]");
    return ret;
}

/* Toggled calls are only used by the Baseline JIT, and then only in Debug mode;
   see Assembler::ToggleCall and Trampoline::generateDebugTrapHandler. Unlike all
   our other calls, these actually do set and branch with LR. */
CodeOffsetLabel
MacroAssemblerPPCCompat::toggledCall(JitCode *target, bool enabled)
{
	ispew("[[ toggledCall(jitcode, bool)");
	
#if DEBUG
	// Currently this code assumes that it will only ever call the debug trap
	// handler, and that this code is executed after the handler was created.
	JitCode *handler = GetJitContext()->runtime->jitRuntime()->debugTrapHandler(GetJitContext()->cx);
	MOZ_ASSERT(handler == target);
#endif
	
	// This call is always long.
    BufferOffset bo = nextOffset();
    CodeOffsetLabel offset(bo.getOffset());
    addPendingJump(bo, ImmPtr(target->raw()), Relocation::JITCODE);
    ma_li32Patchable(tempRegister, ImmPtr(target->raw()));
    if (enabled) {
    	// XXX. Not much we can do here unless we want to bloat all toggled
    	// calls on the G5 (not appealing either).
        x_mtctr(tempRegister);
        // Actually DO branch with LR.
        bctr(LinkB);
    } else {
        x_nop();
        x_nop();
    }
    
    // This should always be 16 bytes long.
    MOZ_ASSERT(nextOffset().getOffset() - offset.offset() == ToggledCallSize(nullptr));
    ispew("   toggledCall(jitcode, bool) ]]");
    return offset;
}

void
MacroAssemblerPPCCompat::branchTest32(Condition cond, Register lhs, Register rhs, Label *label)
{
	ispew("[[ branchTest32(cond, reg, reg, l)");
	MOZ_ASSERT(cond == Zero || cond == NonZero || cond == Signed || cond == NotSigned);
	if (cond == Zero || cond == NonZero) {
		and__rc(tempRegister, lhs, rhs);
		bc(cond, label);
	} else if (cond == Signed || cond == NotSigned) {
		MOZ_ASSERT(lhs == rhs);
		cmpwi(lhs, 0); // less than: signed, greater than: not signed
		bc(cond, label);
	} else {
		// XXX, for future expansion
		bc(ma_cmp(lhs, rhs, cond), label);
	}
	ispew("   branchTest32(cond, reg, reg, l) ]]");
}

void
MacroAssemblerPPCCompat::moveValue(const ValueOperand &src, const ValueOperand &dest)
{
	ispew("moveValue(vo, vo)");
    	
	Register s0 = src.typeReg(), d0 = dest.typeReg(),
		s1 = src.payloadReg(), d1 = dest.payloadReg();

	// Either one or both of the source registers could be the same as a
	// destination register.
	if (s1 == d0) {
		if (s0 == d1) {
			// If both are, this is just a swap of two registers.
			MOZ_ASSERT(d1 != tempRegister);
			MOZ_ASSERT(d0 != tempRegister);
			move32(d1, tempRegister);
			move32(d0, d1);
			move32(tempRegister, d0);
			return;
		}
		// If only one is, copy that source first.
		mozilla::Swap(s0, s1);
		mozilla::Swap(d0, d1);
	}

	if (s0 != d0)
		move32(s0, d0);
	if (s1 != d1)
		move32(s1, d1);
}

void
MacroAssemblerPPCCompat::branchPtrInNurseryRange(Condition cond, Register ptr, Register temp,
                                                  Label *label)
{
	ispew("[[ branchPtrInNurseryRange(cond, reg, reg, l)");
    MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
    MOZ_ASSERT(ptr != temp);
    MOZ_ASSERT(ptr != addressTempRegister);

    const Nursery &nursery = GetJitContext()->runtime->gcNursery();
    movePtr(ImmWord(-ptrdiff_t(nursery.start())), addressTempRegister);
    addPtr(ptr, addressTempRegister);
    branchPtr(cond == Assembler::Equal ? Assembler::Below : Assembler::AboveOrEqual,
              addressTempRegister, Imm32(nursery.nurserySize()), label);
    ispew("   branchPtrInNurseryRange(cond, reg, reg, l) ]]");
}

void
MacroAssemblerPPCCompat::branchValueIsNurseryObject(Condition cond, ValueOperand value,
                                                     Register temp, Label *label)
{
	ispew("[[ branchValueIsNurseryObject(cond, vo, reg, l)");
    MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);

    Label done;

    branchTestObject(Assembler::NotEqual, value, cond == Assembler::Equal ? &done : label);
    branchPtrInNurseryRange(cond, value.payloadReg(), temp, label);

    bind(&done);
    ispew("   branchValueIsNurseryObject(cond, vo, reg, l)");
}

void
MacroAssemblerPPCCompat::profilerEnterFrame(Register framePtr, Register scratch)
{
    AbsoluteAddress activation(GetJitContext()->runtime->addressOfProfilingActivation());
    loadPtr(activation, scratch);
    storePtr(framePtr, Address(scratch, JitActivation::offsetOfLastProfilingFrame()));
    storePtr(ImmPtr(nullptr), Address(scratch, JitActivation::offsetOfLastProfilingCallSite()));
}

void
MacroAssemblerPPCCompat::profilerExitFrame()
{
    branch(GetJitContext()->runtime->jitRuntime()->getProfilerExitFrameTail());
}

// irregexp
// 12 -> 21
void
MacroAssemblerPPC::load16ZeroExtendSwapped(
		const Address &address, const Register &dest)
{
	ispew("load16ZeroExtendSwapped(adr, reg)");
	ma_load(dest, address, SizeHalfWord, ZeroExtend, true);
} 

void
MacroAssemblerPPC::load16ZeroExtendSwapped(
		const BaseIndex &src, const Register &dest)
{
	ispew("load16ZeroExtendSwapped(bi, reg)");
	ma_load(dest, src, SizeHalfWord, ZeroExtend, true);
}

// 4321 -> 1234
void
MacroAssemblerPPC::load32ByteSwapped(const Address &address, Register dest)
{
	ispew("load32ByteSwapped(adr, reg)");
	ma_load(dest, address, SizeWord, ZeroExtend, true);
}

void
MacroAssemblerPPC::load32ByteSwapped(const BaseIndex &src, Register dest)
{
	ispew("load32ByteSwapped(bi, reg)");
	ma_load(dest, src, SizeWord, ZeroExtend, true);
}

// 4321 -> 2143
void
MacroAssemblerPPC::load32WordSwapped(const Address &address, Register dest)
{
	ispew("load32WordSwapped(adr, reg)");
	ma_load(dest, address, SizeWord); // NOT swapped
	rlwinm(dest, dest, 16, 0, 31); // rotlwi
}

void
MacroAssemblerPPC::load32WordSwapped(const BaseIndex &src, Register dest)
{
	ispew("load32WordSwapped(bi, reg)");
	ma_load(dest, src, SizeWord); // NOT swapped
	rlwinm(dest, dest, 16, 0, 31); // rotlwi
}

void
MacroAssemblerPPC::point5Double(FloatRegister fpr)
{
    x_lis(tempRegister, 0x3F00); // 0.5 = 0x3f000000
    stwu(tempRegister, stackPointerRegister, -4);
#if _PPC970_
    // Two nops are the minimum to break the stwu and lfs apart.
    x_nop();
    x_nop();
#endif
    lfs(fpr, stackPointerRegister, 0);
    addi(stackPointerRegister, stackPointerRegister, 4);
}

