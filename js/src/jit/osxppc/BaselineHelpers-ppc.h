/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#if !defined(jsion_baseline_helpers_ppc_h__)
#define jsion_baseline_helpers_ppc_h__

#include "jit/BaselineFrame.h"
#include "jit/BaselineIC.h"
#include "jit/BaselineRegisters.h"
#include "jit/MacroAssembler.h"

namespace js {
namespace jit {

// Distance from sp to the top Value inside an IC stub. Since we emulate x86
// by storing the return address on the stack, this is one word in.
static const size_t ICStackValueOffset = sizeof(void *);

inline void
EmitRestoreTailCallReg(MacroAssembler &masm)
{
    masm.lwz(BaselineTailCallReg, stackPointerRegister, 0);
    masm.addi(stackPointerRegister, stackPointerRegister, 4);
}

inline void
EmitRepushTailCallReg(MacroAssembler &masm)
{
    masm.stwu(BaselineTailCallReg, stackPointerRegister, -4);
}

inline void
EmitCallIC(CodeOffsetLabel *patchOffset, MacroAssembler &masm)
{
    ispew("[[ EmitCallIC");

    // Move ICEntry offset into BaselineStubReg
    CodeOffsetLabel offset = masm.movWithPatch(ImmWord(-1), BaselineStubReg);
    *patchOffset = offset;

    // Load stub pointer into BaselineStubReg
    masm.loadPtr(Address(BaselineStubReg, ICEntry::offsetOfFirstStub()),
        BaselineStubReg);

    // Load stubcode pointer from BaselineStubEntry.
    // Baseline R2 won't be active when we call ICs, so we can use r3.
    MOZ_ASSERT(R2 == ValueOperand(r4, r3));
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfStubCode()), r3);

    // Call the stubcode.
    masm.call(r3);

    ispew("   EmitCallIC ]]");
}

inline void
EmitEnterTypeMonitorIC(MacroAssembler &masm,
                       size_t monitorStubOffset = ICMonitoredStub::offsetOfFirstMonitorStub())
{
    ispew("[[ EmitEnterTypeMonitorIC");

    // This is expected to be called from within an IC, when BaselineStubReg
    // is properly initialized to point to the stub.
    masm.loadPtr(Address(BaselineStubReg, (int32_t)monitorStubOffset),
        BaselineStubReg);

    // Load stubcode pointer from BaselineStubEntry.
    // Baseline R2 won't be active when we call ICs, so we can use r3.
    MOZ_ASSERT(R2 == ValueOperand(r4, r3));
    masm.loadPtr(Address(BaselineStubReg, (int32_t)ICStub::offsetOfStubCode()),
        r3);

    // Jump to the stubcode (not as a subroutine).
// XXX Consider rescheduling this.
    masm.branch(r3);

    ispew("   EmitEnterTypeMonitorIC ]]");
}

inline void
EmitReturnFromIC(MacroAssembler &masm)
{
    ispew("[[ EmitReturnFromIC");
    masm.ret();
    ispew("   EmitReturnFromIC ]]");
}

inline void
EmitChangeICReturnAddress(MacroAssembler &masm, Register reg)
{
    ispew("EmitChangeICReturnAddress");
	masm.stw(reg, stackPointerRegister, 0); // overwrite return address
}

inline void
EmitTailCallVM(JitCode *target, MacroAssembler &masm, uint32_t argSize)
{
    ispew("[[ EmitTailCallVM");
    // We assume during this that Baseline R0 and R1 have been pushed, and
    // that Baseline R2 is unused. The argregs will not be set until we
    // actually call the VMWrapper, so we can trample R2's set.
    MOZ_ASSERT(R2 == ValueOperand(r4, r3));

    // Compute frame size.
    masm.x_mr(r3, BaselineFrameReg);
    masm.addi(r3, r3, BaselineFrame::FramePointerOffset);
    masm.subf(r3, BaselineStackReg, r3); // r3 = r3 - sp (fp > sp)

    // Store frame size without VMFunction arguments for GC marking.
    masm.addi(r4, r3, -(argSize));
    masm.store32(r4,
        Address(BaselineFrameReg, BaselineFrame::reverseOffsetOfFrameSize()));

    // Push frame descriptor and perform the tail call. The call is to
    // Ion code, so it does not need to be ABI compliant.
    masm.addi(stackPointerRegister, stackPointerRegister, -((uint16_t)sizeof(CommonFrameLayout)));
    masm.makeFrameDescriptor(r3, JitFrame_BaselineJS);
    masm.stw(BaselineTailCallReg, stackPointerRegister, CommonFrameLayout::offsetOfReturnAddress());
    masm.stw(r3, stackPointerRegister, CommonFrameLayout::offsetOfDescriptor());
    masm.branch(target);

    ispew("   EmitTailCallVM ]]");
}

inline void
EmitCreateStubFrameDescriptor(MacroAssembler &masm, Register reg)
{
    // Compute stub frame size. We have to add two pointers: the stub reg
    // and previous fake-o frame pointer pushed by EmitEnterStubFrame.
    ispew("EmitCreateStubFrameDescriptor");
    MOZ_ASSERT(reg != r0); // this will turn the addi into an li!
    
    masm.x_mr(reg, BaselineFrameReg);
    masm.addi(reg, reg, sizeof(void *) * 2); // should be 8, should fit
    masm.subf(reg, BaselineStackReg, reg); // fp > sp
    masm.makeFrameDescriptor(reg, JitFrame_BaselineStub);
}

inline void
EmitCallVM(JitCode *target, MacroAssembler &masm)
{
    ispew("[[ EmitCallVM");

    // Use r12, since we can't use r0 with the addi.
    EmitCreateStubFrameDescriptor(masm, r12);
    masm.push(r12);
    masm.call(target);

    ispew("   EmitCallVM ]]");
}

// Size of values pushed by EmitEnterStubFrame.
struct BaselineStubFrame {
    uintptr_t savedFrame;
    uintptr_t savedStub;
    uintptr_t returnAddress;
    uintptr_t descriptor;
};
static const uint32_t STUB_FRAME_SIZE = sizeof(BaselineStubFrame);
static const uint32_t STUB_FRAME_SAVED_STUB_OFFSET = offsetof(BaselineStubFrame, savedStub);

inline void
EmitEnterStubFrame(MacroAssembler &masm, Register scratch)
{
    ispew("[[ EmitEnterStubFrame");
    MOZ_ASSERT(scratch != BaselineTailCallReg);
    MOZ_ASSERT(BaselineStackReg == stackPointerRegister);
    
    EmitRestoreTailCallReg(masm);

    // Compute frame size.
    masm.x_mr(scratch, BaselineFrameReg);
    masm.add32(Imm32(BaselineFrame::FramePointerOffset), scratch);
    masm.subf(scratch, BaselineStackReg, scratch);

    masm.store32(scratch,
        Address(BaselineFrameReg, BaselineFrame::reverseOffsetOfFrameSize()));

    // Push frame descriptor and return address.
    masm.makeFrameDescriptor(scratch, JitFrame_BaselineJS);
    masm.addi(stackPointerRegister, stackPointerRegister, -((uint16_t)STUB_FRAME_SIZE));
    masm.stw(scratch, stackPointerRegister, offsetof(BaselineStubFrame, descriptor));
    masm.stw(BaselineTailCallReg, stackPointerRegister, offsetof(BaselineStubFrame, returnAddress));

    // Save old frame pointer, stack pointer and stub reg.
    masm.stw(BaselineStubReg, stackPointerRegister, offsetof(BaselineStubFrame, savedStub));
    masm.stw(BaselineFrameReg, stackPointerRegister, offsetof(BaselineStubFrame, savedFrame));

    masm.x_mr(BaselineFrameReg, BaselineStackReg);


    ispew("   EmitEnterStubFrame ]]");
}

inline void
EmitLeaveStubFrame(MacroAssembler &masm, bool calledIntoIon = false)
{
    ispew("[[ EmitLeaveStubFrame");
    // Ion frames do not save and restore the frame pointer. If we called
    // into Ion, we have to restore the stack pointer from the frame descriptor.
    // If we performed a VM call, the descriptor has been popped already so
    // in that case we use the frame pointer.
    if (calledIntoIon) {
        // Unwind the descriptor.
        masm.pop(r0);
        masm.x_srwi(r0, r0, FRAMESIZE_SHIFT);
        masm.add(stackPointerRegister, r0, stackPointerRegister);
    } else {
        masm.x_mr(stackPointerRegister, BaselineFrameReg);
    }
    
    // Restore registers.
    masm.lwz(BaselineTailCallReg, stackPointerRegister, offsetof(BaselineStubFrame, returnAddress));
    masm.lwz(BaselineFrameReg, stackPointerRegister, offsetof(BaselineStubFrame, savedFrame));
    masm.lwz(BaselineStubReg, stackPointerRegister, offsetof(BaselineStubFrame, savedStub));
    // Destroy the frame, removing the descriptor with it.
    masm.addi(stackPointerRegister, stackPointerRegister, STUB_FRAME_SIZE-4);
    // Overwrite the remaining word with the return address, so the stack matches the state before entry.
    masm.stw(BaselineTailCallReg, stackPointerRegister, 0);
    ispew("   EmitLeaveStubFrame ]]");
}

inline void
EmitStowICValues(MacroAssembler &masm, int values)
{
    MOZ_ASSERT(values >= 0 && values <= 2);
    masm.pop(BaselineTailCallReg);
    switch(values) {
      case 1:
        // Stow R0
        masm.push3(R0.payloadReg(), R0.typeReg(), BaselineTailCallReg);
        break;
      case 2:
        // Stow R0 and R1
        masm.push5(R0.payloadReg(), R0.typeReg(),
            R1.payloadReg(), R1.typeReg(), BaselineTailCallReg);
        break;
    }
}

inline void
EmitUnstowICValues(MacroAssembler &masm, int values, bool discard = false)
{
    MOZ_ASSERT(values >= 0 && values <= 2);
    if (discard) {
        // sizeof(Value) == double
    	masm.pop(BaselineTailCallReg);
        masm.addi(stackPointerRegister, stackPointerRegister, (8 * values) - 4);
        masm.stw(BaselineTailCallReg, stackPointerRegister, 0);
        return;
    }
    switch(values) {
      case 1:
        // Unstow R0
        masm.pop3(BaselineTailCallReg, R0.typeReg(), R0.payloadReg());
        break;
      case 2:
        // Unstow R0 and R1
        masm.pop5(BaselineTailCallReg, R1.typeReg(), R1.payloadReg(),
            R0.typeReg(), R0.payloadReg());
        break;
    }
    masm.stwu(BaselineTailCallReg, stackPointerRegister, -4);
}

inline void
EmitCallTypeUpdateIC(MacroAssembler &masm, JitCode *code, uint32_t objectOffset)
{
    ispew("[[ EmitCallTypeUpdateIC");
    // Baseline R0 contains the value that needs to be typechecked.
    // The object we're updating is a boxed Value on the stack, at offset
    // objectOffset from SP, excluding the return address. R2 should be
    // free, so we can use it as temporary registers.
    MOZ_ASSERT(R2 == ValueOperand(r4, r3));

    masm.stwu(BaselineStubReg, stackPointerRegister, -4);

    // This is expected to be called from within an IC, when BaselineStubReg
    // is properly initialized to point to the stub. Since we are inside an
    // IC we ourselves generated, ABI-compliance is unimportant.
    masm.loadPtr(Address(BaselineStubReg,
                (int32_t)ICUpdatedStub::offsetOfFirstUpdateStub()),
                BaselineStubReg);

    // Load stubcode pointer from BaselineStubReg into BaselineTailCallReg.
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfStubCode()), r3);

    // Call the stubcode.
    masm.call(r3);

    // Restore the old stub reg.
    masm.lwz(BaselineStubReg, stackPointerRegister, 0);
    masm.addi(stackPointerRegister, stackPointerRegister, 4);

    // The update IC will store 0 or 1 in R1.scratchReg() reflecting if the
    // value in R0 type-checked properly or not.
    masm.cmpwi(R1.scratchReg(), 1);
    // We can't mark this likely; it's used in too many codepaths.
    BufferOffset success = masm._bc(0, Assembler::Equal);

    // If the IC failed, then call the update fallback function.
    EmitEnterStubFrame(masm, R1.scratchReg());
    masm.loadValue(Address(BaselineStackReg,
        STUB_FRAME_SIZE + objectOffset), R1);
    masm.push5(R0.payloadReg(), R0.typeReg(),
        R1.payloadReg(), R1.typeReg(),
        BaselineStubReg);

    // Load previous frame pointer, push BaselineFrame *.
    masm.loadPtr(Address(BaselineFrameReg, 0), R0.scratchReg());
    masm.pushBaselineFramePtr(R0.scratchReg(), R0.scratchReg());

    EmitCallVM(code, masm);
    EmitLeaveStubFrame(masm);

    // Success at end.
    masm.bindSS(success);
    ispew("   EmitCallTypeUpdateIC ]]");
}

template <typename AddrType>
inline void
EmitPreBarrier(MacroAssembler &masm, const AddrType &addr, MIRType type)
{
    ispew("[[ EmitPreBarrier");
    masm.patchableCallPreBarrier(addr, type);
    ispew("   EmitPreBarrier ]]");
}

inline void
EmitStubGuardFailure(MacroAssembler &masm)
{
    ispew("[[ EmitStubGuardFailure");
    // We assume Baseline R2 and component registers are free.
    MOZ_ASSERT(R2 == ValueOperand(r4, r3));

    // NOTE: This routine assumes that the stub guard code left the stack in
    // the same state it was in when it was entered.
    // BaselineStubEntry points to the current stub.

    // Load next stub into BaselineStubReg. (DO THIS FIRST!)
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfNext()), BaselineStubReg);
    // Load stubcode pointer from BaselineStubEntry into scratch register.
    masm.loadPtr(Address(BaselineStubReg, ICStub::offsetOfStubCode()), r3);
    masm.x_mtctr(r3);
#if _PPC970_
	masm.x_nop();
	masm.x_nop();
	masm.x_nop();
	masm.x_nop();
#endif
    // The return address is already on the stack, so just jump.
    masm.bctr();

    ispew("   EmitStubGuardFailure ]]");
}


} // namespace jit
} // namespace js

#endif
