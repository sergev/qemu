/*
 *  MIPS emulation helpers for qemu.
 *
 *  Copyright (c) 2004-2005 Jocelyn Mayer
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <signal.h>

#include "cpu.h"
#include "sysemu/kvm.h"
#include "exec/cpu_ldst.h"
#include "retrobsd_syscall.h"

enum {
    TLBRET_XI = -6,
    TLBRET_RI = -5,
    TLBRET_DIRTY = -4,
    TLBRET_INVALID = -3,
    TLBRET_NOMATCH = -2,
    TLBRET_BADADDR = -1,
    TLBRET_MATCH = 0
};

#if !defined(CONFIG_USER_ONLY)

/* no MMU emulation */
int no_mmu_map_address (CPUMIPSState *env, hwaddr *physical, int *prot,
                        target_ulong address, int rw, int access_type)
{
    *physical = address;
    *prot = PAGE_READ | PAGE_WRITE;
    return TLBRET_MATCH;
}

/* fixed mapping MMU emulation */
int fixed_mmu_map_address (CPUMIPSState *env, hwaddr *physical, int *prot,
                           target_ulong address, int rw, int access_type)
{
    if (address <= (int32_t)0x7FFFFFFFUL) {
        if (!(env->CP0_Status & (1 << CP0St_ERL)))
            *physical = address + 0x40000000UL;
        else
            *physical = address;
    } else if (address <= (int32_t)0xBFFFFFFFUL)
        *physical = address & 0x1FFFFFFF;
    else
        *physical = address;

    *prot = PAGE_READ | PAGE_WRITE;
    return TLBRET_MATCH;
}

/* MIPS32/MIPS64 R4000-style MMU emulation */
int r4k_map_address (CPUMIPSState *env, hwaddr *physical, int *prot,
                     target_ulong address, int rw, int access_type)
{
    uint8_t ASID = env->CP0_EntryHi & 0xFF;
    int i;

    for (i = 0; i < env->tlb->tlb_in_use; i++) {
        r4k_tlb_t *tlb = &env->tlb->mmu.r4k.tlb[i];
        /* 1k pages are not supported. */
        target_ulong mask = tlb->PageMask | ~(TARGET_PAGE_MASK << 1);
        target_ulong tag = address & ~mask;
        target_ulong VPN = tlb->VPN & ~mask;
#if defined(TARGET_MIPS64)
        tag &= env->SEGMask;
#endif

        /* Check ASID, virtual page number & size */
        if ((tlb->G == 1 || tlb->ASID == ASID) && VPN == tag && !tlb->EHINV) {
            /* TLB match */
            int n = !!(address & mask & ~(mask >> 1));
            /* Check access rights */
            if (!(n ? tlb->V1 : tlb->V0)) {
                return TLBRET_INVALID;
            }
            if (rw == MMU_INST_FETCH && (n ? tlb->XI1 : tlb->XI0)) {
                return TLBRET_XI;
            }
            if (rw == MMU_DATA_LOAD && (n ? tlb->RI1 : tlb->RI0)) {
                return TLBRET_RI;
            }
            if (rw != MMU_DATA_STORE || (n ? tlb->D1 : tlb->D0)) {
                *physical = tlb->PFN[n] | (address & (mask >> 1));
                *prot = PAGE_READ;
                if (n ? tlb->D1 : tlb->D0)
                    *prot |= PAGE_WRITE;
                return TLBRET_MATCH;
            }
            return TLBRET_DIRTY;
        }
    }
    return TLBRET_NOMATCH;
}

static int get_physical_address (CPUMIPSState *env, hwaddr *physical,
                                int *prot, target_ulong real_address,
                                int rw, int access_type)
{
    /* User mode can only access useg/xuseg */
    int user_mode = (env->hflags & MIPS_HFLAG_MODE) == MIPS_HFLAG_UM;
    int supervisor_mode = (env->hflags & MIPS_HFLAG_MODE) == MIPS_HFLAG_SM;
    int kernel_mode = !user_mode && !supervisor_mode;
#if defined(TARGET_MIPS64)
    int UX = (env->CP0_Status & (1 << CP0St_UX)) != 0;
    int SX = (env->CP0_Status & (1 << CP0St_SX)) != 0;
    int KX = (env->CP0_Status & (1 << CP0St_KX)) != 0;
#endif
    int ret = TLBRET_MATCH;
    /* effective address (modified for KVM T&E kernel segments) */
    target_ulong address = real_address;

#if 0
    qemu_log("user mode %d h %08x\n", user_mode, env->hflags);
#endif

#define USEG_LIMIT      0x7FFFFFFFUL
#define KSEG0_BASE      0x80000000UL
#define KSEG1_BASE      0xA0000000UL
#define KSEG2_BASE      0xC0000000UL
#define KSEG3_BASE      0xE0000000UL

#define KVM_KSEG0_BASE  0x40000000UL
#define KVM_KSEG2_BASE  0x60000000UL

    if (kvm_enabled()) {
        /* KVM T&E adds guest kernel segments in useg */
        if (real_address >= KVM_KSEG0_BASE) {
            if (real_address < KVM_KSEG2_BASE) {
                /* kseg0 */
                address += KSEG0_BASE - KVM_KSEG0_BASE;
            } else if (real_address <= USEG_LIMIT) {
                /* kseg2/3 */
                address += KSEG2_BASE - KVM_KSEG2_BASE;
            }
        }
    }

    if (address <= USEG_LIMIT) {
        /* useg */
        if (env->CP0_Status & (1 << CP0St_ERL)) {
            *physical = address & 0xFFFFFFFF;
            *prot = PAGE_READ | PAGE_WRITE;
        } else {
            ret = env->tlb->map_address(env, physical, prot, real_address, rw, access_type);
        }
#if defined(TARGET_MIPS64)
    } else if (address < 0x4000000000000000ULL) {
        /* xuseg */
        if (UX && address <= (0x3FFFFFFFFFFFFFFFULL & env->SEGMask)) {
            ret = env->tlb->map_address(env, physical, prot, real_address, rw, access_type);
        } else {
            ret = TLBRET_BADADDR;
        }
    } else if (address < 0x8000000000000000ULL) {
        /* xsseg */
        if ((supervisor_mode || kernel_mode) &&
            SX && address <= (0x7FFFFFFFFFFFFFFFULL & env->SEGMask)) {
            ret = env->tlb->map_address(env, physical, prot, real_address, rw, access_type);
        } else {
            ret = TLBRET_BADADDR;
        }
    } else if (address < 0xC000000000000000ULL) {
        /* xkphys */
        if (kernel_mode && KX &&
            (address & 0x07FFFFFFFFFFFFFFULL) <= env->PAMask) {
            *physical = address & env->PAMask;
            *prot = PAGE_READ | PAGE_WRITE;
        } else {
            ret = TLBRET_BADADDR;
        }
    } else if (address < 0xFFFFFFFF80000000ULL) {
        /* xkseg */
        if (kernel_mode && KX &&
            address <= (0xFFFFFFFF7FFFFFFFULL & env->SEGMask)) {
            ret = env->tlb->map_address(env, physical, prot, real_address, rw, access_type);
        } else {
            ret = TLBRET_BADADDR;
        }
#endif
    } else if (address < (int32_t)KSEG1_BASE) {
        /* kseg0 */
        if (kernel_mode) {
            *physical = address - (int32_t)KSEG0_BASE;
            *prot = PAGE_READ | PAGE_WRITE;
        } else {
            ret = TLBRET_BADADDR;
        }
    } else if (address < (int32_t)KSEG2_BASE) {
        /* kseg1 */
        if (kernel_mode) {
            *physical = address - (int32_t)KSEG1_BASE;
            *prot = PAGE_READ | PAGE_WRITE;
        } else {
            ret = TLBRET_BADADDR;
        }
    } else if (address < (int32_t)KSEG3_BASE) {
        /* sseg (kseg2) */
        if (supervisor_mode || kernel_mode) {
            ret = env->tlb->map_address(env, physical, prot, real_address, rw, access_type);
        } else {
            ret = TLBRET_BADADDR;
        }
    } else {
        /* kseg3 */
        /* XXX: debug segment is not emulated */
        if (kernel_mode) {
            ret = env->tlb->map_address(env, physical, prot, real_address, rw, access_type);
        } else {
            ret = TLBRET_BADADDR;
        }
    }
#if 0
    qemu_log(TARGET_FMT_lx " %d %d => %" HWADDR_PRIx " %d (%d)\n",
            address, rw, access_type, *physical, *prot, ret);
#endif

    return ret;
}
#endif

static void raise_mmu_exception(CPUMIPSState *env, target_ulong address,
                                int rw, int tlb_error)
{
    CPUState *cs = CPU(mips_env_get_cpu(env));
    int exception = 0, error_code = 0;

    if (rw == MMU_INST_FETCH) {
        error_code |= EXCP_INST_NOTAVAIL;
    }

    switch (tlb_error) {
    default:
    case TLBRET_BADADDR:
        /* Reference to kernel address from user mode or supervisor mode */
        /* Reference to supervisor address from user mode */
        if (rw == MMU_DATA_STORE) {
            exception = EXCP_AdES;
        } else {
            exception = EXCP_AdEL;
        }
        break;
    case TLBRET_NOMATCH:
        /* No TLB match for a mapped address */
        if (rw == MMU_DATA_STORE) {
            exception = EXCP_TLBS;
        } else {
            exception = EXCP_TLBL;
        }
        error_code |= EXCP_TLB_NOMATCH;
        break;
    case TLBRET_INVALID:
        /* TLB match with no valid bit */
        if (rw == MMU_DATA_STORE) {
            exception = EXCP_TLBS;
        } else {
            exception = EXCP_TLBL;
        }
        break;
    case TLBRET_DIRTY:
        /* TLB match but 'D' bit is cleared */
        exception = EXCP_LTLBL;
        break;
    case TLBRET_XI:
        /* Execute-Inhibit Exception */
        if (env->CP0_PageGrain & (1 << CP0PG_IEC)) {
            exception = EXCP_TLBXI;
        } else {
            exception = EXCP_TLBL;
        }
        break;
    case TLBRET_RI:
        /* Read-Inhibit Exception */
        if (env->CP0_PageGrain & (1 << CP0PG_IEC)) {
            exception = EXCP_TLBRI;
        } else {
            exception = EXCP_TLBL;
        }
        break;
    }
    /* Raise exception */
    env->CP0_BadVAddr = address;
    env->CP0_Context = (env->CP0_Context & ~0x007fffff) |
                       ((address >> 9) & 0x007ffff0);
    env->CP0_EntryHi =
        (env->CP0_EntryHi & 0xFF) | (address & (TARGET_PAGE_MASK << 1));
#if defined(TARGET_MIPS64)
    env->CP0_EntryHi &= env->SEGMask;
    env->CP0_XContext = (env->CP0_XContext & ((~0ULL) << (env->SEGBITS - 7))) |
                        ((address & 0xC00000000000ULL) >> (55 - env->SEGBITS)) |
                        ((address & ((1ULL << env->SEGBITS) - 1) & 0xFFFFFFFFFFFFE000ULL) >> 9);
#endif
    cs->exception_index = exception;
    env->error_code = error_code;
}

#if !defined(CONFIG_USER_ONLY)
hwaddr mips_cpu_get_phys_page_debug(CPUState *cs, vaddr addr)
{
    MIPSCPU *cpu = MIPS_CPU(cs);
    hwaddr phys_addr;
    int prot;

    if (get_physical_address(&cpu->env, &phys_addr, &prot, addr, 0,
                             ACCESS_INT) != 0) {
        return -1;
    }
    return phys_addr;
}
#endif

int mips_cpu_handle_mmu_fault(CPUState *cs, vaddr address, int rw,
                              int mmu_idx)
{
    MIPSCPU *cpu = MIPS_CPU(cs);
    CPUMIPSState *env = &cpu->env;
#if !defined(CONFIG_USER_ONLY)
    hwaddr physical;
    int prot;
    int access_type;
#endif
    int ret = 0;

#if 0
    log_cpu_state(cs, 0);
#endif
    qemu_log_mask(CPU_LOG_MMU,
              "%s pc " TARGET_FMT_lx " ad %" VADDR_PRIx " rw %d mmu_idx %d\n",
              __func__, env->active_tc.PC, address, rw, mmu_idx);

    /* data access */
#if !defined(CONFIG_USER_ONLY)
    /* XXX: put correct access by using cpu_restore_state()
       correctly */
    access_type = ACCESS_INT;
    ret = get_physical_address(env, &physical, &prot,
                               address, rw, access_type);
    qemu_log_mask(CPU_LOG_MMU,
             "%s address=%" VADDR_PRIx " ret %d physical " TARGET_FMT_plx
             " prot %d\n",
             __func__, address, ret, physical, prot);
    if (ret == TLBRET_MATCH) {
        tlb_set_page(cs, address & TARGET_PAGE_MASK,
                     physical & TARGET_PAGE_MASK, prot | PAGE_EXEC,
                     mmu_idx, TARGET_PAGE_SIZE);
        ret = 0;
    } else if (ret < 0)
#endif
    {
        raise_mmu_exception(env, address, rw, ret);
        ret = 1;
    }

    return ret;
}

#if !defined(CONFIG_USER_ONLY)
hwaddr cpu_mips_translate_address(CPUMIPSState *env, target_ulong address, int rw)
{
    hwaddr physical;
    int prot;
    int access_type;
    int ret = 0;

    /* data access */
    access_type = ACCESS_INT;
    ret = get_physical_address(env, &physical, &prot,
                               address, rw, access_type);
    if (ret != TLBRET_MATCH) {
        raise_mmu_exception(env, address, rw, ret);
        return -1LL;
    } else {
        return physical;
    }
}

static const char * const excp_names[EXCP_LAST + 1] = {
    [EXCP_RESET] = "reset",
    [EXCP_SRESET] = "soft reset",
    [EXCP_DSS] = "debug single step",
    [EXCP_DINT] = "debug interrupt",
    [EXCP_NMI] = "non-maskable interrupt",
    [EXCP_MCHECK] = "machine check",
    [EXCP_EXT_INTERRUPT] = "interrupt",
    [EXCP_DFWATCH] = "deferred watchpoint",
    [EXCP_DIB] = "debug instruction breakpoint",
    [EXCP_IWATCH] = "instruction fetch watchpoint",
    [EXCP_AdEL] = "address error load",
    [EXCP_AdES] = "address error store",
    [EXCP_TLBF] = "TLB refill",
    [EXCP_IBE] = "instruction bus error",
    [EXCP_DBp] = "debug breakpoint",
    [EXCP_SYSCALL] = "syscall",
    [EXCP_BREAK] = "break",
    [EXCP_CpU] = "coprocessor unusable",
    [EXCP_RI] = "reserved instruction",
    [EXCP_OVERFLOW] = "arithmetic overflow",
    [EXCP_TRAP] = "trap",
    [EXCP_FPE] = "floating point",
    [EXCP_DDBS] = "debug data break store",
    [EXCP_DWATCH] = "data watchpoint",
    [EXCP_LTLBL] = "TLB modify",
    [EXCP_TLBL] = "TLB load",
    [EXCP_TLBS] = "TLB store",
    [EXCP_DBE] = "data bus error",
    [EXCP_DDBL] = "debug data break load",
    [EXCP_THREAD] = "thread",
    [EXCP_MDMX] = "MDMX",
    [EXCP_C2E] = "precise coprocessor 2",
    [EXCP_CACHE] = "cache error",
    [EXCP_TLBXI] = "TLB execute-inhibit",
    [EXCP_TLBRI] = "TLB read-inhibit",
    [EXCP_MSADIS] = "MSA disabled",
    [EXCP_MSAFPE] = "MSA floating point",
};
#endif

target_ulong exception_resume_pc (CPUMIPSState *env)
{
    target_ulong bad_pc;
    target_ulong isa_mode;

    isa_mode = !!(env->hflags & MIPS_HFLAG_M16);
    bad_pc = env->active_tc.PC | isa_mode;
    if (env->hflags & MIPS_HFLAG_BMASK) {
        /* If the exception was raised from a delay slot, come back to
           the jump.  */
        bad_pc -= (env->hflags & MIPS_HFLAG_B16 ? 2 : 4);
    }

    return bad_pc;
}

#if !defined(CONFIG_USER_ONLY)
static void set_hflags_for_handler (CPUMIPSState *env)
{
    /* Exception handlers are entered in 32-bit mode.  */
    env->hflags &= ~(MIPS_HFLAG_M16);
    /* ...except that microMIPS lets you choose.  */
    if (env->insn_flags & ASE_MICROMIPS) {
        env->hflags |= (!!(env->CP0_Config3
                           & (1 << CP0C3_ISA_ON_EXC))
                        << MIPS_HFLAG_M16_SHIFT);
    }
}

static inline void set_badinstr_registers(CPUMIPSState *env)
{
    if (env->hflags & MIPS_HFLAG_M16) {
        /* TODO: add BadInstr support for microMIPS */
        return;
    }
    if (env->CP0_Config3 & (1 << CP0C3_BI)) {
        env->CP0_BadInstr = cpu_ldl_code(env, env->active_tc.PC);
    }
    if ((env->CP0_Config3 & (1 << CP0C3_BP)) &&
        (env->hflags & MIPS_HFLAG_BMASK)) {
        env->CP0_BadInstrP = cpu_ldl_code(env, env->active_tc.PC - 4);
    }
}
#endif

static const char * const retrobsd_syscall_name[SYS_LAST + 1] = {
    [SYS_exit] = "_exit",
    [SYS_fork] = "fork",
    [SYS_read] = "read",
    [SYS_write] = "write",
    [SYS_open] = "open",
    [SYS_close] = "close",
    [SYS_wait4] = "wait4",
    [SYS_link] = "link",
    [SYS_unlink] = "unlink",
    [SYS_execv] = "execv",
    [SYS_chdir] = "chdir",
    [SYS_fchdir] = "fchdir",
    [SYS_mknod] = "mknod",
    [SYS_chmod] = "chmod",
    [SYS_chown] = "chown",
    [SYS_chflags] = "chflags",
    [SYS_fchflags] = "fchflags",
    [SYS_lseek] = "lseek",
    [SYS_getpid] = "getpid",
    [SYS_mount] = "mount",
    [SYS_umount] = "umount",
    [SYS___sysctl] = "__sysctl",
    [SYS_getuid] = "getuid",
    [SYS_geteuid] = "geteuid",
    [SYS_ptrace] = "ptrace",
    [SYS_getppid] = "getppid",
    [SYS_statfs] = "statfs",
    [SYS_fstatfs] = "fstatfs",
    [SYS_getfsstat] = "getfsstat",
    [SYS_sigaction] = "sigaction",
    [SYS_sigprocmask] = "sigprocmask",
    [SYS_access] = "access",
    [SYS_sigpending] = "sigpending",
    [SYS_sigaltstack] = "sigaltstack",
    [SYS_sync] = "sync",
    [SYS_kill] = "kill",
    [SYS_stat] = "stat",
    [SYS_lstat] = "lstat",
    [SYS_dup] = "dup",
    [SYS_pipe] = "pipe",
    [SYS_profil] = "profil",
    [SYS_setuid] = "setuid",
    [SYS_seteuid] = "seteuid",
    [SYS_getgid] = "getgid",
    [SYS_getegid] = "getegid",
    [SYS_setgid] = "setgid",
    [SYS_setegid] = "setegid",
    [SYS_kmemdev] = "kmemdev",
    [SYS_phys] = "phys",
    [SYS_lock] = "lock",
    [SYS_ioctl] = "ioctl",
    [SYS_reboot] = "reboot",
    [SYS_sigwait] = "sigwait",
    [SYS_symlink] = "symlink",
    [SYS_readlink] = "readlink",
    [SYS_execve] = "execve",
    [SYS_umask] = "umask",
    [SYS_chroot] = "chroot",
    [SYS_fstat] = "fstat",
    [SYS_pselect] = "pselect",
    [SYS_vfork] = "vfork",
    [SYS_sbrk] = "sbrk",
    [SYS_rdglob] = "rdglob",
    [SYS_wrglob] = "wrglob",
    [SYS_msec] = "msec",
    [SYS_vhangup] = "vhangup",
    [SYS_getgroups] = "getgroups",
    [SYS_setgroups] = "setgroups",
    [SYS_getpgrp] = "getpgrp",
    [SYS_setpgrp] = "setpgrp",
    [SYS_setitimer] = "setitimer",
    [SYS_swapon] = "swapon",
    [SYS_getitimer] = "getitimer",
    [SYS_getdtablesize] = "getdtablesize",
    [SYS_dup2] = "dup2",
    [SYS_fcntl] = "fcntl",
    [SYS_select] = "select",
    [SYS_fsync] = "fsync",
    [SYS_setpriority] = "setpriority",
    [SYS_socket] = "socket",
    [SYS_connect] = "connect",
    [SYS_accept] = "accept",
    [SYS_getpriority] = "getpriority",
    [SYS_send] = "send",
    [SYS_recv] = "recv",
    [SYS_sigreturn] = "sigreturn",
    [SYS_bind] = "bind",
    [SYS_setsockopt] = "setsockopt",
    [SYS_listen] = "listen",
    [SYS_sigsuspend] = "sigsuspend",
    [SYS_sigstack] = "sigstack",
    [SYS_recvmsg] = "recvmsg",
    [SYS_sendmsg] = "sendmsg",
    [SYS_gettimeofday] = "gettimeofday",
    [SYS_getrusage] = "getrusage",
    [SYS_getsockopt] = "getsockopt",
    [SYS_readv] = "readv",
    [SYS_writev] = "writev",
    [SYS_settimeofday] = "settimeofday",
    [SYS_fchown] = "fchown",
    [SYS_fchmod] = "fchmod",
    [SYS_recvfrom] = "recvfrom",
    [SYS_rename] = "rename",
    [SYS_truncate] = "truncate",
    [SYS_ftruncate] = "ftruncate",
    [SYS_flock] = "flock",
    [SYS_sendto] = "sendto",
    [SYS_shutdown] = "shutdown",
    [SYS_socketpair] = "socketpair",
    [SYS_mkdir] = "mkdir",
    [SYS_rmdir] = "rmdir",
    [SYS_utimes] = "utimes",
    [SYS_adjtime] = "adjtime",
    [SYS_getpeername] = "getpeername",
    [SYS_getrlimit] = "getrlimit",
    [SYS_setrlimit] = "setrlimit",
    [SYS_killpg] = "killpg",
    [SYS_setquota] = "setquota",
    [SYS_quota] = "quota",
    [SYS_getsockname] = "getsockname",
    [SYS_ustore] = "ustore",
    [SYS_ufetch] = "ufetch",
    [SYS_ucall] = "ucall",
};

static void print_retrobsd_string(CPUMIPSState *env, target_ulong ptr)
{
    fprintf(qemu_logfile, TARGET_FMT_lx, ptr);
    if (ptr >= 0x7f008000 && ptr < 0x7f020000) {
        // Good pointer.
        fprintf(qemu_logfile, " \"");
        for (;;) {
            char ch = cpu_ldub_data(env, ptr++);
            if (!ch)
                break;
            fputc(ch, qemu_logfile);
        }
        fprintf(qemu_logfile, "\"");
    }
}

static void print_string_vector(CPUMIPSState *env, target_ulong ptr)
{
    fprintf(qemu_logfile, TARGET_FMT_lx, ptr);
    if (ptr >= 0x7f008000 && ptr < 0x7f020000) {
        // Good pointer.
        int sp = cpu_ldl_data(env, ptr);
        fprintf(qemu_logfile, " [");
        while (sp != 0) {
            print_retrobsd_string(env, sp);
            fprintf(qemu_logfile, ", ");
            ptr += 4;
            sp = cpu_ldl_data(env, ptr);
        }
        fprintf(qemu_logfile, "0]");
    }
}

static void print_retrobsd_syscall(CPUMIPSState *env)
{
    int opcode = cpu_ldl_code(env, env->CP0_EPC);
    int syscall = (opcode >> 6) & 0xfffff;

    fprintf(qemu_logfile, "--- Syscall #%u at "TARGET_FMT_lx,
        syscall, env->CP0_EPC);
    if (syscall > 0 && syscall <= SYS_LAST) {
        const char *name = retrobsd_syscall_name[syscall];
        if (name) {
            fprintf(qemu_logfile, ": %s(", name);

            // Print up to 4 arguments.
            const target_ulong *arg = &env->active_tc.gpr[4];
            switch (syscall) {
            case SYS_exit:
            case SYS_close:
            case SYS_dup:
            case SYS_seteuid:
            case SYS_setegid:
                fprintf(qemu_logfile, "%d", arg[0]);
                break;
            case SYS_sbrk:
                fprintf(qemu_logfile, "%#x", arg[0]);
                break;
            case SYS_vfork:
            case SYS_getpid:
            case SYS_getuid:
            case SYS_geteuid:
            case SYS_sync:
            case SYS_setpgrp:
            case SYS_vhangup:
            case SYS_getdtablesize:
                // No arguments.
                break;
            case SYS_read:
            case SYS_write:
                fprintf(qemu_logfile, "%d, "TARGET_FMT_lx", %u", arg[0], arg[1], arg[2]);
                break;
            case SYS_open:
                print_retrobsd_string(env, arg[0]);
                fprintf(qemu_logfile, ", %#x, %#o", arg[1], arg[2]);
                break;
            case SYS_wait4:
                fprintf(qemu_logfile, "%d, "TARGET_FMT_lx", %d, "TARGET_FMT_lx, arg[0], arg[1], arg[2], arg[3]);
                break;
            case SYS_execv:
                print_retrobsd_string(env, arg[0]);
                fprintf(qemu_logfile, ", ");
                print_string_vector(env, arg[1]);
                break;
            case SYS_stat:
            case SYS_lstat:
                print_retrobsd_string(env, arg[0]);
                fprintf(qemu_logfile, ", "TARGET_FMT_lx, arg[1]);
                break;
            case SYS_chdir:
                print_retrobsd_string(env, arg[0]);
                break;
            case SYS_chmod:
            case SYS_access:
                print_retrobsd_string(env, arg[0]);
                fprintf(qemu_logfile, ", %#o", arg[1]);
                break;
            case SYS_chown:
                print_retrobsd_string(env, arg[0]);
                fprintf(qemu_logfile, ", %d, %d", arg[1], arg[2]);
                break;
            case SYS_lseek:
                fprintf(qemu_logfile, "%d, %#x, %d", arg[0], arg[1], arg[2]);
                break;
            case SYS_mount:
                print_retrobsd_string(env, arg[0]);
                fprintf(qemu_logfile, ", ");
                print_retrobsd_string(env, arg[1]);
                fprintf(qemu_logfile, ", %#x", arg[2]);
                break;
            case SYS___sysctl:
                fprintf(qemu_logfile, TARGET_FMT_lx", %u, "TARGET_FMT_lx", "TARGET_FMT_lx", ...",
                        arg[0], arg[1], arg[2], arg[3]);
                break;
            case SYS_sigaction:
            case SYS_sigprocmask:
            case SYS_setitimer:
                fprintf(qemu_logfile, "%d, "TARGET_FMT_lx", "TARGET_FMT_lx, arg[0], arg[1], arg[2]);
                break;
            case SYS_kill:
            case SYS_dup2:
            case SYS_flock:
            case SYS_ftruncate:
                fprintf(qemu_logfile, "%d, %d", arg[0], arg[1]);
                break;
            case SYS_ioctl:
            case SYS_fcntl:
                fprintf(qemu_logfile, "%d, %#x, "TARGET_FMT_lx, arg[0], arg[1], arg[2]);
                break;
            case SYS_execve:
                print_retrobsd_string(env, arg[0]);
                fprintf(qemu_logfile, ", ");
                print_string_vector(env, arg[1]);
                fprintf(qemu_logfile, ", ");
                print_string_vector(env, arg[2]);
                break;
            case SYS_fstat:
                fprintf(qemu_logfile, "%d, "TARGET_FMT_lx, arg[0], arg[1]);
                break;
            case SYS_select:
                fprintf(qemu_logfile, "%d, "TARGET_FMT_lx", "TARGET_FMT_lx", "TARGET_FMT_lx", ...",
                        arg[0], arg[1], arg[2], arg[3]);
                break;
            case SYS_sigsuspend:
                fprintf(qemu_logfile, TARGET_FMT_lx, arg[0]);
                break;
            case SYS_gettimeofday:
                fprintf(qemu_logfile, TARGET_FMT_lx", "TARGET_FMT_lx, arg[0], arg[1]);
                break;
            }
            fprintf(qemu_logfile, ")");
        }
    }
    fprintf(qemu_logfile, "\n");
}

void mips_cpu_do_interrupt(CPUState *cs)
{
#if !defined(CONFIG_USER_ONLY)
    MIPSCPU *cpu = MIPS_CPU(cs);
    CPUMIPSState *env = &cpu->env;
    bool update_badinstr = 0;
    target_ulong offset;
    int cause = -1;

    if (qemu_log_enabled() && cs->exception_index != EXCP_EXT_INTERRUPT) {
        const char *name;
        if (cs->exception_index < 0 || cs->exception_index > EXCP_LAST) {
            name = "unknown";
        } else {
            name = excp_names[cs->exception_index];
        }

        if (! qemu_loglevel_mask(CPU_LOG_INSTR) && ! qemu_loglevel_mask(CPU_LOG_RETROBSD))
            qemu_log("%s enter: PC " TARGET_FMT_lx " EPC " TARGET_FMT_lx " %s exception\n",
                     __func__, env->active_tc.PC, env->CP0_EPC, name);
    }
    if (qemu_loglevel_mask(CPU_LOG_INSTR))
        mips_dump_changed_state(env);

    if (cs->exception_index == EXCP_EXT_INTERRUPT &&
        (env->hflags & MIPS_HFLAG_DM)) {
        cs->exception_index = EXCP_DINT;
    }
    offset = 0x180;
    switch (cs->exception_index) {
    case EXCP_DSS:
        env->CP0_Debug |= 1 << CP0DB_DSS;
        /* Debug single step cannot be raised inside a delay slot and
           resume will always occur on the next instruction
           (but we assume the pc has always been updated during
           code translation). */
        env->CP0_DEPC = env->active_tc.PC | !!(env->hflags & MIPS_HFLAG_M16);
        goto enter_debug_mode;
    case EXCP_DINT:
        env->CP0_Debug |= 1 << CP0DB_DINT;
        goto set_DEPC;
    case EXCP_DIB:
        env->CP0_Debug |= 1 << CP0DB_DIB;
        goto set_DEPC;
    case EXCP_DBp:
        env->CP0_Debug |= 1 << CP0DB_DBp;
        goto set_DEPC;
    case EXCP_DDBS:
        env->CP0_Debug |= 1 << CP0DB_DDBS;
        goto set_DEPC;
    case EXCP_DDBL:
        env->CP0_Debug |= 1 << CP0DB_DDBL;
    set_DEPC:
        env->CP0_DEPC = exception_resume_pc(env);
        env->hflags &= ~MIPS_HFLAG_BMASK;
 enter_debug_mode:
        if (env->insn_flags & ISA_MIPS3) {
            env->hflags |= MIPS_HFLAG_64;
        }
        env->hflags |= MIPS_HFLAG_DM | MIPS_HFLAG_CP0;
        env->hflags &= ~(MIPS_HFLAG_KSU);
        /* EJTAG probe trap enable is not implemented... */
        if (!(env->CP0_Status & (1 << CP0St_EXL)))
            env->CP0_Cause &= ~(1U << CP0Ca_BD);
        env->active_tc.PC = (int32_t)0xBFC00480;
        set_hflags_for_handler(env);
        break;
    case EXCP_RESET:
        cpu_reset(CPU(cpu));
        break;
    case EXCP_SRESET:
        env->CP0_Status |= (1 << CP0St_SR);
        memset(env->CP0_WatchLo, 0, sizeof(*env->CP0_WatchLo));
        goto set_error_EPC;
    case EXCP_NMI:
        env->CP0_Status |= (1 << CP0St_NMI);
 set_error_EPC:
        env->CP0_ErrorEPC = exception_resume_pc(env);
        env->hflags &= ~MIPS_HFLAG_BMASK;
        env->CP0_Status |= (1 << CP0St_ERL) | (1 << CP0St_BEV);
        if (env->insn_flags & ISA_MIPS3) {
            env->hflags |= MIPS_HFLAG_64;
        }
        env->hflags |= MIPS_HFLAG_CP0;
        env->hflags &= ~(MIPS_HFLAG_KSU);
        if (!(env->CP0_Status & (1 << CP0St_EXL)))
            env->CP0_Cause &= ~(1U << CP0Ca_BD);
        env->active_tc.PC = (int32_t)0xBFC00000;
        set_hflags_for_handler(env);
        break;
    case EXCP_EXT_INTERRUPT:
        cause = 0;
        if (env->CP0_Cause & (1 << CP0Ca_IV))
            offset = 0x200;

        if (env->CP0_Config3 & ((1 << CP0C3_VInt) | (1 << CP0C3_VEIC))) {
            /* Vectored Interrupts.  */
            unsigned int spacing;
            unsigned int vector;
            unsigned int pending = (env->CP0_Cause & CP0Ca_IP_mask) >> 8;

            /* Compute the Vector Spacing.  */
            spacing = (env->CP0_IntCtl >> CP0IntCtl_VS) & ((1 << 6) - 1);
            spacing <<= 5;

            if (env->CP0_Config3 & (1 << CP0C3_VEIC)) {
                /* For VEIC mode, the external interrupt controller feeds the
                 * vector through the CP0Cause IP lines. */
                vector = pending;

                /* Architecturally, this is chip-specific behavior.
                 * TODO: some processors, like PIC32MZ,
                 * provide vector in a different way.
                 * Some processors, like PIC32, have a separate
                 * bit INTCON.MVEC to explicitly enable vectored mode,
                 * disabled by default. */
                spacing = 0;
            } else {
                /* For VInt mode, the MIPS computes the vector internally.  */
                pending &= env->CP0_Status >> 8;
                for (vector = 7; vector > 0; vector--) {
                    if (pending & (1 << vector)) {
                        /* Found it.  */
                        break;
                    }
                }
            }
            offset = 0x200 + vector * spacing;
        }
        goto set_EPC;
    case EXCP_LTLBL:
        cause = 1;
        update_badinstr = !(env->error_code & EXCP_INST_NOTAVAIL);
        goto set_EPC;
    case EXCP_TLBL:
        cause = 2;
        update_badinstr = !(env->error_code & EXCP_INST_NOTAVAIL);
        if ((env->error_code & EXCP_TLB_NOMATCH) &&
            !(env->CP0_Status & (1 << CP0St_EXL))) {
#if defined(TARGET_MIPS64)
            int R = env->CP0_BadVAddr >> 62;
            int UX = (env->CP0_Status & (1 << CP0St_UX)) != 0;
            int SX = (env->CP0_Status & (1 << CP0St_SX)) != 0;
            int KX = (env->CP0_Status & (1 << CP0St_KX)) != 0;

            if (((R == 0 && UX) || (R == 1 && SX) || (R == 3 && KX)) &&
                (!(env->insn_flags & (INSN_LOONGSON2E | INSN_LOONGSON2F))))
                offset = 0x080;
            else
#endif
                offset = 0x000;
        }
        goto set_EPC;
    case EXCP_TLBS:
        cause = 3;
        update_badinstr = 1;
        if ((env->error_code & EXCP_TLB_NOMATCH) &&
            !(env->CP0_Status & (1 << CP0St_EXL))) {
#if defined(TARGET_MIPS64)
            int R = env->CP0_BadVAddr >> 62;
            int UX = (env->CP0_Status & (1 << CP0St_UX)) != 0;
            int SX = (env->CP0_Status & (1 << CP0St_SX)) != 0;
            int KX = (env->CP0_Status & (1 << CP0St_KX)) != 0;

            if (((R == 0 && UX) || (R == 1 && SX) || (R == 3 && KX)) &&
                (!(env->insn_flags & (INSN_LOONGSON2E | INSN_LOONGSON2F))))
                offset = 0x080;
            else
#endif
                offset = 0x000;
        }
        goto set_EPC;
    case EXCP_AdEL:
        cause = 4;
        update_badinstr = !(env->error_code & EXCP_INST_NOTAVAIL);
        goto set_EPC;
    case EXCP_AdES:
        cause = 5;
        update_badinstr = 1;
        goto set_EPC;
    case EXCP_IBE:
        cause = 6;
        goto set_EPC;
    case EXCP_DBE:
        cause = 7;
        goto set_EPC;
    case EXCP_SYSCALL:
        cause = 8;
        update_badinstr = 1;
        goto set_EPC;
    case EXCP_BREAK:
        cause = 9;
        update_badinstr = 1;
        goto set_EPC;
    case EXCP_RI:
        cause = 10;
        update_badinstr = 1;
        goto set_EPC;
    case EXCP_CpU:
        cause = 11;
        update_badinstr = 1;
        env->CP0_Cause = (env->CP0_Cause & ~(0x3 << CP0Ca_CE)) |
                         (env->error_code << CP0Ca_CE);
        goto set_EPC;
    case EXCP_OVERFLOW:
        cause = 12;
        update_badinstr = 1;
        goto set_EPC;
    case EXCP_TRAP:
        cause = 13;
        update_badinstr = 1;
        goto set_EPC;
    case EXCP_MSAFPE:
        cause = 14;
        update_badinstr = 1;
        goto set_EPC;
    case EXCP_FPE:
        cause = 15;
        update_badinstr = 1;
        goto set_EPC;
    case EXCP_C2E:
        cause = 18;
        goto set_EPC;
    case EXCP_TLBRI:
        cause = 19;
        update_badinstr = 1;
        goto set_EPC;
    case EXCP_TLBXI:
        cause = 20;
        goto set_EPC;
    case EXCP_MSADIS:
        cause = 21;
        update_badinstr = 1;
        goto set_EPC;
    case EXCP_MDMX:
        cause = 22;
        goto set_EPC;
    case EXCP_DWATCH:
        cause = 23;
        /* XXX: TODO: manage defered watch exceptions */
        goto set_EPC;
    case EXCP_MCHECK:
        cause = 24;
        goto set_EPC;
    case EXCP_THREAD:
        cause = 25;
        goto set_EPC;
    case EXCP_DSPDIS:
        cause = 26;
        goto set_EPC;
    case EXCP_CACHE:
        cause = 30;
        if (env->CP0_Status & (1 << CP0St_BEV)) {
            offset = 0x100;
        } else {
            offset = 0x20000100;
        }
 set_EPC:
        if (!(env->CP0_Status & (1 << CP0St_EXL))) {
            env->CP0_EPC = exception_resume_pc(env);
            if (update_badinstr) {
                set_badinstr_registers(env);
            }
            if (env->hflags & MIPS_HFLAG_BMASK) {
                env->CP0_Cause |= (1U << CP0Ca_BD);
            } else {
                env->CP0_Cause &= ~(1U << CP0Ca_BD);
            }
            env->CP0_Status |= (1 << CP0St_EXL);
            if (env->insn_flags & ISA_MIPS3) {
                env->hflags |= MIPS_HFLAG_64;
            }
            env->hflags |= MIPS_HFLAG_CP0;
            env->hflags &= ~(MIPS_HFLAG_KSU);
        }
        env->hflags &= ~MIPS_HFLAG_BMASK;
        if (env->CP0_Status & (1 << CP0St_BEV)) {
            env->active_tc.PC = (int32_t)0xBFC00200;
        } else {
            env->active_tc.PC = (int32_t)(env->CP0_EBase & ~0x3ff);
        }
        env->active_tc.PC += offset;
        set_hflags_for_handler(env);
        env->CP0_Cause = (env->CP0_Cause & ~(0x1f << CP0Ca_EC)) | (cause << CP0Ca_EC);
        break;
    default:
        qemu_log("Invalid MIPS exception %d. Exiting\n", cs->exception_index);
        printf("Invalid MIPS exception %d. Exiting\n", cs->exception_index);
        exit(1);
    }
    if (qemu_loglevel_mask(CPU_LOG_INSTR)) {
        if (cs->exception_index == EXCP_EXT_INTERRUPT)
            fprintf (qemu_logfile, "--- Interrupt, vector "TARGET_FMT_lx"\n",
                env->active_tc.PC);
        else
            fprintf (qemu_logfile, "--- Exception #%u: vector "TARGET_FMT_lx"\n",
                cause, env->active_tc.PC);
    } else if (qemu_loglevel_mask(CPU_LOG_RETROBSD) && cs->exception_index == EXCP_SYSCALL) {
        print_retrobsd_syscall(env);
    } else
    if (qemu_log_enabled() && cs->exception_index != EXCP_EXT_INTERRUPT) {
        qemu_log("%s: PC " TARGET_FMT_lx " EPC " TARGET_FMT_lx " cause %d\n"
                "    S %08x C %08x A " TARGET_FMT_lx " D " TARGET_FMT_lx "\n",
                __func__, env->active_tc.PC, env->CP0_EPC, cause,
                env->CP0_Status, env->CP0_Cause, env->CP0_BadVAddr,
                env->CP0_DEPC);
    }
#endif
    cs->exception_index = EXCP_NONE;
}

bool mips_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    if (interrupt_request & CPU_INTERRUPT_HARD) {
        MIPSCPU *cpu = MIPS_CPU(cs);
        CPUMIPSState *env = &cpu->env;

        if (cpu_mips_hw_interrupts_pending(env)) {
            /* Raise it */
            cs->exception_index = EXCP_EXT_INTERRUPT;
            env->error_code = 0;
            mips_cpu_do_interrupt(cs);
            return true;
        }
    }
    return false;
}

#if !defined(CONFIG_USER_ONLY)
void r4k_invalidate_tlb (CPUMIPSState *env, int idx, int use_extra)
{
    MIPSCPU *cpu = mips_env_get_cpu(env);
    CPUState *cs;
    r4k_tlb_t *tlb;
    target_ulong addr;
    target_ulong end;
    uint8_t ASID = env->CP0_EntryHi & 0xFF;
    target_ulong mask;

    tlb = &env->tlb->mmu.r4k.tlb[idx];
    /* The qemu TLB is flushed when the ASID changes, so no need to
       flush these entries again.  */
    if (tlb->G == 0 && tlb->ASID != ASID) {
        return;
    }

    if (use_extra && env->tlb->tlb_in_use < MIPS_TLB_MAX) {
        /* For tlbwr, we can shadow the discarded entry into
           a new (fake) TLB entry, as long as the guest can not
           tell that it's there.  */
        env->tlb->mmu.r4k.tlb[env->tlb->tlb_in_use] = *tlb;
        env->tlb->tlb_in_use++;
        return;
    }

    /* 1k pages are not supported. */
    mask = tlb->PageMask | ~(TARGET_PAGE_MASK << 1);
    if (tlb->V0) {
        cs = CPU(cpu);
        addr = tlb->VPN & ~mask;
#if defined(TARGET_MIPS64)
        if (addr >= (0xFFFFFFFF80000000ULL & env->SEGMask)) {
            addr |= 0x3FFFFF0000000000ULL;
        }
#endif
        end = addr | (mask >> 1);
        while (addr < end) {
            tlb_flush_page(cs, addr);
            addr += TARGET_PAGE_SIZE;
        }
    }
    if (tlb->V1) {
        cs = CPU(cpu);
        addr = (tlb->VPN & ~mask) | ((mask >> 1) + 1);
#if defined(TARGET_MIPS64)
        if (addr >= (0xFFFFFFFF80000000ULL & env->SEGMask)) {
            addr |= 0x3FFFFF0000000000ULL;
        }
#endif
        end = addr | mask;
        while (addr - 1 < end) {
            tlb_flush_page(cs, addr);
            addr += TARGET_PAGE_SIZE;
        }
    }
}
#endif
