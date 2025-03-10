DEF_HELPER_3(raise_exception_err, noreturn, env, i32, int)
DEF_HELPER_2(raise_exception, noreturn, env, i32)

DEF_HELPER_1(do_semihosting, void, env)

#ifdef TARGET_MIPS64
DEF_HELPER_4(sdl, void, env, tl, tl, int)
DEF_HELPER_4(sdr, void, env, tl, tl, int)
#endif
DEF_HELPER_4(swl, void, env, tl, tl, int)
DEF_HELPER_4(swr, void, env, tl, tl, int)

#ifndef CONFIG_USER_ONLY
DEF_HELPER_3(ll, tl, env, tl, int)
DEF_HELPER_4(sc, tl, env, tl, tl, int)
#ifdef TARGET_MIPS64
DEF_HELPER_3(lld, tl, env, tl, int)
DEF_HELPER_4(scd, tl, env, tl, tl, int)
#endif
#endif

DEF_HELPER_FLAGS_1(clo, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(clz, TCG_CALL_NO_RWG_SE, tl, tl)
#ifdef TARGET_MIPS64
DEF_HELPER_FLAGS_1(dclo, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(dclz, TCG_CALL_NO_RWG_SE, tl, tl)
#endif

DEF_HELPER_3(muls, tl, env, tl, tl)
DEF_HELPER_3(mulsu, tl, env, tl, tl)
DEF_HELPER_3(macc, tl, env, tl, tl)
DEF_HELPER_3(maccu, tl, env, tl, tl)
DEF_HELPER_3(msac, tl, env, tl, tl)
DEF_HELPER_3(msacu, tl, env, tl, tl)
DEF_HELPER_3(mulhi, tl, env, tl, tl)
DEF_HELPER_3(mulhiu, tl, env, tl, tl)
DEF_HELPER_3(mulshi, tl, env, tl, tl)
DEF_HELPER_3(mulshiu, tl, env, tl, tl)
DEF_HELPER_3(macchi, tl, env, tl, tl)
DEF_HELPER_3(macchiu, tl, env, tl, tl)
DEF_HELPER_3(msachi, tl, env, tl, tl)
DEF_HELPER_3(msachiu, tl, env, tl, tl)

DEF_HELPER_FLAGS_1(bitswap, TCG_CALL_NO_RWG_SE, tl, tl)
#ifdef TARGET_MIPS64
DEF_HELPER_FLAGS_1(dbitswap, TCG_CALL_NO_RWG_SE, tl, tl)
#endif

#ifndef CONFIG_USER_ONLY
/* CP0 helpers */
DEF_HELPER_1(mfc0_mvpcontrol, tl, env)
DEF_HELPER_1(mfc0_mvpconf0, tl, env)
DEF_HELPER_1(mfc0_mvpconf1, tl, env)
DEF_HELPER_1(mftc0_vpecontrol, tl, env)
DEF_HELPER_1(mftc0_vpeconf0, tl, env)
DEF_HELPER_1(mfc0_random, tl, env)
DEF_HELPER_1(mfc0_tcstatus, tl, env)
DEF_HELPER_1(mftc0_tcstatus, tl, env)
DEF_HELPER_1(mfc0_tcbind, tl, env)
DEF_HELPER_1(mftc0_tcbind, tl, env)
DEF_HELPER_1(mfc0_tcrestart, tl, env)
DEF_HELPER_1(mftc0_tcrestart, tl, env)
DEF_HELPER_1(mfc0_tchalt, tl, env)
DEF_HELPER_1(mftc0_tchalt, tl, env)
DEF_HELPER_1(mfc0_tccontext, tl, env)
DEF_HELPER_1(mftc0_tccontext, tl, env)
DEF_HELPER_1(mfc0_tcschedule, tl, env)
DEF_HELPER_1(mftc0_tcschedule, tl, env)
DEF_HELPER_1(mfc0_tcschefback, tl, env)
DEF_HELPER_1(mftc0_tcschefback, tl, env)
DEF_HELPER_1(mfc0_count, tl, env)
DEF_HELPER_1(mftc0_entryhi, tl, env)
DEF_HELPER_1(mftc0_status, tl, env)
DEF_HELPER_1(mftc0_cause, tl, env)
DEF_HELPER_1(mftc0_epc, tl, env)
DEF_HELPER_1(mftc0_ebase, tl, env)
DEF_HELPER_2(mftc0_configx, tl, env, tl)
DEF_HELPER_1(mfc0_lladdr, tl, env)
DEF_HELPER_2(mfc0_watchlo, tl, env, i32)
DEF_HELPER_2(mfc0_watchhi, tl, env, i32)
DEF_HELPER_1(mfc0_debug, tl, env)
DEF_HELPER_1(mftc0_debug, tl, env)
#ifdef TARGET_MIPS64
DEF_HELPER_1(dmfc0_tcrestart, tl, env)
DEF_HELPER_1(dmfc0_tchalt, tl, env)
DEF_HELPER_1(dmfc0_tccontext, tl, env)
DEF_HELPER_1(dmfc0_tcschedule, tl, env)
DEF_HELPER_1(dmfc0_tcschefback, tl, env)
DEF_HELPER_1(dmfc0_lladdr, tl, env)
DEF_HELPER_2(dmfc0_watchlo, tl, env, i32)
#endif /* TARGET_MIPS64 */

DEF_HELPER_2(mtc0_index, void, env, tl)
DEF_HELPER_2(mtc0_mvpcontrol, void, env, tl)
DEF_HELPER_2(mtc0_vpecontrol, void, env, tl)
DEF_HELPER_2(mttc0_vpecontrol, void, env, tl)
DEF_HELPER_2(mtc0_vpeconf0, void, env, tl)
DEF_HELPER_2(mttc0_vpeconf0, void, env, tl)
DEF_HELPER_2(mtc0_vpeconf1, void, env, tl)
DEF_HELPER_2(mtc0_yqmask, void, env, tl)
DEF_HELPER_2(mtc0_vpeopt, void, env, tl)
DEF_HELPER_2(mtc0_entrylo0, void, env, tl)
DEF_HELPER_2(mtc0_tcstatus, void, env, tl)
DEF_HELPER_2(mttc0_tcstatus, void, env, tl)
DEF_HELPER_2(mtc0_tcbind, void, env, tl)
DEF_HELPER_2(mttc0_tcbind, void, env, tl)
DEF_HELPER_2(mtc0_tcrestart, void, env, tl)
DEF_HELPER_2(mttc0_tcrestart, void, env, tl)
DEF_HELPER_2(mtc0_tchalt, void, env, tl)
DEF_HELPER_2(mttc0_tchalt, void, env, tl)
DEF_HELPER_2(mtc0_tccontext, void, env, tl)
DEF_HELPER_2(mttc0_tccontext, void, env, tl)
DEF_HELPER_2(mtc0_tcschedule, void, env, tl)
DEF_HELPER_2(mttc0_tcschedule, void, env, tl)
DEF_HELPER_2(mtc0_tcschefback, void, env, tl)
DEF_HELPER_2(mttc0_tcschefback, void, env, tl)
DEF_HELPER_2(mtc0_entrylo1, void, env, tl)
DEF_HELPER_2(mtc0_context, void, env, tl)
DEF_HELPER_2(mtc0_pagemask, void, env, tl)
DEF_HELPER_2(mtc0_pagegrain, void, env, tl)
DEF_HELPER_2(mtc0_wired, void, env, tl)
DEF_HELPER_2(mtc0_srsconf0, void, env, tl)
DEF_HELPER_2(mtc0_srsconf1, void, env, tl)
DEF_HELPER_2(mtc0_srsconf2, void, env, tl)
DEF_HELPER_2(mtc0_srsconf3, void, env, tl)
DEF_HELPER_2(mtc0_srsconf4, void, env, tl)
DEF_HELPER_2(mtc0_hwrena, void, env, tl)
DEF_HELPER_2(mtc0_count, void, env, tl)
DEF_HELPER_2(mtc0_entryhi, void, env, tl)
DEF_HELPER_2(mttc0_entryhi, void, env, tl)
DEF_HELPER_2(mtc0_compare, void, env, tl)
DEF_HELPER_2(mtc0_status, void, env, tl)
DEF_HELPER_2(mttc0_status, void, env, tl)
DEF_HELPER_2(mtc0_intctl, void, env, tl)
DEF_HELPER_2(mtc0_srsctl, void, env, tl)
DEF_HELPER_2(mtc0_cause, void, env, tl)
DEF_HELPER_2(mttc0_cause, void, env, tl)
DEF_HELPER_2(mtc0_ebase, void, env, tl)
DEF_HELPER_2(mttc0_ebase, void, env, tl)
DEF_HELPER_2(mtc0_config0, void, env, tl)
DEF_HELPER_2(mtc0_config2, void, env, tl)
DEF_HELPER_2(mtc0_config3, void, env, tl)
DEF_HELPER_2(mtc0_config4, void, env, tl)
DEF_HELPER_2(mtc0_config5, void, env, tl)
DEF_HELPER_2(mtc0_lladdr, void, env, tl)
DEF_HELPER_3(mtc0_watchlo, void, env, tl, i32)
DEF_HELPER_3(mtc0_watchhi, void, env, tl, i32)
DEF_HELPER_2(mtc0_xcontext, void, env, tl)
DEF_HELPER_2(mtc0_framemask, void, env, tl)
DEF_HELPER_2(mtc0_debug, void, env, tl)
DEF_HELPER_2(mttc0_debug, void, env, tl)
DEF_HELPER_2(mtc0_performance0, void, env, tl)
DEF_HELPER_2(mtc0_taglo, void, env, tl)
DEF_HELPER_2(mtc0_datalo, void, env, tl)
DEF_HELPER_2(mtc0_taghi, void, env, tl)
DEF_HELPER_2(mtc0_datahi, void, env, tl)

#if defined(TARGET_MIPS64)
DEF_HELPER_2(dmtc0_entrylo0, void, env, i64)
DEF_HELPER_2(dmtc0_entrylo1, void, env, i64)
#endif

/* MIPS MT functions */
DEF_HELPER_2(mftgpr, tl, env, i32)
DEF_HELPER_2(mftlo, tl, env, i32)
DEF_HELPER_2(mfthi, tl, env, i32)
DEF_HELPER_2(mftacx, tl, env, i32)
DEF_HELPER_1(mftdsp, tl, env)
DEF_HELPER_3(mttgpr, void, env, tl, i32)
DEF_HELPER_3(mttlo, void, env, tl, i32)
DEF_HELPER_3(mtthi, void, env, tl, i32)
DEF_HELPER_3(mttacx, void, env, tl, i32)
DEF_HELPER_2(mttdsp, void, env, tl)
DEF_HELPER_0(dmt, tl)
DEF_HELPER_0(emt, tl)
DEF_HELPER_1(dvpe, tl, env)
DEF_HELPER_1(evpe, tl, env)
#endif /* !CONFIG_USER_ONLY */

/* microMIPS functions */
DEF_HELPER_4(lwm, void, env, tl, tl, i32)
DEF_HELPER_4(swm, void, env, tl, tl, i32)
#ifdef TARGET_MIPS64
DEF_HELPER_4(ldm, void, env, tl, tl, i32)
DEF_HELPER_4(sdm, void, env, tl, tl, i32)
#endif

DEF_HELPER_2(fork, void, tl, tl)
DEF_HELPER_2(yield, tl, env, tl)

/* CP1 functions */
DEF_HELPER_2(cfc1, tl, env, i32)
DEF_HELPER_4(ctc1, void, env, tl, i32, i32)

DEF_HELPER_2(float_cvtd_s, i64, env, i32)
DEF_HELPER_2(float_cvtd_w, i64, env, i32)
DEF_HELPER_2(float_cvtd_l, i64, env, i64)
DEF_HELPER_2(float_cvtl_d, i64, env, i64)
DEF_HELPER_2(float_cvtl_s, i64, env, i32)
DEF_HELPER_2(float_cvtps_pw, i64, env, i64)
DEF_HELPER_2(float_cvtpw_ps, i64, env, i64)
DEF_HELPER_2(float_cvts_d, i32, env, i64)
DEF_HELPER_2(float_cvts_w, i32, env, i32)
DEF_HELPER_2(float_cvts_l, i32, env, i64)
DEF_HELPER_2(float_cvts_pl, i32, env, i32)
DEF_HELPER_2(float_cvts_pu, i32, env, i32)
DEF_HELPER_2(float_cvtw_s, i32, env, i32)
DEF_HELPER_2(float_cvtw_d, i32, env, i64)

DEF_HELPER_3(float_addr_ps, i64, env, i64, i64)
DEF_HELPER_3(float_mulr_ps, i64, env, i64, i64)

DEF_HELPER_FLAGS_1(float_class_s, TCG_CALL_NO_RWG_SE, i32, i32)
DEF_HELPER_FLAGS_1(float_class_d, TCG_CALL_NO_RWG_SE, i64, i64)

#define FOP_PROTO(op)                                     \
DEF_HELPER_4(float_ ## op ## _s, i32, env, i32, i32, i32) \
DEF_HELPER_4(float_ ## op ## _d, i64, env, i64, i64, i64)
FOP_PROTO(maddf)
FOP_PROTO(msubf)
#undef FOP_PROTO

#define FOP_PROTO(op)                                \
DEF_HELPER_3(float_ ## op ## _s, i32, env, i32, i32) \
DEF_HELPER_3(float_ ## op ## _d, i64, env, i64, i64)
FOP_PROTO(max)
FOP_PROTO(maxa)
FOP_PROTO(min)
FOP_PROTO(mina)
#undef FOP_PROTO

#define FOP_PROTO(op)                            \
DEF_HELPER_2(float_ ## op ## l_s, i64, env, i32) \
DEF_HELPER_2(float_ ## op ## l_d, i64, env, i64) \
DEF_HELPER_2(float_ ## op ## w_s, i32, env, i32) \
DEF_HELPER_2(float_ ## op ## w_d, i32, env, i64)
FOP_PROTO(round)
FOP_PROTO(trunc)
FOP_PROTO(ceil)
FOP_PROTO(floor)
#undef FOP_PROTO

#define FOP_PROTO(op)                            \
DEF_HELPER_2(float_ ## op ## _s, i32, env, i32)  \
DEF_HELPER_2(float_ ## op ## _d, i64, env, i64)
FOP_PROTO(sqrt)
FOP_PROTO(rsqrt)
FOP_PROTO(recip)
FOP_PROTO(rint)
#undef FOP_PROTO

#define FOP_PROTO(op)                       \
DEF_HELPER_1(float_ ## op ## _s, i32, i32)  \
DEF_HELPER_1(float_ ## op ## _d, i64, i64)  \
DEF_HELPER_1(float_ ## op ## _ps, i64, i64)
FOP_PROTO(abs)
FOP_PROTO(chs)
#undef FOP_PROTO

#define FOP_PROTO(op)                            \
DEF_HELPER_2(float_ ## op ## _s, i32, env, i32)  \
DEF_HELPER_2(float_ ## op ## _d, i64, env, i64)  \
DEF_HELPER_2(float_ ## op ## _ps, i64, env, i64)
FOP_PROTO(recip1)
FOP_PROTO(rsqrt1)
#undef FOP_PROTO

#define FOP_PROTO(op)                                  \
DEF_HELPER_3(float_ ## op ## _s, i32, env, i32, i32)   \
DEF_HELPER_3(float_ ## op ## _d, i64, env, i64, i64)   \
DEF_HELPER_3(float_ ## op ## _ps, i64, env, i64, i64)
FOP_PROTO(add)
FOP_PROTO(sub)
FOP_PROTO(mul)
FOP_PROTO(div)
FOP_PROTO(recip2)
FOP_PROTO(rsqrt2)
#undef FOP_PROTO

#define FOP_PROTO(op)                                      \
DEF_HELPER_4(float_ ## op ## _s, i32, env, i32, i32, i32)  \
DEF_HELPER_4(float_ ## op ## _d, i64, env, i64, i64, i64)  \
DEF_HELPER_4(float_ ## op ## _ps, i64, env, i64, i64, i64)
FOP_PROTO(madd)
FOP_PROTO(msub)
FOP_PROTO(nmadd)
FOP_PROTO(nmsub)
#undef FOP_PROTO

#define FOP_PROTO(op)                                    \
DEF_HELPER_4(cmp_d_ ## op, void, env, i64, i64, int)     \
DEF_HELPER_4(cmpabs_d_ ## op, void, env, i64, i64, int)  \
DEF_HELPER_4(cmp_s_ ## op, void, env, i32, i32, int)     \
DEF_HELPER_4(cmpabs_s_ ## op, void, env, i32, i32, int)  \
DEF_HELPER_4(cmp_ps_ ## op, void, env, i64, i64, int)    \
DEF_HELPER_4(cmpabs_ps_ ## op, void, env, i64, i64, int)
FOP_PROTO(f)
FOP_PROTO(un)
FOP_PROTO(eq)
FOP_PROTO(ueq)
FOP_PROTO(olt)
FOP_PROTO(ult)
FOP_PROTO(ole)
FOP_PROTO(ule)
FOP_PROTO(sf)
FOP_PROTO(ngle)
FOP_PROTO(seq)
FOP_PROTO(ngl)
FOP_PROTO(lt)
FOP_PROTO(nge)
FOP_PROTO(le)
FOP_PROTO(ngt)
#undef FOP_PROTO

#define FOP_PROTO(op) \
DEF_HELPER_3(r6_cmp_d_ ## op, i64, env, i64, i64) \
DEF_HELPER_3(r6_cmp_s_ ## op, i32, env, i32, i32)
FOP_PROTO(af)
FOP_PROTO(un)
FOP_PROTO(eq)
FOP_PROTO(ueq)
FOP_PROTO(lt)
FOP_PROTO(ult)
FOP_PROTO(le)
FOP_PROTO(ule)
FOP_PROTO(saf)
FOP_PROTO(sun)
FOP_PROTO(seq)
FOP_PROTO(sueq)
FOP_PROTO(slt)
FOP_PROTO(sult)
FOP_PROTO(sle)
FOP_PROTO(sule)
FOP_PROTO(or)
FOP_PROTO(une)
FOP_PROTO(ne)
FOP_PROTO(sor)
FOP_PROTO(sune)
FOP_PROTO(sne)
#undef FOP_PROTO

/* Special functions */
#ifndef CONFIG_USER_ONLY
DEF_HELPER_1(tlbwi, void, env)
DEF_HELPER_1(tlbwr, void, env)
DEF_HELPER_1(tlbp, void, env)
DEF_HELPER_1(tlbr, void, env)
DEF_HELPER_1(tlbinv, void, env)
DEF_HELPER_1(tlbinvf, void, env)
DEF_HELPER_1(di, tl, env)
DEF_HELPER_1(ei, tl, env)
DEF_HELPER_1(eret, void, env)
DEF_HELPER_1(eretnc, void, env)
DEF_HELPER_1(deret, void, env)
#endif /* !CONFIG_USER_ONLY */
DEF_HELPER_1(rdhwr_cpunum, tl, env)
DEF_HELPER_1(rdhwr_synci_step, tl, env)
DEF_HELPER_1(rdhwr_cc, tl, env)
DEF_HELPER_1(rdhwr_ccres, tl, env)
DEF_HELPER_2(pmon, void, env, int)
DEF_HELPER_1(wait, void, env)

DEF_HELPER_3(dump_pc, void, env, int, int)
DEF_HELPER_4(dump_store, void, env, int, tl, tl)
DEF_HELPER_4(dump_load, void, env, int, tl, tl)

/* Loongson multimedia functions.  */
DEF_HELPER_FLAGS_2(paddsh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(paddush, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(paddh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(paddw, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(paddsb, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(paddusb, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(paddb, TCG_CALL_NO_RWG_SE, i64, i64, i64)

DEF_HELPER_FLAGS_2(psubsh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(psubush, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(psubh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(psubw, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(psubsb, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(psubusb, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(psubb, TCG_CALL_NO_RWG_SE, i64, i64, i64)

DEF_HELPER_FLAGS_2(pshufh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(packsswh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(packsshb, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(packushb, TCG_CALL_NO_RWG_SE, i64, i64, i64)

DEF_HELPER_FLAGS_2(punpcklhw, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(punpckhhw, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(punpcklbh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(punpckhbh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(punpcklwd, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(punpckhwd, TCG_CALL_NO_RWG_SE, i64, i64, i64)

DEF_HELPER_FLAGS_2(pavgh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(pavgb, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(pmaxsh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(pminsh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(pmaxub, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(pminub, TCG_CALL_NO_RWG_SE, i64, i64, i64)

DEF_HELPER_FLAGS_2(pcmpeqw, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(pcmpgtw, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(pcmpeqh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(pcmpgth, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(pcmpeqb, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(pcmpgtb, TCG_CALL_NO_RWG_SE, i64, i64, i64)

DEF_HELPER_FLAGS_2(psllw, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(psllh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(psrlw, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(psrlh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(psraw, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(psrah, TCG_CALL_NO_RWG_SE, i64, i64, i64)

DEF_HELPER_FLAGS_2(pmullh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(pmulhh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(pmulhuh, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_2(pmaddhw, TCG_CALL_NO_RWG_SE, i64, i64, i64)

DEF_HELPER_FLAGS_2(pasubub, TCG_CALL_NO_RWG_SE, i64, i64, i64)
DEF_HELPER_FLAGS_1(biadd, TCG_CALL_NO_RWG_SE, i64, i64)
DEF_HELPER_FLAGS_1(pmovmskb, TCG_CALL_NO_RWG_SE, i64, i64)

/*** MIPS DSP ***/
/* DSP Arithmetic Sub-class insns */
DEF_HELPER_FLAGS_3(addq_ph, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(addq_s_ph, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(addq_qh, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(addq_s_qh, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(addq_s_w, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(addq_pw, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(addq_s_pw, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(addu_qb, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(addu_s_qb, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_2(adduh_qb, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(adduh_r_qb, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_3(addu_ph, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(addu_s_ph, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_2(addqh_ph, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(addqh_r_ph, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(addqh_w, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(addqh_r_w, TCG_CALL_NO_RWG_SE, tl, tl, tl)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(addu_ob, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(addu_s_ob, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_2(adduh_ob, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(adduh_r_ob, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_3(addu_qh, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(addu_s_qh, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(subq_ph, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(subq_s_ph, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(subq_qh, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(subq_s_qh, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(subq_s_w, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(subq_pw, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(subq_s_pw, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(subu_qb, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(subu_s_qb, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_2(subuh_qb, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(subuh_r_qb, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_3(subu_ph, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(subu_s_ph, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_2(subqh_ph, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(subqh_r_ph, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(subqh_w, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(subqh_r_w, TCG_CALL_NO_RWG_SE, tl, tl, tl)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(subu_ob, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(subu_s_ob, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_2(subuh_ob, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(subuh_r_ob, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_3(subu_qh, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(subu_s_qh, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(addsc, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(addwc, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_2(modsub, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_1(raddu_w_qb, TCG_CALL_NO_RWG_SE, tl, tl)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_1(raddu_l_ob, TCG_CALL_NO_RWG_SE, tl, tl)
#endif
DEF_HELPER_FLAGS_2(absq_s_qb, 0, tl, tl, env)
DEF_HELPER_FLAGS_2(absq_s_ph, 0, tl, tl, env)
DEF_HELPER_FLAGS_2(absq_s_w, 0, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_2(absq_s_ob, 0, tl, tl, env)
DEF_HELPER_FLAGS_2(absq_s_qh, 0, tl, tl, env)
DEF_HELPER_FLAGS_2(absq_s_pw, 0, tl, tl, env)
#endif
DEF_HELPER_FLAGS_2(precr_qb_ph, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(precrq_qb_ph, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_3(precr_sra_ph_w, TCG_CALL_NO_RWG_SE,
                   tl, i32, tl, tl)
DEF_HELPER_FLAGS_3(precr_sra_r_ph_w, TCG_CALL_NO_RWG_SE,
                   tl, i32, tl, tl)
DEF_HELPER_FLAGS_2(precrq_ph_w, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_3(precrq_rs_ph_w, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_2(precr_ob_qh, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_3(precr_sra_qh_pw,
                   TCG_CALL_NO_RWG_SE, tl, tl, tl, i32)
DEF_HELPER_FLAGS_3(precr_sra_r_qh_pw,
                   TCG_CALL_NO_RWG_SE, tl, tl, tl, i32)
DEF_HELPER_FLAGS_2(precrq_ob_qh, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(precrq_qh_pw, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_3(precrq_rs_qh_pw,
                   TCG_CALL_NO_RWG_SE, tl, tl, tl, env)
DEF_HELPER_FLAGS_2(precrq_pw_l, TCG_CALL_NO_RWG_SE, tl, tl, tl)
#endif
DEF_HELPER_FLAGS_3(precrqu_s_qb_ph, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(precrqu_s_ob_qh,
                   TCG_CALL_NO_RWG_SE, tl, tl, tl, env)

DEF_HELPER_FLAGS_1(preceq_pw_qhl, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(preceq_pw_qhr, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(preceq_pw_qhla, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(preceq_pw_qhra, TCG_CALL_NO_RWG_SE, tl, tl)
#endif
DEF_HELPER_FLAGS_1(precequ_ph_qbl, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(precequ_ph_qbr, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(precequ_ph_qbla, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(precequ_ph_qbra, TCG_CALL_NO_RWG_SE, tl, tl)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_1(precequ_qh_obl, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(precequ_qh_obr, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(precequ_qh_obla, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(precequ_qh_obra, TCG_CALL_NO_RWG_SE, tl, tl)
#endif
DEF_HELPER_FLAGS_1(preceu_ph_qbl, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(preceu_ph_qbr, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(preceu_ph_qbla, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(preceu_ph_qbra, TCG_CALL_NO_RWG_SE, tl, tl)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_1(preceu_qh_obl, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(preceu_qh_obr, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(preceu_qh_obla, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_1(preceu_qh_obra, TCG_CALL_NO_RWG_SE, tl, tl)
#endif

/* DSP GPR-Based Shift Sub-class insns */
DEF_HELPER_FLAGS_3(shll_qb, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(shll_ob, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(shll_ph, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(shll_s_ph, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(shll_qh, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(shll_s_qh, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(shll_s_w, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(shll_pw, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(shll_s_pw, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_2(shrl_qb, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(shrl_ph, TCG_CALL_NO_RWG_SE, tl, tl, tl)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_2(shrl_ob, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(shrl_qh, TCG_CALL_NO_RWG_SE, tl, tl, tl)
#endif
DEF_HELPER_FLAGS_2(shra_qb, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(shra_r_qb, TCG_CALL_NO_RWG_SE, tl, tl, tl)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_2(shra_ob, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(shra_r_ob, TCG_CALL_NO_RWG_SE, tl, tl, tl)
#endif
DEF_HELPER_FLAGS_2(shra_ph, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(shra_r_ph, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(shra_r_w, TCG_CALL_NO_RWG_SE, tl, tl, tl)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_2(shra_qh, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(shra_r_qh, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(shra_pw, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(shra_r_pw, TCG_CALL_NO_RWG_SE, tl, tl, tl)
#endif

/* DSP Multiply Sub-class insns */
DEF_HELPER_FLAGS_3(muleu_s_ph_qbl, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(muleu_s_ph_qbr, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(muleu_s_qh_obl, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(muleu_s_qh_obr, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(mulq_rs_ph, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(mulq_rs_qh, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(muleq_s_w_phl, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(muleq_s_w_phr, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(muleq_s_pw_qhl, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(muleq_s_pw_qhr, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_4(dpau_h_qbl, 0, void, i32, tl, tl, env)
DEF_HELPER_FLAGS_4(dpau_h_qbr, 0, void, i32, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_4(dpau_h_obl, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(dpau_h_obr, 0, void, tl, tl, i32, env)
#endif
DEF_HELPER_FLAGS_4(dpsu_h_qbl, 0, void, i32, tl, tl, env)
DEF_HELPER_FLAGS_4(dpsu_h_qbr, 0, void, i32, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_4(dpsu_h_obl, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(dpsu_h_obr, 0, void, tl, tl, i32, env)
#endif
DEF_HELPER_FLAGS_4(dpa_w_ph, 0, void, i32, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_4(dpa_w_qh, 0, void, tl, tl, i32, env)
#endif
DEF_HELPER_FLAGS_4(dpax_w_ph, 0, void, i32, tl, tl, env)
DEF_HELPER_FLAGS_4(dpaq_s_w_ph, 0, void, i32, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_4(dpaq_s_w_qh, 0, void, tl, tl, i32, env)
#endif
DEF_HELPER_FLAGS_4(dpaqx_s_w_ph, 0, void, i32, tl, tl, env)
DEF_HELPER_FLAGS_4(dpaqx_sa_w_ph, 0, void, i32, tl, tl, env)
DEF_HELPER_FLAGS_4(dps_w_ph, 0, void, i32, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_4(dps_w_qh, 0, void, tl, tl, i32, env)
#endif
DEF_HELPER_FLAGS_4(dpsx_w_ph, 0, void, i32, tl, tl, env)
DEF_HELPER_FLAGS_4(dpsq_s_w_ph, 0, void, i32, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_4(dpsq_s_w_qh, 0, void, tl, tl, i32, env)
#endif
DEF_HELPER_FLAGS_4(dpsqx_s_w_ph, 0, void, i32, tl, tl, env)
DEF_HELPER_FLAGS_4(dpsqx_sa_w_ph, 0, void, i32, tl, tl, env)
DEF_HELPER_FLAGS_4(mulsaq_s_w_ph, 0, void, i32, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_4(mulsaq_s_w_qh, 0, void, tl, tl, i32, env)
#endif
DEF_HELPER_FLAGS_4(dpaq_sa_l_w, 0, void, i32, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_4(dpaq_sa_l_pw, 0, void, tl, tl, i32, env)
#endif
DEF_HELPER_FLAGS_4(dpsq_sa_l_w, 0, void, i32, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_4(dpsq_sa_l_pw, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(mulsaq_s_l_pw, 0, void, tl, tl, i32, env)
#endif
DEF_HELPER_FLAGS_4(maq_s_w_phl, 0, void, i32, tl, tl, env)
DEF_HELPER_FLAGS_4(maq_s_w_phr, 0, void, i32, tl, tl, env)
DEF_HELPER_FLAGS_4(maq_sa_w_phl, 0, void, i32, tl, tl, env)
DEF_HELPER_FLAGS_4(maq_sa_w_phr, 0, void, i32, tl, tl, env)
DEF_HELPER_FLAGS_3(mul_ph, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(mul_s_ph, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(mulq_s_ph, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(mulq_s_w, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(mulq_rs_w, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_4(mulsa_w_ph, 0, void, i32, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_4(maq_s_w_qhll, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(maq_s_w_qhlr, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(maq_s_w_qhrl, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(maq_s_w_qhrr, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(maq_sa_w_qhll, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(maq_sa_w_qhlr, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(maq_sa_w_qhrl, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(maq_sa_w_qhrr, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(maq_s_l_pwl, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(maq_s_l_pwr, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(dmadd, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(dmaddu, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(dmsub, 0, void, tl, tl, i32, env)
DEF_HELPER_FLAGS_4(dmsubu, 0, void, tl, tl, i32, env)
#endif

/* DSP Bit/Manipulation Sub-class insns */
DEF_HELPER_FLAGS_1(bitrev, TCG_CALL_NO_RWG_SE, tl, tl)
DEF_HELPER_FLAGS_3(insv, 0, tl, env, tl, tl)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(dinsv, 0, tl, env, tl, tl)
#endif

/* DSP Compare-Pick Sub-class insns */
DEF_HELPER_FLAGS_3(cmpu_eq_qb, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_3(cmpu_lt_qb, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_3(cmpu_le_qb, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_2(cmpgu_eq_qb, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(cmpgu_lt_qb, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(cmpgu_le_qb, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_3(cmp_eq_ph, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_3(cmp_lt_ph, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_3(cmp_le_ph, 0, void, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(cmpu_eq_ob, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_3(cmpu_lt_ob, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_3(cmpu_le_ob, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_3(cmpgdu_eq_ob, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(cmpgdu_lt_ob, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(cmpgdu_le_ob, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_2(cmpgu_eq_ob, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(cmpgu_lt_ob, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_2(cmpgu_le_ob, TCG_CALL_NO_RWG_SE, tl, tl, tl)
DEF_HELPER_FLAGS_3(cmp_eq_qh, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_3(cmp_lt_qh, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_3(cmp_le_qh, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_3(cmp_eq_pw, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_3(cmp_lt_pw, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_3(cmp_le_pw, 0, void, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(pick_qb, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(pick_ph, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(pick_ob, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(pick_qh, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(pick_pw, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_2(packrl_ph, TCG_CALL_NO_RWG_SE, tl, tl, tl)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_2(packrl_pw, TCG_CALL_NO_RWG_SE, tl, tl, tl)
#endif

/* DSP Accumulator and DSPControl Access Sub-class insns */
DEF_HELPER_FLAGS_3(extr_w, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(extr_r_w, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(extr_rs_w, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(dextr_w, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(dextr_r_w, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(dextr_rs_w, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(dextr_l, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(dextr_r_l, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(dextr_rs_l, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(extr_s_h, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(dextr_s_h, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(extp, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(extpdp, 0, tl, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(dextp, 0, tl, tl, tl, env)
DEF_HELPER_FLAGS_3(dextpdp, 0, tl, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(shilo, 0, void, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(dshilo, 0, void, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(mthlip, 0, void, tl, tl, env)
#if defined(TARGET_MIPS64)
DEF_HELPER_FLAGS_3(dmthlip, 0, void, tl, tl, env)
#endif
DEF_HELPER_FLAGS_3(wrdsp, 0, void, tl, tl, env)
DEF_HELPER_FLAGS_2(rddsp, 0, tl, tl, env)

/* MIPS SIMD Architecture */
DEF_HELPER_4(msa_andi_b, void, env, i32, i32, i32)
DEF_HELPER_4(msa_ori_b, void, env, i32, i32, i32)
DEF_HELPER_4(msa_nori_b, void, env, i32, i32, i32)
DEF_HELPER_4(msa_xori_b, void, env, i32, i32, i32)
DEF_HELPER_4(msa_bmnzi_b, void, env, i32, i32, i32)
DEF_HELPER_4(msa_bmzi_b, void, env, i32, i32, i32)
DEF_HELPER_4(msa_bseli_b, void, env, i32, i32, i32)
DEF_HELPER_5(msa_shf_df, void, env, i32, i32, i32, i32)

DEF_HELPER_5(msa_addvi_df, void, env, i32, i32, i32, s32)
DEF_HELPER_5(msa_subvi_df, void, env, i32, i32, i32, s32)
DEF_HELPER_5(msa_maxi_s_df, void, env, i32, i32, i32, s32)
DEF_HELPER_5(msa_maxi_u_df, void, env, i32, i32, i32, s32)
DEF_HELPER_5(msa_mini_s_df, void, env, i32, i32, i32, s32)
DEF_HELPER_5(msa_mini_u_df, void, env, i32, i32, i32, s32)
DEF_HELPER_5(msa_ceqi_df, void, env, i32, i32, i32, s32)
DEF_HELPER_5(msa_clti_s_df, void, env, i32, i32, i32, s32)
DEF_HELPER_5(msa_clti_u_df, void, env, i32, i32, i32, s32)
DEF_HELPER_5(msa_clei_s_df, void, env, i32, i32, i32, s32)
DEF_HELPER_5(msa_clei_u_df, void, env, i32, i32, i32, s32)
DEF_HELPER_4(msa_ldi_df, void, env, i32, i32, s32)

DEF_HELPER_5(msa_slli_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_srai_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_srli_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_bclri_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_bseti_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_bnegi_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_binsli_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_binsri_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_sat_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_sat_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_srari_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_srlri_df, void, env, i32, i32, i32, i32)

DEF_HELPER_5(msa_sll_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_sra_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_srl_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_bclr_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_bset_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_bneg_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_binsl_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_binsr_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_addv_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_subv_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_max_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_max_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_min_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_min_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_max_a_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_min_a_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_ceq_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_clt_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_clt_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_cle_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_cle_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_add_a_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_adds_a_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_adds_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_adds_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_ave_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_ave_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_aver_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_aver_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_subs_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_subs_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_subsus_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_subsuu_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_asub_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_asub_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_mulv_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_maddv_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_msubv_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_div_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_div_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_mod_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_mod_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_dotp_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_dotp_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_dpadd_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_dpadd_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_dpsub_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_dpsub_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_sld_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_splat_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_pckev_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_pckod_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_ilvl_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_ilvr_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_ilvev_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_ilvod_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_vshf_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_srar_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_srlr_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_hadd_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_hadd_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_hsub_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_hsub_u_df, void, env, i32, i32, i32, i32)

DEF_HELPER_5(msa_sldi_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_splati_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_copy_s_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_copy_u_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_insert_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_insve_df, void, env, i32, i32, i32, i32)
DEF_HELPER_3(msa_ctcmsa, void, env, tl, i32)
DEF_HELPER_2(msa_cfcmsa, tl, env, i32)
DEF_HELPER_3(msa_move_v, void, env, i32, i32)

DEF_HELPER_5(msa_fcaf_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fcun_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fceq_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fcueq_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fclt_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fcult_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fcle_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fcule_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fsaf_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fsun_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fseq_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fsueq_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fslt_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fsult_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fsle_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fsule_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fadd_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fsub_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fmul_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fdiv_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fmadd_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fmsub_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fexp2_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fexdo_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_ftq_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fmin_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fmin_a_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fmax_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fmax_a_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fcor_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fcune_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fcne_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_mul_q_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_madd_q_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_msub_q_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fsor_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fsune_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_fsne_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_mulr_q_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_maddr_q_df, void, env, i32, i32, i32, i32)
DEF_HELPER_5(msa_msubr_q_df, void, env, i32, i32, i32, i32)

DEF_HELPER_4(msa_and_v, void, env, i32, i32, i32)
DEF_HELPER_4(msa_or_v, void, env, i32, i32, i32)
DEF_HELPER_4(msa_nor_v, void, env, i32, i32, i32)
DEF_HELPER_4(msa_xor_v, void, env, i32, i32, i32)
DEF_HELPER_4(msa_bmnz_v, void, env, i32, i32, i32)
DEF_HELPER_4(msa_bmz_v, void, env, i32, i32, i32)
DEF_HELPER_4(msa_bsel_v, void, env, i32, i32, i32)
DEF_HELPER_4(msa_fill_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_pcnt_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_nloc_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_nlzc_df, void, env, i32, i32, i32)

DEF_HELPER_4(msa_fclass_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_ftrunc_s_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_ftrunc_u_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_fsqrt_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_frsqrt_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_frcp_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_frint_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_flog2_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_fexupl_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_fexupr_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_ffql_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_ffqr_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_ftint_s_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_ftint_u_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_ffint_s_df, void, env, i32, i32, i32)
DEF_HELPER_4(msa_ffint_u_df, void, env, i32, i32, i32)

#define MSALDST_PROTO(type)                         \
DEF_HELPER_3(msa_ld_ ## type, void, env, i32, tl)   \
DEF_HELPER_3(msa_st_ ## type, void, env, i32, tl)
MSALDST_PROTO(b)
MSALDST_PROTO(h)
MSALDST_PROTO(w)
MSALDST_PROTO(d)
#undef MSALDST_PROTO
