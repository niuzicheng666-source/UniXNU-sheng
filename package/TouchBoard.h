// SPDX-License-Identifier: BSD-2-Clause-Patent
// sheng (SM8550) touch controller board facts and the pure helpers around them. Every constant states its source.
// Nothing here touches hardware; TouchProbe.cpp owns the MMIO. Host-tested by TouchBoardTests.c.
#ifndef SHENG_TOUCH_BOARD_H
#define SHENG_TOUCH_BOARD_H
#include <stdint.h>

// Board DT (build/sheng/sheng.preprocessed.dts): spi4 = wrapper geniqup@ac0000, engine reg 0xa90000 size 0x4000.
#define TB_QUP_WRAPPER_BASE 0x00ac0000ULL
#define TB_SE_BASE 0x00a90000ULL
#define TB_SE_SIZE 0x4000ULL
// sm8550.dtsi: geniqup@ac0000 (spi4's wrapper) has iommus = <&apps_smmu 0xa3 0>; apps_smmu@15000000 (docs/native-ufs.md).
#define TB_QUP_SMMU_SID 0xa3
#define TB_APPS_SMMU_BASE 0x15000000ULL
#define TB_APPS_SMMU_SIZE 0xd0000ULL
// gcc-sm8550.c (Linux mainline): GCC at 0x100000 per the DT (clock-controller@100000).
#define TB_GCC_BASE 0x00100000ULL
#define TB_GCC_VOTE_ENABLE 0x52008            // enable_reg of every QUP branch below
#define TB_GCC_GPLL0_ENABLE 0x52018           // gcc_gpll0 enable_reg, bit 0
#define TB_GCC_GPLL0_MODE 0x0
#define TB_VOTE_WRAP1_CORE_2X (1u<<18)        // gcc_qupv3_wrap1_core_2x_clk, halt 0x23294
#define TB_VOTE_WRAP1_CORE (1u<<19)           // gcc_qupv3_wrap1_core_clk, halt 0x23284
#define TB_VOTE_WRAP1_M_AHB (1u<<20)          // gcc_qupv3_wrap_1_m_ahb_clk, halt 0x2327c
#define TB_VOTE_WRAP1_S_AHB (1u<<21)          // gcc_qupv3_wrap_1_s_ahb_clk, halt 0x23280
#define TB_VOTE_WRAP1_S4 (1u<<26)             // gcc_qupv3_wrap1_s4_clk, halt 0x184e4
#define TB_VOTE_ALL (TB_VOTE_WRAP1_CORE_2X|TB_VOTE_WRAP1_CORE|TB_VOTE_WRAP1_M_AHB|TB_VOTE_WRAP1_S_AHB|TB_VOTE_WRAP1_S4)
#define TB_HALT_CORE_2X 0x23294
#define TB_HALT_CORE 0x23284
#define TB_HALT_M_AHB 0x2327c
#define TB_HALT_S_AHB 0x23280
#define TB_HALT_S4 0x184e4
#define TB_CBCR_CLK_OFF (1u<<31)
// gcc_qupv3_wrap1_s4_clk_src: cmd_rcgr 0x184f0, DFS-managed (clk-rcg2.c SE_* macros).
#define TB_S4_RCG 0x184f0
#define TB_RCG_CMD 0x0
#define TB_RCG_CFG 0x4
#define TB_RCG_DFSR_CMD 0x14                  // bit0 DFS enable, bits 4:1 current level
#define TB_RCG_PERF_CFG(l) (0x1c+4*(l))
#define TB_RCG_PERF_M(l) (0x5c+4*(l))
#define TB_RCG_PERF_N(l) (0x9c+4*(l))
#define TB_DFS_LEVELS 8
// TLMM (DT: pinctrl@f100000; pinctrl-sm8550.c: ctl_reg = 0x1000*gpio, io_reg = +4, mux_bit 2, drv_bit 6, oe_bit 9).
#define TB_TLMM_BASE 0x0f100000ULL
#define TB_GPIO_STRIDE 0x1000
#define TB_GPIO_SPI_MISO 44                   // sheng.dts qup_spi4_data_clk: gpio44/45/46, qup_spi4_cs: gpio47
#define TB_GPIO_SPI_MOSI 45
#define TB_GPIO_SPI_CLK 46
#define TB_GPIO_SPI_CS 47
#define TB_GPIO_IRQ 25                        // novatek,irq-gpio = <&tlmm 25 0x2001>
#define TB_FUNC_QUP1_SE4 1                    // PINGROUP(44..47, qup1_se4, ...): function slot 1
#define TB_DRIVE_6MA 2                        // drive-strength = <6>: 2 mA steps encoded as (mA/2)-1
#define TB_SPI_MODE 0                         // sheng.dts touchscreen@0: spi-max-frequency 19200000, mode 0
#define TB_SPI_DEFAULT_HZ 19200000u   // board DT: touchscreen spi-max-frequency = <19200000>
#define TB_SPI_MAX_HZ 19200000u

static inline uint32_t tb_pin_cfg(unsigned func, unsigned drive_code) {return (func&15u)<<2|(drive_code&7u)<<6;}   // pull disabled

/* ---- ST FingerTip FST2 (babylon's inner touch, "st,spi-pri") ------------------------------------
 * babylon (Xiaomi MIX Fold 3) wires the same serial engine and the same four pins as sheng, so every
 * bus constant above already applies. Read from the stock factory images on 2026-11-27, not assumed:
 *   dtbo.img fragment@36 (qupv3_se4_spi -> /soc/qcom,qupv3_1_geni_se@ac0000/spi@a90000):
 *     compatible "st,spi-pri", interrupt-parent tlmm, interrupts <25 0x2008>,
 *     fts,irq-gpio <&tlmm 25 0x2008>, fts,reset-gpio <&tlmm 24 0x0>,
 *     avdd-supply -> L16B (via __fixups__ L16B), dvdd-supply -> L12B (via __fixups__ L12B)
 *   vendor_boot.img baseline DTB:
 *     spi@a90000 reg <0xa90000 0x4000>; qupv3_se4_spi_{miso,mosi,clk,cs}_active drive-strength 6,
 *     bias-pull-down, function qup1_se4_l0..l3 (one slot per pin)
 *   apps_rsc@17a00000 drv@2: rpmh-regulator-ldob12 -> pm_humu_l12 1800000 uV (min = max = init),
 *                            rpmh-regulator-ldob16 -> pm_humu_l16 2800000 uV (min = max = init)
 * L16B is 2.8 V, NOT 3.3 V: regulator-max-microvolt is 2800000, so a 3.3 V vote would be out of range.
 * The names differ from the dtbo's node names: the vote goes by qcom,resource-name "ldob12"/"ldob16"
 * as AudioProbe::micRail() does for "ldob15". */
#define TB_FTS_SE_BASE TB_SE_BASE            // same engine as sheng: spi@a90000 (0xa90000)
#define TB_FTS_SE_SIZE TB_SE_SIZE
#define TB_FTS_RSC_BASE 0x0017a20000ULL      // apps_rsc drv@2, where micRail's LDO votes already go
#define TB_FTS_RSC_SIZE 0x10000ULL
#define TB_FTS_RSC_TCS 2                     // the TCS micRail() already drives, under the same AppsSmmuLock.
                                             // drv@2 tcs-config [2,3,0, 2,1,2, 3,0,4, 1] allocates 3+1+0 = 4 TCS
                                             // (0-3), so 4 does not exist; 0-2 are the ones working code uses.
#define TB_FTS_GPIO_IRQ 25                    // fts,irq-gpio <&tlmm 25 0x2008>
#define TB_FTS_GPIO_RESET 24                  // fts,reset-gpio <&tlmm 24 0x0>, active high
#define TB_FTS_DRIVE_6MA TB_DRIVE_6MA         // every active config: drive-strength = <6>
#define TB_FTS_SPI_MODE 0                     // st,spi-pri sets neither spi-cpha nor spi-cpol
#define TB_FTS_SPI_HZ 4000000u                // the bring-up clock the task fixed; the part runs far higher
#define TB_FTS_CHIP_ID 0x3652u                // FST2 ("spruce")
/* This board's touch part, not a mode: babylon has the FTS2 and no Novatek device at all, so the
 * probe's Novatek stages are dead code here and are refused structurally in TouchProbe.cpp rather
 * than left reachable through a stage number. */
#define TB_IS_FTS_BOARD 1
/* The chip's own SPI-pin configuration, from fts_configure_spi4() of the zizhan fst2 6.0.3 driver. It
 * must be written before the first read or the chip answers nothing. */
#define TB_FTS_HW_GPIO_PU 0x20000034u
#define TB_FTS_HW_MISO_CONFIG 0x2000003eu
#define TB_FTS_HW_SPI4_CONFIG 0x2000002du
#define TB_FTS_HW_GPIO_PU_VALUE 0x02
#define TB_FTS_HW_MISO_VALUE 0x07
#define TB_FTS_HW_SPI4_VALUE 0x02
#define TB_FTS_HW_CHIP_ID 0x20000000u
/* Power: dvdd (L12B 1.8 V) first, then avdd (L16B 2.8 V), as the task fixed. */
#define TB_FTS_DVDD_RESOURCE "ldob12"
#define TB_FTS_AVDD_RESOURCE "ldob16"
#define TB_FTS_DVDD_UV 1800000u
#define TB_FTS_AVDD_UV 2800000u
static inline uint32_t tb_tlmm_ctl(unsigned gpio) {return (uint32_t)gpio*TB_GPIO_STRIDE;}
static inline uint32_t tb_tlmm_io(unsigned gpio) {return (uint32_t)gpio*TB_GPIO_STRIDE+4;}

// Source clock rates by RCG source-select value (gcc_parent_map_0: 0 = TCXO, 1 = GPLL0 main, 6 = GPLL0 even).
// GPLL0 main is 600 MHz and even 300 MHz, inferred from the frequency table (100 MHz = main / 6, 75 MHz = even / 4).
static inline uint64_t tb_src_hz(unsigned sel) {return sel==0?19200000ULL:sel==1?600000000ULL:sel==6?300000000ULL:0;}
// clk-rcg2.c calc_rate: half-integer divider, then M/N.
static inline uint64_t tb_calc_rate(uint64_t prate, uint32_t m, uint32_t n, unsigned mode, unsigned hid) {
    if (hid) {prate*=2;prate/=hid+1;}
    if (mode && n) {prate*=m;prate/=n;}
    return prate;
}
// Frequency of DFS level l from its CFG (src 10:8, mode 13:12, hid 4:0 on this 5-bit RCG), M register and N register
// (stored inverted: n = (~N & mask) + m, mnd_width 16).
static inline uint64_t tb_dfs_level_hz(uint32_t cfg, uint32_t mreg, uint32_t nreg) {
    unsigned src=(cfg>>8)&7,mode=(cfg>>12)&3,hid=cfg&0x1f;
    uint32_t m=0,n=0;
    if (mode) {m=mreg&0xffff;n=((~nreg)&0xffff)+m;}
    return tb_calc_rate(tb_src_hz(src),m,n,mode,hid);
}
struct tb_clock_choice {unsigned level, div;uint64_t sclk_hz, actual_hz;};
// Choose the DFS level that gives the highest SPI rate not above target with divider div = ceil(sclk / (os * target)).
// Returns 0 on success.
static inline int tb_choose_clock(const uint64_t *level_hz, unsigned n, unsigned os, unsigned target, struct tb_clock_choice *out) {
    int found=0;
    for (unsigned l=0;l<n;++l) {
        uint64_t f=level_hz[l];if (!f) continue;
        uint64_t d=((f+(uint64_t)os*target-1)/((uint64_t)os*target));if (d<1) d=1;if (d>0xfff) continue;
        uint64_t actual=f/(os*d);if (actual>target) continue;
        if (!found || actual>out->actual_hz || (actual==out->actual_hz && d<out->div)) {
            out->level=l;out->div=(unsigned)d;out->sclk_hz=f;out->actual_hz=actual;found=1;
        }
    }
    return found?0:-1;
}
#endif
