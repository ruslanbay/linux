# Exynos9810 (Galaxy S9 SM-G960F) mainline DT gap analysis and proposed improvements

## Scope and method
Compared the current Exynos9810/starlte upstream DT state against:

- Exynos8895 SoC + dreamlte board (closest Samsung flagship predecessor)
- Exynos850 + ExynosAuto v9 (newer upstream Exynos DT architecture patterns)
- Google Tensor GS101 (Exynos-derived, modern upstream conventions)
- Qualcomm S9 variant (`sdm845-samsung-starqltechn`) for board-level common peripherals
- Downstream Samsung Android patch (`patches/0001-Android-4.9.118-G960FXXUHFVB4.patch.gz`) only as hardware hints

## Current state (what exists today)
The current `exynos9810.dtsi` is still a bring-up skeleton: CPUs/PSCI, GIC, chipid, PMU syscon and pinctrl blocks are present, but almost all functional SoC IP blocks are still absent (no CMU, no buses/controllers, no storage, no USB/UFS, no multimedia, no thermal/devfreq scaffolding). `exynos9810-starlte.dts` currently adds memory map, simple framebuffer and GPIO keys only.

## Prioritized improvement backlog

### High priority (high impact + likely/tractable)

| Priority | Component / subsystem | Reference implementation(s) | Why it likely applies to Exynos9810 / starlte | Difficulty | Expected impact |
|---|---|---|---|---|---|
| H1 | **Clock controller topology (CMU blocks) + mandatory sysreg nodes** | `exynos8895.dtsi` has `cmu_*` + `syscon_*`; exynos850/autov9/gs101 all follow this pattern | 9810 currently has no clock providers, so virtually no IP can be enabled cleanly. Exynos family DTS consistently models CMU per domain first. | Medium | **Foundational**: prerequisite for nearly all peripherals |
| H2 | **USI fabric + UART/I2C/SPI controller nodes** | `exynos8895.dtsi`, `exynos850.dtsi`, `exynosautov9.dtsi`, `google/gs101.dtsi` | 9810 pinctrl aliases already include PERIC blocks, implying intended peripheral routing. Samsung platforms rely heavily on USI multiplexed blocks; starlte needs these for sensors/touch/PMIC adjuncts/debug UART. | Medium | **High**: enables board peripheral bring-up and debugging |
| H3 | **Storage controllers: eMMC/SD + UFS host/PHY** | `exynos8895.dtsi` contains `mmc@...`; exynos850 and GS101 model UFS+PHY resources; starqltechn has UFS rails/PHY handling | Smartphone boot/runtime storage depends on this. 9810 starlte currently has no storage DT path beyond bootloader framebuffer/memory. | Medium-High | **Critical**: persistent storage / rootfs feasibility |
| H4 | **USB DRD + PHY + Type-C/PD chain** | `exynos850.dtsi` (`usbdrd`, PHY) and `gs101-pixel-common.dtsi` (Type-C controller/connector modeling) | Downstream and Qualcomm S9 both indicate modern USB-C stack with dedicated PMIC/TCPC integration. 9810 currently lacks any USB node. | Medium-High | **Critical**: charging/data/debug usability |
| H5 | **PMIC + regulator framework for starlte board** | `google/gs101-pixel-common.dtsi` PMIC/regulator modeling; `sdm845-samsung-starqltechn.dts` has explicit MAX77705 charger/battery + regulator rails | Board-level power tree is mandatory before display/camera/radios/stability. User-provided example (MAX77705 absent in current 9810 DT) is corroborated by starqltechn and downstream max77705 presence. | High | **Critical**: power sequencing, peripheral power, battery/charging |
| H6 | **Pinctrl functional groups (not just GPIO banks)** | `exynos8895-pinctrl.dtsi` has bus-specific groups (UART/SPI/I2C/SD/UFS/BT/WLAN wakes) | Current 9810 pinctrl file mostly declares GPIO banks; functional pin mux groups are needed to bind controllers reliably and avoid board-level hacks. | Medium | **High**: unlocks peripheral nodes and stable interrupts/wake lines |

### Medium priority (high usefulness; some uncertainty or larger scope)

| Priority | Component / subsystem | Reference implementation(s) | Why it likely applies | Difficulty | Expected impact |
|---|---|---|---|---|---|
| M1 | **Watchdogs + MCT/RTC timer infrastructure** | `exynos850.dtsi` defines watchdogs + RTC; `exynos8895.dtsi` has MCT | Needed for robust reset/recovery and platform timer integration expected on Samsung SoCs. | Medium | Medium-High (stability, reboot/recovery) |
| M2 | **IOMMU (sysmmu) attachments for multimedia/display/camera blocks** | `exynos850.dtsi` and `exynosautov9.dtsi` define multiple `sysmmu@...` | Exynos multimedia pipelines generally require SYSMMU integration for DMA isolation and functionality. | High | High for camera/display/video enablement |
| M3 | **Thermal zones + cooling maps + cpufreq OPP/cpu-idle states** | GS101 provides OPP tables and idle states; Exynos platforms use thermal/power tuning in DT | 9810 currently has only CPU definitions; no upstream thermal/power operating model. | Medium-High | High for performance, throttling, battery life |
| M4 | **Display pipeline: DECON/DSI/panel graph nodes** | starqltechn has explicit panel wiring; Exynos families use DRM graph-based links | starlte currently relies on simple-framebuffer only; real panel support requires full display graph + regulators/reset gpios. | High | High (native display, suspend/resume correctness) |
| M5 | **Touchscreen + core input peripheral nodes** | `exynos8895-dreamlte.dts` enables touchscreen over HSI2C; downstream patch contains touchscreen stack for starlte | S9 requires touch to be practical; this is typically one of first board peripherals on Samsung phones. | Medium | High for usability |
| M6 | **Reserved-memory cleanup/migration toward upstream-friendly minimal set** | 8895/GS101 upstream use constrained reserved-memory for splash/ramoops etc.; downstream 9810 has large vendor-specific pools | Needed to avoid carrying non-upstream vendor carveouts while preserving required areas (ramoops, splash). | Medium | Medium (maintainability + stability) |

### Low priority (valuable later, but higher uncertainty / dependencies)

| Priority | Component / subsystem | Reference implementation(s) | Why it may apply | Difficulty | Expected impact |
|---|---|---|---|---|---|
| L1 | **Advanced multimedia (camera ISP, codecs, NPU adjunct blocks)** | Downstream patch shows broad camera/media stack; exynos850/autov9 model relevant interconnect pieces | Applies to flagship SoC, but upstream driver readiness and DT binding maturity are limiting factors. | Very High | High when complete, but long-term |
| L2 | **Fingerprint/NFC/sensor-hub detailed bring-up** | starqltechn references fingerprint SPI use; downstream shows many sensor/fingerprint components | Likely present on board, but exact chips/interrupts/regulators must be validated from schematics/logs. | High | Medium-High (feature completeness) |
| L3 | **Fine-grained low-power domains and DVFS domain modeling beyond CPU** | Modern Exynos DTs (gs101/autov9) model richer power/clock domains | Valuable for efficiency and suspend reliability, but depends on preceding CMU/regulator/thermal work. | High | Medium |

## Suggested implementation order (for later coding phase)
1. **Foundational SoC plumbing**: CMU + sysreg + watchdog/MCT/RTC + USI fabric.
2. **Board power**: PMIC/regulators (including battery/charger path), then pinctrl functional groups.
3. **Essential IO**: storage (UFS/eMMC), USB DRD/Type-C.
4. **User-facing baseline**: display pipeline + touchscreen.
5. **Performance/safety**: thermal/cpufreq/idle and SYSMMU wiring.
6. **Feature completion**: camera/audio/sensors/fingerprint/NFC.

## Specific backportable patterns to consider
- Reuse the **USI child-node structure** style from exynos850/gs101 (single USI parent exposing i2c/spi/uart options).
- Reuse **placeholder-regulator strategy** temporarily (as done in dreamlte/pixel-common) only where PMIC work is pending, then replace with real rails.
- Reuse **USB connector graph modeling** from GS101 for Type-C role/orientation wiring.
- Reuse **board bring-up pattern** from dreamlte: enable one peripheral at a time (e.g., touchscreen on a known HSI2C bus) with minimal, auditable pinctrl + supplies.

## Confidence notes
- High confidence on missing **foundational SoC nodes** (clock, buses, storage, USB, power) because these are universally present in mature Exynos DTs but absent in current 9810 files.
- Moderate confidence on exact board peripherals (fingerprint/NFC/sensor variants) without schematic confirmation; downstream patch indicates presence but not always directly upstream-usable bindings.
