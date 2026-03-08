# Exynos9810 (Galaxy S9 SM-G960F) mainline DT analysis report (Codex)

## Goal and constraints
This report focuses on **what to implement in DT using existing upstream bindings/drivers first**.
No DT code changes are proposed here; this is planning only.

Key constraint adopted for prioritization:
1. Prefer components with an upstream DT binding + upstream driver.
2. Use downstream Samsung DT only as hardware hint (addresses, rails, topology), not as direct source.

## Inputs compared
- Current WIP mainline files:
  - `arch/arm64/boot/dts/exynos/exynos9810.dtsi`
  - `arch/arm64/boot/dts/exynos/exynos9810-starlte.dts`
  - `arch/arm64/boot/dts/exynos/exynos9810-pinctrl.dtsi`
- Reference upstream implementations:
  - `exynos8895*`, `exynos850*`, `exynosautov9*`, `exynos/google/gs101*`
  - `arch/arm64/boot/dts/qcom/sdm845-samsung-starqltechn.dts`
- Downstream vendor reference:
  - `patches/0001-Android-4.9.118-G960FXXUHFVB4.patch.gz`
  - specifically `exynos9810-starlte_eur_open_26.dts` section

---

## 1) Current upstream Exynos9810 status (gap baseline)
Current Exynos9810 DT is still bring-up level:
- Present: CPU map, PSCI, timer, PMU PMUv3 nodes, GIC, chipid, PMU syscon, pinctrl banks.
- Missing from SoC DT: CMU/clock providers, most sysregs, USI/UART/I2C/SPI controllers, storage (UFS/MMC), USB DRD, PCIe, display pipeline, thermal zones/cooling maps, most multimedia and IOMMU wiring.
- Board DT (`starlte`) currently contains only memory, splash framebuffer, and gpio-keys.

Implication: the next work should be staged around foundational SoC infra and board power/peripheral bring-up, not feature-complete vendor parity.

---

## 2) Existing upstream drivers/bindings relevant to Galaxy S9

These are immediately useful because they already exist in-tree.

### Display / GPU / input
- **Panel S6E3HA8**
  - Binding: `Documentation/devicetree/bindings/display/panel/samsung,s6e3ha8.yaml`
  - Driver: `drivers/gpu/drm/panel/panel-samsung-s6e3ha8.c`
  - Value: direct path away from simplefb to real DRM panel node.

- **Touchscreen S6SY761**
  - Binding: `Documentation/devicetree/bindings/input/touchscreen/samsung,s6sy761.yaml`
  - Driver: `drivers/input/touchscreen/s6sy761.c`
  - Value: clean upstream replacement for vendor `sec_ts` style nodes.

- **Mali Bifrost + Panfrost**
  - Binding: `Documentation/devicetree/bindings/gpu/arm,mali-bifrost.yaml`
  - Driver family: `drivers/gpu/drm/panfrost/*`
  - Value: enables upstream GPU path for Mali-G72 class hardware.

### Power / charging / PMIC
- **MAX77705 PMIC family**
  - MFD binding: `Documentation/devicetree/bindings/mfd/maxim,max77705.yaml`
  - Charger binding: `Documentation/devicetree/bindings/power/supply/maxim,max77705.yaml`
  - Drivers: `drivers/mfd/max77705.c`, `drivers/power/supply/max77705_charger.c`, `drivers/leds/leds-max77705.c`
  - Value: charger/fuel/power ecosystem has upstream path.

- **S2DOS05 regulator PMIC**
  - Binding: `Documentation/devicetree/bindings/mfd/samsung,s2dos05.yaml`
  - Driver: `drivers/regulator/s2dos05-regulator.c`
  - Value: known panel/touch power rail provider in S9 references.

### Connectivity / sensors / PMU
- **Broadcom FullMAC Wi-Fi (`brcmfmac`)**
  - Binding: `Documentation/devicetree/bindings/net/wireless/brcm,bcm4329-fmac.yaml`
  - Driver: `drivers/net/wireless/broadcom/brcm80211/brcmfmac/*`
  - Value: usable upstream Wi-Fi integration strategy.

- **ST LSM6DSx IMU**
  - Driver stack: `drivers/iio/imu/st_lsm6dsx/*`
  - Value: likely sensor-hub/IMU integration path when exact part is confirmed.

- **ARM PMUv3 / Mongoose PMU compatibility**
  - Binding: `Documentation/devicetree/bindings/arm/pmu.yaml`
  - Driver: `drivers/perf/arm_pmuv3.c`
  - Value: existing 9810 PMU nodes are on an upstream-compatible path.

---

## 3) High-value information extracted from downstream `starlte_eur_open_26.dts`

The downstream file is vendor-heavy, but it still provides strong board-topology hints that are useful for upstream sequencing.

### A. Confirms key board components and buses
- Touch controller appears on `hsi2c@104B0000` at `reg = <0x48>` (vendor compatible is `sec,sec_ts`; upstream target should be `samsung,s6sy761` once validated).
- `s2dos05_pmic@60` appears on `hsi2c@14350000` with rails used like display power rails.
- `max77705@66` appears on `hsi2c@14360000`.
- `dwmmc2@11500000` exists and is configured for external SD slot.
- `ufs@0x11120000` exists with dedicated fixed regulator (`ufs-vcc`) and UFS PHY subnodes.
- `usb@10C00000` + DWC3 subnode + `phy@11100000` exist.
- `pcie0@116A0000` and `pcie1@116B0000` exist.
- Display stack nodes exist: `decon_f@0x16030000`, `dsim@0x16080000`, `displayport@0x11090000`.

### B. Useful pinctrl and wake-line clues
Downstream defines dedicated pin states for:
- `wlan_host_wake`, `bt_hostwake`, `pcie_wake`, `ufs-rst-n`, `ufs-refclk-out`, SD card detect, panel/TE-related lines.
This is valuable for expanding current upstream `exynos9810-pinctrl.dtsi` beyond GPIO banks.

### C. Thermal/cooling and IOMMU blocks exist downstream
- Multiple thermal zones/cooling maps are defined downstream.
- Numerous SYSMMU instances are defined for multimedia/display/camera paths.
These are useful as existence proof, but vendor properties are not directly upstreamable.

### D. What to avoid copying directly
- Vendor-only compatibles/properties such as `sec,sec_ts`, `samsung,brcm-wlan`, `samsung,bcm43xx`, Samsung private battery policy blocks, and large vendor memory carve-outs.
- These should be translated to upstream bindings and minimal required properties.

---

## 4) Revised prioritized backlog (upstream-driver-first)

## High priority
1. **CMU/sysreg + core controller scaffolding** (clock providers, sysregs, USI fabric).
2. **Board power foundation** using upstream MAX77705/S2DOS05 bindings + regulator consumers.
3. **Storage**: UFS host/PHY/fixed-supply and SD/MMC node(s).
4. **USB DRD path**: Exynos USB + DWC3 + PHY and role wiring.
5. **Display baseline**: DSI host graph + `samsung,s6e3ha8` panel.
6. **Touch baseline**: I2C touch node using upstream `samsung,s6sy761` binding.

## Medium priority
1. **Wi-Fi/BT enablement** using upstream brcmfmac-style DT (host-wake/reset/firmware naming) and proper BT binding path.
2. **Thermal zones/cooling maps + devfreq/cpufreq wiring** with upstream thermal framework constructs.
3. **SYSMMU/IOMMU attachments** for display/media/camera pipelines.
4. **Functional pinctrl groups** for buses and wake lines (not only bank declarations).

## Low priority
1. Camera/ISP/NPU feature completeness where bindings and drivers are still fragmented.
2. Vendor extras (security/memory carveout-heavy nodes) unless there is a direct upstream equivalent.

---

## 5) Recommended implementation gates (before adding each DT node)
For each planned component, require all 3 before implementation:
1. Upstream binding exists for the exact compatible.
2. Upstream driver exists and has OF match for that compatible.
3. At least one upstream DT reference pattern exists (same SoC family or same peripheral).

If any gate fails, mark as deferred/experimental and do not include in initial starlte enablement series.

---

## 6) Proposed execution order for future patches
1. SoC infra: CMU/sysreg/USI + basic serial path.
2. PMIC/regulators (MAX77705/S2DOS05), then consumers.
3. Storage + USB.
4. Display panel + touchscreen.
5. Thermal + IOMMU.
6. Connectivity (Wi-Fi/BT) and remaining sensors.

This order minimizes dead-ends and aligns with available upstream drivers.

---

## 7) Commit strategy for upstreaming (bite-size, subsystem-focused)
When turning this plan into patches, use **small, reviewable commits** and follow kernel norms:

- One logical change per commit (single subsystem/topic whenever possible).
- Keep DTSI (SoC) and DTS (board) changes separate unless tightly coupled.
- Add bindings first when new compatibles/properties are introduced, then driver support, then DT usage.
- Prefer enabling infrastructure in one patch and consumers in subsequent patches.
- Avoid mixing refactors/renames with functional enablement.
- Keep vendor-only properties out of upstream series; map only to documented upstream bindings.

### Commit message best practices
- Subject format: `arm64: dts: exynos: <what changed>`
- Subject length ~72 chars, imperative mood.
- Body explains:
  1. Why change is needed (problem / missing support)
  2. What hardware/topology is being modeled
  3. Why binding/compatible choice is correct (upstream reference)
  4. Any limitations/TODOs (explicitly)
- Add tags as appropriate (`Fixes:`, `Link:`, `Signed-off-by:`), and keep changelog concise.

---

## 8) Prioritized high-level commit list (proposed)
The list below is intentionally split into small upstreamable units.

## Phase 1 — SoC foundation (highest priority)
1. **`arm64: dts: exynos: add Exynos9810 CMU/sysreg skeleton`**
   - Introduce core clock controller and required sysreg nodes needed by later IP blocks.
2. **`arm64: dts: exynos: add Exynos9810 USI controller blocks`**
   - Add USI parent nodes with disabled child-mode endpoints (I2C/SPI/UART style).
3. **`arm64: dts: exynos: add Exynos9810 watchdog/MCT/RTC base nodes`**
   - Minimal timer/watchdog infrastructure required for stable bring-up and reboot paths.

## Phase 2 — Board power and pinmux prerequisites
4. **`arm64: dts: exynos: starlte: add PMIC interrupt and base regulator placeholders`**
   - Temporary placeholders only where needed to unblock strict binding requirements.
5. **`arm64: dts: exynos: starlte: model S2DOS05 PMIC regulators`**
   - Add S2DOS05 node and rail definitions used by display/touch and related consumers.
6. **`arm64: dts: exynos: starlte: add MAX77705 PMIC/charger topology`**
   - Add MAX77705 MFD and charger/fuel-related upstream-compatible node layout.
7. **`arm64: dts: exynos: add Exynos9810 pinctrl functional groups`**
   - Add bus/wake-related groups (`ufs-rst`, `wlan_host_wake`, `bt_hostwake`, etc.).

## Phase 3 — Essential IO
8. **`arm64: dts: exynos: add Exynos9810 UFS host/phy nodes`**
   - SoC-level UFS + PHY description with board-side supply hooks left in DTS.
9. **`arm64: dts: exynos: starlte: wire UFS supplies/reset/pinctrl`**
   - Board-level regulator/reset/pinctrl wiring for usable UFS.
10. **`arm64: dts: exynos: add Exynos9810 dw-mshc node`**
    - SoC MMC controller node disabled by default.
11. **`arm64: dts: exynos: starlte: enable external SD slot`**
    - Card-detect GPIO, pinctrl, bus-width, timing, and regulator hookup.
12. **`arm64: dts: exynos: add Exynos9810 USB DRD and PHY nodes`**
    - SoC USB core + DWC3 + PHY framework nodes.
13. **`arm64: dts: exynos: starlte: enable USB-C DRD path`**
    - Board role/data path, interrupts, supplies and connector graph if applicable.

## Phase 4 — User-facing baseline
14. **`arm64: dts: exynos: starlte: add S6E3HA8 panel graph`**
    - DSI panel node with upstream `samsung,s6e3ha8` binding and supplies/reset lines.
15. **`arm64: dts: exynos: starlte: add S6SY761 touchscreen node`**
    - I2C touch node at validated bus/address with IRQ and `avdd/vdd` supplies.
16. **`arm64: dts: exynos: starlte: add ramoops and tighten reserved-memory`**
    - Keep only upstream-relevant carveouts and remove vendor-specific memory regions.

## Phase 5 — Performance and completeness
17. **`arm64: dts: exynos: add Exynos9810 SYSMMU blocks`**
    - Add IOMMU blocks with minimal upstream properties.
18. **`arm64: dts: exynos: add starlte thermal zones and cooling maps`**
    - Thermal sensors, trips and cooling-device links with conservative limits.
19. **`arm64: dts: exynos: starlte: add Wi-Fi/BT board wiring`**
    - Upstream binding-based host-wake/reset wiring and power sequencing.

### Optional later series (only if binding+driver support is ready)
20. Camera/ISP topology incrementally.
21. Extra sensors (e.g., LSM6DSx-class IMU path if component is confirmed).
22. Non-essential board peripherals (fingerprint/NFC variants) with confirmed upstream bindings.

This structure should make rebasing easier and reduce review friction by keeping each patch narrowly scoped.
