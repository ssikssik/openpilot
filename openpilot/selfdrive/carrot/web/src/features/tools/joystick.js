(() => {
  const BTN_ID = "btnToolsJoystick";
  let enabled = false;
  let busy = false;

  const t = (key, fallback) =>
    typeof getUIText === "function" ? getUIText(key, fallback) : fallback;

  const isTruthy = (value) =>
    value === true || value === 1 || value === "1" || value === "true" || value === "True";

  const isExplicitlyOff = (value) =>
    value === false || value === 0 || value === "0" || value === "false" || value === "False";

  function render() {
    const button = document.getElementById(BTN_ID);
    if (!button) return;
    const baseLabel = t("joystick_mode", "Joystick Mode");
    const stateLabel = enabled ? "ON" : "OFF";
    button.textContent = `${baseLabel}: ${stateLabel}`;
    button.dataset.state = enabled ? "on" : "off";
    button.setAttribute("aria-pressed", enabled ? "true" : "false");
    button.classList.toggle("active", enabled);
    button.disabled = busy;
  }

  async function refresh() {
    if (typeof bulkGet !== "function") {
      render();
      return;
    }
    try {
      const values = await bulkGet(["JoystickDebugMode"]);
      enabled = isTruthy(values?.JoystickDebugMode);
    } catch {
      // Keep last known state when offline or unavailable.
    }
    render();
  }

  async function toggle() {
    if (busy || typeof setParam !== "function") return;
    busy = true;
    render();
    try {
      let offroad = true;
      if (typeof bulkGet === "function") {
        try {
          const values = await bulkGet(["JoystickDebugMode", "IsOffroad"]);
          enabled = isTruthy(values?.JoystickDebugMode);
          if (values && Object.prototype.hasOwnProperty.call(values, "IsOffroad")) {
            offroad = !isExplicitlyOff(values.IsOffroad);
          }
        } catch {}
      }

      const nextEnabled = !enabled;
      if (nextEnabled) {
        if (!offroad) {
          if (typeof showAppToast === "function") {
            showAppToast(
              t("joystick_mode_offroad_only", "Turn off the car (offroad) before enabling Joystick Mode."),
              { tone: "error" },
            );
          }
          return;
        }
        if (typeof appConfirm === "function") {
          const ok = await appConfirm(
            t(
              "joystick_mode_confirm",
              "Enable Joystick Mode? Use only in a safe private area with a paired controller.",
            ),
            {
              title: t("joystick_mode", "Joystick Mode"),
              confirmLabel: t("enable", "Enable"),
            },
          );
          if (!ok) return;
        }
        await setParam("JoystickDebugMode", 1);
        await setParam("LongitudinalManeuverMode", 0).catch(() => {});
        enabled = true;
        if (typeof showAppToast === "function") {
          showAppToast(t("joystick_mode_enabled", "Joystick Mode enabled."));
        }
      } else {
        await setParam("JoystickDebugMode", 0);
        enabled = false;
        if (typeof showAppToast === "function") {
          showAppToast(t("joystick_mode_disabled", "Joystick Mode disabled."));
        }
      }
    } catch (err) {
      if (typeof showAppToast === "function") {
        showAppToast(err?.message || t("failed", "Failed"), { tone: "error" });
      }
    } finally {
      busy = false;
      render();
    }
  }

  function init() {
    const button = document.getElementById(BTN_ID);
    if (!button) return;
    if (button.dataset.joystickBound !== "1") {
      button.dataset.joystickBound = "1";
      button.onclick = () => {
        toggle().catch(() => {});
      };
    }
    render();
    refresh().catch(() => {});
  }

  globalThis.CarrotJoystick = {
    init,
    refresh,
    render,
    toggle,
  };
})();
