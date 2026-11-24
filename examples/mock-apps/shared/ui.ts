/** Small DOM helpers the apps share. */

export const $ = <T extends HTMLElement = HTMLElement>(id: string): T => {
  const el = document.getElementById(id);
  if (!el) throw new Error(`no #${id}`);
  return el as T;
};

export const deg = (rad: number | null | undefined, digits = 1): string =>
  rad === null || rad === undefined || !Number.isFinite(rad) ? "-" : ((rad * 180) / Math.PI).toFixed(digits);

export const errorText = (e: unknown): string => (e instanceof Error ? `${e.name === "Error" ? "" : `${e.name}: `}${e.message}` : String(e));

/** A log pane: newest line last, at most `max` lines. */
export function logPane(el: HTMLElement, max = 200): (line: string) => void {
  const lines: string[] = [];
  return (line: string) => {
    lines.push(`${new Date().toISOString().slice(11, 23)} ${line}`);
    if (lines.length > max) lines.splice(0, lines.length - max);
    el.textContent = lines.join("\n");
  };
}

/** The SDK's logger interface, written to the pane (and to the console for warnings and errors). */
export function paneLogger(log: (line: string) => void) {
  return {
    debug: (_m: string) => {},
    info: (m: string) => log(m),
    warn: (m: string, e?: unknown) => (log(`warning: ${m}`), console.warn(m, e ?? "")),
    error: (m: string, e?: unknown) => (log(`error: ${m}`), console.error(m, e ?? "")),
  };
}

/** Runs `action`, putting what it threw in `message`; returns whether it succeeded. */
export async function guarded(message: HTMLElement, action: () => Promise<unknown>): Promise<boolean> {
  try {
    await action();
    return true;
  } catch (e) {
    message.textContent = errorText(e);
    message.dataset.kind = "error";
    return false;
  }
}

export function say(message: HTMLElement, text: string, kind: "info" | "error" = "info"): void {
  message.textContent = text;
  message.dataset.kind = kind;
}

/** One row per joint: name, position (deg, the radians in data-rad), temperature, torque, fault. */
export function renderJoints(
  body: HTMLTableSectionElement,
  joints: Record<string, { position: number | null; temperature: number | null; enabled: boolean | null; fault: string | null }>,
): void {
  const names = Object.keys(joints);
  if (body.rows.length !== names.length || [...body.rows].some((r, i) => r.dataset.joint !== names[i])) {
    body.replaceChildren(
      ...names.map((name) => {
        const row = document.createElement("tr");
        row.dataset.joint = name;
        row.innerHTML = `<th scope="row">${name}</th><td class="pos"></td><td class="temp"></td><td class="torque"></td><td class="fault"></td>`;
        return row;
      }),
    );
  }
  names.forEach((name, i) => {
    const j = joints[name]!;
    const cells = body.rows[i]!.cells;
    cells[1]!.textContent = deg(j.position, 2);
    cells[1]!.dataset.rad = j.position === null ? "" : String(j.position);
    cells[2]!.textContent = j.temperature === null ? "-" : j.temperature.toFixed(0);
    cells[3]!.textContent = j.enabled === null ? "-" : j.enabled ? "on" : "off";
    cells[4]!.textContent = j.fault ?? "ok";
  });
}
