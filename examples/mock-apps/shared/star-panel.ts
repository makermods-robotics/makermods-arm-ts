/**
 * A simulated Star Arm 102 leader with a panel of sliders: each slider turns one simulated FashionStar servo (as a
 * hand turning the leader's joint would), and its checkbox takes the servo off the bus. The StarLeader reads the
 * servos through the real protocol code on the simulated serial port.
 */

import { type Product, StarLeader } from "@makermods/arm-core";
import { simulatedStarLeader, type SimulatedStarPort } from "@makermods/arm-core/testing";

export interface StarPanel {
  leader: StarLeader;
  port: SimulatedStarPort;
}

/** A simulated leader for `product`'s mapping, its sliders put in `container` (ids `<prefix>-servo-<id>`). */
export function starPanel(container: HTMLElement, product: Product, prefix = "star"): StarPanel {
  const [leader, port] = simulatedStarLeader({ product });
  const names = leader.mapper.names;
  container.replaceChildren(
    ...leader.servoIds.map((id, i) => {
      const row = document.createElement("div");
      row.className = "servo";
      row.innerHTML =
        `<label>${names[i] ?? `servo ${id}`} <input type="range" id="${prefix}-servo-${id}" min="-180" max="180" step="1" value="0" /></label>` +
        ` <output id="${prefix}-deg-${id}">0</output>&deg;` +
        ` <label><input type="checkbox" id="${prefix}-online-${id}" checked /> answering</label>`;
      const slider = row.querySelector<HTMLInputElement>("input[type=range]")!;
      const online = row.querySelector<HTMLInputElement>("input[type=checkbox]")!;
      slider.addEventListener("input", () => {
        port.setAngle(id, Number(slider.value));
        row.querySelector("output")!.textContent = slider.value;
      });
      online.addEventListener("change", () => port.setOnline(id, online.checked));
      return row;
    }),
  );
  return { leader, port };
}
