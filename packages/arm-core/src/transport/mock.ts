/** Backend for tests and no hardware: records sends, takes injected receives, and can auto-respond. */

import { CanBackend } from "./base.js";

export type Responder = (canId: number, data: Uint8Array, extended: boolean) => Array<[number, Uint8Array]> | null | undefined | void;

export class MockBackend extends CanBackend {
  sent: Array<[number, Uint8Array]> = [];
  sentExtended: boolean[] = [];
  isOpen = false;
  /** responder(canId, data, extended) returns the frames to inject in reply (standard ids), or nothing. */
  responder: Responder | null = null;

  async open(): Promise<void> {
    this.isOpen = true;
  }

  async close(): Promise<void> {
    this.isOpen = false;
  }

  // Replies are delivered before send() returns, as on the Python MockBackend.
  send(canId: number, data: Uint8Array, extended = true): Promise<void> {
    const frame = Uint8Array.from(data);
    this.sent.push([canId, frame]);
    this.sentExtended.push(extended);
    if (this.responder && this.cb) {
      for (const [id, reply] of this.responder(canId, frame, extended) ?? []) this.cb(id, reply, false);
    }
    return Promise.resolve();
  }

  inject(canId: number, data: Uint8Array, extended = false): void {
    this.cb?.(canId, data, extended);
  }
}
