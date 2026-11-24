/**
 * The server side of the mock playground, run in the test process: a stand-in for the Dream gateway and platform
 * that answers with action chunks recorded from the Python client (the Dream client's test/fixtures/wire.json), and
 * in front of it the Dream client's own proxy (createDreamProxy + nodeHandler) holding the API key. The proxy's
 * upstream fetch reaches only the stand-in: any other address is recorded and refused.
 */

import { existsSync, readFileSync } from "node:fs";
import { createServer, type IncomingMessage, type Server } from "node:http";
import type { AddressInfo } from "node:net";
import { fileURLToPath } from "node:url";

export const DREAMLABS = fileURLToPath(new URL("../../dreamlabs-ts/", import.meta.url));
export const hasDreamlabs = existsSync(`${DREAMLABS}src/proxy.ts`) && existsSync(`${DREAMLABS}test/fixtures/wire.json`);

export interface Recorded {
  method: string;
  path: string;
  headers: IncomingMessage["headers"];
  body: string;
}

export interface DreamStack {
  proxyUrl: string;
  gatewayUrl: string;
  key: string;
  /** Requests the stand-in gateway received (through the proxy). */
  received: Recorded[];
  /** Upstream calls the proxy tried to make to anything but the stand-in. */
  escaped: string[];
  /** "ok": recorded chunks; "error": every get_action answers DA-5000 serve_failed. */
  mode: "ok" | "error";
  close(): Promise<void>;
}

const listen = (server: Server) =>
  new Promise<string>((resolve) =>
    server.listen(0, "127.0.0.1", () => resolve(`127.0.0.1:${(server.address() as AddressInfo).port}`)),
  );

const read = (request: IncomingMessage) =>
  new Promise<string>((resolve) => {
    let text = "";
    request.on("data", (chunk) => (text += chunk));
    request.on("end", () => resolve(text));
  });

interface WireRequest {
  path: string;
  reply: { status: number; body: string };
}

/** The recorded scenario of client IK on a Metal arm for the YAM-trained base model: its models listing, its
 * end-effector chunks and a serve failure from another scenario. */
function recorded() {
  const wire = JSON.parse(readFileSync(`${DREAMLABS}test/fixtures/wire.json`, "utf8")) as {
    scenarios: { name: string; models: unknown[]; steps: { requests: WireRequest[] }[] }[];
  };
  const metal = wire.scenarios.find((s) => s.name.startsWith("client IK on Metal"))!;
  const replies = (s: (typeof wire.scenarios)[number]) =>
    s.steps.flatMap((step) => step.requests).filter((r) => r.path === "/v1/get_action").map((r) => r.reply);
  const chunks = replies(metal)
    .map((r) => JSON.parse(r.body) as { actions?: { per_key?: Record<string, unknown> } })
    .filter((b) => b.actions?.per_key?.["action.right_eef_xyz"]);
  const failure = wire.scenarios.flatMap(replies).find((r) => r.body.includes('"DA-5000"'))!;
  return { models: metal.models, chunks, failure };
}

/** Starts the stand-in gateway and the proxy; `pageOrigin` is the app's origin, the only one the proxy answers. */
export async function startDreamStack(pageOrigin: string): Promise<DreamStack> {
  const { createDreamProxy } = await import("../../dreamlabs-ts/src/proxy.ts");
  const { nodeHandler } = await import("../../dreamlabs-ts/src/node/index.ts");
  const { models, chunks, failure } = recorded();
  const key = `dk_mock_${Math.random().toString(16).slice(2)}${Date.now().toString(16)}`;
  const received: Recorded[] = [];
  const escaped: string[] = [];
  let served = 0;

  const stack = { received, escaped, key, mode: "ok" } as DreamStack;
  const gateway = createServer(async (request, response) => {
    const body = await read(request);
    const path = (request.url ?? "/").split("?")[0]!;
    received.push({ method: request.method ?? "GET", path, headers: request.headers, body });
    const send = (status: number, value: unknown) => {
      response.writeHead(status, { "Content-Type": "application/json" });
      response.end(typeof value === "string" ? value : JSON.stringify(value));
    };
    if (request.headers.authorization !== `Bearer ${key}`) {
      return send(401, { status: { code: "DA-1000", name: "unauthorized", reason: "unknown_key" } });
    }
    if (path === "/v1/models") return send(200, models);
    if (path === "/v1/end_session") return send(200, { status: { code: "DA-0000" } });
    if (path === "/v1/get_action") {
      if (stack.mode === "error") return send(failure.status, failure.body);
      const sent = JSON.parse(body) as { session_id: string | null };
      const reply = structuredClone(chunks[served++ % chunks.length]!) as Record<string, unknown>;
      reply.session_id = sent.session_id ?? "ses_mock";
      reply.status = sent.session_id ? { code: "DA-0000", name: "ok" } : { code: "DA-0001", name: "new_session", reason: "new" };
      return send(200, reply);
    }
    send(404, { status: { code: "DA-4000", name: "bad_request" } });
  });
  const gatewayHost = await listen(gateway);
  const gatewayUrl = `http://${gatewayHost}`;

  const proxyServer = createServer();
  const proxyHost = await listen(proxyServer);
  const upstream = (input: string, init: RequestInit) => {
    if (new URL(input).host !== gatewayHost) {
      escaped.push(input);
      return Promise.reject(new Error(`the test proxy reaches only the stand-in gateway, not ${new URL(input).host}`));
    }
    return fetch(input, init);
  };
  proxyServer.on(
    "request",
    nodeHandler(
      createDreamProxy({
        apiKey: key,
        allowedHosts: [proxyHost],
        allowedOrigins: [pageOrigin],
        basePath: "/api/dream",
        gatewayUrl,
        platformUrl: gatewayUrl,
        dataUrl: gatewayUrl,
        fetch: upstream,
      }),
    ),
  );

  return Object.assign(stack, {
    proxyUrl: `http://${proxyHost}/api/dream`,
    gatewayUrl,
    close: async () => {
      await Promise.all([gateway, proxyServer].map((s) => new Promise<void>((resolve) => (s.closeAllConnections(), s.close(() => resolve())))));
    },
  });
}
