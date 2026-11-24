#!/usr/bin/env node
/**
 * Serve a bundle of examples/dist-check (built against the packages' dist entry points) and run it
 * in headless Chromium. Used by `npm run test:dist`.
 */
import { createServer } from "node:http";
import { readFile } from "node:fs/promises";
import { extname, join } from "node:path";
import { chromium } from "@playwright/test";
const root = process.argv[2];
const types = { ".html": "text/html", ".js": "text/javascript" };
const server = createServer(async (req, res) => {
  try {
    const path = req.url === "/" ? "/index.html" : req.url.split("?")[0];
    const body = await readFile(join(root, path));
    res.writeHead(200, { "content-type": types[extname(path)] ?? "application/octet-stream" }).end(body);
  } catch { res.writeHead(404).end(); }
}).listen(0, "127.0.0.1");
await new Promise((r) => server.on("listening", r));
const browser = await chromium.launch();
const page = await browser.newPage();
const errors = [];
page.on("pageerror", (e) => errors.push(String(e)));
await page.goto(`http://127.0.0.1:${server.address().port}/`);
const report = await page.waitForFunction(() => globalThis.distCheck, null, { timeout: 30000 }).then((h) => h.jsonValue());
console.log(JSON.stringify(report), "errors:", errors.length ? errors : "none");
await browser.close(); server.close();
