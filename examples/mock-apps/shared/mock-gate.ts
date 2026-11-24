// With `?mock=<what>` in the URL, the simulated hardware (../mock/install.ts) is put in place before the app runs.
// Without it the app talks to real adapters.
if (new URLSearchParams(location.search).has("mock")) await import("../mock/install.ts");
