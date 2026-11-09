declare const __BUILD_INFO__: { revision: string; built_at: string } | undefined;

// Vitest nie przechodzi przez define z vite.config, stąd zabezpieczenie typeof.
export const buildInfo = typeof __BUILD_INFO__ === "undefined" ? { revision: "dev", built_at: "" } : __BUILD_INFO__;
export const buildLabel = `${buildInfo.revision}${buildInfo.built_at ? ` · ${new Date(buildInfo.built_at).toLocaleString("pl-PL", { dateStyle: "short", timeStyle: "short" })}` : ""}`;
