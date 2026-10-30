import { execSync } from "node:child_process";
import { defineConfig } from "vite";

// Wersja panelu dla strony Architektura i monitora pracy: commit (+dirty) i czas buildu.
function revision() {
    try {
        const hash = execSync("git rev-parse --short HEAD", { encoding: "utf8" }).trim();
        const dirty = execSync("git status --porcelain", { encoding: "utf8" }).trim() !== "";
        return dirty ? `${hash}+dirty` : hash;
    } catch {
        return "nieznana";
    }
}

export default defineConfig({
    define: {
        __BUILD_INFO__: JSON.stringify({ revision: revision(), built_at: new Date().toISOString() }),
    },
});
