// Koder kodów QR (ISO/IEC 18004) bez zależności: tryb bajtowy (UTF-8), korekcja błędów L/M, wersje 1–40,
// automatyczny wybór maski według kar ze standardu. Wynik: macierz modułów (true = ciemny), bez strefy ciszy.
// Używany do kodu QR „teach pendant” w górnym pasku (PendantLink.tsx); sprawdzany dekoderem w testach.
export type QrEcc = "L" | "M";

const ECC_PER_BLOCK: Record<QrEcc, number[]> = {
    L: [-1, 7, 10, 15, 20, 26, 18, 20, 24, 30, 18, 20, 24, 26, 30, 22, 24, 28, 30, 28, 28, 28, 28, 30, 30, 26, 28, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30],
    M: [-1, 10, 16, 26, 18, 24, 16, 18, 22, 22, 26, 30, 22, 22, 24, 24, 28, 28, 26, 26, 26, 26, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28],
};
const BLOCKS: Record<QrEcc, number[]> = {
    L: [-1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 4, 4, 4, 4, 4, 6, 6, 6, 6, 7, 8, 8, 9, 9, 10, 12, 12, 12, 13, 14, 15, 16, 17, 18, 19, 19, 20, 21, 22, 24, 25],
    M: [-1, 1, 1, 1, 2, 2, 4, 4, 4, 5, 5, 5, 8, 9, 9, 10, 10, 11, 13, 14, 16, 17, 17, 18, 20, 21, 23, 25, 26, 28, 29, 31, 33, 35, 37, 38, 40, 43, 45, 47, 49],
};
const FORMAT_BITS: Record<QrEcc, number> = { L: 1, M: 0 };

function rawModules(version: number) {
    let result = (16 * version + 128) * version + 64;
    if (version >= 2) { const align = Math.floor(version / 7) + 2; result -= (25 * align - 10) * align - 55; if (version >= 7) result -= 36; }
    return result;
}
const dataCodewords = (version: number, ecc: QrEcc) => Math.floor(rawModules(version) / 8) - ECC_PER_BLOCK[ecc][version] * BLOCKS[ecc][version];

// Arytmetyka GF(256), wielomian 0x11D.
function gfMul(x: number, y: number) {
    let z = 0;
    for (let i = 7; i >= 0; i--) { z = (z << 1) ^ ((z >>> 7) * 0x11d); z ^= ((y >>> i) & 1) * x; }
    return z & 0xff;
}
function rsDivisor(degree: number) {
    const result = new Array<number>(degree).fill(0); result[degree - 1] = 1;
    let root = 1;
    for (let i = 0; i < degree; i++) {
        for (let j = 0; j < result.length; j++) { result[j] = gfMul(result[j], root); if (j + 1 < result.length) result[j] ^= result[j + 1]; }
        root = gfMul(root, 0x02);
    }
    return result;
}
function rsRemainder(data: number[], divisor: number[]) {
    const result = new Array<number>(divisor.length).fill(0);
    for (const b of data) {
        const factor = b ^ (result.shift() as number); result.push(0);
        divisor.forEach((coef, i) => { result[i] ^= gfMul(coef, factor); });
    }
    return result;
}
function alignmentPositions(version: number) {
    if (version === 1) return [];
    const count = Math.floor(version / 7) + 2, size = version * 4 + 17;
    const step = Math.floor((version * 8 + count * 3 + 5) / (count * 4 - 4)) * 2;
    const result = [6];
    for (let pos = size - 7; result.length < count; pos -= step) result.splice(1, 0, pos);
    return result;
}

// `mask` 0–7 wymusza maskę (testy porównawcze); domyślnie najlepsza wg kar.
export function encodeQr(text: string, ecc: QrEcc = "M", mask?: number): boolean[][] {
    const bytes = [...new TextEncoder().encode(text)];
    let version = 1;
    for (; version <= 40; version++) if (4 + (version < 10 ? 8 : 16) + bytes.length * 8 <= dataCodewords(version, ecc) * 8) break;
    if (version > 40) throw new Error("Tekst za długi dla kodu QR");
    // Strumień bitów: tryb bajtowy 0100, długość, dane, terminator, wyrównanie, bajty wypełnienia.
    const bits: number[] = [], push = (value: number, length: number) => { for (let i = length - 1; i >= 0; i--) bits.push((value >>> i) & 1); };
    push(0b0100, 4); push(bytes.length, version < 10 ? 8 : 16); bytes.forEach(b => push(b, 8));
    const capacity = dataCodewords(version, ecc) * 8;
    push(0, Math.min(4, capacity - bits.length)); push(0, (8 - bits.length % 8) % 8);
    for (let pad = 0xec; bits.length < capacity; pad ^= 0xec ^ 0x11) push(pad, 8);
    const data: number[] = [];
    for (let i = 0; i < bits.length; i += 8) data.push(bits.slice(i, i + 8).reduce((a, b) => (a << 1) | b, 0));
    // Bloki z korekcją i przeplot.
    const blocks = BLOCKS[ecc][version], eccLen = ECC_PER_BLOCK[ecc][version], raw = Math.floor(rawModules(version) / 8);
    const shortBlocks = blocks - raw % blocks, shortLen = Math.floor(raw / blocks), divisor = rsDivisor(eccLen);
    const parts: number[][] = [];
    for (let i = 0, k = 0; i < blocks; i++) {
        const dat = data.slice(k, k + shortLen - eccLen + (i < shortBlocks ? 0 : 1)); k += dat.length;
        const block = [...dat, ...rsRemainder(dat, divisor)];
        if (i < shortBlocks) block.splice(shortLen - eccLen, 0, 0);
        parts.push(block);
    }
    const codewords: number[] = [];
    for (let i = 0; i < parts[0].length; i++) parts.forEach((block, j) => { if (i !== shortLen - eccLen || j >= shortBlocks) codewords.push(block[i]); });
    // Macierz: wzorce funkcyjne, dane, maska.
    const size = version * 4 + 17;
    const modules = Array.from({ length: size }, () => new Array<boolean>(size).fill(false));
    const fixed = Array.from({ length: size }, () => new Array<boolean>(size).fill(false));
    const set = (x: number, y: number, dark: boolean) => { modules[y][x] = dark; fixed[y][x] = true; };
    for (let i = 0; i < size; i++) { set(6, i, i % 2 === 0); set(i, 6, i % 2 === 0); }
    for (const [cx, cy] of [[3, 3], [size - 4, 3], [3, size - 4]]) for (let dy = -4; dy <= 4; dy++) for (let dx = -4; dx <= 4; dx++) {
        const x = cx + dx, y = cy + dy, d = Math.max(Math.abs(dx), Math.abs(dy));
        if (x >= 0 && x < size && y >= 0 && y < size) set(x, y, d !== 2 && d !== 4);
    }
    const align = alignmentPositions(version), last = align.length - 1;
    align.forEach((ax, i) => align.forEach((ay, j) => {
        if ((i === 0 && j === 0) || (i === 0 && j === last) || (i === last && j === 0)) return;
        for (let dy = -2; dy <= 2; dy++) for (let dx = -2; dx <= 2; dx++) set(ax + dx, ay + dy, Math.max(Math.abs(dx), Math.abs(dy)) !== 1);
    }));
    const formatBits = (mask: number) => {
        const value = (FORMAT_BITS[ecc] << 3) | mask; let rem = value;
        for (let i = 0; i < 10; i++) rem = (rem << 1) ^ ((rem >>> 9) * 0x537);
        const word = ((value << 10) | rem) ^ 0x5412, bit = (i: number) => ((word >>> i) & 1) === 1;
        for (let i = 0; i <= 5; i++) set(8, i, bit(i));
        set(8, 7, bit(6)); set(8, 8, bit(7)); set(7, 8, bit(8));
        for (let i = 9; i < 15; i++) set(14 - i, 8, bit(i));
        for (let i = 0; i < 8; i++) set(size - 1 - i, 8, bit(i));
        for (let i = 8; i < 15; i++) set(8, size - 15 + i, bit(i));
        set(8, size - 8, true);
    };
    formatBits(0);
    if (version >= 7) {
        let rem = version; for (let i = 0; i < 12; i++) rem = (rem << 1) ^ ((rem >>> 11) * 0x1f25);
        const word = (version << 12) | rem;
        for (let i = 0; i < 18; i++) { const bit = ((word >>> i) & 1) === 1, a = size - 11 + i % 3, b = Math.floor(i / 3); set(a, b, bit); set(b, a, bit); }
    }
    let index = 0;
    for (let right = size - 1; right >= 1; right -= 2) {
        if (right === 6) right = 5;
        for (let vert = 0; vert < size; vert++) for (let j = 0; j < 2; j++) {
            const x = right - j, upward = ((right + 1) & 2) === 0, y = upward ? size - 1 - vert : vert;
            if (!fixed[y][x] && index < codewords.length * 8) { modules[y][x] = ((codewords[index >>> 3] >>> (7 - (index & 7))) & 1) === 1; index++; }
        }
    }
    const masked = (mask: number) => modules.map((row, y) => row.map((dark, x) => {
        if (fixed[y][x]) return dark;
        const flip = [(x + y) % 2 === 0, y % 2 === 0, x % 3 === 0, (x + y) % 3 === 0, (Math.floor(x / 3) + Math.floor(y / 2)) % 2 === 0,
            (x * y) % 2 + (x * y) % 3 === 0, ((x * y) % 2 + (x * y) % 3) % 2 === 0, ((x + y) % 2 + (x * y) % 3) % 2 === 0][mask];
        return dark !== flip;
    }));
    let best: boolean[][] | null = null, bestScore = Infinity;
    for (let k = 0; k < 8; k++) {
        if (mask !== undefined && k !== mask) continue;
        formatBits(k);
        const candidate = masked(k), score = mask !== undefined ? 0 : penalty(candidate);
        if (score < bestScore) { best = candidate; bestScore = score; }
    }
    return best!;
}

// Kary ze standardu: serie ≥ 5, bloki 2×2, wzorce podobne do wyszukiwacza, proporcja ciemnych modułów.
function penalty(m: boolean[][]) {
    const size = m.length; let score = 0, dark = 0;
    const lines = [...m, ...m[0].map((_, x) => m.map(row => row[x]))];
    const finder = [true, false, true, true, true, false, true];
    for (const line of lines) {
        let run = 1;
        for (let i = 1; i <= size; i++) {
            if (i < size && line[i] === line[i - 1]) run++;
            else { if (run >= 5) score += 3 + run - 5; run = 1; }
        }
        for (let i = 0; i + 7 <= size; i++) {
            if (!finder.every((v, k) => line[i + k] === v)) continue;
            const before = i >= 4 && [1, 2, 3, 4].every(k => !line[i - k]), after = i + 11 <= size && [7, 8, 9, 10].every(k => !line[i + k]);
            if (before || after) score += 40;
        }
    }
    for (let y = 0; y + 1 < size; y++) for (let x = 0; x + 1 < size; x++) {
        const c = m[y][x]; if (c === m[y][x + 1] && c === m[y + 1][x] && c === m[y + 1][x + 1]) score += 3;
    }
    m.forEach(row => row.forEach(v => { if (v) dark++; }));
    return score + Math.floor(Math.abs(dark * 20 - size * size * 10) / (size * size)) * 10;
}

// Ścieżka SVG modułów (strefa ciszy `margin` modułów) — do <path d=…> w viewBox 0 0 n n.
export function qrSvgPath(modules: boolean[][], margin = 4) {
    let d = "";
    modules.forEach((row, y) => row.forEach((dark, x) => { if (dark) d += `M${x + margin} ${y + margin}h1v1h-1z`; }));
    return { d, size: modules.length + 2 * margin };
}
