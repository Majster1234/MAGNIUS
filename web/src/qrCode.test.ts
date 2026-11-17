import { describe, expect, it } from "vitest";
import { encodeQr, qrSvgPath } from "./qrCode";

// Format (15 bitów) z kopii przy lewym górnym wyszukiwaczu; poprawny, gdy reszta BCH(15,5) = 0 po zdjęciu maski 0x5412.
function formatWord(m: boolean[][]) {
    const bits: boolean[] = [];
    for (let i = 0; i <= 5; i++) bits.push(m[i][8]);
    bits.push(m[7][8], m[8][8], m[8][7]);
    for (let i = 9; i < 15; i++) bits.push(m[8][14 - i]);
    return bits.reduce((word, bit, i) => word | (bit ? 1 << i : 0), 0) ^ 0x5412;
}
const bchOk = (word: number) => { let rem = word; for (let i = 14; i >= 10; i--) if (rem & (1 << i)) rem ^= 0x537 << (i - 10); return rem === 0; };

describe("koder QR", () => {
    it("adres pendanta: wersja 3, wyszukiwacze, linie synchronizacji, poprawny format, deterministyczny", () => {
        const url = "http://192.168.1.69:8080/?fs=1#pendant", m = encodeQr(url, "M");
        expect(m.length).toBe(29);
        for (const [x0, y0] of [[0, 0], [22, 0], [0, 22]]) for (let d = 0; d < 7; d++) {
            expect(m[y0][x0 + d]).toBe(true); expect(m[y0 + 6][x0 + d]).toBe(true); expect(m[y0 + 2][x0 + 2 + (d % 3)]).toBe(true);
        }
        for (let i = 8; i < 21; i++) { expect(m[6][i]).toBe(i % 2 === 0); expect(m[i][6]).toBe(i % 2 === 0); }
        const word = formatWord(m); expect(bchOk(word)).toBe(true); expect(word >> 13).toBe(0);   // poziom M = 00
        expect(m[29 - 8][8]).toBe(true);   // stały ciemny moduł
        expect(encodeQr(url, "M")).toEqual(m);
    });
    it("dobiera wersję do długości i poziomu; wymuszona maska trafia do formatu", () => {
        expect(encodeQr("A", "L").length).toBe(21);
        expect(encodeQr("x".repeat(150), "L").length).toBe(45);   // wersja 7 (informacja o wersji)
        expect(encodeQr("x".repeat(150), "M").length).toBe(49);
        for (let mask = 0; mask < 8; mask++) expect((formatWord(encodeQr("abc", "M", mask)) >> 10) & 7).toBe(mask);
        expect(() => encodeQr("x".repeat(3000), "M")).toThrow();
        const path = qrSvgPath(encodeQr("A", "L")); expect(path.size).toBe(29); expect(path.d.startsWith("M4 4h1v1h-1z")).toBe(true);
    });
});
