import { expect, it } from "vitest";
import { renderToStaticMarkup } from "react-dom/server";
import { ManualPage, MotorPosition } from "./ManualPage";
it("pokazuje sześć niedostępnych osi i blokuje sterowanie bez danych", () => {
    const html = renderToStaticMarkup(<ManualPage />);
    for (let axis = 1; axis <= 6; axis++) {
        expect(html).toContain("CAN " + axis);
        expect(html).toContain("Prędkość J" + axis);
    }
    expect(html.match(/class="jog-arrow"/g)).toHaveLength(12);
    expect(html.match(/class="jog-arrow"[^>]*disabled/g)).toHaveLength(12);
    expect(html).toContain("Ustaw zero"); expect(html).toContain("Wyłącz");
});
it("wskaźnik pozycji pokazuje granice, środek i brak pomiaru", () => {
    for (const [position, percent] of [[-3600, 0], [0, 50], [3600, 100], [7200, 100]]) {
        const html = renderToStaticMarkup(<MotorPosition axis={1} position={position} />);
        expect(html).toContain(`left:${percent}%`);
        expect(html).toContain('role="meter"');
    }
    const empty = renderToStaticMarkup(<MotorPosition axis={3} position={null} />);
    expect(empty).toContain("brak danych");
    expect(empty).not.toContain('class="position-thumb"');
});
