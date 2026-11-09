import { expect, it } from "vitest";
import { renderToStaticMarkup } from "react-dom/server";
import { AxisPosition, ManualPage, MotorPosition } from "./ManualPage";
import type { Axis } from "./manualTypes";
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
it("pokazuje odczyt wału po utracie kalibracji, bez skali przegubu ani znacznika celu na skali wału", () => {
    const axis = { axis: 1, joint_configured: true, calibration_valid: false, joint_position_known: false,
        position_known: true, position_degrees: -12.5, joint_lower_degrees: -171, joint_upper_degrees: 171 } as Axis;
    const html = renderToStaticMarkup(<AxisPosition axis={axis} usable planning plannedPosition={45} planAvailable />);
    expect(html).toContain("-12.5°");
    expect(html).toContain("wał · brak kalibracji");
    expect(html).toContain('aria-label="Pozycja wału J1 względem limitów"');
    expect(html).toContain('aria-valuemin="-3600"');
    expect(html).toContain('aria-valuemax="3600"');
    expect(html).not.toContain('class="position-planned-thumb"');
    expect(html).toContain("Plan: 45.0°");
    const missing = renderToStaticMarkup(<AxisPosition axis={{...axis, position_known: false}} usable />);
    expect(missing).toContain("brak danych");
    expect(missing).not.toContain("-12.5°");
    const stale = renderToStaticMarkup(<AxisPosition axis={axis} usable={false} />);
    expect(stale).toContain("ostatni pomiar");
    expect(stale).toContain('data-stale="true"');
    const calibrated = renderToStaticMarkup(<AxisPosition axis={{...axis, calibration_valid: true, joint_position_known: true, joint_position_degrees: 25}} usable planning plannedPosition={45} />);
    expect(calibrated).toContain('aria-valuenow="25"');
    expect(calibrated).toContain('aria-valuemin="-171"');
    expect(calibrated).toContain('class="position-planned-thumb"');
    expect(calibrated).not.toContain("brak kalibracji");
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
