import { expect, it } from "vitest";
import { PreviewPose } from "./previewPose";
it("zachowuje znaki i wielokrotne obroty bez ekstrapolacji", () => {
    const pose = new PreviewPose();
    pose.push(0, -450, 100); pose.push(0, 720, 150);
    expect(pose.angle(0, 150, true)).toBeCloseTo(-2.5 * Math.PI);
    expect(pose.angle(0, 175, true)).toBeCloseTo(.75 * Math.PI);
    expect(pose.angle(0, 200, true)).toBeCloseTo(4 * Math.PI);
    expect(pose.angle(0, 300, true)).toBeCloseTo(4 * Math.PI);
    pose.push(0, 0, 140); expect(pose.degrees(0)).toBeCloseTo(720);
});
it("nie wymyśla pozycji i zamraża podgląd po utracie danych", () => {
    const pose = new PreviewPose(); expect(pose.degrees(2)).toBeNull();
    expect(pose.angle(2, 100, false)).toBe(0); expect(pose.fresh(2, 100)).toBe(false);
    pose.push(1, 90, 100); expect(pose.angle(1, 120, true)).toBeCloseTo(Math.PI / 2);
    pose.push(1, 180, 150); expect(pose.angle(1, 175, true)).toBeCloseTo(3 * Math.PI / 4);
    expect(pose.angle(1, 190, false)).toBeCloseTo(3 * Math.PI / 4);
    expect(pose.fresh(1, 400)).toBe(false);
    expect(pose.angle(1, 400, true)).toBeCloseTo(3 * Math.PI / 4);
});
