// Dwie próbki na oś; brak historii i ekstrapolacji. Kąty wieloobrotowe nie są zawijane.
type Sample = { angle: number; time: number };
export class PreviewPose {
    private samples: Sample[][] = Array.from({ length: 6 }, () => []);
    private shown = Array(6).fill(0) as number[];
    push(index: number, degrees: number, time: number) {
        if (index < 0 || index >= 6 || !Number.isFinite(degrees) || !Number.isFinite(time)) return;
        const samples = this.samples[index];
        if (samples.length && time <= samples[samples.length - 1].time) return;
        samples.push({ angle: degrees * Math.PI / 180, time });
        if (samples.length > 2) samples.shift();
    }
    fresh(index: number, now: number) {
        const sample = this.samples[index].at(-1);
        return !!sample && now >= sample.time && now - sample.time < 250;
    }
    degrees(index: number) {
        const sample = this.samples[index].at(-1);
        return sample ? sample.angle * 180 / Math.PI : null;
    }
    angle(index: number, now: number, valid: boolean) {
        const samples = this.samples[index];
        if (!valid || !this.fresh(index, now)) return this.shown[index];
        const next = samples[samples.length - 1], previous = samples[0], renderTime = now - 50;
        const t = next.time === previous.time ? 1 : Math.max(0, Math.min(1, (renderTime - previous.time) / (next.time - previous.time)));
        this.shown[index] = previous.angle + t * (next.angle - previous.angle);
        return this.shown[index];
    }
}
