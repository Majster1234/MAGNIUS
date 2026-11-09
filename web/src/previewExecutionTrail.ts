import * as THREE from "three";
import type { URDFRobot } from "urdf-loader";
import type { ManualState } from "./manualTypes";
import type { MotionObservation } from "./motionStatus";

export const EXECUTION_TRAIL_CAPACITY = 20000;
const activeStates = new Set(["arming", "warming", "staging", "running", "waiting_position", "waiting_feedback", "waiting_budget", "settling"]);
export type ExecutionFrame = {
    state: ManualState | null; usable: boolean; receivedAt: number; sampleKey: string;
    execution: MotionObservation | null; revision?: string;
};

// Pierścień punktów TCP: każde miejsce przechowuje punkt i dochodzący do niego odcinek. Przerwa
// ma dwa identyczne wierzchołki, więc nigdy nie łączy oddzielnych ruchów.
function createTrailBuffer(scene: THREE.Scene, robot: URDFRobot, capacity: number, color: number, name: string) {
    if (!Number.isInteger(capacity) || capacity < 2) throw Error("Nieprawidłowa pojemność śladu");
    const positions = new Float32Array(capacity * 3), vertices = new Float32Array(capacity * 6);
    const edges = new Uint8Array(capacity), partial = new Uint8Array(capacity);
    const attribute = new THREE.BufferAttribute(vertices, 3).setUsage(THREE.DynamicDrawUsage);
    const geometry = new THREE.BufferGeometry(); geometry.setAttribute("position", attribute); geometry.setDrawRange(0, 0);
    const material = new THREE.LineBasicMaterial({ color, transparent: true, opacity: .95, depthWrite: false, depthTest: false });
    const line = new THREE.LineSegments(geometry, material); line.name = name;
    line.frustumCulled = false; line.visible = false; line.renderOrder = 4; scene.add(line);
    const reference = robot.clone(true) as URDFRobot, position = new THREE.Vector3(), lastPosition = new THREE.Vector3();
    // FK pomiaru nie może ukrywać rzeczywistego przekroczenia limitu URDF.
    Object.values(reference.joints).forEach(joint => { joint.ignoreLimits = true; });
    let head = 0, size = 0, edgeCount = 0, partialCount = 0, dropped = 0, last = -1, open = false, shown = true;
    function collapse(slot: number) {
        vertices.set(positions.subarray(slot * 3, slot * 3 + 3), slot * 6);
        vertices.set(positions.subarray(slot * 3, slot * 3 + 3), slot * 6 + 3);
    }
    const buffer = {
        reference, position, lastPosition, get open() { return open; }, get hasLast() { return last >= 0; },
        close() { open = false; },
        // Pozycja TCP pomiaru (kąty w radianach) albo null przy niepełnej/nieskończonej FK.
        tcp(angles: number[]) {
            angles.forEach((angle, i) => reference.setJointValue(`joint_${i + 1}`, angle));
            reference.updateMatrixWorld(true); reference.joints.tool_fixed.getWorldPosition(position);
            return [position.x, position.y, position.z].every(Number.isFinite);
        },
        append(incomplete: boolean) {
            const slot = head, next = (slot + 1) % capacity;
            if (size === capacity) {
                partialCount -= partial[slot]; edgeCount -= edges[slot]; dropped++;
                if (edges[next]) { edges[next] = 0; edgeCount--; collapse(next); }
            } else size++;
            position.toArray(positions, slot * 3);
            if (open && last >= 0) {
                vertices.set(positions.subarray(last * 3, last * 3 + 3), slot * 6);
                position.toArray(vertices, slot * 6 + 3); edges[slot] = 1; edgeCount++;
            } else { edges[slot] = 0; collapse(slot); }
            partial[slot] = incomplete ? 1 : 0; partialCount += partial[slot];
            last = slot; head = next; open = true; lastPosition.copy(position);
            attribute.needsUpdate = true; geometry.setDrawRange(0, size * 2); line.visible = shown && edgeCount > 0;
        },
        show(value: boolean) { if (shown === value) return false; shown = value; line.visible = shown && edgeCount > 0; return true; },
        clear() {
            head = size = edgeCount = partialCount = dropped = 0; last = -1; open = false;
            edges.fill(0); partial.fill(0); geometry.setDrawRange(0, 0); line.visible = false;
        },
        snapshot() {
            return { points: size, segments: size - edgeCount, edges: edgeCount, capacity, partial: partialCount > 0,
                dropped, visible: line.visible, color: `#${material.color.getHexString()}`,
                lastPoint: last < 0 ? null : Array.from(positions.subarray(last * 3, last * 3 + 3)) };
        },
    };
    return buffer;
}

export function createExecutionTrail(scene: THREE.Scene, robot: URDFRobot, capacity = EXECUTION_TRAIL_CAPACITY) {
    const buffer = createTrailBuffer(scene, robot, capacity, 0xffc857, "execution-trail"), { reference, position, lastPosition } = buffer;
    let run = "", mask = "", stamp = "", lastSampleAt = -Infinity, lastGoodAt = -Infinity;
    const close = () => buffer.close();
    return {
        update(frame: ExecutionFrame | undefined, now: number) {
            const observation = frame?.execution, status = observation?.status, state = frame?.state;
            if (!frame || !state || !frame.usable || !state.fresh || state.fault || !observation || !status?.busy ||
                !activeStates.has(status.state) || !status.program_id || !status.execution_key ||
                now < observation.receivedAt || now - observation.receivedAt >= 750 ||
                now < frame.receivedAt || now - frame.receivedAt >= 250 || state.active_axis !== 0 ||
                state.operation === "jog" || state.operation === "stopping" ||
                (frame.revision !== undefined && frame.revision !== state.configuration_revision) ||
                status.participating.length !== 6 || !status.participating.some(Boolean) ||
                status.start_positions.length !== 6 || !status.start_positions.every(Number.isFinite) ||
                state.axes.length !== 6 || !reference.joints.tool_fixed) { close(); return false; }
            const nextRun = `${status.program_id}/${status.execution_key}`;
            if (run !== nextRun) {
                run = nextRun; close(); stamp = ""; mask = ""; lastSampleAt = lastGoodAt = -Infinity;
            }
            // Jedna próbka jest renderowana wielokrotnie. Jej starzenie się między
            // klatkami nie jest nowym pomiarem ani powodem do rozcinania historii.
            // Rzeczywistą przerwę odbioru wykrywają limity czasu ramki i lastGoodAt.
            if (stamp === frame.sampleKey) return false;
            const age = (now - frame.receivedAt) / 1000;
            const known = state.axes.map(axis => axis.available && axis.position_known && axis.joint_configured &&
                axis.calibration_valid && axis.joint_position_known && Number.isFinite(axis.joint_position_degrees) &&
                typeof axis.position_age_seconds === "number" && Number.isFinite(axis.position_age_seconds) &&
                axis.position_age_seconds >= 0 && axis.position_age_seconds + age < .25);
            if (status.participating.some((yes, i) => yes && !known[i])) { close(); return false; }
            if (now - lastSampleAt < 50) return false;
            const nextMask = known.map(Boolean).join("/");
            if (mask !== nextMask || now - lastGoodAt >= 250) close();
            mask = nextMask;
            stamp = frame.sampleKey; lastSampleAt = lastGoodAt = now;
            const angles = known.map((yes, i) => yes ? state.axes[i].joint_position_degrees! * Math.PI / 180 : status.start_positions[i]);
            if (!buffer.tcp(angles)) { close(); return false; }
            if (buffer.open && buffer.hasLast && position.distanceToSquared(lastPosition) < 1e-20) return false;
            buffer.append(known.some(value => !value)); return true;
        },
        // Zachowujemy stamp: po czyszczeniu czekamy na nową próbkę pomiarową.
        clear() { buffer.clear(); },
        snapshot: buffer.snapshot,
        // Materiał i geometrię zwalnia właściciel sceny.
    };
}

export const LIVE_TRAIL_CAPACITY = 20000;
// Szum enkoderów w postoju przesuwa TCP o mikrometry: punkt dopiero po 0,2 mm, żeby postój nie wypełniał historii.
export const LIVE_TRAIL_MIN_STEP_M = 2e-4;
export type LiveFrame = { state: ManualState | null; usable: boolean; receivedAt: number; sampleKey: string; revision?: string };

// Trace TCP rzeczywistego modelu z pomiarów enkoderów, niezależnie od rodzaju ruchu (JOG osi, JOG TCP, PTP/LIN).
export function createLiveTrail(scene: THREE.Scene, robot: URDFRobot, capacity = LIVE_TRAIL_CAPACITY) {
    const buffer = createTrailBuffer(scene, robot, capacity, 0x3ad6ff, "live-trail"), { position, lastPosition } = buffer;
    let stamp = "", lastSampleAt = -Infinity, lastGoodAt = -Infinity;
    return {
        update(frame: LiveFrame | undefined, enabled: boolean, now: number) {
            const toggled = buffer.show(enabled);
            const state = frame?.state;
            if (!enabled || !frame || !state || !frame.usable || !state.fresh || now < frame.receivedAt || now - frame.receivedAt >= 250 ||
                (frame.revision !== undefined && frame.revision !== state.configuration_revision) ||
                state.axes.length !== 6 || !buffer.reference.joints.tool_fixed) { buffer.close(); return toggled; }
            if (stamp === frame.sampleKey || now - lastSampleAt < 50) return toggled;
            const age = (now - frame.receivedAt) / 1000;
            // Tylko pełny zestaw sześciu świeżych pozycji; brak którejkolwiek rozcina ścieżkę.
            const complete = state.axes.every(axis => axis.available && axis.position_known && axis.joint_configured &&
                axis.calibration_valid && axis.joint_position_known && Number.isFinite(axis.joint_position_degrees) &&
                typeof axis.position_age_seconds === "number" && Number.isFinite(axis.position_age_seconds) &&
                axis.position_age_seconds >= 0 && axis.position_age_seconds + age < .25);
            if (!complete) { buffer.close(); return toggled; }
            if (now - lastGoodAt >= 250) buffer.close();
            stamp = frame.sampleKey; lastSampleAt = lastGoodAt = now;
            if (!buffer.tcp(state.axes.map(axis => axis.joint_position_degrees! * Math.PI / 180))) { buffer.close(); return toggled; }
            if (buffer.open && buffer.hasLast && position.distanceTo(lastPosition) < LIVE_TRAIL_MIN_STEP_M) return toggled;
            buffer.append(false); return true;
        },
        clear() { buffer.clear(); },
        snapshot: buffer.snapshot,
    };
}
