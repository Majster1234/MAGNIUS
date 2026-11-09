import * as THREE from "three";
import type { URDFRobot } from "urdf-loader";
import type { LimitIndicator } from "./previewLimits";

// Dwa znaczniki na przegub. Zasoby powstają raz; poświata jest lokalnym
// sprite'em i wycinkiem pierścienia, bez renderowania całej sceny do bloom.
export function createLimitGlows(robot: URDFRobot, host: HTMLElement) {
    const canvas = document.createElement("canvas"); canvas.width = canvas.height = 64;
    const context = canvas.getContext("2d");
    if (!context) throw Error("Brak obsługi poświaty");
    const gradient = context.createRadialGradient(32, 32, 0, 32, 32, 32);
    gradient.addColorStop(0, "rgba(255,255,255,.95)");
    gradient.addColorStop(.2, "rgba(255,255,255,.6)");
    gradient.addColorStop(.55, "rgba(255,255,255,.18)");
    gradient.addColorStop(1, "rgba(255,255,255,0)");
    context.fillStyle = gradient; context.fillRect(0, 0, 64, 64);
    const texture = new THREE.CanvasTexture(canvas);
    const geometry = new THREE.RingGeometry(.04, .085, 32, 1, -Math.PI / 4, Math.PI / 2);
    const z = new THREE.Vector3(0, 0, 1);
    const markers = Array.from({ length: 6 }, (_, i) => {
        const joint = robot.joints[`joint_${i + 1}`];
        // Układ sprzed obrotu tego przegubu: granice pozostają nieruchome
        // względem poprzedniego członu, uwzględniając origin/RPY z URDF.
        const origin = new THREE.Group(); origin.position.copy(joint.position); origin.quaternion.copy(joint.quaternion);
        joint.parent!.add(origin);
        const plane = new THREE.Group(); plane.quaternion.setFromUnitVectors(z, joint.axis.clone().normalize()); origin.add(plane);
        return (["lower", "upper"] as const).map(side => {
            const pivot = new THREE.Group(); pivot.visible = false; plane.add(pivot);
            const arc = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({
                transparent: true, depthTest: false, depthWrite: false, side: THREE.DoubleSide, blending: THREE.AdditiveBlending,
            })); arc.renderOrder = 4; pivot.add(arc);
            const glow = new THREE.Sprite(new THREE.SpriteMaterial({
                map: texture, transparent: true, depthTest: false, depthWrite: false, blending: THREE.AdditiveBlending,
            })); glow.position.x = .06; glow.scale.setScalar(.105); glow.renderOrder = 5; pivot.add(glow);
            const label = document.createElement("span"); label.className = "preview-limit-label";
            label.dataset.axis = String(i + 1); label.dataset.side = side; label.hidden = true; host.append(label);
            return { axis: i + 1, side, pivot, arc, glow, label, key: "", point: new THREE.Vector3() };
        });
    }).flat();
    return {
        update(limits: LimitIndicator[][], now: number) {
            let dirty = false;
            for (const marker of markers) {
                const indicator = limits[marker.axis - 1]?.find(value => value.side === marker.side);
                const key = indicator ? `${indicator.level}/${indicator.critical}/${indicator.fresh}/${indicator.limitDegrees}/${indicator.distanceDegrees.toFixed(1)}` : "";
                if (key !== marker.key) {
                    dirty = true; marker.key = key; marker.pivot.visible = !!indicator;
                    if (!indicator) { marker.label.hidden = true; delete marker.label.dataset.level; }
                    else {
                        const color = indicator.level === "warning" ? 0xffca45 : 0xff354c;
                        marker.arc.material.color.setHex(color); marker.glow.material.color.setHex(color);
                        marker.pivot.rotation.z = THREE.MathUtils.degToRad(indicator.limitDegrees);
                        marker.glow.scale.setScalar(indicator.critical ? .135 : .105);
                        marker.label.dataset.level = indicator.level; marker.label.dataset.fresh = String(indicator.fresh);
                        marker.label.dataset.critical = String(indicator.critical); marker.label.dataset.limitDegrees = String(indicator.limitDegrees);
                        const sign = marker.side === "lower" ? "−" : "+";
                        marker.label.textContent = `J${marker.axis} ${sign} LIMIT${indicator.level === "exceeded" ? "!" : ""}${indicator.fresh ? "" : " · ostatni pomiar"}`;
                        marker.label.title = `J${marker.axis}: limit ${sign} ${indicator.limitDegrees}°; ${indicator.level === "exceeded" ? "przekroczony o" : "do limitu"} ${Math.abs(indicator.distanceDegrees).toFixed(1)}°${indicator.fresh ? "" : " (dane nieaktualne)"}`;
                        marker.label.setAttribute("aria-label", marker.label.title);
                    }
                }
                if (!indicator) continue;
                const pulse = indicator.level === "exceeded" && indicator.fresh ? .55 + .45 * (.5 + .5 * Math.sin(now * Math.PI * 2 / 1000)) : 1;
                const opacity = (indicator.fresh ? 1 : .4) * pulse;
                const glowOpacity = opacity * (indicator.critical ? .95 : .7);
                if (marker.glow.material.opacity !== glowOpacity) dirty = true;
                marker.glow.material.opacity = glowOpacity;
                marker.arc.material.opacity = opacity * (indicator.critical ? .34 : .22);
            }
            return dirty;
        },
        project(camera: THREE.Camera, width: number, height: number) {
            for (const { pivot, label, point } of markers) {
                if (!pivot.visible) continue;
                point.set(.14, 0, 0); pivot.localToWorld(point).project(camera);
                label.style.left = `${(point.x + 1) * width / 2}px`; label.style.top = `${(1 - point.y) * height / 2}px`;
                label.hidden = point.z < -1 || point.z > 1 || Math.abs(point.x) > 1 || Math.abs(point.y) > 1;
            }
        },
        // Geometrie i materiały zwalnia właściciel sceny wraz z robotem.
        dispose() { texture.dispose(); markers.forEach(marker => marker.label.remove()); },
    };
}
