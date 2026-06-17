import * as THREE from 'three';

// Basic drum sound synthesizer using Web Audio API
class DrumSynth {
    private ctx: AudioContext | null = null;
    
    // Defer initialization to avoid browser warnings before user interaction
    init() {
        if (!this.ctx) {
            this.ctx = new (window.AudioContext || (window as any).webkitAudioContext)();
        }
    }

    private createPanner(pos: THREE.Vector3) {
        const panner = this.ctx!.createPanner();
        panner.panningModel = 'HRTF';
        panner.positionX.value = pos.x;
        panner.positionY.value = pos.y;
        panner.positionZ.value = pos.z;
        panner.connect(this.ctx!.destination);
        return panner;
    }

    playKick(pos: THREE.Vector3) {
        if (!this.ctx) this.init();
        const t = this.ctx!.currentTime;
        const osc = this.ctx!.createOscillator();
        const gain = this.ctx!.createGain();
        osc.connect(gain);
        gain.connect(this.createPanner(pos));
        osc.frequency.setValueAtTime(150, t);
        osc.frequency.exponentialRampToValueAtTime(0.01, t + 0.5);
        gain.gain.setValueAtTime(1, t);
        gain.gain.exponentialRampToValueAtTime(0.01, t + 0.5);
        osc.start(t);
        osc.stop(t + 0.5);
    }

    playSnare(pos: THREE.Vector3) {
        if (!this.ctx) this.init();
        const t = this.ctx!.currentTime;
        const panner = this.createPanner(pos);
        // Tonal part
        const osc = this.ctx!.createOscillator();
        const oscGain = this.ctx!.createGain();
        osc.type = 'triangle';
        osc.connect(oscGain);
        oscGain.connect(panner);
        osc.frequency.setValueAtTime(250, t);
        oscGain.gain.setValueAtTime(0.5, t);
        oscGain.gain.exponentialRampToValueAtTime(0.01, t + 0.2);
        osc.start(t);
        osc.stop(t + 0.2);
        
        // Noise part
        const bufferSize = this.ctx!.sampleRate * 0.2; // 200ms
        const buffer = this.ctx!.createBuffer(1, bufferSize, this.ctx!.sampleRate);
        const data = buffer.getChannelData(0);
        for (let i = 0; i < bufferSize; i++) {
            data[i] = Math.random() * 2 - 1;
        }
        const noise = this.ctx!.createBufferSource();
        noise.buffer = buffer;
        const noiseFilter = this.ctx!.createBiquadFilter();
        noiseFilter.type = 'highpass';
        noiseFilter.frequency.value = 1000;
        noise.connect(noiseFilter);
        const noiseGain = this.ctx!.createGain();
        noiseFilter.connect(noiseGain);
        noiseGain.connect(panner);
        noiseGain.gain.setValueAtTime(1, t);
        noiseGain.gain.exponentialRampToValueAtTime(0.01, t + 0.2);
        noise.start(t);
    }

    playHiHat(pos: THREE.Vector3) {
        if (!this.ctx) this.init();
        const t = this.ctx!.currentTime;
        const panner = this.createPanner(pos);
        // Noise
        const bufferSize = this.ctx!.sampleRate * 0.1;
        const buffer = this.ctx!.createBuffer(1, bufferSize, this.ctx!.sampleRate);
        const data = buffer.getChannelData(0);
        for (let i = 0; i < bufferSize; i++) {
            data[i] = Math.random() * 2 - 1;
        }
        const noise = this.ctx!.createBufferSource();
        noise.buffer = buffer;
        const bandpass = this.ctx!.createBiquadFilter();
        bandpass.type = 'bandpass';
        bandpass.frequency.value = 10000;
        const highpass = this.ctx!.createBiquadFilter();
        highpass.type = 'highpass';
        highpass.frequency.value = 7000;
        const gain = this.ctx!.createGain();
        noise.connect(bandpass);
        bandpass.connect(highpass);
        highpass.connect(gain);
        gain.connect(panner);
        gain.gain.setValueAtTime(0.5, t);
        gain.gain.exponentialRampToValueAtTime(0.01, t + 0.1);
        noise.start(t);
    }

    playTom(pos: THREE.Vector3) {
        if (!this.ctx) this.init();
        const t = this.ctx!.currentTime;
        const panner = this.createPanner(pos);
        const osc = this.ctx!.createOscillator();
        const gain = this.ctx!.createGain();
        osc.connect(gain);
        gain.connect(panner);
        osc.frequency.setValueAtTime(200, t);
        osc.frequency.exponentialRampToValueAtTime(50, t + 0.3);
        gain.gain.setValueAtTime(0.8, t);
        gain.gain.exponentialRampToValueAtTime(0.01, t + 0.3);
        osc.start(t);
        osc.stop(t + 0.3);
    }
}

interface DrumDef {
    id: string;
    sound: () => void;
    position: THREE.Vector3;
    color: number;
    radius: number;
    active: boolean; // Is currently being hit
    mesh: THREE.Mesh; // hit mesh
    shellMesh: THREE.Object3D; // Visual mesh to animate
    baseRotation: THREE.Euler; // For cymbal sway physics
    sway: THREE.Vector2; // X and Y sway angles
    swayVel: THREE.Vector2; // Sway velocities
    light: THREE.PointLight; // Impact light
}

export class DrumManager {
    private scene: THREE.Scene;
    private synth: DrumSynth;
    private drums: DrumDef[] = [];
    private originalScales = new Map<string, THREE.Vector3>();

    private particleGroup: THREE.Group;
    private particles: { mesh: THREE.Mesh; vel: THREE.Vector3; life: number }[] = [];

    constructor(scene: THREE.Scene) {
        this.scene = scene;
        this.synth = new DrumSynth();
        this.particleGroup = new THREE.Group();
        this.scene.add(this.particleGroup);
        this.initDrums();
    }

    private createWornSkinTexture(radius: number): THREE.CanvasTexture {
        const size = 512;
        const canvas = document.createElement('canvas');
        canvas.width = size;
        canvas.height = size;
        const ctx = canvas.getContext('2d')!;

        // Base color (slightly off-white/beige for a realistic skin)
        ctx.fillStyle = '#f4f0e6';
        ctx.fillRect(0, 0, size, size);

        // Add subtle radial gradient for lighting depth
        const gradient = ctx.createRadialGradient(size/2, size/2, size*0.1, size/2, size/2, size*0.5);
        gradient.addColorStop(0, 'rgba(255, 255, 255, 0.2)');
        gradient.addColorStop(1, 'rgba(0, 0, 0, 0.05)');
        ctx.fillStyle = gradient;
        ctx.fillRect(0, 0, size, size);

        // Add wear marks in the center (stick impacts)
        ctx.save();
        ctx.translate(size/2, size/2);
        
        // Random smudges
        for(let i=0; i<30; i++) {
            const r = Math.random() * size * 0.15;
            const theta = Math.random() * Math.PI * 2;
            const x = Math.cos(theta) * r;
            const y = Math.sin(theta) * r;
            
            ctx.beginPath();
            ctx.arc(x, y, Math.random() * 20 + 10, 0, Math.PI * 2);
            ctx.fillStyle = `rgba(180, 170, 160, ${Math.random() * 0.15 + 0.05})`;
            ctx.fill();
        }
        
        // Central concentrated wear
        ctx.beginPath();
        ctx.arc(0, 0, size * 0.1, 0, Math.PI * 2);
        const centerGradient = ctx.createRadialGradient(0, 0, 0, 0, 0, size*0.15);
        centerGradient.addColorStop(0, 'rgba(150, 140, 130, 0.4)');
        centerGradient.addColorStop(1, 'rgba(200, 190, 180, 0)');
        ctx.fillStyle = centerGradient;
        ctx.fill();
        ctx.restore();

        // Add subtle outer ring wear
        ctx.beginPath();
        ctx.arc(size/2, size/2, size*0.48, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(0,0,0,0.1)';
        ctx.lineWidth = 4;
        ctx.stroke();

        const texture = new THREE.CanvasTexture(canvas);
        texture.anisotropy = 16;
        return texture;
    }

    private createCymbalTexture(): { map: THREE.CanvasTexture, normalMap: THREE.CanvasTexture } {
        const size = 1024;
        const canvas = document.createElement('canvas');
        const normalCanvas = document.createElement('canvas');
        canvas.width = normalCanvas.width = size;
        canvas.height = normalCanvas.height = size;
        const ctx = canvas.getContext('2d')!;
        const nCtx = normalCanvas.getContext('2d')!;

        // Base brass color
        ctx.fillStyle = '#b89942';
        ctx.fillRect(0, 0, size, size);
        
        // Base normal (flat)
        nCtx.fillStyle = '#8080ff';
        nCtx.fillRect(0, 0, size, size);

        const center = size / 2;

        // Draw concentric lathing lines
        for(let r = 10; r < center; r += 2) {
            // Color variation
            ctx.beginPath();
            ctx.arc(center, center, r, 0, Math.PI * 2);
            ctx.strokeStyle = Math.random() > 0.5 ? 'rgba(0,0,0,0.05)' : 'rgba(255,255,255,0.05)';
            ctx.lineWidth = Math.random() * 1.5 + 0.5;
            ctx.stroke();

            // Normal map variation (grooves)
            nCtx.beginPath();
            nCtx.arc(center, center, r, 0, Math.PI * 2);
            // Simulate light/dark banding for normals based on angle
            // A simple approximation of ridges
            const depth = Math.random() * 40;
            nCtx.strokeStyle = `rgb(128, 128, ${255 - depth})`; 
            nCtx.lineWidth = 1.5;
            nCtx.stroke();
        }

        // Add some hammered spots for texture
        for(let i=0; i<100; i++) {
            const r = Math.random() * center * 0.8 + center * 0.1;
            const theta = Math.random() * Math.PI * 2;
            const x = center + Math.cos(theta) * r;
            const y = center + Math.sin(theta) * r;
            const rad = Math.random() * 15 + 5;
            
            ctx.beginPath();
            ctx.arc(x, y, rad, 0, Math.PI * 2);
            const hGrad = ctx.createRadialGradient(x, y, 0, x, y, rad);
            hGrad.addColorStop(0, 'rgba(0,0,0,0.1)');
            hGrad.addColorStop(1, 'rgba(0,0,0,0)');
            ctx.fillStyle = hGrad;
            ctx.fill();
        }

        const map = new THREE.CanvasTexture(canvas);
        const normalMap = new THREE.CanvasTexture(normalCanvas);
        map.anisotropy = 16;
        normalMap.anisotropy = 16;

        return { map, normalMap };
    }

    private initDrums() {
        // Ergonomic classic drum kit layout centered for the robot arms
        // X = Robot's right/left (-X is left, +X is right from robot perspective)
        // Y = Forward (away from robot base)
        // Z = Up
        
        const drumConfigs = [
            // Kick drum: Center, on the table, facing the robot
            // Y=0.2 is slightly forward, Z=0.15 rests on table
            { id: 'kick', p: new THREE.Vector3(0, 0.2, 0.15), shell: 0x111111, skin: 0xffffff, r: 0.15, h: 0.15, tiltX: 0, tiltY: 0, s: (pos: THREE.Vector3) => this.synth.playKick(pos) },
            
            // Snare: Robot's left-center (-X), slightly above table
            { id: 'snare', p: new THREE.Vector3(-0.15, 0.3, 0.25), shell: 0x991111, skin: 0xffffee, r: 0.1, h: 0.08, tiltX: Math.PI/2 - 0.1, tiltY: 0.1, s: (pos: THREE.Vector3) => this.synth.playSnare(pos) },
            
            // Hi-hat: Robot's far left (-X), higher up
            { id: 'hihat', p: new THREE.Vector3(-0.3, 0.2, 0.35), shell: 0x000000, skin: 0xc9b037, r: 0.12, h: 0.005, tiltX: Math.PI/2, tiltY: 0.1, s: (pos: THREE.Vector3) => this.synth.playHiHat(pos) },
            
            // Tom: Robot's right-center (+X), higher
            { id: 'tom', p: new THREE.Vector3(0.15, 0.3, 0.3), shell: 0x113399, skin: 0xffffee, r: 0.09, h: 0.12, tiltX: Math.PI/2 - 0.1, tiltY: -0.1, s: (pos: THREE.Vector3) => this.synth.playTom(pos) }
        ];

        for (const gc of drumConfigs) {
            const group = new THREE.Group();
            group.position.copy(gc.p);

            // Add dynamic light
            const lightColor = gc.id === 'kick' ? 0xff4444 : (gc.id === 'hihat' ? 0xffffaa : 0xaaeeff);
            const light = new THREE.PointLight(lightColor, 0, 0.8);
            group.add(light);

            // Use group as the main hit target
            const hitMesh = new THREE.Mesh(
                new THREE.SphereGeometry(gc.r + (gc.id !== 'hihat' ? 0.02 : 0), 16, 16),
                new THREE.MeshBasicMaterial({ visible: false })
            );
            group.add(hitMesh);

            let shellObject: THREE.Object3D;
            
            if (gc.id === 'hihat') {
                shellObject = new THREE.Group();
                const cymbalTextures = this.createCymbalTexture();
                const bowGeom = new THREE.ConeGeometry(gc.r, 0.02, 32);
                const bowMat = new THREE.MeshStandardMaterial({ 
                    color: gc.skin, 
                    metalness: 0.9, 
                    roughness: 0.3,
                    map: cymbalTextures.map,
                    normalMap: cymbalTextures.normalMap
                });
                const bow = new THREE.Mesh(bowGeom, bowMat);
                
                const bellGeom = new THREE.SphereGeometry(gc.r * 0.2, 16, 16, 0, Math.PI * 2, 0, Math.PI / 2);
                const bell = new THREE.Mesh(bellGeom, bowMat);
                bell.position.y = 0.01;
                
                shellObject.add(bow);
                shellObject.add(bell);
            } else {
                const shellMat = new THREE.MeshPhysicalMaterial({ color: gc.shell, metalness: 0.3, roughness: 0.4, clearcoat: 0.8 });
                const shellGeom = new THREE.CylinderGeometry(gc.r, gc.r, gc.h, 32);
                const shellMesh = new THREE.Mesh(shellGeom, shellMat);
                
                // Add Lugs (tension rod casings)
                const numLugs = gc.id === 'kick' ? 8 : 6;
                const lugGeom = new THREE.CylinderGeometry(0.006, 0.006, gc.h * 0.5, 12);
                const lugMat = new THREE.MeshStandardMaterial({color: 0xdddddd, metalness: 0.9, roughness: 0.2});
                for (let i = 0; i < numLugs; i++) {
                    const angle = (i / numLugs) * Math.PI * 2;
                    const lug = new THREE.Mesh(lugGeom, lugMat);
                    lug.position.set(Math.cos(angle) * (gc.r + 0.001), 0, Math.sin(angle) * (gc.r + 0.001));
                    shellMesh.add(lug);
                }
                shellObject = shellMesh;
            }

            // Apply tilts
            shellObject.rotation.x = gc.tiltX;
            shellObject.rotation.y = gc.tiltY;
            const baseRot = shellObject.rotation.clone();
            
            group.add(shellObject);

            // Drum Skins
            if (gc.id === 'kick') {
                const skinGeom = new THREE.CylinderGeometry(gc.r - 0.005, gc.r - 0.005, gc.h + 0.002, 32);
                const skinTex = this.createWornSkinTexture(gc.r);
                const skinMat = new THREE.MeshStandardMaterial({color: gc.skin, roughness: 0.8, map: skinTex});
                const skinMesh = new THREE.Mesh(skinGeom, skinMat);
                skinMesh.rotation.copy(shellObject.rotation);
                group.add(skinMesh);
            } else if (gc.id !== 'hihat') {
                // Top white skin for vertical drums
                const skinGeom = new THREE.CylinderGeometry(gc.r - 0.002, gc.r - 0.002, gc.h + 0.002, 32);
                const skinTex = this.createWornSkinTexture(gc.r);
                const skinMat = new THREE.MeshStandardMaterial({color: gc.skin, roughness: 0.8, metalness: 0.1, map: skinTex});
                const skinMesh = new THREE.Mesh(skinGeom, skinMat);
                skinMesh.rotation.copy(shellObject.rotation);
                group.add(skinMesh);
            }

            // Stands
            if (gc.id !== 'kick') {
                // Basic vertical pole for stand reaching down to the table (Z=0)
                const standLen = gc.p.z;
                const standGeom = new THREE.CylinderGeometry(0.015, 0.015, standLen, 16);
                const standMat = new THREE.MeshStandardMaterial({color: 0x888888, metalness: 0.8, roughness: 0.3});
                const stand = new THREE.Mesh(standGeom, standMat);
                // Stand is vertical (along Z), Cylinder default is Y.
                stand.rotation.x = Math.PI / 2;
                stand.position.z = -standLen / 2;
                
                // Add a tripod base
                const baseGeom = new THREE.CylinderGeometry(0.05, 0.08, 0.02, 16);
                const base = new THREE.Mesh(baseGeom, standMat);
                base.rotation.x = Math.PI / 2;
                base.position.z = -standLen;
                stand.add(base);

                group.add(stand);
            }

            // Rims
            if (gc.id !== 'hihat') {
                const rimGeom = new THREE.TorusGeometry(gc.r + 0.002, 0.005, 16, 32);
                const rimMat = new THREE.MeshStandardMaterial({color: 0xdddddd, metalness: 0.8, roughness: 0.2});
                
                if (gc.id === 'kick') {
                    // Two rims for kick
                    const rim1 = new THREE.Mesh(rimGeom, rimMat);
                    rim1.rotation.copy(shellObject.rotation);
                    rim1.position.y = gc.h / 2; 
                    group.add(rim1);
                    
                    const rim2 = new THREE.Mesh(rimGeom, rimMat);
                    rim2.rotation.copy(shellObject.rotation);
                    rim2.position.y = -gc.h / 2;
                    group.add(rim2);
                } else {
                    const rim = new THREE.Mesh(rimGeom, rimMat);
                    rim.rotation.copy(shellObject.rotation);
                    // Shift along the local Y axis of the rotated shell
                    const shift = new THREE.Vector3(0, gc.h / 2, 0);
                    shift.applyEuler(shellObject.rotation);
                    rim.position.add(shift);
                    group.add(rim);
                }
            }

            this.scene.add(group);
            this.originalScales.set(gc.id, shellObject.scale.clone());

            this.drums.push({
                id: gc.id,
                sound: () => gc.s(gc.p),
                position: gc.p,
                color: gc.shell,
                radius: gc.r,
                active: false,
                mesh: hitMesh, // We hit test against the main sphere
                shellMesh: shellObject, // For scaling animation
                baseRotation: baseRot,
                sway: new THREE.Vector2(0, 0),
                swayVel: new THREE.Vector2(0, 0),
                light: light
            });
        }
    }

    // Call this each frame loop with the end effector positions
    update(effectors: THREE.Vector3[]) {
        for (const drum of this.drums) {
            let hit = false;
            let hitPos: THREE.Vector3 | null = null;
            
            const drumWorldPos = new THREE.Vector3();
            drum.mesh.getWorldPosition(drumWorldPos);

            // Check distance to any effector
            for (const eff of effectors) {
                // simple sphere collision
                const dist = eff.distanceTo(drumWorldPos);
                // The hit distance is roughly the radius plus some margin
                if (dist < drum.radius + 0.05) { 
                    hit = true;
                    hitPos = eff.clone();
                    break;
                }
            }

            if (hit && !drum.active && hitPos) {
                drum.active = true;
                drum.sound();
                drum.light.intensity = 2.0; // Flash light
                
                // Spawn impact particles (wood chips / sparks)
                const sparkMat = new THREE.MeshBasicMaterial({ color: drum.id === 'hihat' ? 0xffffaa : 0xffcc88 });
                const sparkGeom = new THREE.BoxGeometry(0.006, 0.006, 0.006);
                for(let i=0; i<8; i++) {
                    const mesh = new THREE.Mesh(sparkGeom, sparkMat);
                    mesh.position.copy(hitPos); 
                    // Add slight variance to origin
                    mesh.position.x += (Math.random() - 0.5) * 0.02;
                    mesh.position.y += (Math.random() - 0.5) * 0.02;
                    
                    this.particleGroup.add(mesh);
                    this.particles.push({
                        mesh: mesh,
                        vel: new THREE.Vector3((Math.random()-0.5)*2, (Math.random()-0.5)*2, Math.random()*2).multiplyScalar(0.02),
                        life: 1.0
                    });
                }
                
                // Sway physics impulse for cymbals
                if (drum.id === 'hihat') {
                    drum.swayVel.x += (Math.random() - 0.5) * 1.5;
                    drum.swayVel.y += (Math.random() - 0.5) * 1.5;
                } else {
                    // Visual pop for drums
                    drum.shellMesh.scale.set(1.1, 0.9, 1.1);
                }

                if (drum.shellMesh instanceof THREE.Mesh && drum.shellMesh.material instanceof THREE.MeshPhysicalMaterial) {
                    drum.shellMesh.material.emissive.setHex(0x333333);
                }
            } else if (!hit && drum.active) {
                drum.active = false;
            }

            // Cymbal sway dynamics
            if (drum.id === 'hihat') {
                // Spring force back to center
                drum.swayVel.x += -drum.sway.x * 0.15;
                drum.swayVel.y += -drum.sway.y * 0.15;
                // Damping
                drum.swayVel.multiplyScalar(0.85);
                // Apply velocity
                drum.sway.add(drum.swayVel);
                
                drum.shellMesh.rotation.x = drum.baseRotation.x + drum.sway.x;
                drum.shellMesh.rotation.y = drum.baseRotation.y + drum.sway.y;
            }

            // Light decay
            if (drum.light.intensity > 0.01) {
                drum.light.intensity *= 0.85; // fast decay
            } else {
                drum.light.intensity = 0;
            }

            // Animate scale back to normal
            if (drum.active) {
                drum.shellMesh.scale.lerp(this.originalScales.get(drum.id)!, 0.1);
            } else {
                drum.shellMesh.scale.lerp(this.originalScales.get(drum.id)!, 0.2);
                if (drum.shellMesh instanceof THREE.Mesh && drum.shellMesh.material instanceof THREE.MeshPhysicalMaterial) {
                    drum.shellMesh.material.emissive.setHex(0x000000);
                }
            }
        }

        // Animate independent particles
        for (let i = this.particles.length - 1; i >= 0; i--) {
            const p = this.particles[i];
            p.life -= 0.05;
            p.vel.z -= 0.001; // gravity
            p.mesh.position.add(p.vel);
            p.mesh.rotation.x += p.vel.y;
            p.mesh.rotation.y += p.vel.x;
            p.mesh.scale.setScalar(p.life);
            if (p.life <= 0) {
                this.particleGroup.remove(p.mesh);
                (p.mesh.geometry as THREE.BufferGeometry).dispose();
                (p.mesh.material as THREE.Material).dispose();
                this.particles.splice(i, 1);
            }
        }
    }
}
