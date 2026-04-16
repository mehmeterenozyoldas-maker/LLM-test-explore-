
/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
*/


import * as THREE from 'three';
import { AlohaIK } from './AlohaIK';
import { DragStateManager } from './DragStateManager';
import { IkSystem } from './IkSystem';
import { RenderSystem } from './RenderSystem';
import { RobotLoader } from './RobotLoader';
import { SelectionManager } from './SelectionManager';
import { SequenceAnimator } from './SequenceAnimator';
import { MujocoData, MujocoModel, MujocoModule } from './types';
import { getName } from './utils/StringUtils';

/**
 * MujocoSim: The Central Orchestrator.
 * Manages the connection between the MuJoCo WASM engine and the Three.js visualization.
 */
export class MujocoSim {
    mujoco: MujocoModule;      
    mjModel: MujocoModel | null = null;     
    mjData: MujocoData | null = null;      
    mjvOption: InstanceType<MujocoModule['MjvOption']>;   

    renderSys: RenderSystem;
    ikSys: IkSystem;
    alohaIk: AlohaIK;
    dragStateManager: DragStateManager;
    selectionManager: SelectionManager;
    sequenceAnimator: SequenceAnimator;

    frameId: number | null = null; 
    paused = false;
    gripperActuatorId = -1;
    speedMultiplier = 1;
    
    private userIkEnabled = false; 
    private firstIkEnable = true; // Track first enable to enforce default rotation
    private isFranka = false;
    private isAloha = false;

    // Gizmo Interpolation State
    private gizmoAnim = {
        active: false,
        startPos: new THREE.Vector3(),
        endPos: new THREE.Vector3(),
        startRot: new THREE.Quaternion(),
        endRot: new THREE.Quaternion(),
        startTime: 0,
        duration: 1000
    };

    constructor(container: HTMLElement, mujocoInstance: MujocoModule) {
        this.mujoco = mujocoInstance;
        this.mjvOption = new this.mujoco.MjvOption();
        
        this.renderSys = new RenderSystem(container, this.mujoco);
        
        this.dragStateManager = new DragStateManager(this.renderSys.scene, this.renderSys.renderer, this.renderSys.camera, container, this.renderSys.controls);
        this.selectionManager = new SelectionManager(this.renderSys.scene, this.renderSys.renderer, this.renderSys.camera, container);
        
        this.ikSys = new IkSystem(this.mujoco, this.renderSys.camera, this.renderSys.renderer.domElement, this.renderSys.controls);
        this.renderSys.simGroup.add(this.ikSys.target); 
        this.renderSys.scene.add(this.ikSys.control as unknown as THREE.Object3D);
        
        this.alohaIk = new AlohaIK(this.mujoco, this.renderSys);

        this.sequenceAnimator = new SequenceAnimator();
        
        this.renderSys.initLights(this.dragStateManager);
    }

    async init(robotId: string, onProgress?: (msg: string) => void) {
        const sceneFile = 'scene.xml';
        const loader = new RobotLoader(this.mujoco);
        await loader.load(robotId, sceneFile, onProgress);

        try {
            this.mjModel = this.mujoco.MjModel.loadFromXML(`/working/${sceneFile}`);
            this.mjData = new this.mujoco.MjData(this.mjModel);
        } catch (e: unknown) { 
            throw new Error(`Failed to load model: ${(e as Error).message}`); 
        }

        if (this.mjModel) {
            this.isFranka = robotId === 'franka_emika_panda';
            this.isAloha = robotId === 'aloha';

            // Reset IK stuff
            this.ikSys.gripperSiteId = -1; 
            this.gripperActuatorId = -1;
            
            // Only setup specialized IK for Franka
            if (this.isFranka) {
                for (let i = 0; i < this.mjModel.nsite; i++) {
                     if (getName(this.mjModel, this.mjModel.name_siteadr[i]).includes('tcp')) { 
                         this.ikSys.gripperSiteId = i; break; 
                     }
                }
                for (let i = 0; i < this.mjModel.nu; i++) {
                     if (getName(this.mjModel, this.mjModel.name_actuatoradr[i]).includes('gripper')) { 
                         this.gripperActuatorId = i; break; 
                     }
                }
                this.setInitialPose();
                this.ikSys.init(this.mjModel, false);
                this.sequenceAnimator.init(this.mjModel, false, (addr) => getName(this.mjModel!, addr));
            } else {
                // Disable IK gizmo for other robots initially
                this.ikSys.setGizmoVisible(false);
                this.ikSys.setTargetVisible(false);
            }
            
            // Setup Aloha IK
            if (this.isAloha) {
                this.alohaIk.setup(this.mjModel, this.mjData!);
            }

            this.mujoco.mj_forward(this.mjModel, this.mjData!);
            this.renderSys.initScene(this.mjModel);
            
            if (this.isFranka) {
                this.ikSys.syncToSite(this.mjData!);
                this.ikSys.target.quaternion.setFromEuler(new THREE.Euler(Math.PI, 0, 0));
                this.ikSys.target.position.set(0, 0, 0.45);
            }
            
            if (this.isAloha) {
                this.alohaIk.syncToSim(this.mjModel, this.mjData!);
            }

            this.firstIkEnable = true;
            this.startLoop();
        }
    }
    
    private setInitialPose() {
        if (!this.mjModel || !this.mjData || !this.isFranka) return;
        const initVals = [1.707, -1.754, 0.003, -2.702, 0.003, 0.951, 2.490, 0.000];
        
        for (let i = 0; i < Math.min(initVals.length, this.mjModel.nu); i++) {
            this.mjData.ctrl[i] = initVals[i];
            if (this.mjModel.actuator_trnid[2 * i + 1] === 1) {
                const jointId = this.mjModel.actuator_trnid[2 * i];
                if (jointId >= 0 && jointId < this.mjModel.njnt) {
                    const qposAdr = this.mjModel.jnt_qposadr[jointId];
                    this.mjData.qpos[qposAdr] = initVals[i];
                }
            }
        }
    }

    private startLoop() {
        if (this.frameId) cancelAnimationFrame(this.frameId);

        const loop = () => {
            if (!this.mjModel || !this.mjData) {
                 this.frameId = requestAnimationFrame(loop);
                 return;
            }

            this.dragStateManager.update();
            if (this.draggedBodyId() !== null) this.mjData.xfrc_applied.fill(0);
            if (this.dragStateManager.active && this.dragStateManager.physicsObject) {
                this.applyDragForce();
            }
            
            // Gizmo animation only if IK active and Franka
            if (this.gizmoAnim.active && this.isFranka) {
                const now = performance.now();
                const elapsed = now - this.gizmoAnim.startTime;
                const t = Math.min(elapsed / this.gizmoAnim.duration, 1.0);
                const ease = 1 - Math.pow(1 - t, 3);
                
                this.ikSys.target.position.lerpVectors(this.gizmoAnim.startPos, this.gizmoAnim.endPos, ease);
                this.ikSys.target.quaternion.slerpQuaternions(this.gizmoAnim.startRot, this.gizmoAnim.endRot, ease);
                
                if (t >= 1.0) {
                    this.gizmoAnim.active = false;
                }
            }

            if (!this.paused) {
                if (this.isFranka && this.sequenceAnimator.running) {
                    this.sequenceAnimator.update((1/60) * this.speedMultiplier, this.ikSys.target, this.mjData, this.gripperActuatorId, this.ikSys);
                    this.setIkEnabled(false);
                } else if (this.isFranka) {
                     this.syncIkState();
                     this.ikSys.update(this.mjModel, this.mjData);
                } else if (this.isAloha && this.userIkEnabled) {
                     this.alohaIk.update(this.mjModel, this.mjData);
                }

                const startSimTime = this.mjData.time;
                while (this.mjData.time - startSimTime < (1.0 / 60.0) * this.speedMultiplier) {
                    this.mujoco.mj_step(this.mjModel, this.mjData);
                }
            }

            this.renderSys.update(this.mjData, this.renderSys.contactMarkers.visible);
            this.frameId = requestAnimationFrame(loop);
        };
        this.frameId = requestAnimationFrame(loop);
    }

    private draggedBodyId(): number | null { 
        return this.dragStateManager.active && this.dragStateManager.physicsObject ? this.dragStateManager.physicsObject.userData.bodyID : null; 
    }

    private applyDragForce() {
        if (!this.mjData) return;
        const bodyId = this.draggedBodyId()!;
        const force = new THREE.Vector3().subVectors(this.dragStateManager.currentWorld, this.dragStateManager.worldHit).multiplyScalar(1.5);
        if (force.lengthSq() > 25) force.setLength(5.0); 
        
        const bodyPos = new THREE.Vector3().fromArray(this.mjData.xpos, bodyId * 3);
        const leverArm = new THREE.Vector3().subVectors(this.dragStateManager.worldHit, bodyPos);
        const torque = leverArm.cross(force);

        this.mjData.xfrc_applied.set([force.x, force.y, force.z, torque.x, torque.y, torque.z], bodyId * 6);
    }
    
    private syncIkState() {
        if (this.isFranka) {
            const shouldCalculate = this.userIkEnabled;
            const shouldShowGizmo = this.userIkEnabled && !this.gizmoAnim.active && !this.sequenceAnimator.running; 
            
            this.ikSys.setCalculating(shouldCalculate);
            this.ikSys.setGizmoVisible(shouldShowGizmo);
            
            if (this.sequenceAnimator.running) {
                this.ikSys.setTargetVisible(true);
            } else if(shouldCalculate) {
                this.ikSys.setTargetVisible(true);
            }
        } else if (this.isAloha) {
            this.alohaIk.setEnabled(this.userIkEnabled);
        }
    }

    moveIkTargetTo(pos: THREE.Vector3, duration = 0) {
        if (this.isFranka) {
            if (!this.userIkEnabled) {
                this.setIkEnabled(true);
            }
            
            const targetPos = new THREE.Vector3(pos.x, pos.y, pos.z + 0.05);
            const targetRot = new THREE.Quaternion().setFromEuler(new THREE.Euler(Math.PI, 0, 0));
    
            if (duration > 0) {
                this.gizmoAnim.active = true;
                this.gizmoAnim.startPos.copy(this.ikSys.target.position);
                this.gizmoAnim.endPos.copy(targetPos);
                this.gizmoAnim.startRot.copy(this.ikSys.target.quaternion);
                this.gizmoAnim.endRot.copy(targetRot);
                this.gizmoAnim.startTime = performance.now();
                this.gizmoAnim.duration = duration;
            } else {
                this.gizmoAnim.active = false;
                this.ikSys.target.position.copy(targetPos);
                this.ikSys.target.quaternion.copy(targetRot);
            }
        }
        // TODO: Implement for Aloha if needed (requires choosing which arm)
    }

    reset() {
        if (!this.mjModel || !this.mjData) return;
        this.renderSys.clearErMarkers();
        this.gizmoAnim.active = false;
        
        if (this.isFranka) {
            this.sequenceAnimator.reset(); 
            this.setInitialPose();
        }

        this.mujoco.mj_resetData(this.mjModel, this.mjData);
        if (this.isFranka) this.setInitialPose();
        
        this.mujoco.mj_forward(this.mjModel, this.mjData); 
        
        if (this.isFranka) {
            this.ikSys.syncToSite(this.mjData);
            this.ikSys.target.quaternion.setFromEuler(new THREE.Euler(Math.PI, 0, 0));
            this.ikSys.target.position.set(0, 0, 0.45);
            this.firstIkEnable = true;
        } else if (this.isAloha) {
            this.alohaIk.syncToSim(this.mjModel, this.mjData);
        }
    }
    
    togglePause() { return this.paused = !this.paused; }
    
    setIkEnabled(enabled: boolean) {
        if (!this.isFranka && !this.isAloha) return;
        this.userIkEnabled = enabled;
        this.syncIkState();
        if (this.mjData && enabled) {
             if (this.isFranka && !this.gizmoAnim.active && !this.sequenceAnimator.running) {
                if (this.firstIkEnable) {
                    this.ikSys.target.quaternion.setFromEuler(new THREE.Euler(Math.PI, 0, 0));
                    this.ikSys.target.position.set(0, 0, 0.45);
                    this.firstIkEnable = false;
                } else {
                    this.ikSys.syncToSite(this.mjData);
                }
            } else if (this.isAloha) {
                this.alohaIk.syncToSim(this.mjModel!, this.mjData);
            }
        }
    }
    
    getGizmoStats() { 
        if (this.isFranka && this.ikSys.calculating && this.ikSys.target) {
            return { pos: this.ikSys.target.position.clone(), rot: new THREE.Euler().setFromQuaternion(this.ikSys.target.quaternion) };
        } 
        // For Aloha, just return first active arm stat if available
        if (this.isAloha && this.alohaIk.ikEnabled && this.alohaIk.arms.length > 0) {
            const t = this.alohaIk.arms[0].target;
             return { pos: t.position.clone(), rot: new THREE.Euler().setFromQuaternion(t.quaternion) };
        }
        return null; 
    }
    
    dispose() {
        if (this.frameId) cancelAnimationFrame(this.frameId);
        this.dragStateManager.dispose(); 
        this.selectionManager.dispose(); 
        this.renderSys.dispose(); 
        this.ikSys.dispose();
        this.alohaIk.dispose();
        if (this.mjvOption) this.mjvOption.delete(); 
        if (this.mjModel) this.mjModel.delete(); 
        if (this.mjData) this.mjData.delete();
        try { this.mujoco.FS.unmount('/working'); } catch (e) { /* ignore */ }
    }
}
