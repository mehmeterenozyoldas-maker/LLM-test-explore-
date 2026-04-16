
/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
*/

import * as THREE from 'three';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import * as MatMath from './MatMath';
import { RenderSystem } from './RenderSystem';
import { MujocoData, MujocoModel, MujocoModule } from './types';

interface ArmIK {
    name: string;
    siteId: number;
    target: THREE.Group;
    control: TransformControls;
    jointIndices: number[];
    qposIndices: number[];
    actuatorIndices: number[];
}

export class AlohaIK {
    private mujoco: MujocoModule;
    private renderSys: RenderSystem;

    ikEnabled: boolean = false;
    gizmoMode: string = 'translate';
    arms: ArmIK[] = [];
    
    // Reusable temporary variables for IK
    private _p0 = new THREE.Vector3();
    private _p1 = new THREE.Vector3();
    private _q0 = new THREE.Quaternion();
    private _q1 = new THREE.Quaternion();
    private _qDiff = new THREE.Quaternion();
    private _tempMat4 = new THREE.Matrix4();
    private _currentPosMJ = new THREE.Vector3();
    private _currentQuatMJ = new THREE.Quaternion();
    private _goalQuatMJ = new THREE.Quaternion();
    private _errRotQuat = new THREE.Quaternion();

    ikDamping = 0.2;
    ikStepSize = 0.8;
    ikPosWeight = 1.0;
    ikRotWeight = 0.3;
    ikIterations = 3;

    constructor(mujoco: MujocoModule, renderSys: RenderSystem) {
        this.mujoco = mujoco;
        this.renderSys = renderSys;
    }

    setup(model: MujocoModel, data: MujocoData) {
        const armNames = ['right', 'left'];
        this.arms = [];
        
        // Ensure mjtObj is available (sometimes bindings differ slightly, assume standard or provided)
        const SITE_TYPE = (this.mujoco.mjtObj as any).mjOBJ_SITE?.value ?? 6; // Fallback to known value if enum missing
        const JOINT_TYPE = (this.mujoco.mjtObj as any).mjOBJ_JOINT?.value ?? 3;
        const ACTUATOR_TYPE = (this.mujoco.mjtObj as any).mjOBJ_ACTUATOR?.value ?? 10;

        armNames.forEach(name => {
            const siteName = `${name}/gripper`;
            const siteId = this.mujoco.mj_name2id(model, SITE_TYPE, siteName);
            if (siteId < 0) {
                console.warn(`AlohaIK: Site ${siteName} not found.`);
                return;
            }

            const target = new THREE.Group();
            const targetVis = new THREE.Mesh(
                new THREE.SphereGeometry(0.025, 16, 16),
                new THREE.MeshBasicMaterial({color: name === 'right' ? 0x00ff00 : 0xff3333, transparent: true, opacity: 0.5})
            );
            target.add(targetVis);
            target.add(new THREE.AxesHelper(0.08));
            this.renderSys.simGroup.add(target);
            target.visible = false;

            const control = new TransformControls(this.renderSys.camera, this.renderSys.renderer.domElement);
            control.addEventListener('dragging-changed', (event: any) => {
                this.renderSys.controls.enabled = !event.value;
            });
            control.attach(target);
            // @ts-ignore
            control.visible = false;
            // @ts-ignore
            control.enabled = false;
            control.setSpace("local");
            control.setMode(this.gizmoMode as any);
            this.renderSys.scene.add(control as any);

            const jointNames = [`${name}/waist`, `${name}/shoulder`, `${name}/elbow`, `${name}/forearm_roll`, `${name}/wrist_angle`, `${name}/wrist_rotate`];
            const jointIndices: number[] = [];
            const qposIndices: number[] = [];
            const actuatorIndices: number[] = [];

            jointNames.forEach(jName => {
                 const jntId = this.mujoco.mj_name2id(model, JOINT_TYPE, jName);
                 if (jntId >= 0) {
                      jointIndices.push(model.jnt_dofadr[jntId]);
                      qposIndices.push(model.jnt_qposadr[jntId]);
                      const actId = this.mujoco.mj_name2id(model, ACTUATOR_TYPE, jName);
                      if (actId >= 0) {
                          actuatorIndices.push(actId);
                      }
                 } else {
                     console.warn(`AlohaIK: Joint ${jName} not found`);
                 }
            });

            this.arms.push({ name, siteId, target, control, jointIndices, qposIndices, actuatorIndices });
        });

        // Sync initial positions
        this.syncToSim(model, data);
    }
    
    // Synchronize gizmos to current simulation state
    syncToSim(model: MujocoModel, data: MujocoData) {
        if (!this.arms.length) return;
        
        for (const arm of this.arms) {
            // Get site position
            const off = arm.siteId * 3;
            arm.target.position.set(data.site_xpos[off], data.site_xpos[off+1], data.site_xpos[off+2]);
            
            // Get site rotation
            this.getSiteQuaternionMJ(data, arm.siteId, arm.target.quaternion);
            arm.target.updateMatrixWorld();
            
            // Sync controls
            for (let i = 0; i < arm.actuatorIndices.length; i++) {
                const ctrlIdx = arm.actuatorIndices[i];
                const qposIdx = arm.qposIndices[i];
                data.ctrl[ctrlIdx] = data.qpos[qposIdx];
            }
        }
    }

    setEnabled(enabled: boolean) {
        this.ikEnabled = enabled;
        for(const arm of this.arms) {
            arm.target.visible = enabled;
            // @ts-ignore
            arm.control.visible = enabled;
            // @ts-ignore
            arm.control.enabled = enabled;
        }
    }

    private getSiteQuaternionMJ(simData: MujocoData, siteId: number, targetQuat: THREE.Quaternion) {
        const off = siteId * 9;
        const m = simData.site_xmat;
        this._tempMat4.set(
            m[off+0], m[off+1], m[off+2], 0,
            m[off+3], m[off+4], m[off+5], 0,
            m[off+6], m[off+7], m[off+8], 0,
            0, 0, 0, 1
        );
        targetQuat.setFromRotationMatrix(this._tempMat4);
    }

    private calculateFiniteDifferenceJacobian6D(sim: MujocoData, model: MujocoModel, siteId: number, nDof: number, qposIndices: number[]) {
        const epsilon = 1e-4;
        const jac = new Float64Array(6 * nDof);
        const siteOffset = siteId * 3;

        this.mujoco.mj_forward(model, sim);
        this._p0.fromArray(sim.site_xpos, siteOffset);
        this.getSiteQuaternionMJ(sim, siteId, this._q0);

        for (let i = 0; i < nDof; i++) {
            const qAddr = qposIndices[i];
            const originalVal = sim.qpos[qAddr];

            sim.qpos[qAddr] = originalVal + epsilon;
            this.mujoco.mj_forward(model, sim);
            this._p1.fromArray(sim.site_xpos, siteOffset);
            this.getSiteQuaternionMJ(sim, siteId, this._q1);

            jac[i]          = (this._p1.x - this._p0.x) / epsilon;
            jac[i + nDof]   = (this._p1.y - this._p0.y) / epsilon;
            jac[i + 2*nDof] = (this._p1.z - this._p0.z) / epsilon;

            this._qDiff.copy(this._q1).multiply(this._q0.clone().invert());
            
            // Ensure shortest path for rotation difference
            if (this._qDiff.w < 0) {
              this._qDiff.x = -this._qDiff.x;
              this._qDiff.y = -this._qDiff.y;
              this._qDiff.z = -this._qDiff.z;
              this._qDiff.w = -this._qDiff.w;
            }

            let halfTheta = Math.acos(Math.min(Math.max(this._qDiff.w, -1), 1));
            let sinHalfTheta = Math.sin(halfTheta);
            if (sinHalfTheta > 1e-9) {
                let factor = (2 * halfTheta) / sinHalfTheta / epsilon;
                jac[i + 3*nDof] = this._qDiff.x * factor;
                jac[i + 4*nDof] = this._qDiff.y * factor;
                jac[i + 5*nDof] = this._qDiff.z * factor;
            } else {
                jac[i + 3*nDof] = 0; jac[i + 4*nDof] = 0; jac[i + 5*nDof] = 0;
            }
            sim.qpos[qAddr] = originalVal;
        }
        // Ensure state is restored
        this.mujoco.mj_forward(model, sim);
        return jac;
    }

    update(model: MujocoModel, data: MujocoData) {
        if (!this.ikEnabled || !data || !model || this.arms.length === 0) return;
        
        for (const arm of this.arms) {
            // Target in world space
            const targetPosWorld = new THREE.Vector3();
            arm.target.getWorldPosition(targetPosWorld);
            const targetQuatWorld = new THREE.Quaternion();
            arm.target.getWorldQuaternion(targetQuatWorld);
            
            // Convert to MuJoCo local space (if simGroup is transformed, though usually it's identity)
            const targetPosLocal = this.renderSys.simGroup.worldToLocal(targetPosWorld.clone());
            // For rotation, simGroup inverse world quaternion * target world quaternion
            const simGroupQuat = new THREE.Quaternion();
            this.renderSys.simGroup.getWorldQuaternion(simGroupQuat);
            const targetQuatLocal = simGroupQuat.invert().multiply(targetQuatWorld);
            
            this._goalQuatMJ.copy(targetQuatLocal);
            
            const nDof = arm.jointIndices.length;
            const siteOffset = arm.siteId * 3;

            for(let iter = 0; iter < this.ikIterations; iter++) {
                this.mujoco.mj_forward(model, data);
                this._currentPosMJ.fromArray(data.site_xpos, siteOffset);
                this.getSiteQuaternionMJ(data, arm.siteId, this._currentQuatMJ);

                const error = new Float64Array(6);
                error[0] = this.ikPosWeight * (targetPosLocal.x - this._currentPosMJ.x);
                error[1] = this.ikPosWeight * (targetPosLocal.y - this._currentPosMJ.y);
                error[2] = this.ikPosWeight * (targetPosLocal.z - this._currentPosMJ.z);

                this._errRotQuat.copy(this._goalQuatMJ).multiply(this._currentQuatMJ.clone().invert());
                
                // Ensure shortest path for rotation error
                if (this._errRotQuat.w < 0) {
                    this._errRotQuat.x = -this._errRotQuat.x;
                    this._errRotQuat.y = -this._errRotQuat.y;
                    this._errRotQuat.z = -this._errRotQuat.z;
                    this._errRotQuat.w = -this._errRotQuat.w;
                }

                let halfTheta = Math.acos(Math.min(Math.max(this._errRotQuat.w, -1), 1));
                let sinHalfTheta = Math.sin(halfTheta);
                if (sinHalfTheta > 1e-9) {
                    let factor = (2 * halfTheta) / sinHalfTheta * this.ikRotWeight;
                    error[3] = this._errRotQuat.x * factor;
                    error[4] = this._errRotQuat.y * factor;
                    error[5] = this._errRotQuat.z * factor;
                }

                // Check for convergence
                if ((error[0]**2 + error[1]**2 + error[2]**2) < 1e-7 && (error[3]**2+error[4]**2+error[5]**2) < 1e-5) break;

                const J_flat = this.calculateFiniteDifferenceJacobian6D(data, model, arm.siteId, nDof, arm.qposIndices);
                for(let c=0; c<nDof; c++) {
                    J_flat[c] *= this.ikPosWeight;        J_flat[c+nDof] *= this.ikPosWeight;   J_flat[c+2*nDof] *= this.ikPosWeight;
                    J_flat[c+3*nDof] *= this.ikRotWeight; J_flat[c+4*nDof] *= this.ikRotWeight; J_flat[c+5*nDof] *= this.ikRotWeight;
                }

                const Jt_flat = MatMath.matTranspose(J_flat, 6, nDof);
                const JtJ_flat = MatMath.matMul(Jt_flat, J_flat, nDof, 6, nDof);
                for (let i = 0; i < nDof; i++) JtJ_flat[i * nDof + i] += this.ikDamping;
                const b_vec = MatMath.matVecMul(Jt_flat, error, nDof, 6);

                try {
                   const delta_q = MatMath.solveLinearSystem(JtJ_flat, b_vec, nDof);
                   for (let i = 0; i < nDof; i++) {
                        const qAddr = arm.qposIndices[i];
                        const dofAdr = arm.jointIndices[i];
                        // Get joint id from dof address
                        const jntId = model.dof_jntid[dofAdr];
                        data.qpos[qAddr] += this.ikStepSize * delta_q[i];
                        
                        // Clamp to limits
                        const min = model.jnt_range[jntId * 2];
                        const max = model.jnt_range[jntId * 2 + 1];
                        data.qpos[qAddr] = Math.max(min, Math.min(max, data.qpos[qAddr]));
                   }
                } catch(e) {
                    console.warn("IK solver failed:", e);
                    break; 
                }
            }
            
            // After IK loop finishes, match actuators to new positions to hold state
            for (let i = 0; i < arm.actuatorIndices.length; i++) {
                const ctrlIdx = arm.actuatorIndices[i];
                const qposIdx = arm.qposIndices[i];
                data.ctrl[ctrlIdx] = data.qpos[qposIdx];
            }
        }
    }

    dispose() {
        for(const arm of this.arms) {
            arm.control.detach();
            arm.control.dispose();
            this.renderSys.scene.remove(arm.control as any);
            this.renderSys.simGroup.remove(arm.target);
        }
        this.arms = [];
    }
}
