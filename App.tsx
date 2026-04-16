
/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
*/

import { FilesetResolver, HandLandmarker } from '@mediapipe/tasks-vision';
import { AlertCircle, Camera, Loader2, Video } from 'lucide-react';
import loadMujoco from 'mujoco_wasm';
import { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { MujocoSim } from './MujocoSim';
import { RobotSelector } from './components/RobotSelector';
import { Toolbar } from './components/Toolbar';
import { AxisMode, ControlMap, MujocoModule } from './types';
import { getName } from './utils/StringUtils';

/**
 * Main Application Component
 * Specialized for Franka Panda Embodied Reasoning and Aloha Hand Control
 */
export function App() {
  const containerRef = useRef<HTMLDivElement>(null); 
  const simRef = useRef<MujocoSim | null>(null);      
  const isMounted = useRef(true);                     
  const mujocoModuleRef = useRef<MujocoModule | null>(null);          

  const [isLoading, setIsLoading] = useState(true);
  const [loadingStatus, setLoadingStatus] = useState("Initializing Engine...");
  const [loadError, setLoadError] = useState<string | null>(null);
  const [mujocoReady, setMujocoReady] = useState(false); 
  
  const [isPaused, setIsPaused] = useState(false);
  const [isDarkMode, setIsDarkMode] = useState(false);
  const [isIkEnabled, setIsIkEnabled] = useState(false);
  
  // Vision / Hand Tracking
  const videoRef = useRef<HTMLVideoElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const handLandmarkerRef = useRef<HandLandmarker | null>(null);
  const lastVideoTimeRef = useRef(-1);
  const [isVisionEnabled, setIsVisionEnabled] = useState(false);
  const [visionLoading, setVisionLoading] = useState(false);
  const gripperIndices = useRef<{left: number, right: number}>({ left: -1, right: -1 });
  const [handCount, setHandCount] = useState(0);
  
  // Controls Map
  const [controlMap, setControlMap] = useState<ControlMap>({
    left: { red: 'Left/Right', green: 'Close/Far', blue: 'Up/Down' },
    right: { red: 'Left/Right', green: 'Close/Far', blue: 'Up/Down' }
  });
  // Ref for the loop to access latest controls without re-triggering effects
  const controlMapRef = useRef(controlMap);

  // Fixed to Aloha as requested
  const selectedRobotId: string = 'aloha';
  const robotName = 'Aloha';
  const supportsIk = selectedRobotId === 'franka_emika_panda' || selectedRobotId === 'aloha';

  // Gizmo stats
  const [leftArmStats, setLeftArmStats] = useState<{pos: string, rot: string} | null>(null);
  const [rightArmStats, setRightArmStats] = useState<{pos: string, rot: string} | null>(null);

  const handleControlChange = (arm: 'left'|'right', axis: 'red'|'green'|'blue', mode: AxisMode) => {
      const newMap = { ...controlMap, [arm]: { ...controlMap[arm], [axis]: mode } };
      setControlMap(newMap);
      controlMapRef.current = newMap;
  };

  // Load MediaPipe
  useEffect(() => {
    const initVision = async () => {
        try {
            const vision = await FilesetResolver.forVisionTasks("https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.14/wasm");
            if (isMounted.current) {
                handLandmarkerRef.current = await HandLandmarker.createFromOptions(vision, {
                    baseOptions: {
                        modelAssetPath: `https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task`,
                        delegate: "GPU"
                    },
                    runningMode: "VIDEO",
                    numHands: 2
                });
            }
        } catch (err) {
            console.error("Failed to load MediaPipe:", err);
        }
    };
    initVision();
  }, []);

  useEffect(() => {
    isMounted.current = true;
    loadMujoco({
      locateFile: (path: string) => path.endsWith('.wasm') ? "https://unpkg.com/mujoco-js@0.0.7/dist/mujoco_wasm.wasm" : path,
      printErr: (text: string) => { 
        if (text.includes("Aborted") && isMounted.current) {
            setLoadError(prev => prev ? prev : "Simulation crashed. Reload page."); 
        }
      }
    }).then((inst: unknown) => { 
      if (isMounted.current) { 
        mujocoModuleRef.current = inst as MujocoModule; 
        setMujocoReady(true); 
        setIsLoading(false);
      } 
    }).catch((err: Error) => { 
      if (isMounted.current) { 
        setLoadError(err.message || "Failed to init spatial simulation"); 
        setIsLoading(false); 
      } 
    });
    return () => { 
        isMounted.current = false; 
        simRef.current?.dispose(); 
        if (videoRef.current?.srcObject) {
            (videoRef.current.srcObject as MediaStream).getTracks().forEach(t => t.stop());
        }
    };
  }, []);

  // Effect to load simulation automatically since robot is fixed
  useEffect(() => {
      if (!mujocoReady || !containerRef.current || !mujocoModuleRef.current) return;
      
      setIsLoading(true); 
      setLoadError(null); 
      setIsPaused(false);
      setLoadingStatus(`Loading ${robotName}...`);
      
      simRef.current?.dispose();
      
      try {
          simRef.current = new MujocoSim(containerRef.current, mujocoModuleRef.current);
          simRef.current.renderSys.setDarkMode(isDarkMode);
          
          simRef.current.init(selectedRobotId, (msg) => {
             if (isMounted.current) setLoadingStatus(msg);
          })
             .then(() => {
                 if (isMounted.current) {
                     setIsLoading(false);
                     // Enable IK by default only if supported
                     if (supportsIk) {
                        setIsIkEnabled(true);
                        simRef.current?.setIkEnabled(true);
                     } else {
                        setIsIkEnabled(false);
                     }

                     // Find Gripper Indices for Aloha
                     if (selectedRobotId === 'aloha' && simRef.current?.mjModel) {
                        const m = simRef.current.mjModel;
                        for(let i=0; i<m.nu; i++) {
                            const name = getName(m, m.name_actuatoradr[i]);
                            if (name.includes('left') && name.includes('gripper')) gripperIndices.current.left = i;
                            if (name.includes('right') && name.includes('gripper')) gripperIndices.current.right = i;
                        }
                     }
                 }
             })
             .catch(err => { 
                 if (isMounted.current) { 
                     console.error(err);
                     setLoadError(`Failed to load robot model. Please check connection.`); 
                     setIsLoading(false); 
                 } 
             });
             
      } catch (err: unknown) { 
          if (isMounted.current) { setLoadError((err as Error).message); setIsLoading(false); } 
      }
  }, [mujocoReady]);

  const startCamera = async () => {
    if (isVisionEnabled || visionLoading) return;
    setVisionLoading(true);
    try {
        const stream = await navigator.mediaDevices.getUserMedia({ 
            video: { width: 640, height: 480, facingMode: 'user' } 
        });
        
        if (videoRef.current) {
            videoRef.current.srcObject = stream;
            videoRef.current.onloadeddata = () => {
                videoRef.current?.play();
                setIsVisionEnabled(true);
                setVisionLoading(false);
                if (!isIkEnabled && supportsIk) {
                    toggleIk();
                }
            };
        } else {
             stream.getTracks().forEach(t => t.stop());
             setVisionLoading(false);
        }
    } catch (e) {
        console.error("Camera failed", e);
        setVisionLoading(false);
    }
  };

  const stopCamera = () => {
      setIsVisionEnabled(false);
      if (videoRef.current?.srcObject) {
          (videoRef.current.srcObject as MediaStream).getTracks().forEach(t => t.stop());
          videoRef.current.srcObject = null;
      }
  };

  const toggleVision = () => {
      if (isVisionEnabled) stopCamera();
      else startCamera();
  };

  // Auto-start camera when ready
  useEffect(() => {
      if (!isLoading && !loadError && !isVisionEnabled && !visionLoading) {
          startCamera();
      }
  }, [isLoading, loadError]);

  // UI & Vision Loop
  useEffect(() => {
      if (isLoading) return;
      let animId: number;
      
      const formatStats = (vec: THREE.Vector3, quat: THREE.Quaternion) => {
          const euler = new THREE.Euler().setFromQuaternion(quat);
          return {
              pos: `X: ${vec.x.toFixed(2)}, Y: ${vec.y.toFixed(2)}, Z: ${vec.z.toFixed(2)}`,
              rot: `X: ${euler.x.toFixed(2)}, Y: ${euler.y.toFixed(2)}, Z: ${euler.z.toFixed(2)}`
          };
      };

      const computeAxisValue = (axis: 'red'|'green'|'blue', mode: AxisMode, hand: 'left'|'right', wrist: {x:number, y:number}, size: number) => {
        if (mode === 'Static') {
            if (axis === 'green') return 0; // Center table
            if (axis === 'blue') return 0.25; // Just above table
            if (axis === 'red') return hand === 'left' ? 0.25 : -0.25; // 1/4th and 3/4th positions
            return 0;
        }

        // --- Logic Implementation ---
        
        // RED AXIS (Robot X: Side-to-side)
        if (axis === 'red') {
             if (mode === 'Left/Right') {
                 if (hand === 'left') {
                     return 1.8 - (wrist.x * 2.4);
                 } else {
                     return 0.6 - (wrist.x * 2.4);
                 }
             } else if (mode === 'Up/Down') {
                 return 0.5 - wrist.y;
             } else if (mode === 'Close/Far') {
                 return (size - 0.15) * 4;
             }
        }
        
        // GREEN AXIS (Robot Y: Depth)
        if (axis === 'green') {
            if (mode === 'Close/Far') {
                // Center (0) when size is 2x current base (0.232)
                return 3.0 * (size - 0.232);
            } else if (mode === 'Up/Down') {
                return -0.3 + wrist.y * 0.6;
            } else if (mode === 'Left/Right') {
                return -0.3 + wrist.x * 0.6;
            }
        }

        // BLUE AXIS (Robot Z: Height)
        if (axis === 'blue') {
            if (mode === 'Up/Down') {
                // Standard orientation for both arms: Hand Up (Low Y) -> Robot Up (High Z)
                // Top (0) -> High (0.65), Bottom (1) -> Low (0.02)
                return Math.max(0.02, 0.65 - (wrist.y * 0.65));
            } else if (mode === 'Close/Far') {
                return 0.6 - (size * 2);
            } else if (mode === 'Left/Right') {
                return 0.1 + wrist.x * 0.5;
            }
        }
        
        return 0;
      };

      const loop = () => {
          // Vision Processing
          if (isVisionEnabled && videoRef.current && handLandmarkerRef.current && simRef.current && canvasRef.current) {
              if (videoRef.current.currentTime !== lastVideoTimeRef.current) {
                  lastVideoTimeRef.current = videoRef.current.currentTime;
                  const results = handLandmarkerRef.current.detectForVideo(videoRef.current, performance.now());
                  
                  // Draw Landmarks
                  const ctx = canvasRef.current.getContext('2d');
                  if (ctx) {
                      ctx.clearRect(0, 0, canvasRef.current.width, canvasRef.current.height);
                      
                      // Match canvas size to video if needed
                      if (canvasRef.current.width !== videoRef.current.videoWidth) {
                          canvasRef.current.width = videoRef.current.videoWidth;
                          canvasRef.current.height = videoRef.current.videoHeight;
                      }

                      if (results.landmarks) {
                          setHandCount(results.landmarks.length);
                          
                          for (const landmarks of results.landmarks) {
                              // Draw connections (simple lines)
                              ctx.strokeStyle = '#00FF00';
                              ctx.lineWidth = 2;
                              ctx.beginPath();
                              
                              const drawChain = (indices: number[]) => {
                                  ctx.moveTo(landmarks[indices[0]].x * ctx.canvas.width, landmarks[indices[0]].y * ctx.canvas.height);
                                  for(let i=1; i<indices.length; i++) {
                                      ctx.lineTo(landmarks[indices[i]].x * ctx.canvas.width, landmarks[indices[i]].y * ctx.canvas.height);
                                  }
                              };
                              
                              drawChain([0, 1, 2, 3, 4]); // Thumb
                              drawChain([0, 5, 6, 7, 8]); // Index
                              drawChain([0, 9, 10, 11, 12]); // Middle
                              drawChain([0, 13, 14, 15, 16]); // Ring
                              drawChain([0, 17, 18, 19, 20]); // Pinky
                              drawChain([0, 5, 9, 13, 17, 0]); // Palm
                              ctx.stroke();

                              ctx.fillStyle = '#FF0000';
                              for(const pt of landmarks) {
                                  ctx.beginPath();
                                  ctx.arc(pt.x * ctx.canvas.width, pt.y * ctx.canvas.height, 3, 0, 2 * Math.PI);
                                  ctx.fill();
                              }
                          }
                      } else {
                          setHandCount(0);
                      }
                  }

                  if (results.landmarks) {
                      for (let i = 0; i < results.landmarks.length; i++) {
                          const landmarks = results.landmarks[i];
                          const handedness = results.handedness[i][0]; 
                          
                          const targetArmName = handedness.categoryName.toLowerCase() as 'left' | 'right'; 
                          const arm = simRef.current.alohaIk.arms.find(a => a.name === targetArmName);
                          
                          if (arm) {
                              const wrist = landmarks[0];
                              const indexTip = landmarks[8];
                              const thumbTip = landmarks[4];
                              const middleMCP = landmarks[9];

                              // Hand Size calculation
                              const sizeX = wrist.x - middleMCP.x;
                              const sizeY = wrist.y - middleMCP.y;
                              const handSize = Math.sqrt(sizeX*sizeX + sizeY*sizeY);

                              const map = controlMapRef.current[targetArmName];

                              const robotX = computeAxisValue('red', map.red, targetArmName, wrist, handSize);
                              const robotY = computeAxisValue('green', map.green, targetArmName, wrist, handSize);
                              const robotZ = computeAxisValue('blue', map.blue, targetArmName, wrist, handSize);

                              // Apply
                              arm.target.position.set(robotX, robotY, Math.max(0.02, robotZ));
                              
                              // Orientation
                              // Left: Fix upside down (0,0,0)
                              // Right: Fix upside down (0,0,0) + Rotate 180 deg around Y (Green axis) -> (0, PI, 0)
                              //        PLUS Rotate 180 deg around X (Red) -> (PI, PI, 0)
                              const euler = targetArmName === 'left' 
                                  ? new THREE.Euler(0, 0, 0) 
                                  : new THREE.Euler(Math.PI, Math.PI, 0);

                              arm.target.quaternion.setFromEuler(euler);
                              
                              // Gripper
                              const dx = indexTip.x - thumbTip.x;
                              const dy = indexTip.y - thumbTip.y;
                              const dz = indexTip.z - thumbTip.z;
                              const dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
                              const gripperIdx = targetArmName === 'left' ? gripperIndices.current.left : gripperIndices.current.right;
                              if (gripperIdx >= 0 && simRef.current.mjData) {
                                  const isPinched = dist < 0.05;
                                  simRef.current.mjData.ctrl[gripperIdx] = isPinched ? 0.0 : 0.08;
                              }
                          }
                      }
                  }
              }
          }

          // UI Stats
          if (simRef.current && simRef.current.isAloha) {
              const leftArm = simRef.current.alohaIk.arms.find(a => a.name === 'left');
              const rightArm = simRef.current.alohaIk.arms.find(a => a.name === 'right');
              
              if (leftArm) setLeftArmStats(formatStats(leftArm.target.position, leftArm.target.quaternion));
              if (rightArm) setRightArmStats(formatStats(rightArm.target.position, rightArm.target.quaternion));
          } else {
             setLeftArmStats(null);
             setRightArmStats(null);
          }
          
          animId = requestAnimationFrame(loop);
      };
      loop();
      return () => cancelAnimationFrame(animId);
  }, [isLoading, isVisionEnabled]);

  const toggleDarkMode = () => {
    const next = !isDarkMode;
    setIsDarkMode(next);
    simRef.current?.renderSys.setDarkMode(next);
  };

  const toggleIk = () => {
      if (!supportsIk) return;
      const next = !isIkEnabled;
      setIsIkEnabled(next);
      simRef.current?.setIkEnabled(next);
  };

  const handleReset = () => {
    simRef.current?.reset();
    if (isIkEnabled && supportsIk) {
      simRef.current?.setIkEnabled(true);
    }
  };

  // Simulation View
  return (
    <div className={`w-full h-full relative overflow-hidden font-sans transition-colors duration-500 ${isDarkMode ? 'bg-slate-950 text-slate-100' : 'bg-slate-50 text-slate-800'}`}>
      {/* 3D Container */}
      <div ref={containerRef} className="w-full h-full absolute inset-0 bg-slate-200" />
      
      {/* Robot Info Overlay */}
      {!loadError && (
          <RobotSelector 
            robotName={robotName} 
            leftStats={leftArmStats}
            rightStats={rightArmStats}
            handCount={handCount}
            isDarkMode={isDarkMode} 
            isWebcamOn={isVisionEnabled}
            controlMap={controlMap}
            onControlChange={handleControlChange}
          />
      )}

      {/* Camera View Overlay */}
      {(isVisionEnabled || visionLoading) && (
          <div className={`absolute top-6 right-6 z-20 overflow-hidden rounded-2xl shadow-2xl transition-all border-4 ${isDarkMode ? 'border-slate-800' : 'border-white'} ${visionLoading ? 'opacity-50' : 'opacity-100'}`}>
             <div className="relative w-48 h-36 bg-black">
                 <video 
                    ref={videoRef} 
                    className="absolute inset-0 w-full h-full object-cover -scale-x-100" 
                    muted 
                    playsInline
                 />
                 <canvas 
                    ref={canvasRef}
                    className="absolute inset-0 w-full h-full object-cover -scale-x-100"
                 />
             </div>
             <div className="absolute bottom-2 right-2 flex items-center gap-1 bg-black/50 px-2 py-1 rounded-md pointer-events-none">
                <div className="w-2 h-2 rounded-full bg-red-500 animate-pulse" />
                <span className="text-[10px] text-white font-bold uppercase tracking-wider">Live</span>
             </div>
          </div>
      )}

      {/* Vision Toggle Button */}
      {!isLoading && !loadError && (
        <div className="absolute top-6 right-6 z-20 flex gap-2">
            {!isVisionEnabled && !visionLoading && (
                <button 
                    onClick={startCamera}
                    disabled={visionLoading}
                    className={`flex items-center gap-2 px-4 py-2 rounded-xl font-bold text-sm shadow-lg transition-all ${isDarkMode ? 'bg-slate-800 text-white hover:bg-slate-700' : 'bg-white text-slate-800 hover:bg-slate-50'}`}
                >
                    <Camera className="w-4 h-4" /> Start Camera
                </button>
            )}
            {visionLoading && (
                <button 
                    disabled
                    className={`flex items-center gap-2 px-4 py-2 rounded-xl font-bold text-sm shadow-lg transition-all ${isDarkMode ? 'bg-slate-800 text-white' : 'bg-white text-slate-800'}`}
                >
                    <Loader2 className="w-4 h-4 animate-spin" /> Starting...
                </button>
            )}
            {isVisionEnabled && (
                <button 
                    onClick={stopCamera}
                    className="flex items-center gap-2 px-4 py-2 rounded-xl font-bold text-sm shadow-lg bg-red-500 text-white hover:bg-red-600 transition-all"
                >
                    <Video className="w-4 h-4" /> Stop
                </button>
            )}
        </div>
      )}
      
      {/* Loading Screen */}
      {isLoading && (
          <div className={`absolute inset-0 flex flex-col items-center justify-center z-50 backdrop-blur-md px-6 ${isDarkMode ? 'bg-slate-950/40' : 'bg-slate-50/20'}`}>
              <div className={`glass-panel p-10 rounded-[3rem] flex flex-col items-center justify-center shrink-0 min-w-[300px] shadow-2xl transition-colors ${isDarkMode ? 'bg-slate-900/70 border-white/10' : 'bg-white/70 border-white/80'}`}>
                  <div className="w-16 h-16 rounded-2xl bg-indigo-600 flex items-center justify-center shadow-lg shadow-indigo-100/20 animate-pulse-soft mb-6">
                    <Loader2 className="w-8 h-8 text-white animate-spin" />
                  </div>
                  <h2 className={`text-base font-bold text-center px-2 ${isDarkMode ? 'text-slate-100' : 'text-slate-800'}`}>{loadingStatus}</h2>
              </div>
          </div>
      )}
      
      {/* Error State */}
      {loadError && (
          <div className="absolute inset-0 flex flex-col items-center justify-center bg-white/40 backdrop-blur-xl z-50">
              <div className="glass-panel p-10 rounded-[2.5rem] border-red-100 max-w-md text-center">
                  <div className="w-16 h-16 bg-red-50 text-red-600 rounded-full flex items-center justify-center mx-auto mb-6">
                    <AlertCircle className="w-8 h-8" />
                  </div>
                  <h3 className="text-2xl text-slate-800 font-bold mb-2">Simulation Halted</h3>
                  <p className="text-slate-500 mb-8 leading-relaxed">{loadError}</p>
                  <button 
                    onClick={() => window.location.reload()} 
                    className="w-full py-4 bg-slate-900 text-white rounded-2xl font-bold hover:bg-black transition-all shadow-xl active:scale-95"
                  >
                    Retry Loading
                  </button>
              </div>
          </div>
      )}
      
      {/* Main UI Controls */}
      {!isLoading && !loadError && (
          <Toolbar 
            isPaused={isPaused} 
            togglePause={() => setIsPaused(simRef.current?.togglePause() ?? false)} 
            onReset={handleReset} 
            isDarkMode={isDarkMode}
            toggleDarkMode={toggleDarkMode}
            isIkEnabled={isIkEnabled}
            toggleIk={toggleIk}
            supportsIk={supportsIk}
          />
      )}
    </div>
  );
}
