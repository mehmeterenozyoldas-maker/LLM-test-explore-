
/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
*/

import { AxisMode, ControlMap } from "../types";

interface RobotSelectorProps {
  robotName: string;
  leftStats: { pos: string, rot: string } | null;
  rightStats: { pos: string, rot: string } | null;
  handCount: number;
  isDarkMode: boolean;
  isWebcamOn: boolean;
  controlMap: ControlMap;
  onControlChange: (arm: 'left' | 'right', axis: 'red' | 'green' | 'blue', mode: AxisMode) => void;
}

/**
 * RobotSelector
 * Overlay displaying current robot info and control mappings.
 */
export function RobotSelector({ 
  robotName, 
  leftStats, 
  rightStats, 
  handCount, 
  isDarkMode, 
  isWebcamOn,
  controlMap,
  onControlChange
}: RobotSelectorProps) {
  const panelStyle = isDarkMode ? "bg-slate-900/80 border-white/10 text-slate-100 shadow-slate-900/20" : "bg-white/70 border-white/80 text-slate-800 shadow-slate-100/10";
  const labelStyle = isDarkMode ? "text-slate-400" : "text-slate-500";
  const valueStyle = isDarkMode ? "text-slate-300" : "text-slate-700";
  const subHeaderStyle = isDarkMode ? "text-indigo-400" : "text-indigo-600";
  const selectStyle = isDarkMode ? "bg-slate-800 border-slate-700 text-slate-300" : "bg-white border-slate-200 text-slate-600";

  const renderControl = (arm: 'left'|'right', axis: 'red'|'green'|'blue', colorClass: string) => (
    <div className="flex items-center justify-between gap-2">
      <span className={`text-[10px] font-bold uppercase w-8 ${colorClass}`}>{axis}</span>
      <select 
        value={controlMap[arm][axis]}
        onChange={(e) => onControlChange(arm, axis, e.target.value as AxisMode)}
        className={`flex-1 text-[9px] font-bold py-1 px-1.5 rounded border focus:outline-none focus:ring-1 focus:ring-indigo-500 ${selectStyle}`}
      >
        <option>Up/Down</option>
        <option>Left/Right</option>
        <option>Close/Far</option>
        <option>Static</option>
      </select>
    </div>
  );

  return (
    <div className="absolute top-6 left-6 z-20 flex flex-col gap-4 max-w-sm">
      <div className={`glass-panel px-6 py-4 rounded-[2rem] shadow-2xl pointer-events-none ${panelStyle}`}>
        <h1 className="text-xl font-bold tracking-tight leading-none text-center mb-2">{robotName}</h1>
        <div className="flex justify-center gap-4 text-[10px] font-bold uppercase tracking-widest border-t border-dashed border-current/10 pt-2 opacity-80">
            <span className={isWebcamOn ? "text-emerald-500" : "text-slate-400"}>
                Webcam: {isWebcamOn ? "Active" : "Off"}
            </span>
            <span className={handCount > 0 ? "text-indigo-500" : "text-slate-400"}>
                Hands: {handCount}
            </span>
        </div>
      </div>
      
      {(leftStats || rightStats) && (
        <div className={`glass-card px-5 py-4 rounded-3xl flex flex-col gap-4 shadow-lg backdrop-blur-xl ${isDarkMode ? 'bg-slate-800/80 border-white/10' : 'bg-white/60 border-white/60'}`}>
          
          {leftStats && (
            <div className="space-y-2">
                <h3 className={`text-[10px] font-black uppercase tracking-widest ${subHeaderStyle}`}>Left Arm</h3>
                
                {/* Control Mapping */}
                <div className="space-y-1.5 pb-2 border-b border-dashed border-current/10">
                   {renderControl('left', 'red', 'text-rose-500')}
                   {renderControl('left', 'green', 'text-emerald-500')}
                   {renderControl('left', 'blue', 'text-blue-500')}
                </div>

                <div className="font-mono text-[9px] space-y-1 bg-black/5 rounded-lg p-2 dark:bg-black/20 pointer-events-none">
                    <p className="flex justify-between"><span className={labelStyle}>POS:</span> <span className={valueStyle}>{leftStats.pos}</span></p>
                    <p className="flex justify-between"><span className={labelStyle}>ROT:</span> <span className={valueStyle}>{leftStats.rot}</span></p>
                </div>
            </div>
          )}

          {rightStats && (
            <div className="space-y-2">
                <h3 className={`text-[10px] font-black uppercase tracking-widest ${subHeaderStyle}`}>Right Arm</h3>

                {/* Control Mapping */}
                <div className="space-y-1.5 pb-2 border-b border-dashed border-current/10">
                   {renderControl('right', 'red', 'text-rose-500')}
                   {renderControl('right', 'green', 'text-emerald-500')}
                   {renderControl('right', 'blue', 'text-blue-500')}
                </div>

                <div className="font-mono text-[9px] space-y-1 bg-black/5 rounded-lg p-2 dark:bg-black/20 pointer-events-none">
                    <p className="flex justify-between"><span className={labelStyle}>POS:</span> <span className={valueStyle}>{rightStats.pos}</span></p>
                    <p className="flex justify-between"><span className={labelStyle}>ROT:</span> <span className={valueStyle}>{rightStats.rot}</span></p>
                </div>
            </div>
          )}

        </div>
      )}
    </div>
  );
}