
/**
 * @license
 * SPDX-License-Identifier: Apache-2.0
*/


import { MujocoModule } from "./types";

/**
 * RobotLoader
 * Handles fetching robot XML files and their dependencies (meshes, textures) from remote URLs.
 * It writes these files into MuJoCo's in-memory virtual filesystem so the C++ engine can read them.
 */
export class RobotLoader {
    private mujoco: MujocoModule;

    constructor(mujocoInstance: MujocoModule) {
        this.mujoco = mujocoInstance;
    }

    /**
     * Main entry point. Downloads the main scene XML and recursively finds/downloads all included files.
     * @param robotId The ID of the robot directory in MuJoCo Menagerie.
     * @param sceneFile The entry point XML file (usually 'scene.xml').
     * @param onProgress Optional callback to report loading progress string.
     */
    async load(robotId: string, sceneFile: string, onProgress?: (msg: string) => void): Promise<void> {
        // 1. Clean up the virtual filesystem from previous runs
        try { this.mujoco.FS.unmount('/working'); } catch (e) { /* ignore */ }
        try { this.mujoco.FS.mkdir('/working'); } catch (e) { /* ignore */ }

        // Base URL for standard models from DeepMind's repository
        const baseUrl = `https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/${robotId}/`;

        const downloaded = new Set<string>(); // Keep track to avoid re-downloading same file twice
        const queue: Array<string> = []; // Queue of files to process
        const parser = new DOMParser(); // For parsing XML to find dependencies

        queue.push(sceneFile);

        // Process queue until all dependencies are downloaded
        while (queue.length > 0) {
            const fname = queue.shift()!;
            if (downloaded.has(fname)) continue;
            downloaded.add(fname);

            if (onProgress) {
                onProgress(`Downloading ${fname}...`);
            }

            // Fetch file from network
            const url = baseUrl + fname;
            const res = await fetch(url);
            if (!res.ok) {
                console.warn(`Failed to fetch ${url}: ${res.status} ${res.statusText}`);
                continue;
            }

            // Ensure virtual directory structure exists (e.g., /working/assets/meshes/)
            const dirParts = fname.split('/');
            dirParts.pop(); // remove filename
            let currentPath = '/working';
            for (const part of dirParts) {
                currentPath += '/' + part;
                try { this.mujoco.FS.mkdir(currentPath); } catch (e) { /* ignore */ }
            }

            // If it's an XML, scan it for more dependencies
            if (fname.endsWith('.xml')) {
                const text = await res.text();
                
                // Write text file to virtual FS
                this.mujoco.FS.writeFile(`/working/${fname}`, text);
                
                // Scan for <include file="...">, <mesh file="...">, etc.
                this.scanDependencies(text, fname, parser, downloaded, queue);
            } else {
                // Binary files (STL, PNG, OBJ) just get written directly
                const buffer = new Uint8Array(await res.arrayBuffer());
                this.mujoco.FS.writeFile(`/working/${fname}`, buffer);
            }
        }
    }

    // Finds all files referenced in the XML so we can download them too
    private scanDependencies(xmlString: string, currentFile: string, parser: DOMParser, downloaded: Set<string>, queue: string[]) {
        const xmlDoc = parser.parseFromString(xmlString, 'text/xml');
        
        // Check if the XML defines specific directories for assets
        const compiler = xmlDoc.querySelector('compiler');
        const meshDir = compiler?.getAttribute('meshdir') || '';
        const textureDir = compiler?.getAttribute('texturedir') || '';
        
        // Calculate relative path of current file (to resolve relative paths in includes)
        const currentDir = currentFile.includes('/') ? currentFile.substring(0, currentFile.lastIndexOf('/') + 1) : '';

        // Find all elements with a 'file' attribute
        xmlDoc.querySelectorAll('[file]').forEach(el => {
            const fileAttr = el.getAttribute('file');
            if (!fileAttr) return;
            
            // Prepend appropriate directory based on tag type
            let prefix = '';
            if (el.tagName.toLowerCase() === 'mesh') {
                prefix = meshDir ? meshDir + '/' : '';
            } else if (['texture', 'hfield'].includes(el.tagName.toLowerCase())) {
                prefix = textureDir ? textureDir + '/' : '';
            }
            
            // Normalize path (resolve '..' and '.')
            // Combines current directory of the XML, the asset prefix, and the file attribute
            let fullPath = (currentDir + prefix + fileAttr).replace(/\/\//g, '/');
            
            // Simple path normalization for '..'
            const parts = fullPath.split('/');
            const norm: string[] = [];
            for (const p of parts) { 
                if (p === '..') norm.pop(); 
                else if (p !== '.') norm.push(p); 
            }
            fullPath = norm.join('/');
            
            // Add to queue if we haven't seen it yet
            if (!downloaded.has(fullPath)) queue.push(fullPath);
        });
    }
}
