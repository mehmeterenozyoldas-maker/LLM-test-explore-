# 4-Phase Plan: Drum Design & 3D Maturity

This document outlines the strategic roadmap for elevating the custom 3D drum kit from its current structural layout to a highly mature, photorealistic, and interactive 3D instrument within the robot simulation environment.

## Phase 1: Foundation & Enhanced Geometry
**Objective:** Solidify the base structures and progress from basic primitive shapes to highly detailed, realistic mechanical models.
* **Hardware Detailing:** Model true drum hardware, including tension rods, tension casings (lugs) around the drum shells, and wingnuts on the stands.
* **Cymbal Profiling:** Reshape the hi-hat and future cymbals to feature a genuine acoustic curved profile (bell, bow, edge) instead of thin cylinders.
* **Pedals & Mounts:** Model realistic kick drum pedals and hi-hat foot pedals on the table surface to fully anchor the kit.
* **Drum Thrones & Mounts:** Add suspension mounts for the toms rather than simple stick stands.

## Phase 2: High-Fidelity Texturing & Materials (PBR Pipeline)
**Objective:** Move from simple colors to complete Physically Based Rendering (PBR) materials to capture real-world lighting interaction.
* **Drum Shell Finishes:** Implement advanced textures such as glossy wood veneers, acrylic wraps, or brushed metal with accurate roughness/albedo/normal maps.
* **Worn Drum Skins:** Introduce wear-and-tear textures (e.g., stick impact marks, coating wear) in the center of the snare and tom heads to indicate usage. Add authentic drumhead logos.
* **Photorealistic Chrome:** Upgrade all hardware (rims, stands, lugs) to high-metalness/low-roughness Chrome shaders.
* **Anisotropic Cymbals:** Map concentric groove normal maps to the cymbals, providing the signature radial reflections of lathed brass and bronze.

## Phase 3: Lighting, Shadows & Atmospheric Rendering
**Objective:** Embed the 3D drums seamlessly into the environment through advanced shading and lighting mechanics.
* **Environment Mapping (HDRI):** Implement a high dynamic range environment map specifically for the drum materials so the chrome and cymbals can beautifully reflect a studio environment.
* **Screen Space Ambient Occlusion (SSAO):** Add deep, accurate contact shadows where the metal rims meet the drum skins and where the stands clasp the table.
* **Dynamic Impact Lighting:** Upgrade the current "scale pop" to include point lights that briefly illuminate the inside of the drum shells upon robot collision.
* **Bloom & Glow:** Implement post-processing bloom so that the hit-markers softly glow on the drum skins during fast-paced play.

## Phase 4: Advanced Animation, Physics & Spatial Integration
**Objective:** Achieve peak 3D maturity through hyper-realistic motion and interactive physics.
* **Cymbal Sway Dynamics:** Add rotational physics (damped harmonic oscillators) to the cymbals so they naturally swing, tilt, and settle based on the angle and velocity of the robotic gripper's strike.
* **Skin Deformation Shader:** Write a custom WebGL vertex shader that creates a physical ripple/depression effect across the drum skin topology exactly at the XYZ coordinate of the impact.
* **Particle Systems:** On high-velocity impacts from the robot arms, trigger subtle physical particle systems (e.g., small dust puffs, wood chips, or stylized sparks) to emphasize the force applied by MuJoCo.
* **True 3D Spatial Audio:** Connect the `Three.js` 3D coordinates securely to `WebAudio PannerNodes`, ensuring the synthesized sound actually originates from the precise location of the struck drum in the stereo field.
